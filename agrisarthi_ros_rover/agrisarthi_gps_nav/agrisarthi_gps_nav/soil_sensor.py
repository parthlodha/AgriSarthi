"""One-shot Modbus RTU reader for the Agrisarthi RS485 soil sensor."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class SoilReadResult:
    success: bool
    data: dict | None
    error: str | None
    attempts: int


class SoilSensorError(Exception):
    pass


def crc16(data: bytes | bytearray) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_read_request(slave_id: int, start_addr: int, register_count: int) -> bytes:
    if not 1 <= slave_id <= 247:
        raise ValueError("Modbus slave ID must be between 1 and 247")
    if not 0 <= start_addr <= 0xFFFF or not 1 <= register_count <= 125:
        raise ValueError("Modbus register address/count is outside the protocol range")
    frame = bytearray((slave_id, 0x03, start_addr >> 8, start_addr & 0xFF,
                       register_count >> 8, register_count & 0xFF))
    crc = crc16(frame)
    frame.extend((crc & 0xFF, crc >> 8))
    return bytes(frame)


def read_registers(serial_port, slave_id: int, start_addr: int, register_count: int,
                   response_delay_s: float = 0.3) -> list[int]:
    """Issue function 0x03 and strictly validate address, byte count and CRC."""
    request = build_read_request(slave_id, start_addr, register_count)
    try:
        serial_port.reset_input_buffer()
        serial_port.write(request)
        serial_port.flush()
        if response_delay_s > 0:
            time.sleep(response_delay_s)
        expected_length = 5 + 2 * register_count
        response = serial_port.read(expected_length)
    except Exception as exc:
        raise SoilSensorError(f"serial I/O failed: {exc}") from exc
    if len(response) != expected_length:
        raise SoilSensorError(f"short Modbus response: expected {expected_length} bytes, received {len(response)}")
    if response[0] != slave_id:
        raise SoilSensorError(f"unexpected Modbus slave ID {response[0]}")
    if response[1] == (0x03 | 0x80):
        _validate_crc(response)
        raise SoilSensorError(f"Modbus exception response code 0x{response[2]:02X}")
    if response[1] != 0x03:
        raise SoilSensorError(f"unexpected Modbus function code 0x{response[1]:02X}")
    if response[2] != register_count * 2:
        raise SoilSensorError(f"unexpected Modbus byte count {response[2]}")
    _validate_crc(response)
    return [int.from_bytes(response[3 + i * 2:5 + i * 2], "big")
            for i in range(register_count)]


def _validate_crc(response: bytes) -> None:
    received = response[-2] | (response[-1] << 8)
    calculated = crc16(response[:-2])
    if received != calculated:
        raise SoilSensorError("Modbus CRC16 mismatch")


class SoilSensor:
    """Configurable, retrying sensor acquisition with no display loop or globals."""

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 4800,
                 slave_id: int = 1, timeout_s: float = 1.0, retry_count: int = 3,
                 response_delay_s: float = 0.3, retry_delay_s: float = 0.2,
                 serial_factory: Callable | None = None):
        if baudrate <= 0 or timeout_s <= 0 or retry_count < 1:
            raise ValueError("baudrate/timeout must be positive and retry_count at least one")
        if not 1 <= slave_id <= 247:
            raise ValueError("Modbus slave ID must be between 1 and 247")
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout_s = timeout_s
        self.retry_count = retry_count
        self.response_delay_s = max(0.0, response_delay_s)
        self.retry_delay_s = max(0.0, retry_delay_s)
        self.serial_factory = serial_factory

    def _serial(self):
        if self.serial_factory is not None:
            return self.serial_factory(port=self.port, baudrate=self.baudrate, bytesize=8,
                                       parity="N", stopbits=1, timeout=self.timeout_s)
        try:
            import serial
        except ImportError as exc:
            raise SoilSensorError("pyserial is required for RS485 sensor acquisition") from exc
        return serial.Serial(port=self.port, baudrate=self.baudrate, bytesize=8,
                             parity=serial.PARITY_NONE, stopbits=1, timeout=self.timeout_s)

    def read(self) -> SoilReadResult:
        errors = []
        for attempt in range(1, self.retry_count + 1):
            try:
                with self._serial() as connection:
                    values = self._read_complete_sample(connection)
                return SoilReadResult(True, values, None, attempt)
            except (SoilSensorError, OSError, ValueError) as exc:
                errors.append(f"attempt {attempt}: {exc}")
                if attempt < self.retry_count and self.retry_delay_s:
                    time.sleep(self.retry_delay_s)
        return SoilReadResult(False, None, "; ".join(errors), self.retry_count)

    def _read_complete_sample(self, connection) -> dict:
        basic = read_registers(connection, self.slave_id, 0x0000, 4, self.response_delay_s)
        moisture = basic[0] / 10.0
        temperature = basic[1] / 10.0
        ec = basic[2]  # Preserve the register value; source script labels it µS/cm.
        ph = basic[3] / 10.0
        if not math.isfinite(moisture) or not 0.0 <= moisture <= 100.0:
            raise SoilSensorError(f"invalid moisture register value ({moisture})")
        if not math.isfinite(ph) or not 0.0 <= ph <= 14.0:
            raise SoilSensorError(f"invalid pH register value ({ph})")

        primary = None
        primary_error = None
        try:
            primary = read_registers(connection, self.slave_id, 0x001E, 3, self.response_delay_s)
        except SoilSensorError as exc:
            primary_error = str(exc)
        if primary is None or all(value == 0 for value in primary):
            try:
                fallback = read_registers(connection, self.slave_id, 0x0004, 3, self.response_delay_s)
            except SoilSensorError as exc:
                raise SoilSensorError(f"NPK primary failed ({primary_error or 'all-zero'}); fallback failed ({exc})") from exc
            if not any(fallback):
                raise SoilSensorError("NPK primary and fallback registers both returned all zeros")
            primary = fallback

        if len(primary) != 3 or not any(primary):
            raise SoilSensorError("NPK response is incomplete or all zero")
        return {
            "nitrogen": int(primary[0]),
            "phosphorus": int(primary[1]),
            "potassium": int(primary[2]),
            "ph": ph,
            "moisture": moisture,
            "temperature": temperature,
            "ec": int(ec),
        }
