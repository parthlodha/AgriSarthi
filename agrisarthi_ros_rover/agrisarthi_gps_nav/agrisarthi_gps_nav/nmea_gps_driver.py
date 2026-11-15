"""NMEA-0183 GNSS driver node for u-blox NEO-M8N and compatible UART GPS receivers.

Parses NMEA sentences (GGA, RMC) and publishes sensor_msgs/msg/NavSatFix.
Provides ROS-independent parser and serial abstraction for testing.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


@dataclass(frozen=True)
class NmeaFix:
    valid: bool
    latitude: float | None = None
    longitude: float | None = None
    altitude: float | None = None
    fix_type: int = 0  # 0=Invalid, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float
    satellites: int = 0
    hdop: float | None = None
    timestamp_utc: str | None = None
    error: str | None = None


def verify_nmea_checksum(sentence: str) -> bool:
    """Validate standard NMEA-0183 sentence XOR checksum."""
    sentence = sentence.strip()
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    content, checksum_str = sentence[1:].rsplit("*", 1)
    try:
        expected = int(checksum_str[:2], 16)
    except ValueError:
        return False
    calculated = 0
    for char in content:
        calculated ^= ord(char)
    return calculated == expected


def parse_nmea_coordinate(raw_value: str, direction: str) -> float | None:
    """Convert NMEA format (DDMM.MMMM or DDDMM.MMMM) to signed decimal degrees."""
    if not raw_value or not direction:
        return None
    try:
        val = float(raw_value)
    except ValueError:
        return None
    direction = direction.upper()
    if direction not in ("N", "S", "E", "W"):
        return None

    degrees = int(val / 100.0)
    minutes = val - (degrees * 100.0)
    decimal = degrees + (minutes / 60.0)

    if direction in ("S", "W"):
        decimal = -decimal

    if direction in ("N", "S") and not (-90.0 <= decimal <= 90.0):
        return None
    if direction in ("E", "W") and not (-180.0 <= decimal <= 180.0):
        return None

    return decimal


class NmeaParser:
    """Pure-Python NMEA sentence parser independent of ROS."""

    def __init__(self, hdop_scale_m: float = 2.5):
        self.hdop_scale_m = max(0.5, float(hdop_scale_m))
        self.last_fix: NmeaFix | None = None

    def parse_line(self, line: str) -> NmeaFix | None:
        line = line.strip()
        if not line.startswith("$"):
            return None
        if not verify_nmea_checksum(line):
            return NmeaFix(valid=False, error="CHECKSUM_MISMATCH")

        content = line[1:].split("*")[0]
        fields = content.split(",")
        sentence_id = fields[0].upper()

        # Handle talker IDs: GP (GPS), GN (GNSS/GLONASS/Galileo), GL, GA, BD
        if sentence_id.endswith("GGA"):
            return self._parse_gga(fields)
        if sentence_id.endswith("RMC"):
            return self._parse_rmc(fields)
        return None

    def _parse_gga(self, fields: list[str]) -> NmeaFix:
        # Format: $GNGGA,time,lat,NS,lon,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
        if len(fields) < 10:
            return NmeaFix(valid=False, error="GGA_TOO_SHORT")

        utc_time = fields[1] if fields[1] else None
        raw_lat, ns = fields[2], fields[3]
        raw_lon, ew = fields[4], fields[5]
        raw_quality = fields[6]
        raw_satellites = fields[7]
        raw_hdop = fields[8]
        raw_altitude = fields[9]

        try:
            fix_type = int(raw_quality) if raw_quality else 0
        except ValueError:
            fix_type = 0

        valid_fix = fix_type in (1, 2, 4, 5)
        if not valid_fix:
            return NmeaFix(valid=False, fix_type=fix_type, error="NO_FIX")

        lat = parse_nmea_coordinate(raw_lat, ns)
        lon = parse_nmea_coordinate(raw_lon, ew)
        if lat is None or lon is None:
            return NmeaFix(valid=False, fix_type=fix_type, error="INVALID_COORDINATES")

        try:
            satellites = int(raw_satellites) if raw_satellites else 0
        except ValueError:
            satellites = 0

        try:
            hdop = float(raw_hdop) if raw_hdop else None
        except ValueError:
            hdop = None

        try:
            altitude = float(raw_altitude) if raw_altitude else 0.0
        except ValueError:
            altitude = 0.0

        fix = NmeaFix(
            valid=True,
            latitude=lat,
            longitude=lon,
            altitude=altitude,
            fix_type=fix_type,
            satellites=satellites,
            hdop=hdop,
            timestamp_utc=utc_time,
        )
        self.last_fix = fix
        return fix

    def _parse_rmc(self, fields: list[str]) -> NmeaFix:
        # Format: $GNRMC,time,status,lat,NS,lon,EW,spd,cog,date,mv,mvE,mode*cs
        if len(fields) < 7:
            return NmeaFix(valid=False, error="RMC_TOO_SHORT")

        utc_time = fields[1] if fields[1] else None
        status = fields[2].upper()
        if status != "A":  # 'A' = Active (valid), 'V' = Void (invalid)
            return NmeaFix(valid=False, error="RMC_STATUS_VOID")

        raw_lat, ns = fields[3], fields[4]
        raw_lon, ew = fields[5], fields[6]

        lat = parse_nmea_coordinate(raw_lat, ns)
        lon = parse_nmea_coordinate(raw_lon, ew)
        if lat is None or lon is None:
            return NmeaFix(valid=False, error="INVALID_COORDINATES")

        fix = NmeaFix(
            valid=True,
            latitude=lat,
            longitude=lon,
            altitude=0.0,
            fix_type=1,
            timestamp_utc=utc_time,
        )
        self.last_fix = fix
        return fix


class NMEAGPSDriver(Node):
    """ROS 2 Jazzy node publishing NavSatFix from UART NMEA GNSS stream."""

    def __init__(self, serial_factory: Callable | None = None):
        super().__init__("nmea_gps_driver")
        defaults = {
            "port": "/dev/ttyAMA0",
            "baudrate": 9600,
            "timeout_s": 1.0,
            "gps_topic": "/gps/fix",
            "frame_id": "gps_link",
            "hdop_scale_m": 2.5,
            "default_variance_m2": 25.0,
            "poll_interval_s": 0.05,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value

        self.port = str(p("port"))
        self.baudrate = int(p("baudrate"))
        self.timeout_s = max(0.1, float(p("timeout_s")))
        self.frame_id = str(p("frame_id"))
        self.default_variance = max(0.01, float(p("default_variance_m2")))
        self.parser = NmeaParser(hdop_scale_m=float(p("hdop_scale_m")))
        self.serial_factory = serial_factory
        self.serial_connection = None
        self.buffer = ""
        self.last_error_log = 0.0

        self.fix_pub = self.create_publisher(NavSatFix, str(p("gps_topic")), 10)
        poll_interval = max(0.01, float(p("poll_interval_s")))
        self.timer = self.create_timer(poll_interval, self.poll_serial)

        self.get_logger().info(
            f"NMEA GNSS Driver initialized. Port: {self.port} @ {self.baudrate} baud -> {p('gps_topic')}"
        )

    def _open_serial(self):
        if self.serial_factory is not None:
            return self.serial_factory(port=self.port, baudrate=self.baudrate, timeout=self.timeout_s)
        try:
            import serial
            return serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout_s)
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_error_log >= 5.0:
                self.get_logger().warning(f"Could not open GNSS UART port '{self.port}': {exc}")
                self.last_error_log = now
            return None

    def poll_serial(self):
        if self.serial_connection is None:
            self.serial_connection = self._open_serial()
            if self.serial_connection is None:
                return

        try:
            # Read available bytes without blocking the executor indefinitely
            if hasattr(self.serial_connection, "in_waiting"):
                available = self.serial_connection.in_waiting
                data = self.serial_connection.read(max(1, available)) if available else b""
            else:
                data = self.serial_connection.read(256)

            if not data:
                return

            text = data.decode("ascii", errors="replace")
            self.buffer += text

            while "\n" in self.buffer:
                line, self.buffer = self.buffer.split("\n", 1)
                line = line.strip()
                if line:
                    fix = self.parser.parse_line(line)
                    if fix is not None and fix.valid:
                        self.publish_fix(fix)
        except Exception as exc:
            self.get_logger().warning(f"GNSS serial I/O error: {exc}")
            try:
                if self.serial_connection:
                    self.serial_connection.close()
            except Exception:
                pass
            self.serial_connection = None

    def publish_fix(self, fix: NmeaFix):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Status mapping
        if fix.fix_type == 2:
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        elif fix.fix_type in (1, 4, 5):
            msg.status.status = NavSatStatus.STATUS_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX

        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = float(fix.latitude or 0.0)
        msg.longitude = float(fix.longitude or 0.0)
        msg.altitude = float(fix.altitude or 0.0)

        # Estimate horizontal covariance from HDOP if present
        if fix.hdop and fix.hdop > 0:
            h_var = (fix.hdop * self.parser.hdop_scale_m) ** 2
            v_var = (fix.hdop * self.parser.hdop_scale_m * 2.0) ** 2
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            h_var = self.default_variance
            v_var = self.default_variance * 4.0
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        msg.position_covariance[0] = h_var
        msg.position_covariance[4] = h_var
        msg.position_covariance[8] = v_var

        self.fix_pub.publish(msg)

    def destroy_node(self):
        if self.serial_connection:
            try:
                self.serial_connection.close()
            except Exception:
                pass
            self.serial_connection = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NMEAGPSDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
