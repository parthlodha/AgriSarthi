"""Small session-based UDP command client for the ESP32 motor controller."""

from __future__ import annotations

import json
import secrets
import socket
import time
from typing import Any


class ESP32Link:
    def __init__(self, host: str, port: int, ack_timeout_s: float = 0.6,
                 hello_interval_s: float = 0.5, sock: socket.socket | None = None,
                 session_id: str | None = None):
        self.remote = (socket.gethostbyname(host), int(port))
        self.ack_timeout = max(0.2, float(ack_timeout_s))
        self.hello_interval = max(0.1, float(hello_interval_s))
        self.session_id = session_id or secrets.token_hex(8)
        self.seq = 0
        self.last_ack = 0.0
        self.last_hello = 0.0
        self.last_ack_seq: int | None = None
        self.last_sent_seq: int | None = None
        self.pending_sequences: list[int] = []
        self.online = False
        self.status: dict[str, Any] = {}
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.setblocking(False)

    def _send(self, packet: dict[str, Any]) -> None:
        data = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        self.sock.sendto(data, self.remote)

    def poll(self, now: float | None = None) -> list[dict[str, Any]]:
        now = time.monotonic() if now is None else now
        received = []
        while True:
            try:
                data, sender = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            except OSError:
                self.online = False
                break
            try:
                packet = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if sender != self.remote:
                continue
            if not isinstance(packet, dict) or packet.get("session_id") != self.session_id:
                continue
            kind = packet.get("type")
            if kind == "hello_ack":
                self.online = True
                self.last_ack = now
                self.status = packet
            elif kind in ("ack", "probe_ack", "probe_status"):
                if kind == "probe_status":
                    # Actuator status is also an authenticated, current-session
                    # response and therefore proves that the link is alive.
                    self.online = True
                    self.last_ack = now
                    self.status = packet
                    received.append(packet)
                    continue
                try:
                    ack_seq = int(packet["seq"])
                except (KeyError, TypeError, ValueError):
                    continue
                if ack_seq not in self.pending_sequences:
                    continue
                self.pending_sequences.remove(ack_seq)
                if kind == "ack" and packet.get("accepted") is not True:
                    continue
                self.online = True
                self.last_ack = now
                self.last_ack_seq = ack_seq
                self.status = packet
            else:
                continue
            received.append(packet)
        if now - self.last_ack > self.ack_timeout:
            self.online = False
        return received

    def ready(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return self.online and now - self.last_ack <= self.ack_timeout

    def send_hello(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        if now - self.last_hello < self.hello_interval:
            return False
        self._send({"type": "hello", "protocol": 1, "session_id": self.session_id})
        self.last_hello = now
        return True

    def send_command(self, command: dict[str, Any], now: float | None = None) -> int | None:
        now = time.monotonic() if now is None else now
        if not self.ready(now):
            return None
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        packet = {
            "type": "motor_cmd", "protocol": 1, "session_id": self.session_id,
            "seq": self.seq, "enable": bool(command.get("enable", False)),
            "left": float(command.get("left", 0.0)),
            "right": float(command.get("right", 0.0)),
            "linear": float(command.get("linear", 0.0)),
            "angular": float(command.get("angular", 0.0)),
        }
        self._send(packet)
        self.last_sent_seq = self.seq
        self._remember_pending(self.seq)
        return self.seq

    def send_probe_command(self, action: str, command_id: str,
                           now: float | None = None) -> int | None:
        """Send an idempotency-keyed probe command through the active Pi session."""
        if action not in ("PROBE_INSERT", "PROBE_RETRACT", "PROBE_STATUS"):
            raise ValueError("unsupported probe action")
        if not isinstance(command_id, str) or not command_id or len(command_id) > 64:
            raise ValueError("command_id must be 1..64 characters")
        now = time.monotonic() if now is None else now
        if not self.ready(now):
            return None
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self._send({"type": "probe_cmd", "protocol": 1, "session_id": self.session_id,
                    "seq": self.seq, "command_id": command_id, "action": action})
        self.last_sent_seq = self.seq
        self._remember_pending(self.seq)
        return self.seq

    def _remember_pending(self, seq: int) -> None:
        self.pending_sequences.append(seq)
        if len(self.pending_sequences) > 64:
            del self.pending_sequences[:-64]

    def close(self) -> None:
        try:
            if self.ready():
                for _ in range(3):
                    self.send_command({"enable": False, "left": 0.0, "right": 0.0})
                    time.sleep(0.02)
        except OSError:
            pass
        self.sock.close()
