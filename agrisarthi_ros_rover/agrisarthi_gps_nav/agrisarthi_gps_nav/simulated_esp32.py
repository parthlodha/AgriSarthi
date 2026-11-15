"""ROS-only ESP32 protocol adapter and deterministic ESP32-C6 simulator.

The simulator uses the same JSON packet types, session handling, shared sequence
space, acknowledgements, probe command IDs, and motor watchdog as the UDP client
and firmware. It never opens a socket.
"""

from __future__ import annotations

import json
import math
import secrets
import time
from collections import deque
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _seq_newer(candidate: int, previous: int) -> bool:
    delta = (candidate - previous) & 0xFFFFFFFF
    return delta != 0 and delta < 0x80000000


class ROSSimulatedESP32Link:
    """Navigator-side transport implementing the production ESP32Link API."""

    def __init__(self, node: Node, command_topic: str, feedback_topic: str,
                 ack_timeout_s: float = 0.6, hello_interval_s: float = 0.5,
                 session_id: str | None = None):
        self.node = node
        self.session_id = session_id or secrets.token_hex(8)
        self.ack_timeout = max(0.2, float(ack_timeout_s))
        self.hello_interval = max(0.1, float(hello_interval_s))
        self.seq = 0
        self.last_ack = 0.0
        self.last_hello = 0.0
        self.last_ack_seq: int | None = None
        self.last_sent_seq: int | None = None
        self.pending_sequences: list[int] = []
        self.online = False
        self.status: dict[str, Any] = {}
        self.feedback: deque[dict[str, Any]] = deque()
        self.publisher = node.create_publisher(String, command_topic, 10)
        self.subscription = node.create_subscription(String, feedback_topic, self._on_feedback, 10)

    def _on_feedback(self, msg: String) -> None:
        try:
            packet = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(packet, dict):
            self.feedback.append(packet)

    def _send(self, packet: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(packet, separators=(",", ":"))
        self.publisher.publish(msg)

    def poll(self, now: float | None = None) -> list[dict[str, Any]]:
        now = time.monotonic() if now is None else now
        received = []
        while self.feedback:
            packet = self.feedback.popleft()
            if packet.get("session_id") != self.session_id:
                continue
            kind = packet.get("type")
            if kind == "hello_ack":
                self.online = True
                self.last_ack = now
                self.status = packet
            elif kind == "probe_status":
                self.online = True
                self.last_ack = now
                self.status = packet
            elif kind in ("ack", "probe_ack"):
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

    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self.last_sent_seq = self.seq
        self.pending_sequences.append(self.seq)
        if len(self.pending_sequences) > 64:
            del self.pending_sequences[:-64]
        return self.seq

    def send_command(self, command: dict[str, Any], now: float | None = None) -> int | None:
        now = time.monotonic() if now is None else now
        if not self.ready(now):
            return None
        seq = self._next_seq()
        self._send({"type": "motor_cmd", "protocol": 1, "session_id": self.session_id,
                    "seq": seq, "enable": bool(command.get("enable", False)),
                    "left": float(command.get("left", 0.0)),
                    "right": float(command.get("right", 0.0)),
                    "linear": float(command.get("linear", 0.0)),
                    "angular": float(command.get("angular", 0.0))})
        return seq

    def send_probe_command(self, action: str, command_id: str,
                           now: float | None = None) -> int | None:
        if action not in ("PROBE_INSERT", "PROBE_RETRACT", "PROBE_STATUS"):
            raise ValueError("unsupported probe action")
        if not isinstance(command_id, str) or not command_id or len(command_id) > 64:
            raise ValueError("command_id must be 1..64 characters")
        now = time.monotonic() if now is None else now
        if not self.ready(now):
            return None
        seq = self._next_seq()
        self._send({"type": "probe_cmd", "protocol": 1, "session_id": self.session_id,
                    "seq": seq, "command_id": command_id, "action": action})
        return seq

    def close(self) -> None:
        if self.ready():
            self.send_command({"enable": False})


class SimulatedESP32Node(Node):
    """Deterministic ROS endpoint for the physical ESP32 JSON protocol."""

    def __init__(self):
        super().__init__("simulated_esp32")
        defaults = {
            "command_topic": "/sim/esp32/command", "feedback_topic": "/sim/esp32/feedback",
            "fault_topic": "/sim/esp32/faults", "probe_motion_duration_s": 0.25,
            "motor_watchdog_s": 0.7, "fail_retractions": 0,
            "duplicate_acknowledgements": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.motion_duration = max(0.01, float(p("probe_motion_duration_s")))
        self.watchdog = max(0.1, float(p("motor_watchdog_s")))
        self.fail_retractions = max(0, int(p("fail_retractions")))
        self.duplicate_acks = bool(p("duplicate_acknowledgements"))
        self.publisher = self.create_publisher(String, p("feedback_topic"), 10)
        self.create_subscription(String, p("command_topic"), self.on_command, 10)
        self.create_subscription(String, p("fault_topic"), self.on_fault, 10)
        self.timer = self.create_timer(0.02, self.tick)
        self.session_id = ""
        self.last_seq: int | None = None
        self.command_history: dict[str, tuple[str, bool]] = {}
        self.history_order: deque[str] = deque()
        self.probe_state = "RETRACTED"
        self.probe_error = ""
        self.position_assumed = True
        self.active_command_id = ""
        self.active_action = ""
        self.motion_started = 0.0
        self.fail_active_retraction = False
        self.motors_enabled = False
        self.left = 0.0
        self.right = 0.0
        self.sim_linear = 0.0
        self.sim_angular = 0.0
        self.last_command_at = time.monotonic()

    def publish(self, packet: dict[str, Any]) -> None:
        packet.setdefault("protocol", 1)
        packet.setdefault("session_id", self.session_id)
        packet.setdefault("wifi_connected", True)
        packet.setdefault("motors_enabled", self.motors_enabled)
        packet.setdefault("left", self.left)
        packet.setdefault("right", self.right)
        packet.setdefault("probe_state", self.probe_state)
        packet.setdefault("probe_error", self.probe_error)
        packet.setdefault("position_assumed", self.position_assumed)
        msg = String(); msg.data = json.dumps(packet, separators=(",", ":"))
        self.publisher.publish(msg)
        if self.duplicate_acks and packet.get("type") in ("ack", "probe_ack"):
            self.publisher.publish(msg)

    def stop_motors(self) -> None:
        self.motors_enabled = False
        self.left = self.right = 0.0
        self.sim_linear = self.sim_angular = 0.0

    def on_fault(self, msg: String) -> None:
        try:
            fault = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if fault.get("action") == "FAIL_NEXT_RETRACTION":
            self.fail_retractions += 1

    def on_command(self, msg: String) -> None:
        try:
            packet = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(packet, dict):
            return
        kind = packet.get("type")
        session = packet.get("session_id")
        if not isinstance(session, str) or not session:
            return
        if kind == "hello":
            if session != self.session_id:
                self.stop_motors()
                self.session_id = session
                self.last_seq = None
            self.last_command_at = time.monotonic()
            self.publish({"type": "hello_ack"})
            return
        if session != self.session_id or kind not in ("motor_cmd", "probe_cmd"):
            return
        try:
            seq = int(packet["seq"])
        except (KeyError, TypeError, ValueError):
            return
        if not 0 <= seq <= 0xFFFFFFFF:
            return
        newer = self.last_seq is None or _seq_newer(seq, self.last_seq)
        if not newer:
            self.handle_stale_packet(packet, seq)
            return
        self.last_seq = seq
        self.last_command_at = time.monotonic()
        if kind == "motor_cmd":
            self.handle_motor(packet, seq)
        else:
            self.handle_probe(packet, seq)

    def handle_stale_packet(self, packet: dict[str, Any], seq: int) -> None:
        if packet.get("type") == "motor_cmd":
            self.publish({"type": "ack", "seq": seq, "accepted": False})
            return
        command_id, action = packet.get("command_id"), packet.get("action")
        cached = self.command_history.get(str(command_id))
        accepted = bool(cached and cached == (action, True))
        self.publish({"type": "probe_ack", "seq": seq, "accepted": accepted,
                      "duplicate": cached is not None, "command_id": command_id or ""})

    def remember(self, command_id: str, action: str, accepted: bool) -> None:
        if command_id not in self.command_history:
            self.history_order.append(command_id)
        self.command_history[command_id] = (action, accepted)
        while len(self.history_order) > 32:
            expired = self.history_order.popleft()
            self.command_history.pop(expired, None)

    def handle_motor(self, packet: dict[str, Any], seq: int) -> None:
        try:
            left, right = float(packet.get("left", 0)), float(packet.get("right", 0))
        except (TypeError, ValueError):
            left = right = math.nan
        accepted = math.isfinite(left) and math.isfinite(right)
        if accepted:
            if packet.get("enable") is True and self.probe_state == "RETRACTED":
                self.left = max(-1.0, min(1.0, left))
                self.right = max(-1.0, min(1.0, right))
                self.motors_enabled = self.left != 0.0 or self.right != 0.0
                self.sim_linear = float(packet.get("linear", 0.0)) if self.motors_enabled else 0.0
                self.sim_angular = float(packet.get("angular", 0.0)) if self.motors_enabled else 0.0
            else:
                self.stop_motors()
        self.publish({"type": "ack", "seq": seq, "accepted": accepted,
                      "sim_linear": self.sim_linear, "sim_angular": self.sim_angular})

    def handle_probe(self, packet: dict[str, Any], seq: int) -> None:
        command_id, action = packet.get("command_id"), packet.get("action")
        valid_id = isinstance(command_id, str) and 1 <= len(command_id) <= 64
        accepted = False
        duplicate = False
        if valid_id and action in ("PROBE_INSERT", "PROBE_RETRACT", "PROBE_STATUS"):
            cached = self.command_history.get(command_id)
            if cached is not None:
                duplicate = True
                accepted = cached == (action, True)
            elif action == "PROBE_STATUS":
                accepted = True
                self.active_command_id = command_id
                self.remember(command_id, action, accepted)
            elif action == "PROBE_INSERT" and self.probe_state == "RETRACTED":
                accepted = True
                self.stop_motors()
                self.probe_state = "INSERTING"
                self.probe_error = ""
                self.position_assumed = True
                self.active_action = action
                self.active_command_id = command_id
                self.motion_started = time.monotonic()
                self.remember(command_id, action, accepted)
            elif action == "PROBE_INSERT" and self.probe_state == "INSERTED":
                accepted = True
                self.active_command_id = command_id
                self.remember(command_id, action, accepted)
            elif action == "PROBE_RETRACT" and self.probe_state == "RETRACTED":
                accepted = True
                self.position_assumed = True
                self.active_command_id = command_id
                self.remember(command_id, action, accepted)
            elif action == "PROBE_RETRACT" and self.probe_state in ("INSERTED", "ERROR", "IDLE", "INSERTING"):
                accepted = True
                self.stop_motors()
                self.probe_state = "RETRACTING"
                self.probe_error = ""
                self.position_assumed = True
                self.active_action = action
                self.active_command_id = command_id
                self.motion_started = time.monotonic()
                self.fail_active_retraction = self.fail_retractions > 0
                if self.fail_active_retraction:
                    self.fail_retractions -= 1
                self.remember(command_id, action, accepted)
        elif valid_id:
            self.probe_error = "INVALID_COMMAND"
            self.remember(command_id, str(action), False)
        if not accepted and self.probe_state not in ("INSERTING", "RETRACTING"):
            self.probe_error = self.probe_error or "ACTUATOR_BUSY_OR_INVALID"
        self.stop_motors()
        self.publish({"type": "probe_ack", "seq": seq, "accepted": accepted,
                      "duplicate": duplicate, "command_id": command_id if valid_id else ""})

    def tick(self) -> None:
        now = time.monotonic()
        if self.motors_enabled and now - self.last_command_at > self.watchdog:
            self.stop_motors()
            self.publish({"type": "status", "seq": 0, "accepted": True})
            self.get_logger().warning("Simulated ESP32 motor watchdog stopped the motors.")
        if self.probe_state not in ("INSERTING", "RETRACTING"):
            return
        if now - self.motion_started < self.motion_duration:
            return
        if self.active_action == "PROBE_RETRACT" and self.fail_active_retraction:
            self.probe_state = "ERROR"
            self.probe_error = "SIMULATED_RETRACTION_FAILURE"
        else:
            self.probe_state = "INSERTED" if self.active_action == "PROBE_INSERT" else "RETRACTED"
            self.probe_error = ""
        self.stop_motors()
        self.publish({"type": "probe_status", "seq": 0, "accepted": True,
                      "command_id": self.active_command_id})


def simulated_esp32_main(args=None):
    rclpy.init(args=args)
    node = SimulatedESP32Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
