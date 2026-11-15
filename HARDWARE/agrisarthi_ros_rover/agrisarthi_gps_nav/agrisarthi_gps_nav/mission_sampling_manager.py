#!/usr/bin/env python3
"""Coordinates stopped-at-waypoint sampling, probe actuation and durable results."""

from __future__ import annotations

import json
import math
import re
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String

from agrisarthi_gps_nav.mission_data import append_record, atomic_write, load_document, new_document
from agrisarthi_gps_nav.navigation_core import validate_mission
from agrisarthi_gps_nav.soil_sensor import SoilSensor


class MissionSamplingManager(Node):
    """Fail-closed orchestrator. Sensor I/O runs off the ROS executor thread."""

    def __init__(self):
        super().__init__("mission_sampling_manager")
        defaults = {
            "mission_topic": "/mission/waypoints_json", "start_topic": "/mission/start",
            "navigation_start_topic": "/mission/navigation_start",
            "navigation_complete_topic": "/mission/waypoint_complete",
            "navigation_status_topic": "/rover/nav_status", "gps_topic": "/gps/fix",
            "probe_command_topic": "/probe/command", "probe_status_topic": "/probe/status",
            "sampling_status_topic": "/mission/sampling_status", "recovery_topic": "/mission/sampling_recovery",
            "query_topic": "/mission/query", "status_response_topic": "/mission/status_response",
            "data_response_topic": "/mission/data_response",
            "command_velocity_topic": "/cmd_vel", "rover_id": "ROVER_01",
            "mission_file": "~/agrisarthi_ws/missions/current_mission.json",
            "home_latitude": float("nan"), "home_longitude": float("nan"),
            "data_directory": "~/agrisarthi_ws/missions/data", "sensor_port": "/dev/ttyUSB0",
            "sensor_baudrate": 4800, "sensor_slave_id": 1, "sensor_timeout_s": 1.0,
            "sensor_retry_count": 3, "sample_retries": 2, "actuator_timeout_s": 30.0,
            "probe_ack_retry_s": 2.0, "probe_ack_retries": 3,
            "gps_timeout_s": 2.0, "stop_tolerance": 0.001,
            "sensor_mode": "hardware", "sim_sensor_fail_attempts": 0,
            "sim_sensor_fault_topic": "/sim/soil/faults",
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.rover_id = str(p("rover_id"))
        self.mission_file = Path(str(p("mission_file"))).expanduser()
        self.data_dir = Path(str(p("data_directory"))).expanduser()
        self.home_config = (float(p("home_latitude")), float(p("home_longitude")))
        self.sensor_mode = str(p("sensor_mode")).lower()
        if self.sensor_mode == "hardware":
            self.sensor = SoilSensor(port=str(p("sensor_port")), baudrate=int(p("sensor_baudrate")),
                                     slave_id=int(p("sensor_slave_id")), timeout_s=float(p("sensor_timeout_s")),
                                     retry_count=int(p("sensor_retry_count")))
        elif self.sensor_mode == "simulation":
            from agrisarthi_gps_nav.simulated_soil_sensor import SimulatedSoilSensor
            self.sensor = SimulatedSoilSensor(fail_attempts=int(p("sim_sensor_fail_attempts")))
            self.get_logger().warning("Using deterministic synthetic soil data; no serial port will be opened.")
        else:
            raise ValueError("sensor_mode must be 'hardware' or 'simulation'")
        self.sample_retries = max(1, int(p("sample_retries")))
        self.actuator_timeout = max(1.0, float(p("actuator_timeout_s")))
        self.gps_timeout = max(0.1, float(p("gps_timeout_s")))
        self.stop_tolerance = max(0.0, float(p("stop_tolerance")))
        self.probe_ack_retry_s = max(0.2, float(p("probe_ack_retry_s")))
        self.probe_ack_retries = max(0, int(p("probe_ack_retries")))
        self.probe_action = None
        self.probe_retry_count = 0
        self.probe_last_send = 0.0
        self.mission = None
        self.document = None
        self.output_path = None
        self.state = "IDLE"
        self.failure = ""
        self.current_waypoint = None
        self.sample_attempt = 0
        self.command_id = None
        self.command_started = 0.0
        self.last_probe = {}
        self.boot_safe = False
        self.boot_command_id = None
        self.boot_phase = None
        self.boot_status_last_send = 0.0
        self.explicit_recovery_required = False
        self.mission_paused = False
        self.nav = {}
        self.nav_start_last_send = 0.0
        self.gps = None
        self.gps_time = 0.0
        self.future = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="soil-read")
        self.nav_start_pub = self.create_publisher(Bool, p("navigation_start_topic"), 10)
        self.complete_pub = self.create_publisher(String, p("navigation_complete_topic"), 10)
        self.probe_pub = self.create_publisher(String, p("probe_command_topic"), 10)
        self.status_pub = self.create_publisher(String, p("sampling_status_topic"), 10)
        self.status_response_pub = self.create_publisher(String, p("status_response_topic"), 10)
        self.data_response_pub = self.create_publisher(String, p("data_response_topic"), 10)
        self.create_subscription(String, p("mission_topic"), self.on_mission, 10)
        self.create_subscription(Bool, p("start_topic"), self.on_start, 10)
        self.create_subscription(String, p("navigation_status_topic"), self.on_navigation, 10)
        self.create_subscription(String, p("probe_status_topic"), self.on_probe, 10)
        self.create_subscription(NavSatFix, p("gps_topic"), self.on_gps, 10)
        self.create_subscription(String, p("recovery_topic"), self.on_recovery, 10)
        self.create_subscription(String, p("query_topic"), self.on_query, 10)
        self.create_subscription(Twist, p("command_velocity_topic"), self.on_velocity, 10)
        if self.sensor_mode == "simulation":
            self.create_subscription(String, p("sim_sensor_fault_topic"), self.on_sensor_fault, 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info("Sampling manager ready; public mission start is gated here.")
        if self.mission_file.is_file():
            try:
                self.on_mission(String(data=self.mission_file.read_text(encoding="utf-8")))
            except OSError as exc:
                self.get_logger().error(f"Could not restore saved mission: {exc}")

    def emit(self, state, detail="", update_state=True):
        if update_state:
            self.state = state
        msg = String()
        msg.data = json.dumps({"state": state, "mission_id": self.mission.get("mission_id") if self.mission else None,
                               "waypoint_id": self.current_waypoint, "detail": detail})
        self.status_pub.publish(msg)

    def on_query(self, msg):
        try:
            query = json.loads(msg.data)
            request_id = query.get("request_id")
            if query.get("action") != "GET_SNAPSHOT" or not isinstance(request_id, str) or not request_id or len(request_id) > 64:
                return
        except (json.JSONDecodeError, TypeError, AttributeError):
            return
        status = {"state": self.state, "failure": self.failure or None,
                  "mission_id": self.mission.get("mission_id") if self.mission else None,
                  "waypoint_id": self.current_waypoint,
                  "sensor_mode": self.sensor_mode,
                  "navigation_state": self.nav.get("state"),
                  "probe_state": self.last_probe.get("probe_state")}
        status_msg = String()
        status_msg.data = json.dumps({"request_id": request_id, "status": status})
        data_msg = String()
        data_msg.data = json.dumps({"request_id": request_id, "data": self.document})
        self.status_response_pub.publish(status_msg)
        self.data_response_pub.publish(data_msg)

    def on_mission(self, msg):
        self.nav_start(False)
        try:
            mission = validate_mission(msg.data)
            mid = mission["mission_id"]
            points = mission["waypoints"]
            if not isinstance(mid, str) or not mid or not isinstance(points, list) or not points:
                raise ValueError("mission_id and non-empty waypoints are required")
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", mid) or ".." in mid:
                raise ValueError("mission_id must use safe letters, digits, dot, underscore or hyphen characters")
            home = mission.get("home_point")
            if home is None:
                lat, lon = self.home_config
                if not math.isfinite(lat) or not math.isfinite(lon) or not -90 <= lat <= 90 or not -180 <= lon <= 180:
                    raise ValueError("configure home_latitude/home_longitude or include mission home_point")
                home = {"latitude": lat, "longitude": lon}
            doc = new_document(self.rover_id, mid, home)
            self.data_dir.mkdir(parents=True, exist_ok=True)
            output = self.data_dir / f"mission_{mid}_samples.json"
            if output.exists():
                existing = load_document(output)
                if existing["mission_id"] != mid or existing["rover_id"] != self.rover_id:
                    raise ValueError("existing result file belongs to a different mission or rover")
                if existing["home_point"] != doc["home_point"]:
                    raise ValueError("configured home point differs from persisted mission")
                doc = existing
            else:
                atomic_write(output, doc)
            self.mission, self.document, self.output_path = mission, doc, output
            self.failure = ""
            self.current_waypoint = None
            self.sample_attempt = 0
            self.command_id = None
            self.probe_action = None
            self.boot_safe = False
            self.boot_command_id = None
            self.boot_phase = None
            self.boot_status_last_send = 0.0
            self.last_probe = {}
            self.mission_paused = False
            self.explicit_recovery_required = False
            self.emit("MISSION_READY", f"Results will be saved incrementally to {output}")
        except (ValueError, KeyError, OSError, TypeError) as exc:
            self.mission = self.document = self.output_path = None
            self.failure = str(exc)
            self.emit("MISSION_REJECTED", str(exc))

    def nav_start(self, value):
        msg = Bool(); msg.data = bool(value); self.nav_start_pub.publish(msg)
        if value:
            self.nav_start_last_send = time.monotonic()

    def on_start(self, msg):
        if msg.data and self.state in ("NAVIGATING", "NAVIGATING_TO_NEXT_WAYPOINT", "RETURNING_HOME"):
            self.emit(self.state, "Duplicate start ignored; mission is already active.", update_state=False)
            return
        if not msg.data:
            self.mission_paused = True
            self.nav_start(False)
            sampling_states = {"WAYPOINT_REACHED", "PROBE_INSERTING", "SENSOR_READING", "SENSOR_VALIDATION",
                               "DATA_SAVED", "PROBE_RETRACTING", "FAILURE_RETRACTING", "WAIT_NAV_COMPLETE",
                               "RECOVERY_INSERTING", "RETURNING_HOME"}
            self.emit("PAUSED", "Navigation paused; any active stationary sampling cycle will finish safely.",
                      update_state=self.state not in sampling_states)
        elif not self.mission or self.failure or self.state not in (
                "MISSION_READY", "WAYPOINT_COMPLETE", "PAUSED", "START_REJECTED"):
            sampling_states = {"WAYPOINT_REACHED", "PROBE_INSERTING", "SENSOR_READING", "SENSOR_VALIDATION",
                               "DATA_SAVED", "PROBE_RETRACTING", "WAIT_NAV_COMPLETE", "RECOVERY_INSERTING",
                               "RETURNING_HOME"}
            if self.mission and not self.failure and self.state in sampling_states:
                self.mission_paused = False
                if self.state == "RETURNING_HOME":
                    self.nav_start(True)
                self.emit(self.state, "Sampling continues; navigation resumes only after safe completion.")
            else:
                self.nav_start(False)
                self.emit("RECOVERY_REQUIRED" if self.failure else "START_REJECTED", self.failure or "Mission is not ready.")
        elif not self.boot_safe:
            self.nav_start(False)
            self.emit("START_REJECTED", "Waiting for ESP32 probe retraction state confirmation.")
        else:
            self.mission_paused = False
            self.failure = ""
            self.nav_start(True)
            self.emit("NAVIGATING", "Mission explicitly started.")

    def on_navigation(self, msg):
        try:
            nav = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.nav = nav
        if (nav.get("state") == "WAITING_FOR_ESP32" and not self.failure
                and self.current_waypoint is None
                and self.state in ("IDLE", "MISSION_READY", "PAUSED", "START_REJECTED")):
            # Initial discovery before an explicit start is readiness, not a
            # mission-time communications failure; no navigation was authorized.
            self.emit(self.state, "Waiting for ESP32 handshake; movement remains disabled.", update_state=False)
            return
        if nav.get("state") == "WAYPOINT_REACHED" and self.current_waypoint is None and not self.failure:
            waypoint_id = nav.get("waypoint_id")
            if waypoint_id is None:
                index = nav.get("waypoint_index", -1)
                points = self.mission.get("waypoints", []) if self.mission else []
                waypoint_id = points[index].get("id") if isinstance(index, int) and 0 <= index < len(points) else None
            if waypoint_id is not None:
                self.current_waypoint = waypoint_id
                if self.document and any(str(r["waypoint_id"]) == str(waypoint_id)
                                         for r in self.document["records"]):
                    if self.last_probe.get("probe_state") == "RETRACTED":
                        self.complete_waypoint()
                    else:
                        self.nav_start(False)
                        self.failure = "SAVED_SAMPLE_PROBE_STATE_UNKNOWN"
                        self.emit("RECOVERY_REQUIRED", self.failure)
                    return
                self.emit("WAYPOINT_REACHED", "Waiting for stopped-motor confirmation.")
        if nav.get("state") in ("GEOFENCE_BREACH", "ESTOP", "COMMUNICATION_LOSS",
                                 "WAITING_FOR_ESP32", "WAITING_FOR_SENSORS"):
            self.nav_start(False)
            self.failure = str(nav.get("state"))
            if self.state != "FAILURE_RETRACTING" and self.last_probe.get("probe_state") in ("INSERTING", "INSERTED", "RETRACTING"):
                self.state = "FAILURE_RETRACTING"
                self.send_probe("PROBE_RETRACT")
            self.emit("RECOVERY_REQUIRED", self.failure)
        if nav.get("state") == "MISSION_REJECTED":
            self.nav_start(False)
            self.failure = "NAVIGATION_MISSION_REJECTED"
            self.emit("MISSION_REJECTED", self.failure)
        if self.state == "WAIT_NAV_COMPLETE" and nav.get("state") in (
                "WAYPOINT_COMPLETE", "RETURNING_HOME", "MISSION_COMPLETE"):
            completed = self.current_waypoint
            self.current_waypoint = None
            self.failure = ""
            if nav.get("state") == "MISSION_COMPLETE":
                self.emit("MISSION_COMPLETE", f"Waypoint {completed} sampled and saved.")
            elif self.explicit_recovery_required or self.mission_paused:
                self.explicit_recovery_required = False
                self.mission_paused = False
                self.emit("PAUSED", f"Waypoint {completed} complete; explicit start required after recovery.")
            elif nav.get("state") == "RETURNING_HOME":
                self.nav_start(True)
                self.emit("RETURNING_HOME", f"All samples saved; returning home after waypoint {completed}.")
            else:
                self.nav_start(True)
                self.emit("NAVIGATING_TO_NEXT_WAYPOINT", f"Waypoint {completed} sampled and saved.")
        elif nav.get("state") == "MISSION_COMPLETE" and self.current_waypoint is None and self.state != "MISSION_COMPLETE":
            self.emit("MISSION_COMPLETE", "Mission complete; rover returned to home.")

    def on_velocity(self, msg):
        if self.last_probe.get("probe_state") in ("INSERTING", "INSERTED", "RETRACTING"):
            if abs(msg.linear.x) > self.stop_tolerance or abs(msg.angular.z) > self.stop_tolerance:
                self.failure = "MOTION_REQUESTED_WHILE_PROBE_NOT_RETRACTED"
                self.nav_start(False)
                self.fail_sampling(self.failure)

    def on_sensor_fault(self, msg):
        try:
            fault = json.loads(msg.data)
            if fault.get("action") == "FAIL_NEXT_READS":
                self.sensor.fail_next(int(fault.get("count", 1)))
                self.get_logger().warning("Injected deterministic simulated soil-read failure.")
        except (json.JSONDecodeError, TypeError, ValueError, AttributeError):
            return

    def on_gps(self, msg):
        lat, lon = float(msg.latitude), float(msg.longitude)
        valid = (msg.status.status >= 0 and math.isfinite(lat) and math.isfinite(lon)
                 and -90 <= lat <= 90 and -180 <= lon <= 180)
        self.gps = {"latitude": lat, "longitude": lon} if valid else None
        self.gps_time = time.monotonic()

    def send_probe(self, action):
        self.command_id = uuid.uuid4().hex
        self.command_started = time.monotonic()
        self.probe_action = action
        self.probe_retry_count = 0
        self.probe_last_send = self.command_started
        self.publish_probe_command()
        return self.command_id

    def publish_probe_command(self):
        msg = String(); msg.data = json.dumps({"action": self.probe_action, "command_id": self.command_id})
        self.probe_pub.publish(msg)

    def on_probe(self, msg):
        try:
            packet = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.last_probe = packet
        if packet.get("type") == "hello_ack" and not self.boot_safe and self.boot_command_id is None:
            self.boot_phase = "STATUS"
            self.boot_command_id = self.send_probe("PROBE_STATUS")
            return
        if packet.get("command_id") == self.boot_command_id and self.boot_command_id is not None:
            if packet.get("accepted") is not True:
                self.failure = "PROBE_BOOTSTRAP_COMMAND_REJECTED"
                self.emit("RECOVERY_REQUIRED", self.failure)
                return
            probe_state = packet.get("probe_state")
            if self.boot_phase == "STATUS":
                if probe_state == "RETRACTED":
                    self.boot_safe = True
                    self.boot_command_id = None
                    self.boot_phase = None
                    if self.mission and self.state in ("START_REJECTED", "RECOVERY_REQUIRED"):
                        self.emit("MISSION_READY", "ESP32 confirms probe retracted; explicit start required.")
                    elif not self.mission:
                        self.emit("IDLE", "ESP32 confirms probe retracted.")
                else:
                    self.boot_phase = "RETRACT"
                    self.boot_command_id = self.send_probe("PROBE_RETRACT")
                return
            if self.boot_phase == "RETRACT" and probe_state == "RETRACTED":
                self.boot_safe = True
                self.boot_command_id = None
                self.boot_phase = None
                if self.mission and self.state in ("START_REJECTED", "RECOVERY_REQUIRED"):
                    self.emit("MISSION_READY", "ESP32 confirms safe probe retraction; explicit start required.")
                elif not self.mission:
                    self.emit("IDLE", "ESP32 confirms safe probe retraction.")
                return
        if packet.get("command_id") != self.command_id:
            return
        if packet.get("probe_state") == "ERROR":
            if self.probe_action == "PROBE_INSERT":
                self.fail_sampling(packet.get("probe_error") or "probe actuator entered ERROR")
            else:
                self.failure = packet.get("probe_error") or "probe actuator entered ERROR"
                self.nav_start(False)
                self.state = "RECOVERY_REQUIRED"
                self.emit("RECOVERY_REQUIRED", self.failure)
            return
        if packet.get("type") == "probe_ack" and packet.get("accepted") is not True:
            self.fail_sampling(packet.get("probe_error") or "probe command rejected")
            return
        state = packet.get("probe_state")
        if self.state == "PROBE_INSERTING" and state == "INSERTED":
            if packet.get("motors_enabled") is True or abs(float(packet.get("left", 0))) > self.stop_tolerance or abs(float(packet.get("right", 0))) > self.stop_tolerance:
                self.fail_sampling("ESP32 reports motors active with probe inserted")
                return
            self.emit("PROBE_INSERTED", "ESP32 reports the probe inserted.")
            self.submit_sample()
        elif self.state == "PROBE_RETRACTING" and state == "RETRACTED":
            if packet.get("motors_enabled") is True:
                self.fail_sampling("ESP32 reports motors active after probe retraction")
                return
            self.emit("PROBE_RETRACTED", "ESP32 reports the probe retracted.")
            self.complete_waypoint()
        elif self.state == "RECOVERY_INSERTING" and state == "INSERTED":
            if packet.get("motors_enabled") is True:
                self.fail_sampling("ESP32 reports motors active with probe inserted")
            else:
                self.emit("PROBE_INSERTED", "ESP32 reports the probe inserted during recovery.")
                self.submit_sample()
        elif self.state == "FAILURE_RETRACTING" and state == "RETRACTED":
            self.boot_safe = True
            self.emit("RECOVERY_REQUIRED", self.failure or "Sampling failed; explicit retry is required.")

    def submit_sample(self):
        self.sample_attempt += 1
        self.state = "SENSOR_READING"
        self.future = self.pool.submit(self.sensor.read)
        self.emit("SENSOR_READING", f"Acquisition attempt {self.sample_attempt}.")

    def save_sample(self, data):
        if not self.gps or time.monotonic() - self.gps_time > self.gps_timeout:
            raise ValueError("fresh valid GPS fix unavailable at sample time")
        record = {"waypoint_id": self.current_waypoint,
                  "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
                  "gps": dict(self.gps), "soil": data}
        updated = append_record(self.document, record)
        atomic_write(self.output_path, updated)
        self.document = updated

    def fail_sampling(self, reason):
        self.nav_start(False)
        self.failure = str(reason)
        self.emit("SAMPLE_FAILED", self.failure)
        # Attempt safe retraction. No waypoint-complete event is sent on failure.
        self.state = "FAILURE_RETRACTING"
        self.send_probe("PROBE_RETRACT")

    def complete_waypoint(self):
        message = String()
        message.data = json.dumps({"mission_id": self.mission["mission_id"], "waypoint_id": self.current_waypoint})
        self.complete_pub.publish(message)
        self.state = "WAIT_NAV_COMPLETE"
        self.emit("WAYPOINT_COMPLETE", "Sample saved and probe retracted; completion sent to navigator.",
                  update_state=False)

    def on_recovery(self, msg):
        action = msg.data.strip().upper()
        if not self.failure or not self.mission:
            return
        if action == "RETRY_SAMPLE" and self.last_probe.get("probe_state") == "INSERTED" and self.current_waypoint is not None:
            self.failure = ""
            self.sample_attempt = 0
            self.submit_sample()
        elif action == "COMPLETE_SAVED_WAYPOINT" and self.last_probe.get("probe_state") == "RETRACTED" and self.document:
            saved = any(str(row["waypoint_id"]) == str(self.current_waypoint) for row in self.document["records"])
            if saved:
                self.failure = ""
                self.explicit_recovery_required = True
                self.complete_waypoint()
        elif action == "RETRY_SAMPLE" and self.last_probe.get("probe_state") == "RETRACTED" and self.current_waypoint is not None:
            self.failure = ""
            self.explicit_recovery_required = True
            self.sample_attempt = 0
            self.state = "RECOVERY_INSERTING"
            self.send_probe("PROBE_INSERT")
        elif action == "RESUME" and self.current_waypoint is None and self.boot_safe:
            self.failure = ""
            self.mission_paused = False
            self.nav_start(True)
            self.emit("NAVIGATING", "Explicit recovery resume accepted.")
        elif action == "RETRACT":
            self.state = "FAILURE_RETRACTING"
            self.send_probe("PROBE_RETRACT")

    def tick(self):
        now = time.monotonic()
        active_states = ("NAVIGATING", "NAVIGATING_TO_NEXT_WAYPOINT", "RETURNING_HOME")
        held_nav_states = ("MISSION_READY", "PAUSED", "START_REQUESTED", "WAYPOINT_COMPLETE", "RETURNING_HOME")
        if (self.state in active_states and self.nav.get("state") in held_nav_states
                and now - self.nav_start_last_send >= 0.35):
            # Reliable handoff over ROS discovery and after waypoint completion:
            # keep the explicit manager authorization asserted until navigator
            # status confirms it left a held state.
            self.nav_start(True)
        # Handshake feedback can be published before ROS discovery has connected
        # the manager subscriber. Retry a status request with a stable command ID
        # until the ESP32 explicitly reports a safe retracted state.
        if not self.boot_safe and self.boot_command_id is None and now - self.boot_status_last_send >= self.probe_ack_retry_s:
            self.boot_phase = "STATUS"
            self.boot_command_id = self.send_probe("PROBE_STATUS")
            self.boot_status_last_send = now
        pending_probe = self.state in ("PROBE_INSERTING", "PROBE_RETRACTING", "FAILURE_RETRACTING",
                                       "RECOVERY_INSERTING", "BOOTSTRAP_STATUS", "BOOTSTRAP_RETRACTING") or self.boot_phase is not None
        if (pending_probe and self.command_id and self.probe_retry_count < self.probe_ack_retries
                and now - self.probe_last_send >= self.probe_ack_retry_s):
            self.publish_probe_command()  # same command ID makes firmware retries idempotent
            self.probe_retry_count += 1
            self.probe_last_send = now
        if self.state == "WAYPOINT_REACHED":
            feedback = self.nav.get("esp32_status", {})
            stopped = (self.nav.get("esp32_connected") is True and feedback.get("motors_enabled") is False
                       and abs(float(feedback.get("left", 0))) <= self.stop_tolerance
                       and abs(float(feedback.get("right", 0))) <= self.stop_tolerance)
            if stopped:
                self.emit("ROVER_STOPPED", "ESP32 confirms both motors are stopped.")
                self.state = "PROBE_INSERTING"
                self.sample_attempt = 0
                self.send_probe("PROBE_INSERT")
                self.emit("PROBE_INSERTING", "ESP32 confirmed stopped motors.")
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            try:
                result = future.result()
            except Exception as exc:
                if self.state == "SENSOR_READING" and not self.failure:
                    self.fail_sampling(f"sensor acquisition raised: {exc}")
                return
            if self.state != "SENSOR_READING" or self.failure:
                return
            if result.success and result.data is not None:
                try:
                    self.emit("SENSOR_VALIDATION", "Validating complete sensor result.")
                    self.emit("DATA_SAVING", "Writing the validated sample atomically.")
                    self.save_sample(result.data)
                    self.state = "PROBE_RETRACTING"
                    self.send_probe("PROBE_RETRACT")
                    self.emit("DATA_SAVED", "Atomic sample record saved; requesting probe retraction.",
                              update_state=False)
                except (ValueError, OSError) as exc:
                    self.fail_sampling(str(exc))
            elif self.sample_attempt < self.sample_retries:
                self.submit_sample()
            else:
                self.fail_sampling(result.error or "soil acquisition failed")
        if self.state in ("PROBE_INSERTING", "PROBE_RETRACTING", "FAILURE_RETRACTING",
                          "RECOVERY_INSERTING", "BOOTSTRAP_STATUS", "BOOTSTRAP_RETRACTING") and now - self.command_started > self.actuator_timeout:
            self.failure = "PROBE_COMMAND_TIMEOUT"
            self.nav_start(False)
            self.emit("RECOVERY_REQUIRED", "No actuator confirmation before timeout; motor interlock must remain active.")
            self.state = "RECOVERY_REQUIRED"
        if self.boot_phase is not None and now - self.command_started > self.actuator_timeout:
            self.failure = "PROBE_BOOTSTRAP_TIMEOUT"
            self.nav_start(False)
            self.emit("RECOVERY_REQUIRED", "Could not confirm the probe state; motors remain disabled.")
            self.boot_command_id = None
            self.boot_phase = None

    def destroy_node(self):
        try:
            self.nav_start(False)
        except Exception:
            pass  # ROS context may already be shutting down.
        self.pool.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionSamplingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
