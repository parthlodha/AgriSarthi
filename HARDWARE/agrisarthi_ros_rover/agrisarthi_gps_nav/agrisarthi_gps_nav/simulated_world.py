"""Deterministic planar rover world publishing simulated GPS and heading."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String

EARTH_RADIUS_M = 6_371_000.0


class SimulatedWorldNode(Node):
    def __init__(self):
        super().__init__("simulated_world")
        defaults = {
            "gps_topic": "/sim/gps/fix", "heading_topic": "/sim/heading_deg",
            "feedback_topic": "/sim/esp32/feedback", "fault_topic": "/sim/world/faults",
            "home_latitude": 37.4219999, "home_longitude": -122.0840575,
            "initial_heading_deg": 0.0, "publish_rate_hz": 20.0,
            "gps_loss_after_s": 0.0, "gps_covariance_m2": 0.04,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.lat = float(p("home_latitude"))
        self.lon = float(p("home_longitude"))
        self.heading = float(p("initial_heading_deg")) % 360.0
        self.covariance = max(0.0, float(p("gps_covariance_m2")))
        self.gps_loss_after = max(0.0, float(p("gps_loss_after_s")))
        self.start_time = time.monotonic()
        self.last_tick = self.start_time
        self.linear = 0.0
        self.angular = 0.0
        self.motors_enabled = False
        self.probe_retracted = True
        self.gps_enabled = True
        self.fix_pub = self.create_publisher(NavSatFix, p("gps_topic"), 10)
        self.heading_pub = self.create_publisher(Float32, p("heading_topic"), 10)
        self.create_subscription(String, p("feedback_topic"), self.on_esp32_feedback, 10)
        self.create_subscription(String, p("fault_topic"), self.on_fault, 10)
        rate = max(1.0, float(p("publish_rate_hz")))
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"Simulation world initialized at {self.lat:.7f},{self.lon:.7f}; no hardware I/O enabled.")

    def on_esp32_feedback(self, msg: String) -> None:
        try:
            packet = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.probe_retracted = packet.get("probe_state") == "RETRACTED"
        self.motors_enabled = packet.get("motors_enabled") is True
        if self.motors_enabled and self.probe_retracted and packet.get("type") == "ack" and packet.get("accepted") is True:
            self.linear = float(packet.get("sim_linear", 0.0))
            # Positive angular values mean clockwise/right for the rover controller.
            self.angular = float(packet.get("sim_angular", 0.0))
        else:
            self.linear = self.angular = 0.0

    def on_fault(self, msg: String) -> None:
        try:
            fault = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        action = fault.get("action")
        if action == "GPS_LOSS":
            self.gps_enabled = False
            self.linear = self.angular = 0.0
        elif action == "GPS_RESTORE":
            self.gps_enabled = True
            self.linear = self.angular = 0.0

    def tick(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_tick))
        self.last_tick = now
        if self.gps_loss_after and now - self.start_time >= self.gps_loss_after:
            self.gps_enabled = False
            self.gps_loss_after = 0.0
        if self.gps_enabled and self.motors_enabled and self.probe_retracted:
            distance = self.linear * dt
            middle_heading = math.radians(self.heading + math.degrees(self.angular * dt) / 2.0)
            north = distance * math.cos(middle_heading)
            east = distance * math.sin(middle_heading)
            self.lat += math.degrees(north / EARTH_RADIUS_M)
            scale = max(1e-8, math.cos(math.radians(self.lat)))
            self.lon += math.degrees(east / (EARTH_RADIUS_M * scale))
            self.heading = (self.heading + math.degrees(self.angular * dt)) % 360.0
        if self.gps_enabled:
            # Publish a valid stationary fix too, so the navigator can initialize
            # before its first motor command (and managers can geotag samples).
            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.latitude, fix.longitude, fix.altitude = self.lat, self.lon, 0.0
            fix.position_covariance[0] = self.covariance
            fix.position_covariance[4] = self.covariance
            fix.position_covariance[8] = self.covariance * 2.0
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.fix_pub.publish(fix)
        heading = Float32(); heading.data = float(self.heading)
        self.heading_pub.publish(heading)


def simulated_world_main(args=None):
    rclpy.init(args=args)
    node = SimulatedWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
