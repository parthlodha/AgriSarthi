"""Heading sensor interface node for Agrisarthi rover.

Preserves the std_msgs/Float32 heading topic (degrees clockwise from North [0..360]).
When physical heading hardware is unavailable, the node remains disabled and does NOT
publish any data, ensuring the navigator safely latches in WAITING_FOR_SENSORS.
When physical hardware is connected, it reads and converts sensor data to calibrated degrees.
"""

from __future__ import annotations

import math
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def normalize_heading_deg(heading: float, declination: float = 0.0, offset: float = 0.0) -> float | None:
    """Normalize heading to degrees clockwise from North in range [0.0, 360.0)."""
    if not math.isfinite(heading):
        return None
    deg = (heading + declination + offset) % 360.0
    return deg if deg >= 0.0 else deg + 360.0


class HeadingNode(Node):
    """ROS 2 Jazzy node for physical digital compass / IMU heading integration."""

    def __init__(self, sensor_factory: Callable | None = None):
        super().__init__("heading_node")
        defaults = {
            "enabled": False,  # Physical sensor unavailable by default
            "heading_topic": "/heading_deg",
            "device_type": "generic_serial",
            "port": "/dev/ttyUSB1",
            "baudrate": 9600,
            "timeout_s": 1.0,
            "publish_rate_hz": 10.0,
            "magnetic_declination_deg": 0.0,
            "calibration_offset_deg": 0.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value

        self.enabled = bool(p("enabled"))
        self.heading_topic = str(p("heading_topic"))
        self.device_type = str(p("device_type"))
        self.port = str(p("port"))
        self.baudrate = int(p("baudrate"))
        self.timeout_s = max(0.1, float(p("timeout_s")))
        self.declination = float(p("magnetic_declination_deg"))
        self.offset = float(p("calibration_offset_deg"))
        self.sensor_factory = sensor_factory
        self.sensor_connection = None
        self.last_error_log = 0.0

        self.heading_pub = self.create_publisher(Float32, self.heading_topic, 10)
        rate = max(1.0, float(p("publish_rate_hz")))
        self.timer = self.create_timer(1.0 / rate, self.tick)

        if not self.enabled:
            self.get_logger().info(
                f"Heading sensor is DISABLED in configuration (sensor hardware unavailable). "
                f"Topic '{self.heading_topic}' will NOT be published. Navigator will safely remain in "
                f"WAITING_FOR_SENSORS until physical compass/IMU is connected and enabled."
            )
        else:
            self.get_logger().info(
                f"Heading sensor ENABLED: {self.device_type} on {self.port} -> {self.heading_topic}"
            )

    def _open_sensor(self):
        if self.sensor_factory is not None:
            return self.sensor_factory(port=self.port, baudrate=self.baudrate, timeout=self.timeout_s)
        try:
            import serial
            return serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout_s)
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_error_log >= 5.0:
                self.get_logger().warning(f"Could not open heading sensor port '{self.port}': {exc}")
                self.last_error_log = now
            return None

    def tick(self):
        # Fail-closed safety: if disabled, do NOT publish constant or fake values
        if not self.enabled:
            return

        if self.sensor_connection is None:
            self.sensor_connection = self._open_sensor()
            if self.sensor_connection is None:
                return

        try:
            # Read line or binary frame from sensor
            line = self.sensor_connection.readline()
            if not line:
                return
            raw_text = line.decode("ascii", errors="replace").strip()
            # Generic format support: e.g. "H:123.4", "$HCHDG,123.4...", or raw float string
            heading_val = self._parse_raw_heading(raw_text)
            if heading_val is not None:
                calibrated = normalize_heading_deg(heading_val, self.declination, self.offset)
                if calibrated is not None:
                    msg = Float32()
                    msg.data = float(calibrated)
                    self.heading_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f"Heading sensor I/O error: {exc}")
            try:
                if self.sensor_connection:
                    self.sensor_connection.close()
            except Exception:
                pass
            self.sensor_connection = None

    def _parse_raw_heading(self, text: str) -> float | None:
        """Parse heading angle from text representation."""
        if not text:
            return None
        # Handle NMEA HDT/HDG sentences: e.g. "$HCHDT,123.4,T*cs"
        if text.startswith("$") and ("HDT" in text or "HDG" in text):
            parts = text.split("*")[0].split(",")
            if len(parts) >= 2:
                try:
                    return float(parts[1])
                except ValueError:
                    return None
        # Handle prefix format e.g. "H:123.45"
        if text.startswith("H:") or text.startswith("HDG:"):
            _, val_str = text.split(":", 1)
            try:
                return float(val_str)
            except ValueError:
                return None
        # Direct float string
        try:
            return float(text)
        except ValueError:
            return None

    def destroy_node(self):
        if self.sensor_connection:
            try:
                self.sensor_connection.close()
            except Exception:
                pass
            self.sensor_connection = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
