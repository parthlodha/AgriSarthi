#!/usr/bin/env python3
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String

# ROS adapter kept in this module so the existing console entry point remains stable.
# Navigation calculations and protocol handling live in independently testable modules.
from pathlib import Path
from agrisarthi_gps_nav.esp32_link import ESP32Link
from agrisarthi_gps_nav.navigation_core import NavigationController
from agrisarthi_gps_nav.simulated_esp32 import ROSSimulatedESP32Link


class GPSWaypointNavigator(Node):
    def __init__(self):
        super().__init__('gps_waypoint_navigator')
        defaults = {
            'mission_topic': '/mission/waypoints_json', 'mission_file': '~/agrisarthi_ws/missions/current_mission.json',
            'start_topic': '/mission/navigation_start', 'gps_topic': '/gps/fix', 'heading_topic': '/heading_deg',
            'estop_topic': '/cmd/estop', 'status_topic': '/rover/nav_status', 'esp32_ip': '192.168.1.50',
            'enable_esp32_link': True, 'command_topic': '/cmd_vel',
            'esp32_port': 5005, 'command_rate_hz': 10.0, 'communication_timeout_s': 0.6,
            'gps_timeout_s': 2.0, 'heading_timeout_s': 1.0, 'max_gps_accuracy_m': 10.0,
            'require_gps_covariance': False, 'waypoint_tolerance_m': 1.5, 'max_waypoints': 5000,
            'max_linear_speed': 0.18, 'max_angular_speed': 0.35, 'heading_kp': 0.012,
            'turn_in_place_error_deg': 55.0, 'max_heading_error_for_full_speed_deg': 35.0,
            'minimum_speed_scale': 0.2, 'slow_down_distance_m': 3.0,
            'waypoint_complete_topic': '/mission/waypoint_complete',
            'probe_command_topic': '/probe/command', 'probe_status_topic': '/probe/status',
            'esp32_transport': 'udp', 'sim_esp32_command_topic': '/sim/esp32/command',
            'sim_esp32_feedback_topic': '/sim/esp32/feedback', 'return_home': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.mission_file = str(param('mission_file'))
        self.max_waypoints = max(1, int(param('max_waypoints')))
        self.max_gps_accuracy = max(0.0, float(param('max_gps_accuracy_m')))
        self.require_gps_covariance = bool(param('require_gps_covariance'))
        self.controller = NavigationController(
            waypoint_tolerance_m=float(param('waypoint_tolerance_m')),
            gps_timeout_s=float(param('gps_timeout_s')), heading_timeout_s=float(param('heading_timeout_s')),
            max_linear_speed=float(param('max_linear_speed')), max_angular_speed=float(param('max_angular_speed')),
            heading_kp=float(param('heading_kp')), turn_in_place_error_deg=float(param('turn_in_place_error_deg')),
            max_heading_error_for_full_speed_deg=float(param('max_heading_error_for_full_speed_deg')),
            minimum_speed_scale=float(param('minimum_speed_scale')),
            slow_down_distance_m=float(param('slow_down_distance_m')),
            return_home=bool(param('return_home')),
        )
        self.estop = False
        self.last_status = 0.0
        self.link = None
        if bool(param('enable_esp32_link')):
            transport = str(param('esp32_transport')).lower()
            if transport == 'udp':
                self.link = ESP32Link(str(param('esp32_ip')), int(param('esp32_port')),
                                      ack_timeout_s=float(param('communication_timeout_s')))
            elif transport == 'ros_sim':
                self.link = ROSSimulatedESP32Link(
                    self, str(param('sim_esp32_command_topic')), str(param('sim_esp32_feedback_topic')),
                    ack_timeout_s=float(param('communication_timeout_s')))
            else:
                raise ValueError("esp32_transport must be 'udp' or 'ros_sim'")
        self.command_pub = self.create_publisher(Twist, param('command_topic'), 10)
        self.create_subscription(String, param('mission_topic'), self.on_mission, 10)
        self.create_subscription(Bool, param('start_topic'), self.on_start, 10)
        self.create_subscription(String, param('waypoint_complete_topic'), self.on_waypoint_complete, 10)
        self.create_subscription(String, param('probe_command_topic'), self.on_probe_command, 10)
        self.create_subscription(NavSatFix, param('gps_topic'), self.on_gps, 10)
        self.create_subscription(Float32, param('heading_topic'), self.on_heading, 10)
        self.create_subscription(Bool, param('estop_topic'), self.on_estop, 10)
        self.status_pub = self.create_publisher(String, param('status_topic'), 10)
        self.probe_status_pub = self.create_publisher(String, param('probe_status_topic'), 10)
        self.timer = self.create_timer(1.0 / max(1.0, float(param('command_rate_hz'))), self.tick)
        self.get_logger().info(f"Ready. GPS={param('gps_topic')} heading={param('heading_topic')} "
                               f"ESP32 enabled={self.link is not None}")
        self.load_saved_mission()

    def mission_path(self):
        return Path(self.mission_file).expanduser()

    def remove_saved_mission(self):
        try:
            path = self.mission_path()
            path.unlink(missing_ok=True)
            path.with_suffix(path.suffix + '.tmp').unlink(missing_ok=True)
        except OSError as exc:
            self.get_logger().error(f'Could not remove invalid saved mission: {exc}')

    def on_mission(self, msg):
        self.controller.clear_mission('MISSION_REJECTED')
        self.remove_saved_mission()
        try:
            packet = self.controller.load_mission(msg.data, max_waypoints=self.max_waypoints)
            path = self.mission_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            temp = path.with_suffix(path.suffix + '.tmp')
            temp.write_text(json.dumps(packet, indent=2), encoding='utf-8')
            temp.replace(path)
        except (ValueError, TypeError, OSError) as exc:
            self.controller.clear_mission('MISSION_REJECTED')
            self.remove_saved_mission()
            self.send_stop()
            self.get_logger().error(f'Mission rejected; prior mission cleared: {exc}')
            self.publish_status('MISSION_REJECTED', str(exc))
            return
        self.send_stop()
        self.publish_status('MISSION_READY', f'Saved to {path}; explicit start required')

    def load_saved_mission(self):
        path = self.mission_path()
        if not path.is_file():
            self.get_logger().info('No saved mission; waiting for a planner mission.')
            return
        try:
            self.controller.load_mission(json.loads(path.read_text(encoding='utf-8')),
                                         max_waypoints=self.max_waypoints)
            self.publish_status('MISSION_READY', f'Loaded {path}; explicit start required')
        except (OSError, ValueError, TypeError) as exc:
            self.controller.clear_mission('MISSION_REJECTED')
            self.remove_saved_mission()
            self.get_logger().error(f'Saved mission rejected and removed: {exc}')
            self.publish_status('MISSION_REJECTED', str(exc))

    def on_start(self, msg):
        if not msg.data:
            self.controller.pause('PAUSED')
            self.send_stop()
            self.publish_status('PAUSED', 'Explicit start is required to resume.')
        elif self.estop:
            self.send_stop()
            self.publish_status('ESTOP', 'Cannot start while software E-stop is active.')
        elif not self.controller.start():
            self.send_stop()
            self.publish_status(self.controller.state, 'No valid runnable mission or navigation fault.')
        else:
            self.publish_status('START_REQUESTED', 'Waiting for fresh sensors and ESP32 acknowledgement.')

    def on_waypoint_complete(self, msg):
        try:
            packet = json.loads(msg.data)
            if packet.get('mission_id') != self.controller.mission_id:
                raise ValueError('mission_id does not match the active mission')
            state = self.controller.complete_waypoint(packet.get('waypoint_id'))
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Ignored waypoint completion: {exc}')
            return
        self.send_stop()
        self.publish_status(state, f"Sampling acknowledged for waypoint {packet['waypoint_id']}")

    def on_probe_command(self, msg):
        try:
            packet = json.loads(msg.data)
            if self.link is None or not self.link.ready():
                return
            self.link.send_probe_command(str(packet['action']), str(packet['command_id']))
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self.get_logger().warning(f'Could not send probe command: {exc}')

    def publish_probe_feedback(self, packet):
        if packet.get('type') not in ('probe_ack', 'probe_status', 'hello_ack'):
            return
        if packet.get('type') == 'hello_ack' and 'probe_state' not in packet:
            return
        message = String()
        message.data = json.dumps(packet)
        self.probe_status_pub.publish(message)

    def on_gps(self, msg):
        lat, lon = float(msg.latitude), float(msg.longitude)
        valid = (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90
                 and -180 <= lon <= 180 and msg.status.status >= 0)
        covariance_type = int(msg.position_covariance_type)
        if self.require_gps_covariance and covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            valid = False
        if valid and covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            xx, yy = float(msg.position_covariance[0]), float(msg.position_covariance[4])
            if not math.isfinite(xx) or not math.isfinite(yy) or xx < 0 or yy < 0:
                valid = False
            elif self.max_gps_accuracy and math.sqrt(max(xx, yy)) > self.max_gps_accuracy:
                valid = False
        self.controller.set_gps(lat, lon, time.monotonic(), valid=valid)

    def on_heading(self, msg):
        self.controller.set_heading(float(msg.data), time.monotonic())

    def on_estop(self, msg):
        self.estop = bool(msg.data)
        self.controller.estop = self.estop
        if self.estop:
            self.controller.pause('ESTOP')
            self.send_stop()
            self.publish_status('ESTOP', 'Explicit mission start required after clearing E-stop.')

    def send_stop(self):
        if self.link is None:
            return
        now = time.monotonic()
        self.link.poll(now)
        if self.link.ready(now):
            try:
                self.link.send_command({'enable': False}, now)
            except OSError as exc:
                self.get_logger().warning(f'Could not send stop command: {exc}')
        else:
            self.link.send_hello(now)

    def tick(self):
        now = time.monotonic()
        if self.link is not None:
            for packet in self.link.poll(now):
                self.publish_probe_feedback(packet)
        if self.link is not None and not self.link.ready(now):
            if self.controller.started:
                self.controller.pause('COMMUNICATION_LOSS')
            self.link.send_hello(now)
            self.status_throttled('WAITING_FOR_ESP32', 'ESP32 handshake/ack missing; motors remain stopped.')
            return
        command = self.controller.step(now)
        twist = Twist()
        twist.linear.x = command['linear']
        twist.angular.z = command['angular']
        self.command_pub.publish(twist)
        if self.link is not None:
            try:
                self.link.send_command(command, now)
            except OSError as exc:
                self.controller.pause('COMMUNICATION_LOSS')
                self.get_logger().warning(f'UDP send failed; mission paused: {exc}')
                command = {**command, 'state': 'COMMUNICATION_LOSS',
                           'detail': 'UDP send failed; explicit mission start required after recovery.'}
        link_status = self.link.status if self.link is not None else {}
        feedback = {k: link_status[k] for k in
                    ('motors_enabled', 'left', 'right', 'wifi_connected') if k in link_status}
        self.status_throttled(command['state'], command['detail'], feedback)

    def publish_status(self, state, detail, esp32=None):
        msg = String()
        msg.data = json.dumps({'state': state, 'mission_id': self.controller.mission_id,
                               'waypoint_index': self.controller.index,
                               'waypoint_id': (self.controller.points[self.controller.index]['id']
                                               if self.controller.index < len(self.controller.points) else None),
                               'waypoint_count': len(self.controller.points), 'detail': detail,
                               'esp32_connected': self.link.ready() if self.link is not None else False,
                               'esp32_status': esp32 if esp32 is not None else
                               (self.link.status if self.link is not None else {})})
        self.status_pub.publish(msg)

    def status_throttled(self, state, detail, esp32=None):
        now = time.monotonic()
        if now - self.last_status >= 1.0:
            self.publish_status(state, detail, esp32)
            self.last_status = now

    def destroy_node(self):
        if self.link is not None:
            try:
                self.link.close()
            except Exception:
                pass  # ROS context may already be shutting down.
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
