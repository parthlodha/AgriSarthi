import json
import time
import unittest

import rclpy
from std_msgs.msg import String

from agrisarthi_gps_nav.simulated_esp32 import ROSSimulatedESP32Link, SimulatedESP32Node
from agrisarthi_gps_nav.simulated_soil_sensor import SimulatedSoilSensor


class SimulatedSoilTests(unittest.TestCase):
    def test_deterministic_results_and_failure_are_structured(self):
        sensor = SimulatedSoilSensor(fail_attempts=1)
        failed = sensor.read()
        self.assertFalse(failed.success)
        self.assertIsNone(failed.data)
        first = sensor.read()
        second = sensor.read()
        self.assertTrue(first.success)
        self.assertEqual(first.data["nitrogen"], 70)
        self.assertEqual(first.data["ec"], 640)
        self.assertEqual(second.data["nitrogen"], 72)
        self.assertEqual(set(first.data), {"nitrogen", "phosphorus", "potassium", "ph",
                                           "moisture", "temperature", "ec"})


class SimulatedESP32Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SimulatedESP32Node()
        self.node.timer.cancel()
        self.output = []
        self.node.publish = lambda packet: self.output.append(dict(packet))
        self.session = "sitl-test-session"
        self.send({"type": "hello", "session_id": self.session})
        self.assertEqual(self.output[-1]["type"], "hello_ack")

    def tearDown(self):
        self.node.destroy_node()

    def send(self, packet):
        packet.setdefault("protocol", 1)
        packet.setdefault("session_id", self.session)
        self.node.on_command(String(data=json.dumps(packet)))

    def test_shared_sequence_probe_interlock_and_failed_retraction(self):
        self.send({"type": "motor_cmd", "seq": 1, "enable": True, "left": 0.2, "right": 0.2})
        self.assertTrue(self.node.motors_enabled)
        self.send({"type": "probe_cmd", "seq": 2, "command_id": "insert-1", "action": "PROBE_INSERT"})
        self.assertEqual(self.node.probe_state, "INSERTING")
        self.assertFalse(self.node.motors_enabled)
        started = self.node.motion_started
        self.send({"type": "probe_cmd", "seq": 3, "command_id": "insert-1", "action": "PROBE_INSERT"})
        self.assertEqual(self.node.motion_started, started)
        self.node.motion_started = time.monotonic() - 1.0
        self.node.tick()
        self.assertEqual(self.node.probe_state, "INSERTED")
        self.send({"type": "motor_cmd", "seq": 4, "enable": True, "left": 0.4, "right": 0.4})
        self.assertFalse(self.node.motors_enabled)

        self.node.fail_retractions = 1
        self.send({"type": "probe_cmd", "seq": 5, "command_id": "retract-1", "action": "PROBE_RETRACT"})
        self.node.motion_started = time.monotonic() - 1.0
        self.node.tick()
        self.assertEqual(self.node.probe_state, "ERROR")
        self.send({"type": "motor_cmd", "seq": 6, "enable": True, "left": 0.5, "right": 0.5})
        self.assertFalse(self.node.motors_enabled)
        self.assertEqual(self.node.left, 0.0)

    def test_duplicate_ack_does_not_refresh_a_pending_sequence_twice(self):
        link = ROSSimulatedESP32Link(self.node, "/test/command", "/test/feedback", session_id=self.session)
        try:
            link.online = True
            link.last_ack = 100.0
            seq = link.send_command({"enable": False}, now=100.01)
            response = {"type": "ack", "session_id": self.session, "seq": seq, "accepted": True}
            link._on_feedback(String(data=json.dumps(response)))
            link._on_feedback(String(data=json.dumps(response)))
            self.assertEqual(len(link.poll(now=100.02)), 1)
            self.assertEqual(link.pending_sequences, [])
            self.assertEqual(link.poll(now=100.03), [])
        finally:
            link.close()


if __name__ == "__main__":
    unittest.main()
