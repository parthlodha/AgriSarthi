import unittest
import rclpy
from std_msgs.msg import Float32

from agrisarthi_gps_nav.heading_node import HeadingNode, normalize_heading_deg


class MockHeadingSerial:
    def __init__(self, lines: list[bytes]):
        self.lines = list(lines)
        self.closed = False

    def readline(self) -> bytes:
        return self.lines.pop(0) if self.lines else b""

    def close(self):
        self.closed = True


class NormalizeHeadingTests(unittest.TestCase):
    def test_in_range_values(self):
        self.assertEqual(normalize_heading_deg(0.0), 0.0)
        self.assertEqual(normalize_heading_deg(180.5), 180.5)
        self.assertEqual(normalize_heading_deg(359.9), 359.9)

    def test_wrapping_positive(self):
        self.assertAlmostEqual(normalize_heading_deg(365.0), 5.0)
        self.assertAlmostEqual(normalize_heading_deg(720.0), 0.0)

    def test_wrapping_negative(self):
        self.assertAlmostEqual(normalize_heading_deg(-10.0), 350.0)
        self.assertAlmostEqual(normalize_heading_deg(-370.0), 350.0)

    def test_declination_and_offset(self):
        # 10 deg heading + 2 deg declination - 1 deg offset = 11 deg
        self.assertAlmostEqual(normalize_heading_deg(10.0, declination=2.0, offset=-1.0), 11.0)

    def test_non_finite_returns_none(self):
        self.assertIsNone(normalize_heading_deg(float("nan")))
        self.assertIsNone(normalize_heading_deg(float("inf")))


class HeadingNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_disabled_node_does_not_publish(self):
        node = HeadingNode()
        published = []
        node.heading_pub.publish = lambda msg: published.append(msg)
        try:
            self.assertFalse(node.enabled)
            node.tick()
            self.assertEqual(len(published), 0)
        finally:
            node.destroy_node()

    def test_enabled_node_parses_and_publishes_calibrated_heading(self):
        mock_serial = MockHeadingSerial([b"$HCHDT,270.5,T*2D\r\n", b"H:95.0\r\n", b"180.0\r\n"])
        node = HeadingNode(sensor_factory=lambda **kwargs: mock_serial)
        # Enable manually for test
        node.enabled = True
        node.declination = 1.0  # +1 degree declination
        node.offset = -0.5      # -0.5 degree offset -> net +0.5 degree

        published = []
        node.heading_pub.publish = lambda msg: published.append(msg)

        try:
            # 1. $HCHDT,270.5,T -> 270.5 + 0.5 = 271.0
            node.tick()
            self.assertEqual(len(published), 1)
            self.assertAlmostEqual(published[-1].data, 271.0, places=2)

            # 2. H:95.0 -> 95.0 + 0.5 = 95.5
            node.tick()
            self.assertEqual(len(published), 2)
            self.assertAlmostEqual(published[-1].data, 95.5, places=2)

            # 3. 180.0 -> 180.0 + 0.5 = 180.5
            node.tick()
            self.assertEqual(len(published), 3)
            self.assertAlmostEqual(published[-1].data, 180.5, places=2)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
