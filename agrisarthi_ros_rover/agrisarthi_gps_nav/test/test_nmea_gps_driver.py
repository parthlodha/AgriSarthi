import unittest
import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from agrisarthi_gps_nav.nmea_gps_driver import (
    NmeaParser,
    NMEAGPSDriver,
    parse_nmea_coordinate,
    verify_nmea_checksum,
)


def format_nmea(content: str) -> str:
    cs = 0
    for ch in content:
        cs ^= ord(ch)
    return f"${content}*{cs:02X}"


class MockSerial:
    def __init__(self, data: bytes):
        self._data = data
        self._pos = 0
        self.closed = False

    @property
    def in_waiting(self) -> int:
        return max(0, len(self._data) - self._pos)

    def read(self, size: int = 1) -> bytes:
        if self._pos >= len(self._data):
            return b""
        chunk = self._data[self._pos : self._pos + size]
        self._pos += len(chunk)
        return chunk

    def close(self):
        self.closed = True


class NmeaChecksumTests(unittest.TestCase):
    def test_valid_checksum(self):
        sentence = format_nmea("GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,")
        self.assertTrue(verify_nmea_checksum(sentence))

    def test_corrupt_checksum_fails(self):
        sentence = format_nmea("GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,")[:-2] + "00"
        self.assertFalse(verify_nmea_checksum(sentence))

    def test_malformed_string_fails(self):
        self.assertFalse(verify_nmea_checksum("NOT_NMEA"))
        self.assertFalse(verify_nmea_checksum("$GNGGA,123456*ZZ"))


class NmeaCoordinateParsingTests(unittest.TestCase):
    def test_north_east_coordinate(self):
        # 48 deg 07.038 min = 48 + 7.038/60 = 48.1173
        lat = parse_nmea_coordinate("4807.038", "N")
        self.assertAlmostEqual(lat, 48.1173, places=4)

        # 011 deg 31.000 min = 11 + 31/60 = 11.5166667
        lon = parse_nmea_coordinate("01131.000", "E")
        self.assertAlmostEqual(lon, 11.5166667, places=4)

    def test_south_west_coordinate(self):
        lat = parse_nmea_coordinate("1904.560", "S")
        self.assertAlmostEqual(lat, -19.0760, places=4)

        lon = parse_nmea_coordinate("07252.662", "W")
        self.assertAlmostEqual(lon, -72.8777, places=4)

    def test_invalid_values_return_none(self):
        self.assertIsNone(parse_nmea_coordinate("", "N"))
        self.assertIsNone(parse_nmea_coordinate("1234.5", "X"))
        self.assertIsNone(parse_nmea_coordinate("99999.0", "N"))


class NmeaParserTests(unittest.TestCase):
    def setUp(self):
        self.parser = NmeaParser(hdop_scale_m=2.5)

    def test_parse_valid_gga_sentence(self):
        line = format_nmea("GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,")
        fix = self.parser.parse_line(line)
        self.assertIsNotNone(fix)
        self.assertTrue(fix.valid)
        self.assertAlmostEqual(fix.latitude, 48.1173, places=4)
        self.assertAlmostEqual(fix.longitude, 11.5166667, places=4)
        self.assertEqual(fix.altitude, 545.4)
        self.assertEqual(fix.fix_type, 1)
        self.assertEqual(fix.satellites, 8)
        self.assertEqual(fix.hdop, 0.9)

    def test_parse_no_fix_gga_sentence(self):
        line = format_nmea("GNGGA,123519.00,,,,,0,00,99.99,,,,,,")
        fix = self.parser.parse_line(line)
        self.assertIsNotNone(fix)
        self.assertFalse(fix.valid)
        self.assertEqual(fix.error, "NO_FIX")

    def test_parse_valid_rmc_sentence(self):
        line = format_nmea("GNRMC,123519.00,A,4807.038,N,01131.000,E,0.224,84.4,230394,003.1,W")
        fix = self.parser.parse_line(line)
        self.assertIsNotNone(fix)
        self.assertTrue(fix.valid)
        self.assertAlmostEqual(fix.latitude, 48.1173, places=4)

    def test_parse_void_rmc_sentence(self):
        line = format_nmea("GNRMC,123519.00,V,,,,,,,230394,,,")
        fix = self.parser.parse_line(line)
        self.assertIsNotNone(fix)
        self.assertFalse(fix.valid)
        self.assertEqual(fix.error, "RMC_STATUS_VOID")


class NMEAGPSDriverNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_driver_publishes_navsatfix_from_stream(self):
        gga = (format_nmea("GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,") + "\r\n").encode()
        mock_serial = MockSerial(gga)

        node = NMEAGPSDriver(serial_factory=lambda **kwargs: mock_serial)
        received_msgs = []
        node.fix_pub.publish = lambda msg: received_msgs.append(msg)

        try:
            node.poll_serial()
            self.assertEqual(len(received_msgs), 1)
            fix_msg = received_msgs[0]
            self.assertIsInstance(fix_msg, NavSatFix)
            self.assertEqual(fix_msg.status.status, NavSatStatus.STATUS_FIX)
            self.assertAlmostEqual(fix_msg.latitude, 48.1173, places=4)
            self.assertAlmostEqual(fix_msg.longitude, 11.5166667, places=4)
            self.assertEqual(fix_msg.altitude, 545.4)
            # HDOP = 0.9, scale = 2.5 -> var = (0.9 * 2.5)^2 = 5.0625
            self.assertAlmostEqual(fix_msg.position_covariance[0], 5.0625, places=3)
            self.assertEqual(fix_msg.position_covariance_type, NavSatFix.COVARIANCE_TYPE_APPROXIMATED)
        finally:
            node.destroy_node()

    def test_driver_accumulates_chunked_stream(self):
        full_line = (format_nmea("GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,") + "\r\n").encode()
        chunk1 = full_line[:25]
        chunk2 = full_line[25:]

        class FragmentedSerial:
            def __init__(self, c1, c2):
                self.chunks = [c1, c2]
            @property
            def in_waiting(self):
                return len(self.chunks[0]) if self.chunks else 0
            def read(self, size=256):
                return self.chunks.pop(0) if self.chunks else b""
            def close(self):
                pass

        mock_serial = FragmentedSerial(chunk1, chunk2)
        node = NMEAGPSDriver(serial_factory=lambda **kwargs: mock_serial)
        received_msgs = []
        node.fix_pub.publish = lambda msg: received_msgs.append(msg)

        try:
            node.poll_serial()  # First chunk read
            self.assertEqual(len(received_msgs), 0)
            node.poll_serial()  # Second chunk read and parsed
            self.assertEqual(len(received_msgs), 1)
            self.assertAlmostEqual(received_msgs[0].latitude, 48.1173, places=4)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
