import unittest

from agrisarthi_gps_nav.soil_sensor import SoilSensor, SoilSensorError, crc16, read_registers


def response(slave_id, values, corrupt_crc=False):
    frame = bytearray((slave_id, 0x03, len(values) * 2))
    for value in values:
        frame.extend((value >> 8, value & 0xFF))
    crc = crc16(frame)
    if corrupt_crc:
        crc ^= 0x0001
    frame.extend((crc & 0xFF, crc >> 8))
    return bytes(frame)


class FakeSerial:
    def __init__(self, values_by_register, *, corrupt=None, short=None, timed_out=False, **_kwargs):
        self.values_by_register = values_by_register
        self.corrupt = set(corrupt or ())
        self.short = set(short or ())
        self.timed_out = timed_out
        self.start = None

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def reset_input_buffer(self):
        pass

    def write(self, request):
        self.start = (request[2] << 8) | request[3]

    def flush(self):
        pass

    def read(self, count):
        if self.timed_out:
            return b""
        values = self.values_by_register.get(self.start)
        if values is None:
            return b""
        result = response(1, values, corrupt_crc=self.start in self.corrupt)
        return result[:-1] if self.start in self.short else result


class SoilSensorTests(unittest.TestCase):
    def sensor(self, factory, attempts=1):
        return SoilSensor(port="/dev/fake-rs485", baudrate=4800, slave_id=1,
                          timeout_s=0.1, retry_count=attempts, response_delay_s=0,
                          retry_delay_s=0, serial_factory=factory)

    def test_valid_modbus_values_preserve_register_mapping_and_scaling(self):
        factory = lambda **kwargs: FakeSerial({0x0000: [583, 275, 1450, 64],
                                                0x001E: [72, 38, 165]}, **kwargs)
        result = self.sensor(factory).read()
        self.assertTrue(result.success)
        self.assertEqual(result.data, {
            "nitrogen": 72, "phosphorus": 38, "potassium": 165,
            "ph": 6.4, "moisture": 58.3, "temperature": 27.5, "ec": 1450,
        })

    def test_fallback_npk_register_is_used_when_primary_is_all_zero(self):
        factory = lambda **kwargs: FakeSerial({0x0000: [583, 275, 1450, 64],
                                                0x001E: [0, 0, 0],
                                                0x0004: [71, 39, 166]}, **kwargs)
        result = self.sensor(factory).read()
        self.assertTrue(result.success)
        self.assertEqual(result.data["nitrogen"], 71)

    def test_crc_error_is_reported_and_retried(self):
        factories = [
            lambda **kwargs: FakeSerial({0x0000: [583, 275, 1450, 64]}, corrupt={0x0000}, **kwargs),
            lambda **kwargs: FakeSerial({0x0000: [583, 275, 1450, 64], 0x001E: [72, 38, 165]}, **kwargs),
        ]
        result = self.sensor(lambda **kwargs: factories.pop(0)(**kwargs), attempts=2).read()
        self.assertTrue(result.success)
        self.assertEqual(result.attempts, 2)

    def test_short_response_and_timeout_return_failure_not_zero_sample(self):
        short = self.sensor(lambda **kwargs: FakeSerial({0x0000: [583, 275, 1450, 64]},
                                                        short={0x0000}, **kwargs)).read()
        timeout = self.sensor(lambda **kwargs: FakeSerial({}, timed_out=True, **kwargs)).read()
        self.assertFalse(short.success)
        self.assertIsNone(short.data)
        self.assertIn("short Modbus response", short.error)
        self.assertFalse(timeout.success)
        self.assertIsNone(timeout.data)

    def test_failed_npk_primary_and_fallback_never_become_zero_success(self):
        factory = lambda **kwargs: FakeSerial({0x0000: [583, 275, 1450, 64],
                                                0x001E: [0, 0, 0],
                                                0x0004: [0, 0, 0]}, **kwargs)
        result = self.sensor(factory).read()
        self.assertFalse(result.success)
        self.assertIsNone(result.data)
        self.assertIn("all zeros", result.error)

    def test_read_registers_rejects_bad_crc(self):
        class OneShot(FakeSerial):
            def read(self, count):
                return response(1, [12], corrupt_crc=True)

        with OneShot({}) as fake:
            with self.assertRaisesRegex(SoilSensorError, "CRC16"):
                read_registers(fake, 1, 0, 1, response_delay_s=0)


if __name__ == "__main__":
    unittest.main()
