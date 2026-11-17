import math
import unittest

from tb3_sensor_bringup.core import (
    SensorBuffer,
    WheelSpeed,
    wheel_speed_from_joint_state,
)


class WheelSpeedTests(unittest.TestCase):
    def test_nominal_conversion_and_order(self):
        result = wheel_speed_from_joint_state(
            ['wheel_left_joint', 'wheel_right_joint'],
            [1.0, 2.0], 0.033)
        self.assertIsInstance(result, WheelSpeed)
        self.assertAlmostEqual(result.left_rad_s, 1.0)
        self.assertAlmostEqual(result.right_rad_s, 2.0)
        self.assertAlmostEqual(result.left_m_s, 0.033)
        self.assertAlmostEqual(result.right_m_s, 0.066)

    def test_reversed_name_order_is_respected(self):
        result = wheel_speed_from_joint_state(
            ['wheel_right_joint', 'wheel_left_joint'],
            [2.0, 1.0], 0.033)
        self.assertAlmostEqual(result.left_m_s, 0.033)
        self.assertAlmostEqual(result.right_m_s, 0.066)

    def test_missing_wheel_raises(self):
        with self.assertRaises(KeyError):
            wheel_speed_from_joint_state(['wheel_left_joint'], [1.0], 0.033)

    def test_duplicate_name_raises(self):
        with self.assertRaises(ValueError):
            wheel_speed_from_joint_state(
                ['wheel_left_joint', 'wheel_left_joint'], [1.0, 2.0], 0.033)

    def test_bad_radius_or_velocity_raises(self):
        with self.assertRaises(ValueError):
            wheel_speed_from_joint_state(
                ['wheel_left_joint', 'wheel_right_joint'], [1.0, 2.0], 0.0)
        with self.assertRaises(ValueError):
            wheel_speed_from_joint_state(
                ['wheel_left_joint', 'wheel_right_joint'], [math.nan, 2.0], 0.033)

    def test_mismatched_lengths_raise(self):
        with self.assertRaises(ValueError):
            wheel_speed_from_joint_state(
                ['wheel_left_joint', 'wheel_right_joint'], [1.0], 0.033)


class SensorBufferTests(unittest.TestCase):
    def fill(self, buffer, stamp=10.0):
        for kind in SensorBuffer.KINDS:
            buffer.update(kind, stamp, kind)

    def test_all_four_ready(self):
        buffer = SensorBuffer()
        self.fill(buffer)
        snapshot = buffer.ready(now=10.1, tolerance=0.1, staleness=0.5)
        self.assertIsNotNone(snapshot)
        self.assertEqual(snapshot.span_s, 0.0)
        self.assertEqual(snapshot.payloads['imu'], 'imu')

    def test_missing_sensor_is_not_ready(self):
        buffer = SensorBuffer()
        buffer.update('image', 10.0, 'image')
        self.assertIsNone(buffer.ready(10.0, 0.1, 0.5))
        self.assertEqual(buffer.missing(), ['scan', 'imu', 'wheel'])

    def test_span_over_tolerance_is_not_ready(self):
        buffer = SensorBuffer()
        buffer.update('image', 10.0, 'image')
        buffer.update('scan', 10.05, 'scan')
        buffer.update('imu', 10.20, 'imu')
        buffer.update('wheel', 10.20, 'wheel')
        self.assertIsNone(buffer.ready(10.2, 0.1, 0.5))

    def test_stale_sample_is_not_ready(self):
        buffer = SensorBuffer()
        self.fill(buffer, stamp=10.0)
        self.assertIsNone(buffer.ready(11.0, 0.1, 0.5))

    def test_duplicate_and_backward_stamps_are_ignored(self):
        buffer = SensorBuffer()
        self.assertTrue(buffer.update('image', 10.0, 'first'))
        self.assertFalse(buffer.update('image', 10.0, 'duplicate'))
        self.assertFalse(buffer.update('image', 9.0, 'backward'))
        self.assertEqual(buffer.latest['image'][1], 'first')

    def test_invalid_configuration_and_kind(self):
        with self.assertRaises(ValueError):
            SensorBuffer(required=('image', 'bogus'))
        buffer = SensorBuffer()
        with self.assertRaises(ValueError):
            buffer.update('bogus', 1.0, None)
        with self.assertRaises(ValueError):
            buffer.ready(math.nan, 0.1, 0.5)
        with self.assertRaises(ValueError):
            buffer.ready(1.0, 0.0, 0.5)


if __name__ == '__main__':
    unittest.main()
