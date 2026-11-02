"""Adapter tests use an explicit bridge double, not a simulated ROS installation."""

from copy import deepcopy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from tb3_camera_attack.relay import process_image


class RelayAdapterTests(unittest.TestCase):
    def setUp(self):
        self.header = SimpleNamespace(
            stamp=SimpleNamespace(sec=123, nanosec=456789012), frame_id='camera_optical_frame')
        self.message = SimpleNamespace(
            header=self.header, height=24, width=32, encoding='bgr8',
            is_bigendian=0, step=96, data=b'synthetic-input')
        self.frame = np.full((24, 32, 3), (17, 67, 201), dtype=np.uint8)
        self.bridge = Mock()
        self.bridge.imgmsg_to_cv2.return_value = self.frame
        self.bridge.cv2_to_imgmsg.return_value = SimpleNamespace(
            header=None, height=24, width=32, encoding='bgr8')

    def test_clean_returns_exact_input_without_conversion(self):
        before = deepcopy(self.message)
        result = process_image(self.message, self.bridge, attack_enabled=False)
        self.assertIs(result, self.message)
        self.assertEqual(result, before)
        self.bridge.imgmsg_to_cv2.assert_not_called()
        self.bridge.cv2_to_imgmsg.assert_not_called()

    def test_occlusion_preserves_header_and_converts_bgr8(self):
        result = process_image(self.message, self.bridge, attack_enabled=True)
        self.assertEqual(result.header, self.header)
        self.assertIsNot(result.header, self.header)
        self.assertEqual((result.height, result.width), (24, 32))
        self.bridge.imgmsg_to_cv2.assert_called_once_with(self.message, desired_encoding='bgr8')
        args, kwargs = self.bridge.cv2_to_imgmsg.call_args
        self.assertEqual(kwargs, {'encoding': 'bgr8'})
        expected = self.frame.copy()
        expected[6:, 6:25] = 0
        np.testing.assert_array_equal(args[0], expected)
        self.assertEqual(result.header.stamp.nanosec, 456789012)
        self.assertEqual(result.header.frame_id, 'camera_optical_frame')

    def test_invalid_rectangle_does_not_emit_message(self):
        with self.assertRaises(ValueError):
            process_image(self.message, self.bridge, attack_enabled=True, mask_x=-0.2)
        self.bridge.cv2_to_imgmsg.assert_not_called()

    def test_conversion_error_is_not_silently_replaced_with_clean_image(self):
        self.bridge.imgmsg_to_cv2.side_effect = ValueError('invalid image')
        with self.assertRaises(ValueError):
            process_image(self.message, self.bridge, attack_enabled=True)
        self.bridge.cv2_to_imgmsg.assert_not_called()


if __name__ == '__main__':
    unittest.main()
