"""Real CvBridge and DDS tests; explicitly skipped when ROS 2 is unavailable."""

import time
import unittest

import numpy as np

try:
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from tb3_camera_attack.node import CameraOcclusion
except ModuleNotFoundError as exc:
    if exc.name.split('.')[0] not in ('rclpy', 'cv_bridge', 'sensor_msgs', 'rcl_interfaces'):
        raise
    ROS_UNAVAILABLE = str(exc)
else:
    ROS_UNAVAILABLE = ''


@unittest.skipIf(bool(ROS_UNAVAILABLE), f'ROS 2 unavailable: {ROS_UNAVAILABLE}')
class RosRelayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.relay = CameraOcclusion(namespace='/occlusion_test', parameter_overrides=[
            Parameter('input_topic', value='raw'), Parameter('output_topic', value='attacked')])
        self.probe = Node('probe', namespace='/occlusion_test')
        self.received = []
        self.publisher = self.probe.create_publisher(Image, 'raw', qos_profile_sensor_data)
        self.subscription = self.probe.create_subscription(
            Image, 'attacked', self.received.append, qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.relay)
        self.executor.add_node(self.probe)
        self.bridge = CvBridge()
        self.frame = np.random.default_rng(7).integers(
            1, 256, size=(240, 320, 3), dtype=np.uint8)
        self.message = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        self.message.header.stamp.sec = 123
        self.message.header.stamp.nanosec = 456789012
        self.message.header.frame_id = 'camera_optical_frame'

    def tearDown(self):
        self.executor.shutdown()
        self.probe.destroy_node()
        self.relay.destroy_node()

    def exchange(self):
        # Republish a fixed synthetic sample until DDS discovery and delivery finish.
        self.received.clear()
        self.message.header.stamp.nanosec += 1
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self.publisher.publish(self.message)
            self.executor.spin_once(timeout_sec=0.05)
            matching = [msg for msg in self.received if msg.header == self.message.header]
            if matching:
                return matching[-1]
        self.fail('No relayed image received within five seconds')

    def test_clean_message_equal_over_dds(self):
        self.assertEqual(self.exchange(), self.message)

    def test_occlusion_pixels_dimensions_and_header_over_dds(self):
        result = self.relay.set_parameters_atomically([Parameter('attack_enabled', value=True)])
        self.assertTrue(result.successful, result.reason)
        output = self.exchange()
        expected = self.frame.copy()
        expected[60:, 64:256] = 0
        np.testing.assert_array_equal(self.bridge.imgmsg_to_cv2(output, 'bgr8'), expected)
        self.assertEqual((output.height, output.width), (240, 320))
        self.assertEqual(output.header, self.message.header)
        np.testing.assert_array_equal(self.bridge.imgmsg_to_cv2(self.message, 'bgr8'), self.frame)

    def test_rgb_input_preserves_colors_outside_mask_and_header(self):
        self.message = self.bridge.cv2_to_imgmsg(self.frame[:, :, ::-1].copy(), encoding='rgb8')
        self.message.header.stamp.sec = 42
        self.message.header.frame_id = 'rgb_camera'
        self.relay.set_parameters_atomically([Parameter('attack_enabled', value=True)])
        output = self.exchange()
        expected = self.frame.copy()
        expected[60:, 64:256] = 0
        np.testing.assert_array_equal(self.bridge.imgmsg_to_cv2(output, 'bgr8'), expected)
        self.assertEqual(output.header, self.message.header)

    def test_invalid_runtime_coordinates_rejected_and_node_keeps_processing(self):
        self.relay.set_parameters_atomically([Parameter('attack_enabled', value=True)])
        original = self.relay.rectangle()
        for name, value in (('mask_x', -0.1), ('mask_y', 1.0), ('mask_width', 0.0),
                            ('mask_height', 2.0), ('mask_x', float('nan'))):
            with self.subTest(parameter=name, value=value):
                result = self.relay.set_parameters_atomically([Parameter(name, value=value)])
                self.assertFalse(result.successful)
                self.assertEqual(self.relay.rectangle(), original)
        self.assertEqual(self.exchange().header, self.message.header)

    def test_valid_atomic_rectangle_and_clean_toggle(self):
        result = self.relay.set_parameters_atomically([
            Parameter('attack_enabled', value=True), Parameter('mask_x', value=0.0),
            Parameter('mask_y', value=0.0), Parameter('mask_width', value=1.0),
            Parameter('mask_height', value=1.0)])
        self.assertTrue(result.successful, result.reason)
        output = self.exchange()
        self.assertFalse(self.bridge.imgmsg_to_cv2(output, 'bgr8').any())
        self.relay.set_parameters_atomically([Parameter('attack_enabled', value=False)])
        self.assertEqual(self.exchange(), self.message)

    def test_invalid_startup_coordinates_use_valid_defaults(self):
        node = CameraOcclusion(namespace='/invalid_startup_test', parameter_overrides=[
            Parameter('mask_x', value=-100.0), Parameter('mask_width', value=2.0)])
        try:
            self.assertEqual(node.rectangle(), dict(
                mask_x=0.2, mask_y=0.25, mask_width=0.6, mask_height=0.75))
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
