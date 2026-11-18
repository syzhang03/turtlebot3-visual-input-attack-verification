"""Real DDS checks for camera input QoS and full-size replay messages. No motion."""
import tempfile
import time
import unittest

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from tb3_visual_baseline.replay_node import DelayReplay
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


@unittest.skipUnless(HAS_ROS, 'Requires ROS Jazzy and real DDS')
class RelayRosTests(unittest.TestCase):
    def exercise(self, delay, reliability, exchange):
        with tempfile.TemporaryDirectory(prefix='replay-qos-test-') as folder:
            rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false',
                '-p', 'input_topic:=/replay_qos_test/input',
                '-p', 'output_topic:=/replay_qos_test/output',
                '-p', f'input_reliability:={reliability}', '-p', 'output_reliability:=reliable',
                '-p', f'delay_s:={delay:.1f}', '-p', f'log_dir:={folder}'])
            relay = peer = executor = None
            try:
                relay = DelayReplay()
                inputs = [s for s in relay.subscriptions if s.topic_name == '/replay_qos_test/input']
                self.assertEqual(len(inputs), 1)
                qos = inputs[0].qos_profile
                self.assertEqual(qos.reliability, ReliabilityPolicy.RELIABLE if reliability == 'reliable'
                                 else ReliabilityPolicy.BEST_EFFORT)
                if reliability == 'reliable':
                    self.assertEqual(qos.depth, 1)
                    self.assertEqual(qos.history, HistoryPolicy.KEEP_LAST)
                if not exchange:
                    return
                peer = rclpy.create_node('replay_qos_test_peer')
                reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
                source = peer.create_publisher(Image, '/replay_qos_test/input', reliable)
                received = []
                sink = peer.create_subscription(Image, '/replay_qos_test/output', received.append, reliable)
                executor = SingleThreadedExecutor()
                executor.add_node(relay)
                executor.add_node(peer)

                def spin_until(predicate):
                    deadline = time.monotonic()+5
                    while not predicate() and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.02)
                    self.assertTrue(predicate(), 'DDS delivery/discovery did not complete within 5s')

                spin_until(lambda: source.get_subscription_count() == 1
                           and relay.publisher.get_subscription_count() == 1)
                for stamp, color in ((10, 17), (11, 29)):
                    msg = Image()
                    msg.header.stamp.sec, msg.header.frame_id = stamp, 'camera_frame'
                    msg.height, msg.width, msg.step, msg.encoding = 480, 640, 1920, 'rgb8'
                    msg.data = bytes([color])*921600
                    source.publish(msg)
                    spin_until(lambda: any(m.header.stamp.sec == stamp for m in received))
                output = next(m for m in received if m.header.stamp.sec == 11)
                self.assertEqual((output.height, output.width, output.step, output.encoding),
                                 (480, 640, 1920, 'rgb8'))
                self.assertEqual(output.header.frame_id, 'camera_frame')
                self.assertEqual(bytes(output.data), bytes([17 if delay else 29])*921600)
                self.assertFalse(any(p.topic_name == '/cmd_vel' for p in relay.publishers))
            finally:
                if executor is not None:
                    executor.shutdown()
                if peer is not None:
                    peer.destroy_node()
                if relay is not None:
                    relay.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()

    def test_reliable_input_zero_delay_large_image(self):
        self.exercise(0., 'reliable', True)

    def test_reliable_input_delayed_payload_current_header(self):
        self.exercise(1., 'reliable', True)

    def test_best_effort_input_still_selectable(self):
        self.exercise(0., 'best_effort', False)


if __name__ == '__main__':
    unittest.main()
