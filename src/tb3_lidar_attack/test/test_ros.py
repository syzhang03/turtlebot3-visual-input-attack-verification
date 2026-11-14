"""Real LaserScan serialization and DDS tests. Never publish velocity commands."""
import math
from pathlib import Path
import tempfile
import time
import unittest
import uuid

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tb3_lidar_attack.core import RangeBias
    from tb3_lidar_attack.node import LidarBias, transform_message
except ModuleNotFoundError:
    rclpy = None


def scan():
    msg = LaserScan()
    msg.header.frame_id = 'laser_frame'
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 345
    msg.angle_min, msg.angle_max, msg.angle_increment = -math.pi/2, math.pi/2, math.pi/6
    msg.time_increment, msg.scan_time = .001, .2
    msg.range_min, msg.range_max = .1, 10.
    msg.ranges = [2.]*7
    msg.intensities = [1., 2., 3., 4., 5., 6., 7.]
    return msg


@unittest.skipIf(rclpy is None, 'ROS 2 libraries are not available')
class MessageTests(unittest.TestCase):
    def test_payload_only_transform_preserves_metadata_and_input(self):
        msg = scan()
        attack = RangeBias(sector_half_width_deg=31.)
        result = attack.process(12*10**9+345, msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max)
        out = transform_message(msg, result)
        self.assertEqual(list(out.ranges), [2., 2., 3., 3., 3., 2., 2.])
        self.assertEqual(list(msg.ranges), [2.]*7)
        self.assertIsNot(msg.header, out.header)
        for field in msg.get_fields_and_field_types():
            if field != 'ranges':
                self.assertEqual(getattr(msg, field), getattr(out, field), field)


@unittest.skipIf(rclpy is None, 'ROS 2 libraries are not available')
class DDSTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.folder = tempfile.TemporaryDirectory()
        self.prefix = '/joint_test_' + uuid.uuid4().hex
        self.driver = Node('joint_test_driver_' + uuid.uuid4().hex)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.driver)
        self.relay = None

    def tearDown(self):
        self.executor.shutdown()
        if self.relay is not None:
            self.relay.destroy_node()
        self.driver.destroy_node()
        rclpy.shutdown()
        self.folder.cleanup()

    def spin_until(self, predicate, timeout=8.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.executor.spin_once(timeout_sec=.05)
        self.fail('DDS discovery or message delivery timed out')

    def exercise(self, bias):
        self.relay = LidarBias(parameter_overrides=[Parameter(k, value=v) for k, v in dict(
            input_topic=self.prefix+'/raw', output_topic=self.prefix+'/attacked',
            bias_m=bias, sector_half_width_deg=31., log_dir=self.folder.name).items()])
        self.executor.add_node(self.relay)
        publisher = self.driver.create_publisher(LaserScan, self.prefix+'/raw', qos_profile_sensor_data)
        reliable_received, best_effort_received = [], []
        self.driver.create_subscription(LaserScan, self.prefix+'/attacked', reliable_received.append,
                                        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.driver.create_subscription(LaserScan, self.prefix+'/attacked', best_effort_received.append,
                                        qos_profile_sensor_data)
        self.spin_until(lambda: publisher.get_subscription_count() == 1
                        and self.relay.publisher.get_subscription_count() == 2)
        msg = scan()
        # Fresh timestamps allow recovery from best-effort loss at either hop.
        deadline = time.monotonic()+8
        common = set()
        while not common and time.monotonic() < deadline:
            msg.header.stamp.nanosec += 1
            publisher.publish(msg)
            self.executor.spin_once(timeout_sec=.05)
            common = ({m.header.stamp.nanosec for m in reliable_received}
                      & {m.header.stamp.nanosec for m in best_effort_received})
        self.assertTrue(common)
        stamp = min(common)
        reliable = next(m for m in reliable_received if m.header.stamp.nanosec == stamp)
        best_effort = next(m for m in best_effort_received if m.header.stamp.nanosec == stamp)
        self.assertEqual(reliable, best_effort)
        expected = [2., 2., 2.+bias, 2.+bias, 2.+bias, 2., 2.]
        self.assertEqual(list(reliable.ranges), expected)
        self.assertEqual(reliable.header.stamp.sec, msg.header.stamp.sec)
        self.assertEqual(reliable.header.frame_id, msg.header.frame_id)
        self.assertEqual(reliable.intensities, msg.intensities)
        self.assertTrue(list(Path(self.folder.name).glob('*/scans.csv')))
        # An invalid ray must survive injection and transport.
        msg.header.stamp.sec += 1
        msg.ranges[3] = math.nan
        deadline = time.monotonic()+8
        while not any(m.header.stamp.sec == 13 for m in reliable_received) and time.monotonic() < deadline:
            publisher.publish(msg)
            self.executor.spin_once(timeout_sec=.05)
        last = [m for m in reliable_received if m.header.stamp.sec == 13]
        self.assertTrue(last)
        self.assertTrue(math.isnan(last[-1].ranges[3]))

    def test_bias_over_dds_to_reliable_and_sensor_data_subscribers(self):
        self.exercise(1.)

    def test_passthrough_over_dds(self):
        self.exercise(0.)


if __name__ == '__main__':
    unittest.main()
