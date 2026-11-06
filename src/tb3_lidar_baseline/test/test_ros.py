"""Real DDS checks; run in ROS Jazzy, not with mocked ROS modules."""
import math
import tempfile
import time
import unittest

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tb3_lidar_baseline.node import LidarBaseline
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


@unittest.skipUnless(HAS_ROS, 'Requires real ROS Jazzy packages')
class RosTests(unittest.TestCase):
    def setUp(self):
        self.logs = tempfile.TemporaryDirectory(prefix='lidar-ros-test-')
        rclpy.init(args=['--ros-args', '-p', 'enabled:=true', '-p', 'use_sim_time:=false',
                        '-p', 'scan_topic:=/lidar_test/scan', '-p', 'odom_topic:=/lidar_test/odom',
                        '-p', 'cmd_topic:=/lidar_test/cmd', '-p', 'speed:=0.1',
                        '-p', 'goal_distance:=0.2', '-p', 'sensor_timeout_wall:=0.6',
                        '-p', 'max_stamp_age_sim:=0.5', '-p', f'log_dir:={self.logs.name}'])
        self.baseline = LidarBaseline()
        self.peer = rclpy.create_node('lidar_test_peer')
        self.scans = self.peer.create_publisher(LaserScan, '/lidar_test/scan', qos_profile_sensor_data)
        self.odoms = self.peer.create_publisher(Odometry, '/lidar_test/odom', qos_profile_sensor_data)
        self.commands = []
        self.sub = self.peer.create_subscription(TwistStamped, '/lidar_test/cmd',
                                                 lambda m: self.commands.append(m.twist.linear.x), 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.baseline)
        self.executor.add_node(self.peer)
        self.frozen_scan = None

    def tearDown(self):
        self.executor.shutdown()
        self.baseline.log.close()
        self.baseline.destroy_node()
        self.peer.destroy_node()
        rclpy.shutdown()
        self.logs.cleanup()

    def pump(self, duration, scan=True, obstacle=False, invalid=False, frozen=False, x=0.0):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            stamp = self.peer.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.0
            odom.pose.pose.position.x = x
            self.odoms.publish(odom)
            if scan:
                msg = LaserScan()
                msg.header.stamp, msg.header.frame_id = stamp, 'base_footprint'
                msg.angle_min, msg.angle_increment = 0.0, math.pi/180
                msg.angle_max = 359*math.pi/180
                msg.range_min, msg.range_max = 0.12, 3.5
                msg.ranges = [math.inf]*360
                if obstacle:
                    msg.ranges[0] = 0.3
                if invalid:
                    msg.ranges[0] = math.nan
                if frozen and self.frozen_scan is not None:
                    msg = self.frozen_scan
                else:
                    self.frozen_scan = msg
                self.scans.publish(msg)
            for _ in range(6):
                self.executor.spin_once(timeout_sec=0.005)

    def assert_stopped(self, state):
        self.assertEqual(self.baseline.controller.state, state)
        self.assertTrue(self.commands)
        self.assertEqual(self.commands[-1], 0.0)

    def test_clear_route_then_goal(self):
        self.pump(0.5)
        self.assertIn(0.1, self.commands)
        self.pump(0.25, x=0.25)
        self.assert_stopped('GOAL_REACHED')
        topics = [s.topic_name for s in self.baseline.subscriptions]
        self.assertFalse(any('image' in t or 'camera' in t for t in topics))

    def test_obstacle_stop_is_latched(self):
        self.pump(0.5)
        self.pump(0.2, obstacle=True)
        self.pump(0.2)
        self.assert_stopped('OBSTACLE_STOP')

    def test_missing_scan_stops_while_odom_continues(self):
        self.pump(0.5)
        self.assertIn(0.1, self.commands)
        self.pump(0.85, scan=False)
        self.assert_stopped('SENSOR_TIMEOUT')

    def test_repeated_old_scan_does_not_refresh_watchdog(self):
        self.pump(0.5)
        self.assertIn(0.1, self.commands)
        self.pump(0.85, frozen=True)
        self.assert_stopped('SENSOR_TIMEOUT')

    def test_invalid_scan_is_not_clear(self):
        self.pump(0.5)
        self.pump(0.2, invalid=True)
        self.assert_stopped('SENSOR_INVALID')


if __name__ == '__main__':
    unittest.main()
