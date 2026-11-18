import math
import tempfile
import time
import unittest
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from nav_msgs.msg import Odometry
    from tb3_camera_lidar_monitor.node import CameraLidarMonitor
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


@unittest.skipUnless(HAS_ROS, 'Requires real ROS and cv_bridge')
class FusionRosTests(unittest.TestCase):
    image_reliability = 'best_effort'

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        args = ['--ros-args']
        for value in ['enabled:=true', 'use_sim_time:=false', 'cmd_topic:=/fusion_test/cmd',
                      'scan_topic:=/fusion_test/scan', 'odom_topic:=/fusion_test/odom',
                      'image_topic:=/fusion_test/image', f'image_reliability:={self.image_reliability}', 'speed:=0.1',
                      'sensor_timeout_wall:=0.5', 'pair_timeout:=0.4',
                      f'log_dir:={self.directory.name}']:
            args += ['-p', value]
        rclpy.init(args=args)
        self.node = CameraLidarMonitor()
        self.peer = rclpy.create_node('fusion_test_peer')
        qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
               if self.image_reliability == 'reliable' else qos_profile_sensor_data)
        self.images = self.peer.create_publisher(Image, '/fusion_test/image', qos)
        self.scans = self.peer.create_publisher(LaserScan, '/fusion_test/scan', qos_profile_sensor_data)
        self.odoms = self.peer.create_publisher(Odometry, '/fusion_test/odom', qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.peer)

    def tearDown(self):
        self.executor.shutdown()
        self.node.pair_log.close()
        self.node.log.close()
        self.node.destroy_node()
        self.peer.destroy_node()
        rclpy.shutdown()
        self.directory.cleanup()

    def pump(self, duration, distance=math.inf, red=False, camera=True):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            stamp = self.peer.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.
            self.odoms.publish(odom)
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_footprint'
            scan.angle_increment, scan.angle_max = math.pi/180, 359*math.pi/180
            scan.range_min, scan.range_max = .12, 3.5
            scan.ranges = [math.inf]*360
            scan.ranges[0] = distance
            self.scans.publish(scan)
            if camera:
                image = Image()
                image.header.stamp = stamp
                image.height = image.width = 40
                image.encoding, image.step = 'bgr8', 120
                pixels = bytearray(4800)
                if red:
                    for y in range(20,30):
                        for x in range(15,25): pixels[(y*40+x)*3+2] = 255
                image.data = bytes(pixels)
                self.images.publish(image)
            for _ in range(6): self.executor.spin_once(timeout_sec=.005)

    def test_normal_pairing_and_single_command_owner(self):
        self.pump(.6)
        self.assertEqual(self.node.controller.state, 'RUNNING')
        self.assertFalse(self.node.monitor.alert)
        self.assertEqual(self.node.count_publishers('/fusion_test/cmd'), 1)

    def test_persistent_visual_miss_and_reverse_direction(self):
        self.pump(.6, distance=2.)
        self.assertEqual(self.node.controller.state, 'ANOMALY_STOP')
        self.assertEqual(self.node.monitor.direction, 'VISUAL_MISS')

    def test_visual_extra_also_alerts_without_nominal_visual_stop(self):
        self.pump(.6, red=True)
        self.assertEqual(self.node.controller.state, 'ANOMALY_STOP')
        self.assertEqual(self.node.monitor.direction, 'VISUAL_EXTRA')

    def test_camera_loss_is_not_agreement(self):
        self.pump(.5)
        self.assertEqual(self.node.controller.state, 'RUNNING')
        self.pump(.8, camera=False)
        self.assertEqual(self.node.controller.state, 'SENSOR_TIMEOUT')

    def test_lidar_danger_has_immediate_priority(self):
        self.pump(.5)
        self.pump(.2, distance=.3)
        self.assertEqual(self.node.controller.state, 'OBSTACLE_STOP')

    def test_camera_timestamp_rollback_latches_fault(self):
        self.pump(.5)
        self.images.publish(Image())  # Zero stamp is older than the accepted clock.
        for _ in range(20): self.executor.spin_once(timeout_sec=.01)
        self.assertEqual(self.node.controller.state, 'FAULT')


class FusionReliableRosTests(FusionRosTests):
    """Run the same guard, pairing and timeout checks with reliable image delivery."""
    image_reliability = 'reliable'
