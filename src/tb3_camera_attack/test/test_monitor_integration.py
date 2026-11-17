"""Real DDS test against main's actual monitor, with isolated command output."""
import math
import tempfile
import time
import unittest
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from nav_msgs.msg import Odometry
    from tb3_camera_attack.node import CameraOcclusion
    from tb3_camera_lidar_monitor.node import CameraLidarMonitor
except ModuleNotFoundError as exc:
    MISSING=str(exc)
else:
    MISSING=''

@unittest.skipIf(bool(MISSING), 'Real ROS integration Not Tested: '+MISSING)
class MonitorIntegration(unittest.TestCase):
    def test_clean_and_attack_reach_monitor(self):
        with tempfile.TemporaryDirectory() as directory:
            rclpy.init(args=['--ros-args','-p','use_sim_time:=false','-p','enabled:=false',
                '-p','image_topic:=/camera_attack_test/image','-p','scan_topic:=/camera_attack_test/scan',
                '-p','odom_topic:=/camera_attack_test/odom','-p','cmd_topic:=/camera_attack_test/cmd',
                '-p',f'log_dir:={directory}'])
            monitor=relay=peer=executor=None
            try:
                monitor=CameraLidarMonitor()
                relay=CameraOcclusion(parameter_overrides=[
                    Parameter('input_topic',value='/camera_attack_test/raw'),
                    Parameter('output_topic',value='/camera_attack_test/image')])
                peer=rclpy.create_node('camera_attack_test_peer')
                images=peer.create_publisher(Image,'/camera_attack_test/raw',qos_profile_sensor_data)
                scans=peer.create_publisher(LaserScan,'/camera_attack_test/scan',qos_profile_sensor_data)
                odoms=peer.create_publisher(Odometry,'/camera_attack_test/odom',qos_profile_sensor_data)
                received=[]
                sub=peer.create_subscription(Image,'/camera_attack_test/image',received.append,
                    QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
                executor=SingleThreadedExecutor()
                for n in (monitor,relay,peer):executor.add_node(n)
                for enabled in (False,True):
                    result=relay.set_parameters_atomically([Parameter('attack_enabled',value=enabled)])
                    self.assertTrue(result.successful,result.reason)
                    received.clear(); deadline=time.monotonic()+5
                    while time.monotonic()<deadline:
                        stamp=peer.get_clock().now().to_msg()
                        image=Image();image.header.stamp=stamp;image.header.frame_id='optical'
                        image.height=image.width=40;image.encoding='bgr8';image.step=120
                        image.data=bytes([50]*4800);images.publish(image)
                        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_footprint'
                        scan.angle_increment=math.pi/180;scan.angle_max=359*math.pi/180
                        scan.range_min=.12;scan.range_max=3.5;scan.ranges=[math.inf]*360;scans.publish(scan)
                        odom=Odometry();odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.;odoms.publish(odom)
                        for _ in range(10):executor.spin_once(timeout_sec=.005)
                        matching=[m for m in received if m.header==image.header]
                        if (matching and monitor.image_stamp == stamp.sec + stamp.nanosec/1e9
                                and monitor.pair_at is not None):
                            if enabled:self.assertIn(0,matching[-1].data)
                            else:self.assertEqual(matching[-1].data,image.data)
                            break
                    else:self.fail('No reliable output / actual monitor pairing within 5 seconds')
                self.assertEqual(peer.count_publishers('/camera_attack_test/image'),1)
            finally:
                if executor:executor.shutdown()
                if monitor:
                    monitor.pair_log.close();monitor.log.close();monitor.destroy_node()
                if relay:relay.destroy_node()
                if peer:peer.destroy_node()
                rclpy.shutdown()
