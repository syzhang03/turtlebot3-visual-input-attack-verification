import csv
import json
import math
from pathlib import Path
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from .core import Trial, yaw_from_quaternion, wrap_angle
from .vision import detect_red


class Baseline(Node):
    def __init__(self):
        super().__init__('visual_baseline')
        defaults = {
            'enabled': False, 'image_topic': '/camera/image_raw',
            'odom_topic': '/odom', 'cmd_topic': '/cmd_vel',
            'speed': 0.05, 'goal_distance': 1.0, 'sensor_timeout': 2.0,
            'red_area_threshold': 0.06, 'roi_top': 0.25,
            'heading_gain': 1.0, 'max_angular_speed': 0.3,
            'max_trial_seconds': 120.0, 'log_dir': '~/baseline-results',
        }
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for k in ('red_area_threshold', 'roi_top'):
            if not 0 < self.cfg[k] < 1:
                raise ValueError(f'{k} must lie strictly between 0 and 1')
        for k in ('heading_gain', 'max_angular_speed', 'max_trial_seconds'):
            if not math.isfinite(self.cfg[k]) or self.cfg[k] <= 0:
                raise ValueError(f'{k} must be finite and positive')
        self.trial = Trial(self.cfg['goal_distance'], self.cfg['speed'], self.cfg['sensor_timeout'])
        self.bridge = CvBridge()
        self.image_at = self.odom_at = None
        self.image_stamp = self.odom_stamp = None
        self.last_image_stamp = self.last_odom_stamp = None
        self.origin = self.pose = None
        self.blocked, self.ratio = False, 0.0
        self.measured_v = 0.0
        self.previous_state = None
        self.last_sim = None
        self.created = time.monotonic()
        self.running_since = None
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.run_dir = Path(self.cfg['log_dir']).expanduser() / run_id
        self.run_dir.mkdir(parents=True, exist_ok=False)
        metadata = dict(self.cfg, use_sim_time=self.get_parameter('use_sim_time').value,
                        run_id=run_id, status='LOCAL_BASELINE_NOT_FORMAL_EVALUATION')
        (self.run_dir / 'config.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        self.log = (self.run_dir / 'events.csv').open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.log)
        self.writer.writerow(['wall_elapsed_s', 'sim_time_s', 'state', 'distance_m',
                              'red_area_ratio', 'visual_blocked', 'cmd_v', 'cmd_w',
                              'odom_v', 'image_age_wall_s', 'odom_age_wall_s',
                              'image_stamp_s', 'odom_stamp_s'])
        self.command = self.create_publisher(TwistStamped, self.cfg['cmd_topic'], 10)
        self.debug = self.create_publisher(Image, '/baseline/debug_image', qos_profile_sensor_data)
        self.create_subscription(Image, self.cfg['image_topic'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom, qos_profile_sensor_data)
        # A steady timer also publishes zero if the simulator's clock stops.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.tick, clock=self.wall_clock)
        self.get_logger().info(f'Logs: {self.run_dir}; enabled={self.cfg["enabled"]}')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_image(self, msg):
        stamp = self.stamp(msg)
        if self.last_image_stamp is not None and stamp <= self.last_image_stamp:
            if stamp < self.last_image_stamp:
                self.trial.state = 'FAULT'
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.blocked, self.ratio, annotated = detect_red(
                frame, self.cfg['red_area_threshold'], self.cfg['roi_top'])
            self.image_at = time.monotonic()
            self.image_stamp = self.last_image_stamp = stamp
            # Preserve a detected obstacle between control ticks.
            if self.cfg['enabled'] and self.blocked:
                self.trial.state = 'OBSTACLE_STOP'
            if self.debug.get_subscription_count():
                output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                output.header = msg.header
                self.debug.publish(output)
        except Exception as exc:
            self.trial.state = 'FAULT'
            self.get_logger().error(f'Image processing failed: {exc}')

    def on_odom(self, msg):
        stamp = self.stamp(msg)
        if self.last_odom_stamp is not None and stamp <= self.last_odom_stamp:
            if stamp < self.last_odom_stamp:
                self.trial.state = 'FAULT'
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w, msg.twist.twist.linear.x)
        if not all(math.isfinite(v) for v in values):
            self.trial.state = 'FAULT'
            return
        self.pose = (p.x, p.y, yaw_from_quaternion(q))
        self.measured_v = msg.twist.twist.linear.x
        self.odom_at = time.monotonic()
        self.odom_stamp = self.last_odom_stamp = stamp

    def publish(self, linear=0.0, angular=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.angular.z = float(linear), float(angular)
        self.command.publish(msg)

    def tick(self):
        wall = time.monotonic()
        sim = self.get_clock().now().nanoseconds * 1e-9
        if self.last_sim is not None and sim < self.last_sim:
            self.trial.state = 'FAULT'
        self.last_sim = sim
        image_age = wall - self.image_at if self.image_at is not None else float('inf')
        odom_age = wall - self.odom_at if self.odom_at is not None else float('inf')
        ready = self.image_at is not None and self.pose is not None and sim > 0
        if ready and self.origin is None:
            self.origin = self.pose
        distance = 0.0
        if self.origin is not None:
            x, y, yaw = self.origin
            distance = (self.pose[0] - x) * math.cos(yaw) + (self.pose[1] - y) * math.sin(yaw)
        if self.running_since is not None and sim - self.running_since > self.cfg['max_trial_seconds']:
            if self.trial.state == 'RUNNING':
                self.trial.state = 'FAULT'
                self.get_logger().error('Maximum simulation-time trial duration exceeded')
        linear = self.trial.step(self.cfg['enabled'], ready, image_age, odom_age,
                                 self.blocked, distance)
        if linear and self.running_since is None:
            self.running_since = sim
        angular = 0.0
        if linear:
            error = wrap_angle(self.origin[2] - self.pose[2])
            limit = self.cfg['max_angular_speed']
            angular = max(-limit, min(limit, self.cfg['heading_gain'] * error))
        self.publish(linear, angular)
        self.writer.writerow([wall - self.created, sim, self.trial.state, distance,
                              self.ratio, self.blocked, linear, angular, self.measured_v,
                              image_age, odom_age, self.image_stamp, self.odom_stamp])
        self.log.flush()
        if self.trial.state != self.previous_state:
            self.get_logger().info(f'{self.trial.state}: distance={distance:.3f}, red_area={self.ratio:.4f}')
            self.previous_state = self.trial.state


def main(args=None):
    # Keep the ROS context alive while sending final zero commands on Ctrl+C.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Baseline()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                for _ in range(5):
                    node.publish()
                    time.sleep(0.05)
            node.log.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
