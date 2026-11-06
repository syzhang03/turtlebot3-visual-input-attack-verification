"""LiDAR-only ROS adapter. No image or visual-decision subscriptions."""
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

from .core import Controller, ScanResult, corridor_scan, stopping_distance, wrap


def yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values) - 1) > 0.02:
        raise ValueError('Invalid quaternion')
    if abs(q.x) > 0.01 or abs(q.y) > 0.01:
        raise ValueError('Only planar transforms/poses are supported')
    return math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)


class LidarBaseline(Node):
    def __init__(self, node_name='lidar_baseline'):
        super().__init__(node_name)
        defaults = dict(
            enabled=False, scan_topic='/scan', odom_topic='/odom', cmd_topic='/cmd_vel',
            base_frame='base_footprint', speed=1.0, goal_distance=4.0,
            enable_goal_stop=True, front_extent=0.15, corridor_half_width=0.20,
            min_clearance=0.35, reaction_time=0.30, braking_deceleration=1.0,
            sensor_timeout_wall=2.0, max_stamp_age_sim=0.5, control_period=0.05,
            allow_positive_infinity=True, heading_gain=1.0, max_angular_speed=0.3,
            max_trial_seconds=120.0, log_dir='~/lidar-results')
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for key, default in defaults.items():
            value = self.cfg[key]
            if isinstance(default, bool):
                if not isinstance(value, bool):
                    raise ValueError(f'{key} must be boolean')
            elif isinstance(default, float):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{key} must be finite and positive')
            elif not value:
                raise ValueError(f'{key} must be nonempty')
        self.controller = Controller()
        self.scan_result = ScanResult(False, reason='NO_SCAN')
        self.scan_at = self.odom_at = None
        self.scan_stamp = self.odom_stamp = None
        self.pose = self.origin = None
        self.odom_v = 0.0
        self.created = time.monotonic()
        self.start_sim = self.last_sim = None
        self.previous_state = None
        self.threshold = self.stop_threshold(self.cfg['speed'])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.command = self.create_publisher(TwistStamped, self.cfg['cmd_topic'], 10)
        self.create_subscription(LaserScan, self.cfg['scan_topic'], self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom, qos_profile_sensor_data)
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.run_dir = Path(self.cfg['log_dir']).expanduser() / run_id
        self.run_dir.mkdir(parents=True, exist_ok=False)
        (self.run_dir / 'config.json').write_text(json.dumps(dict(
            self.cfg, run_id=run_id, use_sim_time=self.get_parameter('use_sim_time').value,
            revision=('lidar-only-v1' if node_name == 'lidar_baseline' else 'camera-lidar-monitor-v1'),
            status='DEVELOPMENT_NOT_FORMAL_EVALUATION'), indent=2))
        self.log = (self.run_dir / 'events.csv').open('w', newline='')
        self.writer = csv.writer(self.log)
        self.writer.writerow(['wall_elapsed_s', 'sim_time_s', 'state', 'distance_m',
                              'corridor_clearance_m', 'stop_threshold_m', 'scan_usable',
                              'scan_reason', 'invalid_rays', 'cmd_v', 'cmd_w', 'odom_v',
                              'scan_age_wall_s', 'odom_age_wall_s', 'scan_stamp_s', 'odom_stamp_s'])
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.cfg['control_period'], self.tick, clock=self.wall_clock)
        self.get_logger().info(f'Logs: {self.run_dir}; enabled={self.cfg["enabled"]}')

    def stop_threshold(self, speed):
        return stopping_distance(speed, self.cfg['min_clearance'],
                                 self.cfg['reaction_time'], self.cfg['braking_deceleration'])

    def accept_stamp(self, msg, previous):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp < 0 or (previous is not None and stamp < previous):
            self.controller.stop('FAULT')
            return None
        if previous is not None and stamp == previous:
            return None  # Repeated messages must not refresh the watchdog.
        return stamp

    def on_scan(self, msg):
        stamp = self.accept_stamp(msg, self.scan_stamp)
        if stamp is None:
            return
        self.scan_stamp, self.scan_at = stamp, time.monotonic()
        self.threshold = self.stop_threshold(max(self.cfg['speed'], max(0.0, self.odom_v)))
        try:
            if not msg.header.frame_id:
                raise ValueError('Missing scan frame')
            transform = self.tf.lookup_transform(self.cfg['base_frame'], msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp))
            t = transform.transform.translation
            rotation = yaw(transform.transform.rotation)
            self.scan_result = corridor_scan(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                t.x, t.y, rotation, self.cfg['corridor_half_width'], self.cfg['front_extent'],
                self.threshold, self.cfg['allow_positive_infinity'])
        except (TransformException, ValueError) as exc:
            self.scan_result = ScanResult(False, reason=f'TRANSFORM: {exc}')
        # Do not lose a hazard/invalid scan if a clear scan arrives before a control tick.
        if self.cfg['enabled'] and self.controller.started:
            if not self.scan_result.usable:
                self.controller.stop('SENSOR_INVALID')
            elif self.scan_result.clearance <= self.threshold:
                self.controller.stop('OBSTACLE_STOP')

    def on_odom(self, msg):
        stamp = self.accept_stamp(msg, self.odom_stamp)
        if stamp is None:
            return
        p = msg.pose.pose.position
        v = msg.twist.twist.linear.x
        try:
            heading = yaw(msg.pose.pose.orientation)
            if not all(math.isfinite(n) for n in (p.x, p.y, v)):
                raise ValueError('Nonfinite odometry')
        except ValueError:
            self.controller.stop('FAULT')
            return
        self.pose, self.odom_v = (p.x, p.y, heading), v
        self.odom_stamp, self.odom_at = stamp, time.monotonic()

    def publish(self, linear=0.0, angular=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['base_frame']
        msg.twist.linear.x, msg.twist.angular.z = float(linear), float(angular)
        self.command.publish(msg)

    def extra_input_status(self, wall, sim):
        return True, True

    def tick(self):
        wall = time.monotonic()
        sim = self.get_clock().now().nanoseconds * 1e-9
        if self.last_sim is not None and sim < self.last_sim:
            self.controller.stop('FAULT')
        self.last_sim = sim
        scan_age = wall - self.scan_at if self.scan_at is not None else math.inf
        odom_age = wall - self.odom_at if self.odom_at is not None else math.inf
        ready = self.scan_at is not None and self.pose is not None and sim > 0
        fresh = (max(scan_age, odom_age) <= self.cfg['sensor_timeout_wall']
                 and all(s is not None and -0.05 <= sim - s <= self.cfg['max_stamp_age_sim']
                         for s in (self.scan_stamp, self.odom_stamp)))
        extra_ready, extra_fresh = self.extra_input_status(wall, sim)
        ready, fresh = ready and extra_ready, fresh and extra_fresh
        if ready and fresh and self.origin is None:
            self.origin = self.pose
        distance = 0.0
        if self.origin is not None:
            x, y, heading = self.origin
            distance = (self.pose[0] - x)*math.cos(heading) + (self.pose[1] - y)*math.sin(heading)
        if self.count_publishers(self.cfg['cmd_topic']) > 1:
            self.controller.stop('COMMAND_CONFLICT')
        if self.start_sim is not None and sim - self.start_sim > self.cfg['max_trial_seconds']:
            self.controller.stop('TRIAL_TIMEOUT')
        self.threshold = self.stop_threshold(max(self.cfg['speed'], max(0.0, self.odom_v)))
        linear = self.controller.step(
            self.cfg['enabled'], ready, fresh, self.scan_result.usable,
            self.scan_result.clearance <= self.threshold,
            self.cfg['enable_goal_stop'] and distance >= self.cfg['goal_distance'], self.cfg['speed'])
        angular = 0.0
        if linear:
            if self.start_sim is None:
                self.start_sim = sim
            limit = self.cfg['max_angular_speed']
            angular = max(-limit, min(limit, self.cfg['heading_gain'] * wrap(self.origin[2] - self.pose[2])))
        self.publish(linear, angular)
        self.writer.writerow([wall-self.created, sim, self.controller.state, distance,
                              self.scan_result.clearance, self.threshold, self.scan_result.usable,
                              self.scan_result.reason, self.scan_result.invalid_rays, linear, angular,
                              self.odom_v, scan_age, odom_age, self.scan_stamp, self.odom_stamp])
        self.log.flush()
        if self.previous_state != self.controller.state:
            self.get_logger().info(f'{self.controller.state}: distance={distance:.3f}; '
                                   f'clearance={self.scan_result.clearance:.3f}; '
                                   f'reason={self.scan_result.reason}')
            self.previous_state = self.controller.state


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = LidarBaseline()
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
