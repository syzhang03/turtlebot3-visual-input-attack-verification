"""LaserScan input -> logged range bias -> LaserScan output; no motion control."""
import copy
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .core import RangeBias


def transform_message(message, result):
    if not result.changed_rays:
        return message
    output = copy.deepcopy(message)
    output.ranges = list(result.ranges)
    return output


class LidarBias(Node):
    def __init__(self, **kwargs):
        super().__init__('lidar_bias', **kwargs)
        defaults = dict(input_topic='/scan', output_topic='/attack/scan',
                        output_reliability='reliable', bias_m=1.0, sector_center_deg=0.0,
                        sector_half_width_deg=30.0, start_s=0.0, duration_s=600.0,
                        log_dir='~/lidar-attack-results')
        self.cfg = {k: self.declare_parameter(k, v, ParameterDescriptor(read_only=True)).value
                    for k, v in defaults.items()}
        if self.cfg['output_reliability'] not in ('reliable', 'best_effort'):
            raise ValueError('output_reliability must be reliable or best_effort')
        if not all(self.cfg[k].strip() for k in ('input_topic', 'output_topic', 'log_dir')):
            raise ValueError('Topic names and log_dir must not be empty')
        if self.resolve_topic_name(self.cfg['input_topic']) == self.resolve_topic_name(self.cfg['output_topic']):
            raise ValueError('Input and output must resolve to distinct topics')
        self.attack = RangeBias(**{k: self.cfg[k] for k in (
            'bias_m', 'sector_center_deg', 'sector_half_width_deg', 'start_s', 'duration_s')})
        qos = QoSProfile(depth=1, reliability=(ReliabilityPolicy.RELIABLE
                         if self.cfg['output_reliability'] == 'reliable' else ReliabilityPolicy.BEST_EFFORT))
        self.publisher = self.create_publisher(LaserScan, self.cfg['output_topic'], qos)
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.run_dir = Path(self.cfg['log_dir']).expanduser() / run_id
        self.run_dir.mkdir(parents=True, exist_ok=False)
        (self.run_dir / 'config.json').write_text(json.dumps(dict(
            self.cfg, run_id=run_id, use_sim_time=self.get_parameter('use_sim_time').value,
            time_basis='original scan header stamps; schedule starts at first scan',
            sector_frame='input scan frame (+X = 0 degrees)', header_policy='unchanged',
        ), indent=2), encoding='utf-8')
        self.log = (self.run_dir / 'scans.csv').open('w', newline='', encoding='utf-8')
        fields = ['input_stamp_ns', 'output_stamp_ns', 'elapsed_s', 'state', 'reset',
                  'eligible_rays', 'changed_rays', 'clipped_rays', 'invalid_rays',
                  'min_delta_m', 'max_delta_m', 'input_sector_min_m', 'output_sector_min_m',
                  'duplicates_total', 'callback_monotonic_ns', 'input_gap_wall_s',
                  'publish_call_s', 'output_subscribers']
        self.writer = csv.DictWriter(self.log, fieldnames=fields)
        self.writer.writeheader()
        self.log.flush()
        self.previous_callback_ns = self.previous_state = None
        self.create_subscription(LaserScan, self.cfg['input_topic'], self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'LiDAR attack logs: {self.run_dir}')

    def on_scan(self, msg):
        now = time.monotonic_ns()
        gap = (now - self.previous_callback_ns) / 1e9 if self.previous_callback_ns is not None else ''
        self.previous_callback_ns = now
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        result = self.attack.process(stamp, msg.ranges, msg.angle_min, msg.angle_increment,
                                     msg.range_min, msg.range_max)
        if result is None:
            return
        output = transform_message(msg, result)
        before_publish = time.monotonic_ns()
        self.publisher.publish(output)
        publish_s = (time.monotonic_ns() - before_publish) / 1e9
        row = asdict(result)
        del row['ranges']
        row.update(input_stamp_ns=stamp, output_stamp_ns=stamp,
                   duplicates_total=self.attack.duplicates, callback_monotonic_ns=now,
                   input_gap_wall_s=gap, publish_call_s=publish_s,
                   output_subscribers=self.publisher.get_subscription_count())
        self.writer.writerow(row)
        self.log.flush()
        if result.state != self.previous_state or result.reset:
            self.get_logger().info(
                f'{result.state}; bias={self.cfg["bias_m"]:.3f}m; changed={result.changed_rays}; '
                f'reset={result.reset}; elapsed={result.elapsed_s:.3f}s')
            self.previous_state = result.state

    def destroy_node(self):
        if hasattr(self, 'log'):
            self.log.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarBias()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
