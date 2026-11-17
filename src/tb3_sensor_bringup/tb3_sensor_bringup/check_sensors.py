#!/usr/bin/env python3
"""Check that camera, LiDAR, IMU and wheel speed are alive together.

Passive diagnostic: subscribes to the four topics for a fixed wall-clock
duration, reports counts, simulation-stamp gaps and monotonicity, and writes a
JSON record.  It publishes no commands and does not decode image pixels.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState, LaserScan


class SensorCheck(Node):
    TOPICS = {
        'camera': ('/camera/image_raw', Image),
        'lidar': ('/scan', LaserScan),
        'imu': ('/imu', Imu),
        'wheel': ('/wheel_speed', JointState),
    }

    def __init__(self, allow_missing=False):
        super().__init__('sensor_check')
        self.allow_missing = allow_missing
        self.counts = {kind: 0 for kind in self.TOPICS}
        self.first = {kind: None for kind in self.TOPICS}
        self.last = {kind: None for kind in self.TOPICS}
        self.max_gap = {kind: 0.0 for kind in self.TOPICS}
        self.backwards = {kind: 0 for kind in self.TOPICS}
        self.duplicates = {kind: 0 for kind in self.TOPICS}
        for kind, (topic, msg_type) in self.TOPICS.items():
            qos = qos_profile_sensor_data if msg_type is not JointState else 10
            self.create_subscription(msg_type, topic, self.make_callback(kind), qos)

    def make_callback(self, kind):
        def callback(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.counts[kind] += 1
            self.first[kind] = stamp if self.first[kind] is None else self.first[kind]
            if self.last[kind] is not None:
                delta = stamp - self.last[kind]
                if delta < 0:
                    self.backwards[kind] += 1
                elif delta == 0:
                    self.duplicates[kind] += 1
                else:
                    self.max_gap[kind] = max(self.max_gap[kind], delta)
            self.last[kind] = stamp
        return callback

    def summary(self):
        rows = {}
        for kind in self.TOPICS:
            rows[kind] = {
                'topic': self.TOPICS[kind][0],
                'count': self.counts[kind],
                'first_stamp_s': self.first[kind],
                'last_stamp_s': self.last[kind],
                'max_stamp_gap_s': self.max_gap[kind],
                'backwards_stamps': self.backwards[kind],
                'duplicate_stamps': self.duplicates[kind],
            }
        return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--max-gap', type=float, default=1.0,
                        help='Maximum allowed simulation-stamp gap per sensor (s)')
    parser.add_argument('--allow-missing', action='store_true',
                        help='Report missing topics but do not fail')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error('--seconds must be between 1 and 300')
    if args.max_gap <= 0:
        parser.error('--max-gap must be positive')

    rclpy.init()
    allow_missing = args.allow_missing
    node = SensorCheck(allow_missing)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        end = time.monotonic()
        summary = node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    failures = []
    for kind, row in summary.items():
        if row['count'] == 0:
            failures.append(f'{kind}: no messages on {row["topic"]}')
        elif row['backwards_stamps']:
            failures.append(f'{kind}: {row["backwards_stamps"]} backward stamps')
        elif row['max_stamp_gap_s'] > args.max_gap:
            failures.append(
                f'{kind}: max stamp gap {row["max_stamp_gap_s"]:.3f} s > {args.max_gap:.3f} s')
    if allow_missing:
        failures = [f for f in failures if 'no messages' not in f]
    passed = not failures

    print('Sensor check (simulation stamps; passive, no commands):')
    for kind, row in summary.items():
        print(f"  {kind:7s} {row['topic']:20s} count={row['count']:6d} "
              f"max_gap={row['max_stamp_gap_s']:.3f}s "
              f"back={row['backwards_stamps']} dup={row['duplicate_stamps']}")
    print(f'{"SENSOR_CHECK_PASS" if passed else "SENSOR_CHECK_FAIL"}: '
          f'{len(failures)} failure(s), duration={end - start:.1f} s')
    for failure in failures:
        print('  - ' + failure)

    result = {
        'started_utc': datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'),
        'duration_wall_s': end - start,
        'max_gap_s': args.max_gap,
        'allow_missing': node.allow_missing,
        'sensors': summary,
        'failures': failures,
        'passed': passed,
    }
    out = Path(os.path.expanduser('~/sensor-checks')) / result['started_utc']
    out.mkdir(parents=True, exist_ok=False)
    (out / 'check.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(f'Record saved: {out}')
    raise SystemExit(0 if passed else 2)


if __name__ == '__main__':
    main()
