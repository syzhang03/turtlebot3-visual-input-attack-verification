#!/usr/bin/env python3
"""Check that the forward corridor is clear at the current robot pose.

Run while the robot is stationary, after `start_scene.sh empty` reports
SCENE_READY.  This is a passive check: it subscribes to /scan, publishes no
commands, and exits non-zero if a finite return appears inside the configured
forward sector nearer than the threshold.

The red box in the full scene is at X=4.0 m and the LiDAR range is 3.5 m, so
this check does **not** distinguish full from empty at the start pose.  Use the
baseline outcome (full stops near 2.95 m, empty reaches the 4.0 m endpoint) and
the static world tests to distinguish the scenes.  This is a development
check, not a collision proof.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class EmptyRoadCheck(Node):
    def __init__(self, scan_topic, half_angle):
        super().__init__('empty_road_check')
        self.half_angle = half_angle
        self.scans = 0
        self.samples = 0
        self.min_forward = math.inf
        self.first_stamp = None
        self.last_stamp = None
        self.backwards = 0
        self.duplicates = 0
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None:
            if stamp < self.last_stamp:
                self.backwards += 1
            elif stamp == self.last_stamp:
                self.duplicates += 1
        self.first_stamp = stamp if self.first_stamp is None else self.first_stamp
        self.last_stamp = stamp
        self.scans += 1
        if len(msg.ranges) < 2 or not math.isfinite(msg.angle_increment) or msg.angle_increment == 0:
            return
        for index, distance in enumerate(msg.ranges):
            angle = wrap(msg.angle_min + index * msg.angle_increment)
            if abs(angle) > self.half_angle:
                continue
            if not math.isfinite(distance) or distance < msg.range_min:
                continue
            self.samples += 1
            self.min_forward = min(self.min_forward, distance)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--min-clearance', type=float, default=2.0,
                        help='Fail if a finite forward return is nearer than this (m)')
    parser.add_argument('--half-angle-deg', type=float, default=10.0)
    parser.add_argument('--scan-topic', default='/scan')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error('--seconds must be between 1 and 60')
    if args.min_clearance <= 0:
        parser.error('--min-clearance must be positive')
    if not 0 < args.half_angle_deg <= 90:
        parser.error('--half-angle-deg must be between 0 and 90')

    rclpy.init()
    node = EmptyRoadCheck(args.scan_topic, math.radians(args.half_angle_deg))
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        end = time.monotonic()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    observed = node.min_forward
    passed = node.scans > 0 and (not math.isfinite(observed) or observed >= args.min_clearance)
    result = {
        'started_utc': datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'),
        'duration_wall_s': end - start,
        'scan_topic': args.scan_topic,
        'half_angle_deg': args.half_angle_deg,
        'min_clearance_required_m': args.min_clearance,
        'scans': node.scans,
        'forward_finite_samples': node.samples,
        'min_forward_range_m': None if not math.isfinite(observed) else observed,
        'first_stamp_s': node.first_stamp,
        'last_stamp_s': node.last_stamp,
        'backwards_stamps': node.backwards,
        'duplicate_stamps': node.duplicates,
        'passed': passed,
    }
    print(f'EMPTY_ROAD_{"PASS" if passed else "FAIL"}: '
          f'scans={node.scans}, forward_samples={node.samples}, '
          f'min_forward={result["min_forward_range_m"]}, '
          f'required>={args.min_clearance:.2f} m')
    if node.scans == 0:
        print('No scans received. Check the scene, /scan bridge and ROS_DOMAIN_ID.')
    elif not passed:
        print('A finite forward return is inside the corridor; this is not the empty scene '
              'or an obstacle is still present.')
    out = Path.home() / 'empty-road-checks' / result['started_utc']
    out.mkdir(parents=True, exist_ok=False)
    (out / 'check.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(f'Record saved: {out}')
    raise SystemExit(0 if passed else 2)


if __name__ == '__main__':
    main()
