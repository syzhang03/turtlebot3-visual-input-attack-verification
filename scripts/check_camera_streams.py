#!/usr/bin/env python3
"""Observe raw and relayed camera timing together, without publishing commands."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--relay-reliability', choices=('reliable', 'best_effort'),
                        default='reliable', help='Match the downstream controller reliability')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 300:
        parser.error('--seconds must be between 1 and 300')
    topics = ('/camera/image_raw', '/attack/image_raw')
    samples = {topic: [] for topic in topics}
    rclpy.init()
    node = Node('camera_stream_diagnostic')
    subscriptions = []
    for topic in topics:
        def receive(msg, topic=topic):
            samples[topic].append((time.monotonic(),
                                  msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec))
        qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
               if topic == '/attack/image_raw' and args.relay_reliability == 'reliable'
               else qos_profile_sensor_data)
        subscriptions.append(node.create_subscription(Image, topic, receive, qos))
    start = time.monotonic()
    started_utc = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    print(f'正在同时检查两路图像，持续 {args.seconds:g} 秒；不会发布速度指令。', flush=True)
    print(f'中继输出订阅模式：{args.relay_reliability}。旧版中继请加 --relay-reliability best_effort。', flush=True)
    next_notice = start + 10
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() >= next_notice:
                print('已接收：' + '，'.join(f'{t} = {len(samples[t])} 帧' for t in topics), flush=True)
                next_notice += 10
    except KeyboardInterrupt:
        pass
    finally:
        end = time.monotonic()
        publishers = {t: node.count_publishers(t) for t in topics} if rclpy.ok() else {}
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    result = {'started_utc': started_utc, 'duration_wall_s': end - start,
              'relay_reliability': args.relay_reliability,
              'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'), 'topics': {}}
    for topic, events in samples.items():
        arrivals = [t for t, stamp in events]
        fresh = []
        previous = None
        duplicates = backwards = 0
        for wall, stamp in events:
            if previous is None or stamp > previous:
                fresh.append(wall)
            elif stamp == previous:
                duplicates += 1
            else:
                backwards += 1
                fresh.append(wall)
            previous = stamp
        gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
        fresh_gaps = [b - a for a, b in zip(fresh, fresh[1:])]
        summary = dict(
            publishers_at_end=publishers.get(topic), received=len(events),
            max_receive_gap_s=max(gaps, default=None),
            max_fresh_stamp_gap_s=max(fresh_gaps, default=None),
            last_receive_age_s=end - arrivals[-1] if arrivals else None,
            last_fresh_stamp_age_s=end - fresh[-1] if fresh else None,
            first_receive_after_start_s=arrivals[0] - start if arrivals else None,
            duplicate_stamps=duplicates, backwards_stamps=backwards,
        )
        result['topics'][topic] = summary
        print(f'\n{topic}：发布者 {summary["publishers_at_end"]}，收到 {len(events)} 帧')
        if not events:
            print('  整段检查没有收到图像。')
            continue
        def fmt(value):
            return '样本不足' if value is None else f'{value:.3f} 秒'
        print(f'  最大接收间隔：{fmt(summary["max_receive_gap_s"])}')
        print(f'  时间戳更新的最大间隔：{fmt(summary["max_fresh_stamp_gap_s"])}')
        print(f'  检查结束时距最后更新时间：{fmt(summary["last_fresh_stamp_age_s"])}')
        print(f'  重复时间戳：{duplicates}，倒退时间戳：{backwards}')

    out = Path.home() / 'camera-checks' / started_utc
    out.mkdir(parents=True, exist_ok=False)
    (out / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    with (out / 'arrivals.csv').open('w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['topic', 'wall_elapsed_s', 'source_stamp_ns', 'receive_monotonic_ns'])
        writer.writerows((topic, wall - start, stamp, round(wall * 1e9))
                         for topic, events in samples.items() for wall, stamp in events)
    print(f'\n记录已保存：{out}')
    print('这是独立订阅者的接收观察；帧数差异不能直接认定为中继丢帧。')
    print('两路都超过 2 秒：检查原相机/仿真/传输；仅中继输出超时：检查中继链路。')
    print('本次都正常：只能说明本次观察未复现，仍需检查控制器运行时的情况。')


if __name__ == '__main__':
    main()
