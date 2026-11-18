#!/usr/bin/env python3
"""Observe raw and relayed camera timing together, without publishing commands."""
import argparse
from bisect import bisect_left, bisect_right
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
from threading import Lock
import time

def stamp_gaps(events):
    """Observed positive header-time jumps; not camera generation/drop proof."""
    return [dict(from_stamp_ns=a[1], to_stamp_ns=b[1],
                 gap_sim_s=(b[1]-a[1])/1e9, gap_wall_s=b[0]-a[0])
            for a, b in zip(events, events[1:]) if b[1] > a[1]]


def compare_gaps(raw, relayed, threshold):
    raw_stamps = sorted({stamp for _, stamp in raw})
    raw_set = set(raw_stamps)
    result = []
    for gap in stamp_gaps(relayed):
        if gap['gap_sim_s'] <= threshold:
            continue
        low, high = gap['from_stamp_ns'], gap['to_stamp_ns']
        inside = bisect_left(raw_stamps, high)-bisect_right(raw_stamps, low)
        both = low in raw_set and high in raw_set
        observation = ('RAW_INTERMEDIATE_OBSERVED' if inside else
                       'SAME_GAP_OBSERVED' if both else 'INSUFFICIENT_MATCHING_SAMPLES')
        result.append(dict(gap, raw_intermediate_stamps=inside,
                           raw_both_boundaries_seen=both, observation=observation))
    return result


def machine_snapshot():
    result = dict(cpu_count=os.cpu_count())
    if hasattr(os, 'getloadavg'):
        result['load_average_1_5_15_min'] = os.getloadavg()
    try:
        fields = ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree')
        result['memory_kib'] = {k: int(v.split()[0]) for k, v in
            (line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())
            if k in fields}
    except (OSError, ValueError):
        result['memory_kib'] = None
    # Only named transport settings, never a dump of the process environment.
    result['diagnostic_process_environment'] = {k: os.environ.get(k) for k in (
        'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY', 'GZ_PARTITION')}
    return result


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.utilities import get_rmw_implementation_identifier, get_available_rmw_implementations
    from sensor_msgs.msg import Image

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--relay-reliability', choices=('reliable', 'best_effort'),
                        default='reliable', help='Match the downstream controller reliability')
    parser.add_argument('--raw-reliability', choices=('reliable', 'best_effort'),
                        default='best_effort', help='Match the camera relay input; reliable requires a reliable publisher')
    parser.add_argument('--stamp-gap-threshold', type=float, default=.5,
                        help='Diagnostic header-gap marker in simulation seconds; does not change controller settings')
    parser.add_argument('--gazebo', action='store_true', help='Also observe Gazebo native images before the ROS bridge')
    parser.add_argument('--gazebo-topic', default='/camera/image_raw')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 300:
        parser.error('--seconds must be between 1 and 300')
    if not math.isfinite(args.stamp_gap_threshold) or args.stamp_gap_threshold <= 0:
        parser.error('--stamp-gap-threshold must be finite and positive')
    if args.gazebo:
        try:
            from gz.transport13 import Node as GazeboNode
            from gz.msgs10.image_pb2 import Image as GazeboImage
        except ImportError as exc:
            parser.exit(2, f'三路检查未开始：缺少 Gazebo Harmonic Python 绑定：{exc}\n'
                           '需要 gz.transport13 和 gz.msgs10；请保留这条错误，不能用两路结果代替原生检查。\n')
    topics = ('/camera/image_raw', '/attack/image_raw')
    samples = {topic: [] for topic in topics}
    shapes = {}
    machine_start = machine_snapshot()
    rclpy.init()
    machine_start['diagnostic_rmw_implementation'] = get_rmw_implementation_identifier()
    machine_start['available_rmw_implementations'] = sorted(get_available_rmw_implementations())
    node = Node('camera_stream_diagnostic')
    subscriptions = []
    for topic in topics:
        def receive(msg, topic=topic):
            samples[topic].append((time.monotonic(),
                                  msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec))
            shapes.setdefault(topic, dict(width=msg.width, height=msg.height,
                                          step=msg.step, encoding=msg.encoding))
        reliability = args.relay_reliability if topic == '/attack/image_raw' else args.raw_reliability
        qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
               if reliability == 'reliable' else qos_profile_sensor_data)
        subscriptions.append(node.create_subscription(Image, topic, receive, qos))
    native_node = None
    native_key = 'gazebo:' + args.gazebo_topic
    native_events, native_shape = [], {}
    native_lock, native_active = Lock(), [True]
    if args.gazebo:
        native_node = GazeboNode()

        def receive_native(msg):
            wall = time.monotonic()
            stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nsec
            with native_lock:
                if native_active[0]:
                    native_events.append((wall, stamp))
                    if not native_shape:
                        native_shape.update(width=msg.width, height=msg.height,
                                            step=msg.step, pixel_format_type=msg.pixel_format_type)

        if not native_node.subscribe(GazeboImage, args.gazebo_topic, receive_native):
            node.destroy_node()
            rclpy.shutdown()
            parser.exit(2, 'Gazebo 原生图像订阅创建失败，三路检查未开始。\n')
    start = time.monotonic()
    started_utc = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    label = 'Gazebo 原生及 ROS 两路图像' if args.gazebo else '两路图像'
    print(f'正在同时检查{label}，持续 {args.seconds:g} 秒；不会发布速度指令。', flush=True)
    print(f'中继输出订阅模式：{args.relay_reliability}。旧版中继请加 --relay-reliability best_effort。', flush=True)
    print(f'原始相机订阅模式：{args.raw_reliability}。需要与本次中继输入设置对应。', flush=True)
    if args.gazebo:
        print('虚拟机资源：' + json.dumps(machine_start, ensure_ascii=False), flush=True)
    next_notice = start + 10
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() >= next_notice:
                notice = [f'{t} = {len(samples[t])} 帧' for t in topics]
                if args.gazebo:
                    with native_lock:
                        notice.insert(0, f'Gazebo 原生 = {len(native_events)} 帧')
                print('已接收：' + '，'.join(notice), flush=True)
                next_notice += 10
    except KeyboardInterrupt:
        pass
    finally:
        end = time.monotonic()
        publishers = {t: node.count_publishers(t) for t in topics} if rclpy.ok() else {}
        publisher_qos = {}
        if rclpy.ok():
            for topic in topics:
                publisher_qos[topic] = [dict(node_name=info.node_name,
                    reliability=getattr(info.qos_profile.reliability, 'name', str(info.qos_profile.reliability)),
                    depth=info.qos_profile.depth,
                    history=getattr(info.qos_profile.history, 'name', str(info.qos_profile.history)))
                    for info in node.get_publishers_info_by_topic(topic)]
        if native_node is not None:
            with native_lock:
                native_active[0] = False
                samples[native_key] = list(native_events)
                shapes[native_key] = dict(native_shape)
            native_node.unsubscribe(args.gazebo_topic)
            try:
                publishers[native_key] = len(native_node.topic_info(args.gazebo_topic)[0])
            except Exception:
                publishers[native_key] = None  # Discovery unavailable is not zero publishers.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    samples = {key: [(wall, stamp) for wall, stamp in events if start <= wall <= end]
               for key, events in samples.items()}

    result = {'started_utc': started_utc, 'duration_wall_s': end - start,
              'relay_reliability': args.relay_reliability,
              'raw_reliability': args.raw_reliability,
              'stamp_gap_threshold_sim_s': args.stamp_gap_threshold,
              'gazebo_native_enabled': args.gazebo, 'image_metadata': shapes,
              'machine_start': machine_start, 'machine_end': machine_snapshot(),
              'ros_publisher_qos_at_end': publisher_qos,
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
        source_gaps = stamp_gaps(events)
        summary = dict(
            publishers_at_end=publishers.get(topic), received=len(events),
            max_receive_gap_s=max(gaps, default=None),
            max_fresh_stamp_gap_s=max(fresh_gaps, default=None),
            last_receive_age_s=end - arrivals[-1] if arrivals else None,
            last_fresh_stamp_age_s=end - fresh[-1] if fresh else None,
            first_receive_after_start_s=arrivals[0] - start if arrivals else None,
            duplicate_stamps=duplicates, backwards_stamps=backwards,
            max_header_stamp_gap_sim_s=max((g['gap_sim_s'] for g in source_gaps), default=None),
            header_gaps_over_threshold=sum(g['gap_sim_s'] > args.stamp_gap_threshold for g in source_gaps),
        )
        result['topics'][topic] = summary
        print(f'\n{topic}：发布者 {summary["publishers_at_end"]}，收到 {len(events)} 帧')
        if not events:
            print('  整段检查没有收到图像。')
            continue
        def fmt(value):
            return '样本不足' if value is None else f'{value:.3f} 秒'
        print(f'  最大接收间隔（墙钟）：{fmt(summary["max_receive_gap_s"])}')
        print(f'  新时间戳到达的最大间隔（墙钟）：{fmt(summary["max_fresh_stamp_gap_s"])}')
        print(f'  相邻图像头时间戳最大差值（仿真）：{fmt(summary["max_header_stamp_gap_sim_s"])}')
        print(f'  图像头时间戳间隔 > {args.stamp_gap_threshold:g} 仿真秒的次数：{summary["header_gaps_over_threshold"]}')
        print(f'  检查结束时距最后更新时间（墙钟）：{fmt(summary["last_fresh_stamp_age_s"])}')
        print(f'  重复时间戳：{duplicates}，倒退时间戳：{backwards}')
        if topic in shapes:
            print(f'  图像格式：{shapes[topic]}')

    comparisons = compare_gaps(samples[topics[0]], samples[topics[1]], args.stamp_gap_threshold)
    result['relay_gap_comparisons'] = comparisons
    result['comparison_usable_without_reset'] = all(
        result['topics'][t]['backwards_stamps'] == 0 for t in topics)
    print('\n中继长间隔与原始图像对照（最多列出最大的 5 次）：')
    if not result['comparison_usable_without_reset']:
        print('检测到时间戳倒退，跨重置的对照不可用；重新开始一次检查。')
    elif comparisons:
        labels = {'RAW_INTERMEDIATE_OBSERVED': '原始订阅者在空档内收到过其他帧',
                  'SAME_GAP_OBSERVED': '两路观察到相同的时间戳空档',
                  'INSUFFICIENT_MATCHING_SAMPLES': '缺少匹配边界，不能比较'}
        for row in sorted(comparisons, key=lambda r: r['gap_sim_s'], reverse=True)[:5]:
            print(f'  {row["from_stamp_ns"]/1e9:.3f} → {row["to_stamp_ns"]/1e9:.3f} s; '
                  f'仿真间隔={row["gap_sim_s"]:.3f}s; 墙钟间隔={row["gap_wall_s"]:.3f}s; '
                  f'原始中间帧数={row["raw_intermediate_stamps"]}; {labels[row["observation"]]}')
    else:
        print('未记录到可列出的中继长间隔；若无图像或样本不足，也不能判定正常。')

    if args.gazebo:
        print('\nROS 发布者 QoS：' + json.dumps(publisher_qos, ensure_ascii=False))
        result['gazebo_gap_comparisons'] = {}
        native_valid = (len(samples[native_key]) >= 2
                        and result['topics'][native_key]['backwards_stamps'] == 0
                        and any(stamp > 0 for _, stamp in samples[native_key]))
        result['gazebo_comparison_usable'] = native_valid
        print('\nGazebo 原生图像与 ROS 空档对照：')
        if not native_valid:
            print('原生样本不足或时间戳无效，无法定位；检查原生话题、GZ_PARTITION 和 Python 绑定。')
        else:
            for target in topics:
                if result['topics'][target]['backwards_stamps']:
                    print(f'{target} 时间倒退，本次对照无效。')
                    continue
                compared = compare_gaps(samples[native_key], samples[target], args.stamp_gap_threshold)
                result['gazebo_gap_comparisons'][target] = compared
                print(f'{target} 的长间隔（最大 5 次）：')
                if not compared:
                    print('  没有可比较的长间隔；缺失全部数据或样本不足不能视为正常。')
                for row in sorted(compared, key=lambda r: r['gap_sim_s'], reverse=True)[:5]:
                    finding = {'RAW_INTERMEDIATE_OBSERVED': '原生有中间帧：优先查桥接/ROS传输/接收调度',
                               'SAME_GAP_OBSERVED': '原生也观察到相同空档：优先查仿真/原生传输/共同调度',
                               'INSUFFICIENT_MATCHING_SAMPLES': '边界不匹配，暂不能定位'}[row['observation']]
                    print(f'  {row["from_stamp_ns"]/1e9:.3f} → {row["to_stamp_ns"]/1e9:.3f} s; '
                          f'间隔={row["gap_sim_s"]:.3f}s; 原生中间帧数={row["raw_intermediate_stamps"]}; {finding}')
        print('原生订阅也可能受传输/调度影响，仍不能把未收到直接等同于未生成；此次额外订阅会增加负载。')

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
    print('两路出现相同空档：优先查共同的图像生成/传输/系统调度；仍不证明相机未生成这些帧。')
    print('原始订阅者在中继空档内收到其他帧：继续查中继输入接收/调度/输出链路；独立订阅者接收可不同。')
    print('仿真头时间戳差值与墙钟接收间隔是两种量；此脚本不替代控制器的新鲜度/配对检查。')
    print('没有复现空档不能证明控制器运行时正常；检查本身也会增加图像订阅负载。')


if __name__ == '__main__':
    main()
