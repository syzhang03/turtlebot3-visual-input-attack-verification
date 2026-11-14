#!/usr/bin/env python3
"""Read existing trial logs; do not connect to ROS or change experiment settings."""
import argparse
import csv
import json
from pathlib import Path


def rows(path):
    if not path.is_file():
        print(f'缺少日志：{path}')
        return []
    with path.open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def config(path):
    return json.loads(path.read_text(encoding='utf-8')) if path.is_file() else {}


def number(row, field):
    value = row.get(field)
    return float(value) if value not in (None, '') else None


def table(records, fields):
    print(' | '.join(fields))
    for record in records:
        print(' | '.join(str(record.get(field, '')) for field in fields))


def near(records, time_of, target, before=5, after=2):
    # Selection is by logged timestamps, not an assertion of callback order.
    valid = [(time_of(row), row) for row in records]
    valid = [(stamp, row) for stamp, row in valid if stamp is not None]
    earlier = [row for stamp, row in valid if stamp <= target]
    later = [row for stamp, row in valid if stamp > target]
    return earlier[-before:] + later[:after]


def relay_timing(records):
    """Pair a callback gap with the PRECEDING callback's work, not the next one."""
    previous = None
    for original in records:
        row = dict(original)
        stamp = number(row, 'input_stamp_ns')
        old_stamp = number(previous, 'input_stamp_ns') if previous is not None else None
        row['input_gap_sim_s'] = (round((stamp-old_stamp)/1e9, 6)
                                  if stamp is not None and old_stamp is not None else '')
        for field in ('prepare_s', 'publish_call_s'):
            row['preceding_'+field] = previous.get(field, '') if previous is not None else ''
        # previous_log_write_s already describes the preceding callback's CSV write.
        # This is only measured work; it excludes scheduling/receive wait and any
        # uninstrumented tail work. Do not label the remaining time "camera delay".
        measured = [number(row, name) for name in (
            'preceding_prepare_s', 'preceding_publish_call_s', 'previous_log_write_s')]
        row['preceding_measured_work_wall_s'] = (round(sum(measured), 6)
                                                if all(x is not None for x in measured) else '')
        yield row
        previous = row


def diagnose(directory):
    base = config(directory / 'config.json')
    visual = config(directory / 'visual-config.json')
    events = rows(directory / 'events.csv')
    index = next((i for i, row in enumerate(events) if row['state'] == 'SENSOR_TIMEOUT'), None)
    if index is None:
        print('未找到 SENSOR_TIMEOUT 记录。')
        return
    stop = float(events[index]['sim_time_s'])
    print(f'控制器日志：{directory}\n首次超时：仿真时间 {stop:.6f} s')
    for key in ('speed', 'scan_topic', 'sensor_timeout_wall', 'max_stamp_age_sim', 'enable_goal_stop'):
        print(f'{key}={base.get(key, "未记录")}')
    for key in ('image_topic', 'pair_tolerance', 'pair_timeout', 'confirmations', 'visual_stop_threshold'):
        print(f'{key}={visual.get(key, "未记录")}')
    print('\n停车前后的控制器记录（最后两列为计算得到的仿真时间戳年龄）：')
    excerpt = []
    for original in events[max(0, index-6):index+3]:
        row = dict(original)
        for sensor in ('scan', 'odom'):
            stamp = number(row, sensor+'_stamp_s')
            row[sensor+'_age_sim_s'] = (round(float(row['sim_time_s'])-stamp, 6)
                                        if stamp is not None else '')
        excerpt.append(row)
    table(excerpt, ['wall_elapsed_s', 'sim_time_s', 'state', 'cmd_v', 'odom_v',
                    'scan_age_wall_s', 'odom_age_wall_s', 'scan_age_sim_s', 'odom_age_sim_s'])
    images = rows(directory / 'image-acceptance.csv')
    print('\n相机实际接收记录，按接收仿真时间选取停车附近行：')
    selected = near(images, lambda row: number(row, 'accepted_sim_s'), stop)
    for row in selected:
        row['image_age_at_accept_sim_s'] = round(float(row['accepted_sim_s'])
                                                - int(row['header_stamp_ns'])/1e9, 6)
    table(selected, ['accepted_sim_s', 'header_stamp_ns', 'image_age_at_accept_sim_s'])
    print('\n邻近配对（按消息时间筛选；此表不重建配对接收时刻）：')
    pairs = rows(directory / 'pairs.csv')
    selected = near(pairs, lambda row: max(float(row['image_stamp']), float(row['scan_stamp'])), stop)
    table(selected, ['image_stamp', 'scan_stamp', 'direction', 'count', 'alert', 'red_ratio'])
    root = directory.parent.parent
    manifest = config(root / 'manifest.json')
    print('\n攻击模式与实际启用参数：', manifest.get('mode'), manifest.get('effective'))
    for sensor, name, fields in (
        ('camera', 'frames.csv', ['input_stamp_ns', 'actual_delay_s', 'state', 'input_gap_sim_s',
                                'input_gap_wall_s', 'output_subscribers']),
        ('lidar', 'scans.csv', ['input_stamp_ns', 'state', 'changed_rays', 'input_gap_wall_s', 'output_subscribers']),
    ):
        for path in sorted((root / sensor).glob('*/'+name)):
            print(f'\n{sensor} 中继在停车消息时间附近的输出：')
            records = rows(path)
            if sensor == 'camera':
                records = list(relay_timing(records))
            selected = near(records, lambda row: int(row['input_stamp_ns'])/1e9, stop)
            table(selected, fields)
            if sensor == 'camera':
                print('\n同一批相机输入间隔中的上一帧处理耗时（墙钟秒）：')
                table(selected, ['input_stamp_ns', 'input_gap_wall_s', 'preceding_prepare_s',
                                 'preceding_publish_call_s', 'previous_log_write_s',
                                 'preceding_measured_work_wall_s'])
                print('input_gap_wall_s 是相邻回调开始的间隔，包含处理/排队/等待，不能直接当作相机生成间隔。')
                print('上一帧耗时与间隔一起看：数值很小只能排除这些已计时的处理段，不能区分仿真、传输或调度。')
    print('\n说明：现有日志没有记录控制器逐次检查时的 image_age_wall 和 pair_age_wall。')
    print('同一仿真时刻的事件先后、最后一次配对到达时间不能据此精确还原；此报告不自动认定根因。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('controller_dir', type=Path)
    args = parser.parse_args()
    try:
        diagnose(args.controller_dir.expanduser().resolve())
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f'无法读取这轮日志：{exc}\n')


if __name__ == '__main__':
    main()
