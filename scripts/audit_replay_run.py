#!/usr/bin/env python3
"""Audit a paired controller/relay run without ROS or changing its source logs."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import mean


STOP_STATES = {'OBSTACLE_STOP', 'GOAL_REACHED', 'SENSOR_TIMEOUT', 'FAULT'}


def read_csv(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def distribution(values):
    return {'count': len(values), 'min': min(values), 'mean': mean(values),
            'max': max(values)} if values else None


def sample(row, index, stop_wall):
    result = {'csv_line': index + 2, 'state': row['state']}
    for key in ('wall_elapsed_s', 'sim_time_s', 'distance_m', 'red_area_ratio',
                'cmd_v', 'cmd_w', 'odom_v', 'image_stamp_s', 'odom_stamp_s'):
        value = float(row[key]) if row.get(key) else None
        result[key] = value if value is None or math.isfinite(value) else None
    result['relative_to_stop_wall_s'] = result['wall_elapsed_s'] - stop_wall
    return result


def audit(baseline, relay):
    events = read_csv(baseline / 'events.csv')
    frames = read_csv(relay / 'frames.csv')
    config = json.loads((baseline / 'config.json').read_text(encoding='utf-8'))
    relay_config = json.loads((relay / 'config.json').read_text(encoding='utf-8'))
    first_run = next((i for i, row in enumerate(events) if row['state'] == 'RUNNING'), None)
    first_stop = next((i for i, row in enumerate(events) if row['state'] in STOP_STATES), None)
    if first_run is None or first_stop is None or first_stop < first_run:
        raise ValueError('A RUNNING interval followed by a stop is required for this audit')
    by_stamp = {}
    for frame in frames:
        stamp = int(frame['input_stamp_ns'])
        if stamp in by_stamp:
            raise ValueError('Duplicate relay input stamps: split clock-reset sessions before joining')
        by_stamp[stamp] = frame
    active = events[first_run:first_stop + 1]
    image_stamps = sorted({round(float(row['image_stamp_s']) * 1e9)
                           for row in active if row['image_stamp_s']})
    matched = [by_stamp[stamp] for stamp in image_stamps if stamp in by_stamp]
    delays = [(int(row['input_stamp_ns']) - int(row['selected_source_stamp_ns'])) / 1e9
              for row in matched]
    stop = events[first_stop]
    stop_wall = float(stop['wall_elapsed_s'])
    after = events[first_stop:]
    low_index = next((i for i in range(first_stop, len(events))
                      if abs(float(events[i]['odom_v'])) < 0.02), None)
    stop_frame = by_stamp.get(round(float(stop['image_stamp_s']) * 1e9))
    transitions = [sample(row, i, stop_wall) for i, row in enumerate(events)
                   if i == 0 or row['state'] != events[i - 1]['state']]
    # This is a diagnostic sample filter, not a collision detector. Exclude startup.
    low_before = [sample(row, i, stop_wall)
                  for i, row in enumerate(events[:first_stop])
                  if row['state'] == 'RUNNING' and float(row['distance_m']) > 1.0
                  and float(row['cmd_v']) > 0.1 and abs(float(row['odom_v'])) < 0.02]
    sources = []
    for directory, names in ((baseline, ('config.json', 'events.csv')),
                             (relay, ('config.json', 'frames.csv'))):
        for name in names:
            data = (directory / name).read_bytes()
            sources.append({'path': f'runs/{directory.name}/{name}', 'bytes': len(data),
                            'sha256': hashlib.sha256(data).hexdigest()})
    return {
        'schema_version': 1,
        'baseline_run_id': baseline.name, 'relay_run_id': relay.name,
        'baseline_config': config, 'relay_config': relay_config,
        'source_files': sources,
        'method': {
            'window': 'First RUNNING through first stop, inclusive',
            'join': 'round(image_stamp_s * 1e9) equals relay input_stamp_ns; unique input stamps',
            'delay': '(input_stamp_ns - selected_source_stamp_ns) / 1e9; simulation seconds',
            'weighting': 'One sample per distinct image header observed in controller CSV ticks',
            'coverage_limit': 'Control ticks may omit intermediate callbacks; this is not every received image',
            'speed': 'cmd_v/cmd_w are controller publish-call values; odom_v is reported odometry',
            'contact': 'No contact detector or independent cmd_vel recording is present',
            'low_speed_filter': 'RUNNING, distance_m > 1, cmd_v > 0.1, abs(odom_v) < 0.02',
        },
        'row_counts': {'events': len(events), 'frames': len(frames)},
        'image_join': {'unique_stamps': len(image_stamps), 'matched': len(matched),
                       'unmatched': len(image_stamps) - len(matched)},
        'actual_replay_delay_sim_s': distribution(delays),
        'matched_relay_states': sorted({row['state'] for row in matched}),
        'matched_buffer_resets': sum(int(row['buffer_reset']) for row in matched),
        'delay_arithmetic_mismatch_rows': sum(
            abs(float(row['actual_delay_s']) -
                (int(row['input_stamp_ns']) - int(row['selected_source_stamp_ns'])) / 1e9) > 1e-9
            for row in frames),
        'first_stop': sample(stop, first_stop, stop_wall),
        'stop_frame': ({'input_stamp_ns': int(stop_frame['input_stamp_ns']),
                        'selected_source_stamp_ns': int(stop_frame['selected_source_stamp_ns']),
                        'actual_delay_s': float(stop_frame['actual_delay_s']),
                        'content_age_at_control_tick_sim_s': float(stop['sim_time_s']) -
                        int(stop_frame['selected_source_stamp_ns']) / 1e9}
                       if stop_frame else None),
        'state_transitions': transitions,
        'stop_window': [sample(events[i], i, stop_wall)
                        for i in range(max(first_run, first_stop - 3), min(len(events), first_stop + 5))],
        'after_stop': {
            'rows': len(after),
            'nonzero_command_rows': sum(float(row['cmd_v']) != 0 or float(row['cmd_w']) != 0
                                        for row in after),
            'first_odom_abs_below_0_02': sample(events[low_index], low_index, stop_wall)
            if low_index is not None else None,
            'odom_after_first_low': distribution([float(row['odom_v']) for row in events[low_index:]])
            if low_index is not None else None,
            'final_sample': sample(events[-1], len(events) - 1, stop_wall),
        },
        'prestop_nonzero_command_low_odom_samples': low_before,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--relay', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(audit(args.baseline, args.relay), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
