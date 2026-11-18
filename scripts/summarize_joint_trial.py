#!/usr/bin/env python3
"""Read-only audit of one joint trial. Does not infer collision from sensor data."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import statistics


def read_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def only_file(root, pattern):
    matches = list(root.glob(pattern))
    if len(matches) != 1:
        raise ValueError(f'Expected exactly one {pattern} in {root}, found {len(matches)}')
    return matches[0]


def stats(values):
    return dict(count=len(values), minimum=min(values), mean=statistics.mean(values), maximum=max(values)) if values else None


def summarize(root):
    manifest = json.loads((root / 'manifest.json').read_text(encoding='utf-8'))
    candidates = [(p, json.loads(p.read_text(encoding='utf-8'))) for p in root.glob('controller/*/config.json')]
    drives = [(p, cfg) for p, cfg in candidates if cfg['enabled']]
    if len(drives) != 1:
        raise ValueError(f'Expected one enabled controller run; found {len(drives)}. Reset the scene and relays for each trial.')
    path, cfg = drives[0]
    visual = json.loads((path.parent / 'visual-config.json').read_text(encoding='utf-8'))
    events = read_rows(path.parent / 'events.csv')
    if not events:
        raise ValueError('Controller events.csv has no records')
    forward_index = next((i for i, r in enumerate(events) if float(r['cmd_v']) > 0.), None)
    stop_index = (next((i for i in range(forward_index+1, len(events)) if float(events[i]['cmd_v']) == 0.), None)
                  if forward_index is not None else None)
    start = float(events[forward_index]['sim_time_s']) if forward_index is not None else None
    end = float(events[stop_index]['sim_time_s']) if stop_index is not None else float(events[-1]['sim_time_s'])
    camera = read_rows(only_file(root, 'camera/*/frames.csv'))
    camera_cfg = json.loads(only_file(root, 'camera/*/config.json').read_text(encoding='utf-8'))
    lidar = read_rows(only_file(root, 'lidar/*/scans.csv'))
    accepted = read_rows(path.parent / 'image-acceptance.csv')
    stamps = {int(r['header_stamp_ns']) for r in accepted
              if start is not None and start <= float(r['accepted_sim_s']) <= end}
    consumed_camera = [r for r in camera if int(r['input_stamp_ns']) in stamps]
    # These scans were generated during the motion interval, not a proof of DDS delivery.
    motion_lidar = [r for r in lidar if start is not None and start <= int(r['input_stamp_ns'])/1e9 <= end]
    issues = []
    expected_transport = manifest.get('camera_transport')
    if expected_transport and any(camera_cfg.get(k) != expected_transport[k]
                                  for k in ('input_reliability', 'output_reliability')):
        issues.append('Camera relay transport does not match this run manifest; check the built overlay')
    expected = dict(speed=1., enable_goal_stop=False, scan_topic='/attack/scan', sensor_timeout_wall=2.)
    if any(cfg.get(k) != v for k, v in expected.items()):
        issues.append('Controller differs from the prescribed joint experiment configuration')
    if visual.get('visual_stop_threshold') != .4 or visual.get('calibration_mode') is not False:
        issues.append('Visual threshold or calibration mode differs from the prescribed configuration')
    stop_mode = cfg.get('stop_distance_mode', 'dynamic')
    if stop_mode == 'fixed':
        critical = cfg.get('fixed_stop_distance', float('nan'))
        if (not math.isfinite(critical) or critical <= 0
                or visual.get('visual_verify_threshold') != .4
                or not visual.get('verify_near', math.inf) < critical < visual.get('verify_far', -math.inf)):
            issues.append('Fixed-stop profile does not align visual 0.4 with the consistency distance band')
        if any(abs(float(e['stop_threshold_m'])-critical) > 1e-6 for e in events):
            issues.append('Logged LiDAR threshold differs from the fixed stop distance')
    elif stop_mode != 'dynamic':
        issues.append('Unknown LiDAR stopping mode')
    if forward_index is None:
        issues.append('No positive forward command was logged')
    if stop_index is None:
        issues.append('No zero command after forward motion was logged')
    if events[-1]['state'] in ('SENSOR_TIMEOUT', 'SENSOR_INVALID', 'FAULT', 'COMMAND_CONFLICT', 'TRIAL_TIMEOUT'):
        issues.append('Trial ended with an infrastructure, sensor or time-limit stop; investigate separately')
    if not consumed_camera:
        issues.append('No relay frames matched images accepted during motion')
    desired_camera = 'REPLAY' if manifest['effective']['camera_delay_s'] else 'ZERO_DELAY'
    if any(r['state'] != desired_camera for r in consumed_camera):
        issues.append('Accepted camera frames include warmup, out-of-window or buffer fallback states')
    if not motion_lidar:
        issues.append('No LiDAR relay records during motion')
    desired_lidar = 'BIAS_ACTIVE' if manifest['effective']['lidar_bias_m'] else 'PASSTHROUGH'
    if any(r['state'] != desired_lidar for r in motion_lidar):
        issues.append('LiDAR records include invalid metadata or out-of-window states')
    if manifest['effective']['lidar_bias_m'] and not any(int(r['changed_rays']) for r in motion_lidar):
        issues.append('LiDAR attack was configured but no ranges changed during motion')
    if any(r['buffer_reset'] == '1' for r in camera) or any(r['reset'] == 'True' for r in lidar):
        issues.append('A relay reset occurred; inspect timestamps/layout and repeat from a fresh scene')
    command_fields = ('sim_time_s', 'state', 'distance_m', 'cmd_v', 'cmd_w', 'odom_v',
                      'corridor_clearance_m', 'stop_threshold_m')
    excerpt = ([{k: r[k] for k in command_fields}
                for r in events[max(forward_index, stop_index-2):stop_index+4]] if stop_index is not None else [])
    return dict(
        run_dir=str(root), mode=manifest['mode'], effective_attack=manifest['effective'],
        controller_profile='aligned_fixed_stop' if stop_mode == 'fixed' else 'legacy_dynamic_stop',
        camera_transport=dict(input_reliability=camera_cfg.get('input_reliability', 'not recorded'),
                              output_reliability=camera_cfg.get('output_reliability', 'not recorded'),
                              revision=camera_cfg.get('transport_revision', 'not recorded')),
        controller_config=cfg, visual_config=visual, issues=issues,
        motion_start_sim_s=start, first_zero_after_motion_sim_s=end if stop_index is not None else None,
        final_state=events[-1]['state'],
        accepted_camera_delay_s=stats([float(r['actual_delay_s']) for r in consumed_camera]),
        accepted_camera_states=dict(Counter(r['state'] for r in consumed_camera)),
        lidar_relay_states_during_motion=dict(Counter(r['state'] for r in motion_lidar)),
        lidar_changed_rays_total=sum(int(r['changed_rays']) for r in motion_lidar),
        lidar_changed_scan_max_delta_m=stats([float(r['max_delta_m']) for r in motion_lidar if int(r['changed_rays'])]),
        stop_command_excerpt=excerpt,
        contact='not determined by this audit; use independent simulation/contact evidence',
        clearance='not computed from attacked sensors; record independent ground truth',
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path, help='Timestamped joint-attack-results run directory')
    args = parser.parse_args()
    try:
        result = summarize(args.run_dir.expanduser().resolve())
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(2, f'Cannot summarize trial: {exc}\n')
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
