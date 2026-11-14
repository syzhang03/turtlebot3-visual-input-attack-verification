#!/usr/bin/env python3
"""Fit the visual 0.4 crossing from a clean raw-sensor calibration approach."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil


def rows(path):
    with path.open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def fit_crossing(pairs, threshold, speed, tolerance, max_gap=.3):
    """Interpolate one bracket; keep sampling/pairing uncertainty explicit."""
    observations = []
    for row in pairs:
        r, d, ti, ts = (float(row[k]) for k in
                        ('red_ratio', 'clearance', 'image_stamp', 'scan_stamp'))
        if not all(math.isfinite(v) for v in (r, ti, ts)) or not 0 <= r <= 1:
            raise ValueError('Invalid ratio or timestamp in pairs.csv')
        if observations and (ti <= observations[-1][2] or ts <= observations[-1][3]):
            raise ValueError('Repeated/backward paired timestamp; repeat calibration')
        observations.append((r, d, ti, ts))
    index = next((i for i, item in enumerate(observations) if item[0] >= threshold), None)
    if index is None or index == 0:
        raise ValueError('Need a clean approach with paired observations below and above 0.4')
    below, above = observations[index-1:index+1]
    r0, d0, t0, s0 = below
    r1, d1, t1, s1 = above
    if not all(math.isfinite(d) and d > 0 for d in (d0, d1)) or d1 > d0:
        raise ValueError('Crossing must have finite, positive, decreasing corridor clearance')
    if t1-t0 > max_gap or s1-s0 > max_gap:
        raise ValueError('Crossing has a >0.3s sampling gap; repeat the clean calibration')
    offset = max(abs(t0-s0), abs(t1-s1))
    if offset > tolerance + 1e-9:
        raise ValueError('Pair timestamps exceed the configured tolerance')
    estimate = d0 + (threshold-r0) / (r1-r0) * (d1-d0)
    # This is an operational tolerance band, not a statistical confidence interval.
    # Include the measured transition bracket and travel during timestamp mismatch,
    # then round outward to centimetres. Other scene/noise errors require validation.
    allowance = speed * offset
    near = math.floor((d1-allowance)*100) / 100
    far = math.ceil((d0+allowance)*100) / 100
    critical = round(estimate, 3)
    if not 0 < near < critical < far:
        raise ValueError('Cannot form a positive tolerance band around the crossing')
    return dict(critical_distance_m=critical, verify_near=near, verify_far=far,
                visual_ratio=threshold, below=dict(zip(('ratio', 'clearance_m', 'image_stamp', 'scan_stamp'), below)),
                above=dict(zip(('ratio', 'clearance_m', 'image_stamp', 'scan_stamp'), above)),
                interpolation_estimate_m=estimate, maximum_pair_offset_s=offset,
                pairing_travel_allowance_m=allowance, speed_bound_m_s=speed)


def calibrate(root):
    candidates = list(root.glob('controller/*/config.json'))
    if len(candidates) != 1:
        raise ValueError('Expected exactly one calibration controller in this run')
    folder = candidates[0].parent
    cfg = json.loads(candidates[0].read_text())
    visual = json.loads((folder/'visual-config.json').read_text())
    expected = dict(enabled=True, scan_topic='/scan', speed=.3, enable_goal_stop=False,
                    stop_distance_mode='dynamic')
    if any(cfg.get(k) != v for k, v in expected.items()):
        raise ValueError('Use run_stop_calibration.sh with raw sensors and speed=0.3')
    if (visual.get('image_topic') != '/camera/image_raw'
            or visual.get('calibration_mode') is not True
            or visual.get('visual_stop_threshold') != .4
            or visual.get('visual_verify_threshold') != .4
            or visual.get('pair_tolerance') != .06):
        raise ValueError('Unexpected calibration visual settings')
    events = rows(folder/'events.csv')
    start = next((i for i, e in enumerate(events) if float(e['cmd_v']) > 0), None)
    stop = next((i for i in range(start+1, len(events)) if float(events[i]['cmd_v']) == 0), None) if start is not None else None
    if stop is None or events[stop]['state'] != 'VISUAL_STOP':
        raise ValueError('Calibration must move and first stop for VISUAL_STOP, not timeout/LiDAR/anomaly')
    if not any(float(e['odom_v']) == 0 and float(e['cmd_v']) == 0 for e in events[stop:]):
        raise ValueError('No fully stopped odometry record; wait after VISUAL_STOP before Ctrl+C')
    speeds = [abs(float(e['odom_v'])) for e in events[start:stop+1]]
    if not all(math.isfinite(v) for v in speeds):
        raise ValueError('Invalid odometry speed')
    fit = fit_crossing(rows(folder/'pairs.csv'), .4, max(cfg['speed'], *speeds), .06)
    if abs(fit['above']['image_stamp']-float(events[stop]['sim_time_s'])) > .3:
        raise ValueError('Paired crossing is too far from the first stop; repeat calibration')
    params = dict(stop_distance_mode='fixed', fixed_stop_distance=fit['critical_distance_m'],
                  visual_stop_threshold=.4, visual_verify_threshold=.4,
                  verify_near=fit['verify_near'], verify_far=fit['verify_far'],
                  speed=1.0, enable_goal_stop=False, calibration_mode=False,
                  front_extent=cfg['front_extent'], corridor_half_width=cfg['corridor_half_width'],
                  base_frame=cfg['base_frame'], roi_top=visual['roi_top'], pair_tolerance=.06)
    # At formal 1m/s, a 0.06s pair offset can exceed the slow-run travel allowance.
    # Extend the consistency band for that configured bound, without moving D.
    formal_allowance = params['speed'] * params['pair_tolerance']
    distances = (fit['below']['clearance_m'], fit['above']['clearance_m'])
    params['verify_near'] = math.floor((min(distances)-formal_allowance)*100) / 100
    params['verify_far'] = math.ceil((max(distances)+formal_allowance)*100) / 100
    if not 0 < params['verify_near'] < params['fixed_stop_distance'] < params['verify_far']:
        raise ValueError('Invalid band after accounting for formal 1m/s pairing tolerance')
    report = dict(source=str(folder.resolve()), status='SINGLE_APPROACH_REQUIRES_CLEAN_VALIDATION',
                  fit=fit, formal_pairing_travel_allowance_m=formal_allowance, parameters=params,
                  limits=['Scene-specific interpolated trigger boundary, not final stopping clearance.',
                          'Distance is the TF-transformed forward-corridor clearance with front_extent subtracted.',
                          'Band covers sampled bracket and nominal pairing travel; not all sensor error or delivery delay.',
                          'Validate the new clean condition at 1m/s before camera-only and LiDAR-only trials.',
                          'Collision and final physical clearance still require independent simulation evidence.'])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--latest', action='store_true')
    source.add_argument('--run-dir', type=Path)
    parser.add_argument('--profile', type=Path, default=Path(__file__).resolve().parents[1]/'configs/aligned_stop.yaml')
    parser.add_argument('--check-only', action='store_true', help='Print report without writing files')
    args = parser.parse_args()
    try:
        root = (Path(Path(os.environ.get('STOP_CALIBRATION_RESULTS', '~/stop-point-calibration')).expanduser()
                     .joinpath('latest-run.txt').read_text().strip()) if args.latest else args.run_dir).expanduser()
        report = calibrate(root)
        if not args.check_only:
            profile = args.profile.expanduser()
            profile.parent.mkdir(parents=True, exist_ok=True)
            if profile.exists():
                backup = profile.with_name(profile.name + '.bak-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
                shutil.copy2(profile, backup)
            contents = '# Generated from ' + str(root.resolve()) + '\n'
            contents += 'camera_lidar_monitor:\n  ros__parameters:\n'
            contents += ''.join(f'    {k}: {json.dumps(v)}\n' for k, v in report['parameters'].items())
            pending = profile.with_name(profile.name + f'.tmp-{os.getpid()}')
            pending.write_text(contents, encoding='utf-8')
            (root/'calibration-report.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
            pending.replace(profile)
            report['written_profile'] = str(profile.resolve())
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f'Calibration rejected: {exc}\nNo new profile was generated.\n')


if __name__ == '__main__':
    main()
