"""Audit protocol compatibility and report the three-controller comparison."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess

from issue13_metrics import read_csv
from report_issue13 import describe, format_mean, format_rate, outcome_rate


CONDITIONS = [('clean', True), ('clean', False), ('replay', False), ('occlusion', False)]
LABELS = {'camera': 'Camera-only', 'lidar': 'LiDAR-only', 'fusion': 'Camera–LiDAR'}
GUARD_KEYS = ('scan_topic', 'odom_topic', 'cmd_topic', 'base_frame', 'front_extent',
              'corridor_half_width', 'min_clearance', 'reaction_time', 'braking_deceleration',
              'sensor_timeout_wall', 'max_stamp_age_sim', 'control_period', 'allow_positive_infinity',
              'heading_gain', 'max_angular_speed', 'max_trial_seconds')
PROTOCOL_KEYS = ('speed_m_s', 'goal_m', 'minimum_margin_m', 'max_cross_track_m',
                 'no_stop_horizon_sim_s', 'fault_injection', 'seed', 'replay_delay_s',
                 'occlusion_parameters', 'confirmation_pairs')


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def batch(folder, repeats, controllers, pilot=False):
    protocol = load(folder/'protocol.json')
    audited = (folder/'analysis-status.json').exists()
    status = load(folder/('analysis-status.json' if audited else 'status.json'))
    rows = load(folder/('analysis-runs.json' if audited else 'runs.json'))
    expected = {(c, attack, empty, n) for c in controllers for attack, empty in CONDITIONS
                for n in range(1, repeats+1)}
    observed = Counter((r['controller'], r['input'], r['empty'], r['repeat']) for r in rows)
    if observed != Counter(expected) or protocol['pilot'] != pilot or not status['source_unchanged']:
        raise ValueError(f'Incomplete or unfrozen matrix in {folder}')
    return protocol, rows


def controller_config(folder, row):
    case = 'clear_segment' if row['empty'] else 'obstacle'
    files = list((folder/row['trial']/case).glob('*/config.json'))
    if len(files) != 1:
        raise ValueError(f'Missing unique controller configuration for {row["trial"]}')
    cfg = load(files[0])
    if (cfg['speed'] != 1.0 or cfg['goal_distance'] != 4.0
            or cfg['enable_goal_stop'] != row['empty'] or not cfg['use_sim_time']):
        raise ValueError(f'Controller configuration mismatch for {row["trial"]}')
    return cfg, files[0]


def exposure_before_stop(folder, row):
    trial = folder/row['trial']
    delivery = read_csv(trial/'image-delivery.csv')
    if row['input'] == 'replay':
        altered = {r['input_stamp_ns'] for r in read_csv(next((trial/'attack').glob('*/frames.csv')))
                   if r['state'] == 'REPLAY'}
    else:
        altered = {r['header_stamp_ns'] for r in delivery if r['black_roi'] == 'True'}
    end = row['stop_command_sim_s']
    if end is None:
        end = load(trial/'summary.json')['evaluation_end_sim_s']
    count = sum(r['header_stamp_ns'] in altered and
                row['motion_start_sim_s'] <= float(r['delivery_sim_s']) < end for r in delivery)
    if not count:
        raise ValueError(f'No manipulated image verified before stopping in {row["trial"]}')
    return count


def audit(root, previous, added, pilot):
    old, reused = batch(previous, 5, ('camera', 'fusion'))
    new, lidar = batch(added, 1 if pilot else 5, ('lidar',), pilot)
    if any(old[k] != new[k] for k in PROTOCOL_KEYS):
        raise ValueError('Historical and LiDAR-only protocols differ')
    if not all(r['valid'] for r in reused):
        raise ValueError('Historical record is not analyzable')
    changed = []
    for name, digest in old['sources_sha256'].items():
        blob = subprocess.check_output(['git', 'show', f'0aec88b:{name}'], cwd=root)
        if hashlib.sha256(blob).hexdigest() != digest:
            raise ValueError(f'Historical frozen source mismatch: {name}')
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != digest:
            changed.append(name)
    allowed = {'scripts/run_issue13.py', 'scripts/issue13_metrics.py', 'scripts/verify_lidar_gazebo.py'}
    if set(changed) - allowed:
        raise ValueError(f'Unreviewed source difference: {set(changed)-allowed}')
    shared_guard = None
    for folder, rows in [(previous, reused), (added, lidar)]:
        for row in rows:
            if not row['valid']:
                continue  # Preserve collection failures and their denominator.
            cfg, path = controller_config(folder, row)
            if row['controller'] == 'camera':
                if (cfg['red_area_threshold'], cfg['roi_top']) != (.4, .25):
                    raise ValueError('Camera baseline threshold mismatch')
                continue
            guard = {key:cfg[key] for key in GUARD_KEYS}
            if shared_guard is None:
                shared_guard = guard
            if guard != shared_guard:
                raise ValueError(f'LiDAR guard parameter mismatch in {path}')
            if row['controller'] == 'fusion':
                visual = load(path.with_name('visual-config.json'))
                if (visual['confirmations'], visual['calibration_mode'], visual['visual_stop_threshold'],
                        visual['roi_top']) != (3, False, .4, .25):
                    raise ValueError('Fusion monitor configuration mismatch')
    result = dict(reused_records=len(reused), added_records=len(lidar), historical_commit='0aec88b',
                  verified_historical_sources=len(old['sources_sha256']),
                  reviewed_harness_only_differences=sorted(changed), shared_guard=shared_guard,
                  matched_protocol={k:new[k] for k in PROTOCOL_KEYS},
                  lidar_manipulated_frames_before_stop={r['trial']:exposure_before_stop(added,r)
                      for r in lidar if r['valid'] and r['input'] != 'clean'},
                  timing_note='Separate collection batches. Controller code, shared geometry and scoring are matched; DDS startup and scheduling can vary.')
    return result, reused + lidar


def statistics(rows):
    conditions = []
    for attack, empty in CONDITIONS:
        for controller in LABELS:
            sample = [r for r in rows if (r['controller'],r['input'],r['empty'])==(controller,attack,empty)]
            clean = attack == 'clean'
            conditions.append(dict(controller=controller,input=attack,scene='empty' if empty else 'obstacle',
                planned=len(sample),valid=sum(r['valid'] for r in sample),
                attack_success=None if clean else outcome_rate(sample,'attack_success'),
                normal_task_success=outcome_rate(sample,'normal_task_success') if clean else None,
                premature_stop=outcome_rate(sample,'premature_stop') if empty else None,
                empty_task_completion_s=describe([r['motion_to_stop_command_s'] + r['settling_latency_s']
                    for r in sample if empty and r.get('normal_task_success')
                    and r.get('motion_to_stop_command_s') is not None and r.get('settling_latency_s') is not None]),
                false_alarm=outcome_rate(sample,'false_alarm') if clean and controller=='fusion' else None,
                alerts=sum(r.get('first_alert_sim_s') is not None for r in sample),
                missing_braking=[r.get('braking_note',r.get('error')) for r in sample if r.get('braking_latency_s') is None],
                missing_detection=[r.get('detection_note',r.get('error')) for r in sample
                                   if not clean and r.get('detection_latency_s') is None],
                **{key:describe([r.get(key) for r in sample]) for key in
                   ('min_clearance_lower_m','min_clearance_upper_m','motion_to_stop_command_s',
                    'detection_latency_s','braking_latency_s','settling_latency_s')}))
    return conditions


def write_report(output, result, rows, groups):
    labels = {('clean','empty'):'Clean empty route', ('clean','obstacle'):'Clean obstacle',
              ('replay','obstacle'):'1 s replay, obstacle', ('occlusion','obstacle'):'Occlusion, obstacle'}
    lines = ['# Issue 15 three-controller comparison', '',
             f"{result['reused_records']} audited Issue #13 records and {result['added_records']} new LiDAR-only records. "
             f"{sum(r['valid'] for r in rows)}/{len(rows)} analyzable trials.", '',
             '| Condition | Controller | Attack success | Normal task success | Premature stops | False anomaly alarms | Mean empty task completion (sim s) |',
             '|---|---|---:|---:|---:|---:|---:|']
    for g in groups:
        cells = [labels[g['input'],g['scene']],LABELS[g['controller']]]
        cells += [format_rate(g[key]) for key in ('attack_success','normal_task_success','premature_stop','false_alarm')]
        cells.append(format_mean(g['empty_task_completion_s'],g['planned']))
        lines.append('| '+' | '.join(cells)+' |')
    lines += ['', '| Condition | Controller | Clearance bounds across runs (m) | Mean start to stop command (sim s) | Mean detection (sim s) | Mean braking onset (sim s) | Mean settling (sim s) |',
              '|---|---|---:|---:|---:|---:|---:|']
    for g in groups:
        lo, hi = g['min_clearance_lower_m'],g['min_clearance_upper_m']
        bounds = f"[{lo['minimum']:.3f}, {hi['maximum']:.3f}]" if lo and hi else 'N/A'
        cells = [labels[g['input'],g['scene']],LABELS[g['controller']],bounds]
        cells += [format_mean(g[key],g['planned']) for key in ('motion_to_stop_command_s','detection_latency_s',
                                                              'braking_latency_s','settling_latency_s')]
        lines.append('| '+' | '.join(cells)+' |')
    lines += ['', 'Clearance entries span the lowest sampled lower bound and highest sampled upper bound of the per-run minima. '
              'The data support geometry-based threshold classification. Direct contact events were not recorded.', '',
              'Empty-task completion time runs from motion start to the onset of sustained near-zero speed at the goal. '
              'The start-to-command and command-to-settling intervals are also shown separately. '
              'Visual alert timing is applicable to the fusion monitor. Reasons for unavailable measurements '
              'and timing ranges are in `metrics.json` and the per-trial records.', '',
              'The two collection batches use the same protocol and source-verified controller configuration. '
              'The design was not contemporaneously interleaved across all three controllers. Exact seeds were not controlled. '
              'Interpret results within these fixed scenes and five repeats per cell.', '']
    (output/'REPORT.md').write_text('\n'.join(lines), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('previous', type=Path)
    parser.add_argument('lidar', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pilot', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    result, rows = audit(root,args.previous,args.lidar,args.pilot)
    if args.pilot and not all(r['valid'] for r in rows):
        raise ValueError('Pilot has invalid records')
    args.output.mkdir(parents=True, exist_ok=True)
    prefix = 'pilot-' if args.pilot else ''
    (args.output/f'{prefix}compatibility.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    if not args.pilot:
        groups = statistics(rows)
        (args.output/'metrics.json').write_text(json.dumps(dict(conditions=groups),indent=2)+'\n',encoding='utf-8')
        write_report(args.output,result,rows,groups)
    print(f"Audited {result['reused_records']} historical and {result['added_records']} new records")


if __name__ == '__main__':
    main()
