#!/usr/bin/env python3
"""Frozen-parameter reset-scene validation, retaining every attempted trial."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def trial_metrics(folder):
    summaries = json.loads((folder/'summary.json').read_text())
    files = list(folder.glob('*/*/events.csv'))
    if len(files) != 1:
        return dict(summary_passed=False, error='Missing unique controller CSV')
    rows = read_rows(files[0])
    terminal = {'ANOMALY_STOP','VISUAL_STOP','OBSTACLE_STOP','GOAL_REACHED',
                'SENSOR_TIMEOUT','SENSOR_INVALID','FAULT','TRIAL_TIMEOUT','COMMAND_CONFLICT'}
    start = next((r for r in rows if r['state']=='RUNNING'), None)
    stop = next((r for r in rows if r['state'] in terminal), None)
    pairs = read_rows(files[0].with_name('pairs.csv'))
    alert = next((r for r in pairs if r['alert']=='True'), None)
    gt_gap = summaries['ground_truth_min_clearance_lower_bound_m']
    last = rows[-1]
    return dict(summary_passed=summaries['all_passed'], state=last['state'],
        final_distance_m=float(last['distance_m']), final_speed_m_s=float(last['odom_v']),
        min_pose_clearance_bound_m=gt_gap,
        stop_time_sim_s=(float(stop['sim_time_s'])-float(start['sim_time_s']) if start and stop else None),
        stop_time_wall_s=(float(stop['wall_elapsed_s'])-float(start['wall_elapsed_s']) if start and stop else None),
        anomaly_alert=alert is not None, first_alert_direction=alert['direction'] if alert else None,
        first_alert_clearance_m=float(alert['clearance']) if alert else None,
        independent_pose_samples=summaries['ground_truth_samples'],
        fault_activated_sim_s=summaries.get('fault_activated_sim_s'),
        fault_to_stop_sim_s=(float(stop['sim_time_s'])-summaries['fault_activated_sim_s']
                            if stop and summaries.get('fault_activated_sim_s') is not None else None),
        pose_margin_passed=(gt_gap is None or gt_gap >= .35))


def describe(values):
    values=[v for v in values if v is not None]
    return dict(n=len(values), mean=statistics.mean(values) if values else None,
                sd=statistics.stdev(values) if len(values)>1 else None,
                minimum=min(values) if values else None, maximum=max(values) if values else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--fault-repeats', type=int, default=3)
    args=parser.parse_args()
    if min(args.repeats,args.fault_repeats)<1: parser.error('Repeat counts must be positive')
    root=Path(__file__).resolve().parents[1]
    output=Path(args.output).resolve(); output.mkdir(parents=True,exist_ok=False)
    obstacle=['--case','obstacle']
    empty=['--empty-world','--case','clear','--goal-distance','4.0']
    plan={
        'obstacle-clean':[*obstacle,'--delay','0.0'],
        'obstacle-delay-1':[*obstacle,'--delay','1.0'],
        'obstacle-occlusion':[*obstacle,'--occlusion'],
        'empty-clean':[*empty,'--delay','0.0'],
        'empty-delay-1':[*empty,'--delay','1.0'],
        'empty-occlusion':[*empty,'--occlusion'],
        'obstacle-delay-0.5':[*obstacle,'--delay','0.5','--expected-stop','ANOMALY_STOP|OBSTACLE_STOP'],
        'camera-loss':[*obstacle,'--delay','0.0','--fault','camera-loss'],
        'scan-loss':[*obstacle,'--delay','0.0','--fault','scan-loss'],
        'camera-freeze':[*obstacle,'--delay','0.0','--fault','camera-freeze']}
    sources=[root/'configs/camera_lidar.yaml',root/'scripts/verify_lidar_gazebo.py',Path(__file__)]
    sources += list((root/'src/tb3_camera_lidar_monitor/tb3_camera_lidar_monitor').glob('*.py'))
    sources += list((root/'src/tb3_lidar_baseline/tb3_lidar_baseline').glob('*.py'))
    hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (output/'protocol.json').write_text(json.dumps(dict(
        status='FROZEN_SCENE_VALIDATION_NOT_GENERAL_ROBUSTNESS',
        plan=plan,repeats=args.repeats,fault_repeats=args.fault_repeats,
        fixed_speed=1.0,min_pose_margin_m=.35,seed='No noise injected; no explicit random seed',
        short_delay_policy='Safety stop can be anomaly or LiDAR; report detection separately',
        sha256=hashes),indent=2)+'\n')
    runs=[]
    for repeat in range(max(args.repeats,args.fault_repeats)):
        names=[n for n in plan if repeat<(args.fault_repeats if n in ('camera-loss','scan-loss','camera-freeze') else args.repeats)]
        rotation=repeat%len(names); names=names[rotation:]+names[:rotation]
        for name in names:
            folder=output/f'{name}-{repeat+1}'
            env=dict(os.environ,ROS_DOMAIN_ID='85',GZ_PARTITION=f'fusion-campaign-{output.name}-{name}-{repeat+1}')
            command=[sys.executable,str(root/'scripts/verify_lidar_gazebo.py'),
                     '--controller','fusion','--ground-truth','--output',str(folder),*plan[name]]
            with (output/f'{name}-{repeat+1}.log').open('w') as log:
                process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT)
                try: code=process.wait(timeout=240)
                except (subprocess.TimeoutExpired,KeyboardInterrupt) as error:
                    process.send_signal(signal.SIGINT)
                    try: process.wait(timeout=60)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()
                    if isinstance(error,KeyboardInterrupt): raise
                    code=124
            entry=dict(condition=name,repeat=repeat+1,exit_code=code,passed=False)
            if (folder/'summary.json').exists():
                try: entry.update(trial_metrics(folder))
                except Exception as error: entry['metrics_error']=str(error)
            entry['passed']=(code==0 and entry.get('summary_passed',False)
                             and entry.get('pose_margin_passed',False))
            runs.append(entry)
            (output/'runs.json').write_text(json.dumps(runs,indent=2)+'\n')
            print(json.dumps(entry),flush=True)
    groups={}
    for name in plan:
        rows=[r for r in runs if r['condition']==name]
        groups[name]=dict(total=len(rows),passed=sum(r['passed'] for r in rows),
            alerts=sum(r.get('anomaly_alert',False) for r in rows),
            states={s:sum(r.get('state')==s for r in rows) for s in {r.get('state') for r in rows}},
            ground_truth_gap_m=describe([r.get('min_pose_clearance_bound_m') for r in rows]),
            stop_time_sim_s=describe([r.get('stop_time_sim_s') for r in rows]),
            fault_to_stop_sim_s=describe([r.get('fault_to_stop_sim_s') for r in rows]),
            endpoint_overshoot_m=describe([r['final_distance_m']-4 for r in rows
                                          if name.startswith('empty') and 'final_distance_m' in r]))
    unchanged=all(hashlib.sha256((root/p).read_bytes()).hexdigest()==digest for p,digest in hashes.items())
    (output/'summary.json').write_text(json.dumps(dict(groups=groups,source_unchanged=unchanged,
        all_passed=unchanged and all(r['passed'] for r in runs),
        limits=['Only this calibrated scene, robot, parameters and tested attacks.',
                'Pose-derived enclosure separation, not a contact sensor collision rate.',
                'No statistical comparison with other controllers in this batch.']),indent=2)+'\n')
    return 0 if unchanged and all(r['passed'] for r in runs) else 1


if __name__=='__main__': main_code=main(); raise SystemExit(main_code)
