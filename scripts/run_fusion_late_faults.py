#!/usr/bin/env python3
"""Late sensor faults; emergency LiDAR stop may correctly beat the watchdog."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from run_fusion_campaign import trial_metrics

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',required=True)
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
paths=[root/'scripts/verify_lidar_gazebo.py',root/'configs/camera_lidar.yaml',Path(__file__)]
paths += list((root/'src/tb3_camera_lidar_monitor/tb3_camera_lidar_monitor').glob('*.py'))
hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
(output/'protocol.json').write_text(json.dumps(dict(
    repeats=3,trigger_lidar_clearance_m=1.5,min_pose_margin_m=.35,
    accepted_states=['SENSOR_TIMEOUT','OBSTACLE_STOP'],
    explanation='A fresh LiDAR emergency stop can precede camera watchdog expiry.',sha256=hashes),indent=2)+'\n')
runs=[]
for repeat in range(3):
    for fault in ('camera-loss','scan-loss','camera-freeze'):
        folder=output/f'{fault}-{repeat+1}'
        env=dict(os.environ,ROS_DOMAIN_ID='86',GZ_PARTITION=f'fusion-late-{fault}-{repeat+1}')
        command=[sys.executable,str(root/'scripts/verify_lidar_gazebo.py'),
                 '--controller','fusion','--case','obstacle','--delay','0.0','--ground-truth',
                 '--fault',fault,'--fault-clearance','1.5',
                 '--expected-stop','SENSOR_TIMEOUT|OBSTACLE_STOP','--output',str(folder)]
        with (output/f'{fault}-{repeat+1}.log').open('w') as log:
            process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT)
            try: code=process.wait(timeout=240)
            except (subprocess.TimeoutExpired,KeyboardInterrupt) as error:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=60)
                except subprocess.TimeoutExpired: process.kill();process.wait()
                if isinstance(error,KeyboardInterrupt):raise
                code=124
        entry=dict(condition=fault,repeat=repeat+1,exit_code=code,passed=False)
        if (folder/'summary.json').exists():entry.update(trial_metrics(folder))
        entry['passed']=(code==0 and entry.get('summary_passed',False)
                         and entry.get('pose_margin_passed',False)
                         and entry.get('fault_activated_sim_s') is not None)
        runs.append(entry)
        (output/'runs.json').write_text(json.dumps(runs,indent=2)+'\n')
        print(json.dumps(entry),flush=True)
unchanged=all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
(output/'summary.json').write_text(json.dumps(dict(total=len(runs),passed=sum(r['passed'] for r in runs),
    source_unchanged=unchanged,all_passed=unchanged and all(r['passed'] for r in runs)),indent=2)+'\n')
raise SystemExit(0 if unchanged and all(r['passed'] for r in runs) else 1)
