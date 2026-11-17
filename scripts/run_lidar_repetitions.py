#!/usr/bin/env python3
"""Reset-scene LiDAR-only trials; retains unsuccessful runs and missing metrics."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys


def metrics(folder):
    files = list(folder.glob('*/*/events.csv'))
    if len(files) != 1:
        return {'metrics_error': f'Expected one control CSV, found {len(files)}'}
    with files[0].open() as stream:
        rows = list(csv.DictReader(stream))
    running = next((r for r in rows if r['state'] == 'RUNNING'), None)
    stopped = next((r for r in rows if r['state'] in {
        'GOAL_REACHED', 'OBSTACLE_STOP', 'SENSOR_TIMEOUT', 'SENSOR_INVALID',
        'FAULT', 'TRIAL_TIMEOUT', 'COMMAND_CONFLICT'}), None)
    final = rows[-1] if rows else {}
    finite_gaps = [float(r['corridor_clearance_m']) for r in rows
                   if math.isfinite(float(r['corridor_clearance_m']))]
    return dict(final_state=final.get('state'),
                final_distance_m=float(final['distance_m']) if final else None,
                final_speed_m_s=float(final['odom_v']) if final else None,
                min_lidar_corridor_gap_m=min(finite_gaps) if finite_gaps else None,
                time_to_stop_sim_s=(float(stopped['sim_time_s'])-float(running['sim_time_s'])
                                    if running and stopped else None),
                time_to_stop_wall_s=(float(stopped['wall_elapsed_s'])-float(running['wall_elapsed_s'])
                                     if running and stopped else None),
                collision_ground_truth=None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--repeats', type=int, default=5)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    verifier = Path(__file__).with_name('verify_lidar_gazebo.py')
    plan = {'obstacle-clean': ['--case', 'obstacle', '--delay', '0.0'],
            'obstacle-delay': ['--case', 'obstacle', '--delay', '1.0'],
            'obstacle-occlusion': ['--case', 'obstacle', '--occlusion'],
            'empty-clean': ['--empty-world', '--case', 'clear', '--goal-distance', '4.0']}
    outcomes = []
    for repeat in range(args.repeats):
        names = list(plan)
        names = names[repeat % len(names):] + names[:repeat % len(names)]
        for condition in names:
            folder = output / f'{condition}-{repeat+1}'
            environment = dict(os.environ, ROS_DOMAIN_ID='78',
                               GZ_PARTITION=f'lidar-reset-{output.name}-{condition}-{repeat+1}')
            command = [sys.executable, str(verifier), '--output', str(folder), *plan[condition]]
            with (output / f'{condition}-{repeat+1}.log').open('w') as log:
                process = subprocess.Popen(command, env=environment, stdout=log,
                                           stderr=subprocess.STDOUT)
                try:
                    code = process.wait(timeout=240)
                except subprocess.TimeoutExpired:
                    # Let the verifier's finally block stop its scene and controller.
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=60)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    code = 124
                except KeyboardInterrupt:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=60)
                    raise
            entry = dict(condition=condition, repeat=repeat+1, exit_code=code, passed=False)
            summary = folder / 'summary.json'
            if summary.exists():
                entry['passed'] = code == 0 and json.loads(summary.read_text())['all_passed']
                entry.update(metrics(folder))
            outcomes.append(entry)
            (output / 'runs.json').write_text(json.dumps(outcomes, indent=2)+'\n')
            print(json.dumps(entry), flush=True)
    aggregate = {name: dict(passed=sum(r['passed'] for r in outcomes if r['condition']==name),
                           total=sum(r['condition']==name for r in outcomes)) for name in plan}
    (output / 'summary.json').write_text(json.dumps(dict(
        checked_at_utc=datetime.now(timezone.utc).isoformat(), conditions=aggregate,
        limits=['Sensor-derived gaps; no independent collision ground truth.',
                'No added sensor noise or configured random seeds.',
                'Only LiDAR-only is evaluated; no multi-sensor benefit claim.']), indent=2)+'\n')
    return 0 if all(r['passed'] for r in outcomes) else 1


if __name__ == '__main__':
    raise SystemExit(main())
