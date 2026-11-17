#!/usr/bin/env python3
"""Run the predeclared four-group protocol, keeping every planned trial."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

from issue13_metrics import analyze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--pilot', action='store_true')
    parser.add_argument('--lidar-only', action='store_true', help='Collect the Issue 15 comparator using this protocol')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    plan = [
        ('A', 'camera', 'clean', True), ('C', 'fusion', 'clean', True),
        ('A', 'camera', 'clean', False), ('C', 'fusion', 'clean', False),
        ('B', 'camera', 'replay', False), ('D', 'fusion', 'replay', False),
        ('B', 'camera', 'occlusion', False), ('D', 'fusion', 'occlusion', False),
    ]
    if args.lidar_only:
        plan = [('E', 'lidar', 'clean', True), ('E', 'lidar', 'clean', False),
                ('E', 'lidar', 'replay', False), ('E', 'lidar', 'occlusion', False)]
    repetitions = 1 if args.pilot else 5
    sources = ['scripts/run_issue13.py', 'scripts/issue13_metrics.py', 'scripts/verify_lidar_gazebo.py',
               'scripts/ground_truth.py', 'scripts/start_scene.launch.py', 'configs/delay_replay.yaml',
               'configs/camera_lidar.yaml', 'configs/baseline_white_line.yaml', 'configs/turtlebot3.repos',
               'worlds/intersection-white-line.sdf']
    sources += [p.relative_to(root).as_posix() for p in (root/'src').glob('tb3_*/*/*.py')]
    if args.lidar_only:
        sources.append('configs/lidar_only.yaml')
    hashes = {name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in sources}
    (output/'protocol.json').write_text(json.dumps(dict(
        issue=15 if args.lidar_only else 13, pilot=args.pilot, plan=plan, repetitions=repetitions, speed_m_s=1., goal_m=4.,
        minimum_margin_m=.35, max_cross_track_m=.15, no_stop_horizon_sim_s=8.,
        fault_injection=False, seed='Same pinned world defaults; no added noise or configured seed',
        replay_delay_s=1., occlusion_parameters='Pinned teammate default rectangle',
        confirmation_pairs=3, sources_sha256=hashes), indent=2)+'\n')
    rows = []
    for repetition in range(repetitions):
        offset = (2 * repetition) % len(plan)
        for group, controller, attack, empty in plan[offset:]+plan[:offset]:
            name = f'{group}-{attack}-{"empty" if empty else "obstacle"}-{repetition+1}'
            trial = output/name
            command = [sys.executable, str(root/'scripts/verify_lidar_gazebo.py'), '--output', str(trial),
                       '--evaluate', '--ground-truth', '--controller', controller,
                       '--case', 'clear' if empty else 'obstacle']
            if empty:
                command += ['--empty-world', '--goal-distance', '4.0']
            if attack == 'occlusion':
                command += ['--occlusion']
            else:
                command += ['--delay', '1.0' if attack == 'replay' else '0.0']
            env = dict(os.environ, ROS_DOMAIN_ID='88', GZ_PARTITION=f'issue13-{output.name}-{name}')
            with (output/f'{name}.log').open('w') as log:
                child = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
                try:
                    code = child.wait(timeout=220)
                except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=50)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
                    if isinstance(error, KeyboardInterrupt):
                        raise
                    code = 124
            row = dict(trial=name, group=group, controller=controller, input=attack, empty=empty,
                       repeat=repetition+1, exit_code=code, valid=False)
            if code == 0:
                try:
                    row.update(analyze(trial, attack, empty, controller=='fusion'))
                except (ValueError, OSError, StopIteration, KeyError) as error:
                    row['error'] = str(error)
            else:
                row['error'] = f'Verifier exit {code}; see {name}.log'
            rows.append(row)
            (output/'runs.json').write_text(json.dumps(rows, indent=2)+'\n')
            print(json.dumps(row), flush=True)
    unchanged = all(hashlib.sha256((root/name).read_bytes()).hexdigest()==digest
                    for name, digest in hashes.items())
    (output/'status.json').write_text(json.dumps(dict(planned=len(plan)*repetitions,
        completed=len(rows), valid=sum(r['valid'] for r in rows), source_unchanged=unchanged), indent=2)+'\n')
    return 0 if unchanged and all(r['valid'] for r in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
