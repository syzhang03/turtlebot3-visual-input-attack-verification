#!/usr/bin/env python3
"""Sequential development checks, not matched formal evaluation trials.

Run in a sourced ROS/TurtleBot3/project environment. No other controller may run.
Creates its own scene; sends real simulation velocity commands; closes on exit.
"""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
import queue
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from ament_index_python.packages import get_package_share_directory
from ground_truth import collision_radius, box_clearance_lower_bound


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--empty-world', action='store_true', help='Only test endpoint stopping in an empty world')
    parser.add_argument('--delay', type=float, help='Run the existing camera delay attack concurrently')
    parser.add_argument('--occlusion', action='store_true', help='Use the separately installed teammate camera-occlusion package')
    parser.add_argument('--case', choices=['suite', 'obstacle', 'clear'], default='suite')
    parser.add_argument('--goal-distance', type=float)
    parser.add_argument('--ground-truth', action='store_true', help='Record evaluation-only simulator poses')
    args = parser.parse_args()
    if args.delay is not None and (not math.isfinite(args.delay) or args.delay < 0):
        parser.error('--delay must be finite and nonnegative')
    if args.occlusion and args.delay is not None:
        parser.error('Select delay or occlusion, not both')
    if args.case == 'obstacle' and args.empty_world:
        parser.error('Obstacle case requires the obstacle scene')
    if args.goal_distance is not None and (not math.isfinite(args.goal_distance) or args.goal_distance <= 0):
        parser.error('--goal-distance must be finite and positive')
    project = Path(__file__).resolve().parents[1]
    os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle_pi')
    os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    processes, handles, outcomes = [], [], []
    attack_verified = None if args.delay is None and not args.occlusion else False
    expected_cases = 1 if args.empty_world or args.case != 'suite' else 3

    def start(command, name):
        handle = (output / f'{name}.log').open('w')
        handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)

    rclpy.init()
    peer = rclpy.create_node('lidar_gazebo_verification')
    relay = peer.create_publisher(LaserScan, '/validation/scan', qos_profile_sensor_data)
    forwarding = [True]
    scans = [0]
    attacked_images = [0]
    black_roi_images = [0]
    gt_samples = []
    gt_queue = queue.SimpleQueue()
    gt_log = None
    gt_writer = None
    if args.ground_truth:
        model = Path(get_package_share_directory('turtlebot3_gazebo'))/'models/turtlebot3_waffle_pi/model.sdf'
        required_radius = collision_radius(model)
        if required_radius > 0.30:
            raise ValueError(f'Robot collision bound {required_radius} exceeds evaluation sphere')
        gt_log = (output/'ground-truth.csv').open('w', newline='')
        gt_writer = csv.writer(gt_log)
        gt_writer.writerow(['sim_time_s', 'model_x', 'model_y', 'clearance_lower_bound_m'])

    def receive_pose(message):
        for pose in message.pose:
            if pose.name == 'waffle_pi':
                p = pose.position
                stamp = message.header.stamp.sec + message.header.stamp.nsec*1e-9
                gap = None if args.empty_world else box_clearance_lower_bound(p.x, p.y, .30)
                gt_queue.put((stamp, p.x, p.y, gap))

    def receive_image(message):
        attacked_images[0] += 1
        if args.occlusion:
            # Validate the actual default mask in received BGR8 messages.
            import numpy as np
            if message.encoding == 'bgr8' and message.width and message.height:
                rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
                pixels = rows[:, :message.width*3].reshape(message.height, message.width, 3)
                roi = pixels[int(message.height*.25):, int(message.width*.2):int(message.width*.8)]
                if roi.size and not np.any(roi):
                    black_roi_images[0] += 1

    def receive_scan(message):
        scans[0] += 1
        if forwarding[0]:
            relay.publish(message)

    subscription = peer.create_subscription(LaserScan, '/scan', receive_scan, qos_profile_sensor_data)
    image_subscription = peer.create_subscription(Image, '/attack/image_raw', receive_image, qos_profile_sensor_data)
    if args.ground_truth:
        from gz.transport13 import Node as GazeboNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        gt_node = GazeboNode()
        gt_node.subscribe(Pose_V, '/world/default/pose/info', receive_pose)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(peer, timeout_sec=0.03)
            while not gt_queue.empty():
                sample = gt_queue.get()
                gt_samples.append(sample)
                gt_writer.writerow(sample)
                gt_log.flush()

    def last_event(case):
        events = sorted((output / case).glob('*/events.csv'))
        if not events:
            return None
        try:
            with events[-1].open() as stream:
                rows = list(csv.DictReader(stream))
            return rows[-1] if rows else None
        except (OSError, IndexError):
            return None

    try:
        scene_command = (['ros2', 'launch', 'turtlebot3_gazebo', 'empty_world.launch.py']
                         if args.empty_world else ['bash', str(project / 'scripts/start_scene.sh')])
        scene = start(scene_command, 'scene')
        if args.delay is not None:
            start(['ros2', 'run', 'tb3_visual_baseline', 'delay_replay', '--ros-args',
                   '--params-file', str(project / 'configs/delay_replay.yaml'),
                   '-p', f'delay_s:={args.delay}', '-p', 'duration_s:=600.0',
                   '-p', f'log_dir:={output / "attack"}'], 'attack')
        if args.occlusion:
            start(['ros2', 'run', 'tb3_camera_attack', 'camera_occlusion', '--ros-args',
                   '-p', 'use_sim_time:=true', '-p', 'attack_enabled:=true',
                   '-p', 'output_topic:=/attack/image_raw'], 'attack')
        deadline = time.monotonic() + 110
        while time.monotonic() < deadline:
            spin_for(0.2)
            if scene.poll() is not None:
                raise RuntimeError('Scene exited during startup')
            if scans[0] and (args.empty_world or 'SCENE_READY' in (output / 'scene.log').read_text(errors='replace')):
                break
        else:
            raise RuntimeError('No ready scene and scan within 110 seconds')
        if args.ground_truth:
            deadline = time.monotonic()+10
            while not gt_samples and time.monotonic() < deadline:
                spin_for(.1)
            if not gt_samples:
                raise RuntimeError('No independent simulator model poses received')
        # Start movement only after the selected image manipulation is active.
        if args.delay is not None or args.occlusion:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                spin_for(0.1)
                if args.occlusion and black_roi_images[0]:
                    break
                if args.delay is not None:
                    files = list((output / 'attack').glob('*/frames.csv'))
                    if files:
                        with files[0].open() as stream:
                            state = 'REPLAY' if args.delay > 0 else 'ZERO_DELAY'
                            if any(row.get('state') == state for row in csv.DictReader(stream)):
                                break
            else:
                raise RuntimeError('Image manipulation was not active before movement')
        cases = [('clear_segment', 'GOAL_REACHED')]
        if not args.empty_world:
            cases += [('scan_interruption', 'SENSOR_TIMEOUT'), ('obstacle', 'OBSTACLE_STOP')]
        if args.case == 'obstacle':
            cases = [('obstacle', 'OBSTACLE_STOP')]
        elif args.case == 'clear':
            cases = [('clear_segment', 'GOAL_REACHED')]
        for case, expected in cases:
            forwarding[0] = True
            command = ['ros2', 'run', 'tb3_lidar_baseline', 'lidar_baseline', '--ros-args',
                       '--params-file', str(project / 'configs/lidar_only.yaml'),
                       '-p', 'enabled:=true', '-p', 'scan_topic:=/validation/scan',
                       '-p', f'log_dir:={output / case}']
            if case == 'clear_segment':
                command += ['-p', f'goal_distance:={args.goal_distance or 0.5}']
            process = start(command, case)
            interrupted = False
            deadline = time.monotonic() + 100
            while time.monotonic() < deadline:
                spin_for(0.1)
                event = last_event(case)
                if process.poll() is not None:
                    raise RuntimeError(f'{case} controller exited unexpectedly')
                if not event:
                    continue
                if case == 'scan_interruption' and event['state'] == 'RUNNING' and not interrupted:
                    forwarding[0], interrupted = False, True
                if event['state'] in ('GOAL_REACHED', 'OBSTACLE_STOP', 'SENSOR_TIMEOUT',
                                      'SENSOR_INVALID', 'FAULT', 'TRIAL_TIMEOUT', 'COMMAND_CONFLICT'):
                    spin_for(2)
                    final = last_event(case)
                    passed = (final['state'] == expected and float(final['cmd_v']) == 0
                              and abs(float(final['odom_v'])) < 0.02)
                    outcome = dict(case=case, expected=expected, passed=passed, final_event=final)
                    outcomes.append(outcome)
                    print(json.dumps(outcome), flush=True)
                    if not passed:
                        raise RuntimeError(f'{case} failed')
                    break
            else:
                raise RuntimeError(f'{case} did not stop within 100 seconds')
            stop(process)
            spin_for(0.3)
        if args.delay is not None:
            if attacked_images[0] == 0:
                raise RuntimeError('Delay relay did not publish images')
            frame_files = list((output / 'attack').glob('*/frames.csv'))
            if not frame_files:
                raise RuntimeError('Missing delay relay frame log')
            with frame_files[0].open() as stream:
                expected_relay_state = 'REPLAY' if args.delay > 0 else 'ZERO_DELAY'
                if not any(row['state'] == expected_relay_state for row in csv.DictReader(stream)):
                    raise RuntimeError(f'Relay never entered {expected_relay_state}')
            attack_verified = True
        if args.occlusion:
            if black_roi_images[0] < 1:
                raise RuntimeError('No received image has the expected black detector ROI')
            attack_verified = True
        if args.ground_truth and not args.empty_world:
            if min(s[3] for s in gt_samples) < .35:
                raise RuntimeError('Simulator-pose conservative clearance below 0.35 m')
        return 0
    finally:
        for process in reversed(processes):
            stop(process)
        peer.destroy_node()
        rclpy.shutdown()
        for handle in handles:
            handle.close()
        if gt_log is not None:
            gt_log.close()
        (output / 'summary.json').write_text(json.dumps(dict(
            checked_at_utc=datetime.now(timezone.utc).isoformat(),
            status='DEVELOPMENT_CHECK_NOT_FORMAL_EVALUATION',
            cases=outcomes, empty_world=args.empty_world, camera_delay_s=args.delay,
            received_attacked_images=attacked_images[0],
            camera_occlusion=args.occlusion, black_roi_images=black_roi_images[0],
            attack_verified=attack_verified,
            case_selection=args.case, goal_distance_override=args.goal_distance,
            ground_truth_samples=len(gt_samples),
            ground_truth_robot_enclosure_radius_m=.30 if args.ground_truth else None,
            model_collision_radius_bound_m=required_radius if args.ground_truth else None,
            ground_truth_min_clearance_lower_bound_m=(min(s[3] for s in gt_samples)
                                                      if gt_samples and not args.empty_world else None),
            all_passed=(len(outcomes) == expected_cases
                        and all(o['passed'] for o in outcomes) and attack_verified is not False
                        and (not args.ground_truth or (bool(gt_samples) and (args.empty_world
                             or min(s[3] for s in gt_samples) >= .35)))),
            limitations=(['Single empty-world endpoint check; no repeated trials.'] if args.empty_world
                         else ['Sequential runs in one scene, not reset matched trials.',
                               'Clear segment ends before the distant red obstacle.']) + [
                         ('Simulator-pose conservative clearance bound; no contact sensor.' if args.ground_truth
                          else 'No contact sensor or independent ground-truth clearance measurement.'),
                         'No formal attack success rate or repeated noise study.']), indent=2) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
