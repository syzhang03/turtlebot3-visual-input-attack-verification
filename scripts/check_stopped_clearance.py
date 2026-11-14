#!/usr/bin/env python3
"""Read current Gazebo poses; bound collision-geometry clearance after stopping.

No ROS subscriptions, commands, resets or model mutations. This cannot recover
contacts that happened before this observer started. Local SDFs must match the
models loaded by start_scene.sh. No visual mesh or attacked LaserScan is used.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from threading import Lock
import time
import xml.etree.ElementTree as ET


IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))


def matvec(r, p):
    return tuple(sum(a*b for a, b in zip(row, p)) for row in r)


def transpose(r):
    return tuple(zip(*r))


def multiply(a, b):
    return tuple(tuple(sum(x*y for x, y in zip(row, col))
                       for col in zip(*b)) for row in a)


def compose(a, b):
    pa, ra = a
    pb, rb = b
    return tuple(x+y for x, y in zip(pa, matvec(ra, pb))), multiply(ra, rb)


def inverse(pose):
    p, r = pose
    rt = transpose(r)
    return matvec(rt, tuple(-x for x in p)), rt


def xml_pose(element):
    pose = element.find('pose')
    if pose is None:
        return (0., 0., 0.), IDENTITY
    if pose.get('relative_to') or pose.get('rotation_format', 'euler_rpy') != 'euler_rpy':
        raise ValueError('Unsupported SDF relative frame / rotation format')
    values = [float(v) for v in pose.text.split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Invalid SDF pose')
    angles = values[3:]
    if pose.get('degrees', 'false').lower() in ('true', '1'):
        angles = list(map(math.radians, angles))
    roll, pitch, yaw = angles
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return tuple(values[:3]), ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                              (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                              (-sp, cp*sr, cp*cr))


def message_pose(pose):
    p, q = pose['position'], pose['quaternion_xyzw']
    if not all(math.isfinite(v) for v in p+q):
        raise ValueError('Non-finite native pose')
    n = math.sqrt(sum(v*v for v in q))
    if n < 1e-9:
        raise ValueError('Invalid native quaternion')
    x, y, z, w = (v/n for v in q)
    return p, ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
               (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
               (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def shape(collision):
    geometry = collision.find('geometry')
    if geometry is None or len(geometry) != 1:
        raise ValueError('Missing / ambiguous collision geometry')
    kind = geometry[0].tag
    if kind == 'box':
        dimensions = tuple(float(v) for v in geometry.findtext('box/size').split())
        if len(dimensions) != 3:
            raise ValueError('Invalid box size')
    elif kind == 'sphere':
        dimensions = (float(geometry.findtext('sphere/radius')),)
    elif kind == 'cylinder':
        dimensions = (float(geometry.findtext('cylinder/radius')),
                      float(geometry.findtext('cylinder/length')))
    else:
        raise ValueError(f'Unsupported collision geometry: {kind}')
    if not all(math.isfinite(v) and v > 0 for v in dimensions):
        raise ValueError('Invalid collision dimensions')
    return kind, dimensions


def load_geometry(robot_file, scene_file):
    robot = ET.parse(robot_file).getroot().find('model')
    scene = ET.parse(scene_file).getroot().find('model')
    if (robot is None or scene is None or any(model.findall('model') or model.findall('include')
                                             for model in (robot, scene))):
        raise ValueError('Expected flat robot and scene SDF models')
    collisions = []
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            collisions.append(dict(link=link.get('name'), name=collision.get('name'),
                                   pose=xml_pose(collision), shape=shape(collision)))
    if not collisions:
        raise ValueError('No robot collision shapes')
    targets = [(link, collision) for link in scene.findall('link')
               for collision in link.findall('collision')
               if collision.get('name') == 'red_box_collision']
    if len(targets) != 1:
        raise ValueError('Expected exactly one red_box_collision')
    link, collision = targets[0]
    kind, dimensions = shape(collision)
    if kind != 'box':
        raise ValueError('Red obstacle must have box collision geometry')
    return collisions, dict(model=scene.get('name'), link=link.get('name'),
                           pose=xml_pose(collision), half=tuple(v/2 for v in dimensions))


def aabb(pose, geometry):
    """Exact axis projection extrema for a rotated primitive (not mesh bounds)."""
    center, rotation = pose
    kind, dimensions = geometry
    if kind == 'box':
        half = [sum(abs(v)*s/2 for v, s in zip(row, dimensions)) for row in rotation]
    elif kind == 'sphere':
        half = [dimensions[0]]*3
    elif kind == 'cylinder':
        radius, length = dimensions
        half = [abs(row[2])*length/2 + radius*math.sqrt(max(0., 1-row[2]**2))
                for row in rotation]
    else:
        raise ValueError('Unsupported primitive')
    return tuple(c-h for c, h in zip(center, half)), tuple(c+h for c, h in zip(center, half))


def measure(snapshot, collisions, target, robot_name):
    # SceneBroadcaster publishes model poses relative to world and link poses
    # relative to model. Compose both, including live wheel/link rotations.
    # Source: gazebosim/gz-sim, gz-sim8, SceneBroadcaster.cc PoseUpdate.
    poses = {name: message_pose(value) for name, value in snapshot['poses'].items()}
    obstacle = compose(compose(poses[target['model']], poses[target['link']]), target['pose'])
    obstacle_inverse = inverse(obstacle)
    rows = []
    for collision in collisions:
        world = compose(compose(poses[robot_name], poses[collision['link']]), collision['pose'])
        relative = compose(obstacle_inverse, world)
        low, high = aabb(relative, collision['shape'])
        # Each shape lies inside this bounding box. Bounding-box separation
        # bounds physical separation below; overlap does NOT prove contact.
        gaps = [max(0., lo-half, -half-hi)
                for lo, hi, half in zip(low, high, target['half'])]
        rows.append(dict(link=collision['link'], collision=collision['name'],
                         clearance_lower_bound_m=math.sqrt(sum(v*v for v in gaps)),
                         bounds_in_obstacle_frame=[low, high]))
    return dict(sim_time_s=snapshot['sim_time_s'], robot_position=poses[robot_name][0],
                clearance_lower_bound_m=min(r['clearance_lower_bound_m'] for r in rows),
                collision_bounds=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path, help='Label only; does not prove scene has not been reset')
    parser.add_argument('--scene-file', type=Path,
                        default=Path.home()/'camera-lidar-attack/worlds/intersection-white-line.sdf')
    parser.add_argument('--robot-file', type=Path)
    parser.add_argument('--robot-name', default='waffle_pi')
    parser.add_argument('--world', default='default')
    parser.add_argument('--seconds', type=float, default=5.)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 30:
        parser.error('--seconds must be between 2 and 30')
    if not (args.run_dir/'manifest.json').is_file():
        parser.error('找不到这次实验的 manifest.json；请核对日志目录。')
    if args.robot_file is None:
        from ament_index_python.packages import get_package_share_directory
        args.robot_file = Path(get_package_share_directory('turtlebot3_gazebo'))/'models/turtlebot3_waffle_pi/model.sdf'
    collisions, target = load_geometry(args.robot_file, args.scene_file)
    names = {args.robot_name, target['model'], target['link']} | {c['link'] for c in collisions}
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    node = Node()
    topic = f'/world/{args.world}/pose/info'
    lock, snapshots, failures = Lock(), [], []

    def receive(msg):
        poses = {}
        duplicate = []
        for p in msg.pose:
            if p.name not in names:
                continue
            if p.name in poses:
                duplicate.append(p.name)
            poses[p.name] = dict(id=p.id, position=[p.position.x, p.position.y, p.position.z],
                                 quaternion_xyzw=[p.orientation.x, p.orientation.y,
                                                  p.orientation.z, p.orientation.w])
        missing = names-poses.keys()
        with lock:
            if duplicate or missing:
                if not failures:
                    failures.append(dict(duplicate_names=duplicate, missing_names=sorted(missing)))
                return
            snapshots.append(dict(sim_time_s=msg.header.stamp.sec+msg.header.stamp.nsec/1e9, poses=poses))

    if not node.subscribe(Pose_V, topic, receive):
        parser.exit(2, 'Gazebo 原生位置订阅失败。\n')
    print(f'读取 Gazebo 碰撞几何和原生位置 {args.seconds:g} 秒；不会发布速度指令。', flush=True)
    try:
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            time.sleep(.1)
    finally:
        node.unsubscribe(topic)
    with lock:
        observed = list(snapshots)
    if not observed:
        parser.exit(2, '没有收到完整的模型/连杆位置。请保留当前场景，确认仿真仍在运行。\n'
                       + json.dumps(failures, ensure_ascii=False)+'\n')
    measurements = [measure(s, collisions, target, args.robot_name) for s in observed]
    times = [m['sim_time_s'] for m in measurements]
    reset = any(b < a for a, b in zip(times, times[1:]))
    # Check every primitive's bound, including moving links, not odometry.
    first = measurements[0]
    variation = max(abs(value-ref)
                    for m in measurements
                    for a, b in zip(m['collision_bounds'], first['collision_bounds'])
                    for av, bv in zip(a['bounds_in_obstacle_frame'], b['bounds_in_obstacle_frame'])
                    for value, ref in zip(av, bv))
    stationary = len(set(times)) >= 3 and times[-1]-times[0] >= .1 and variation <= .001 and not reset
    minimum = min(m['clearance_lower_bound_m'] for m in measurements)
    status = ('CURRENTLY_SEPARATED' if stationary and minimum > .001 else
              'INDETERMINATE_NEAR_CONTACT' if stationary else 'NOT_STATIONARY_OR_INSUFFICIENT_DATA')
    folder = Path.home()/'clearance-checks'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    folder.mkdir(parents=True, exist_ok=False)
    report = dict(status=status, trial_label=str(args.run_dir.resolve()),
                  measurement_scope='Only the current observation window; past contact is unknown',
                  history_contact='UNKNOWN', stationary_during_check=stationary,
                  pose_samples=len(observed), sim_time_start=times[0], sim_time_end=times[-1],
                  collision_bound_variation_m=variation, time_reset=reset,
                  clearance_lower_bound_m=minimum, final_measurement=measurements[-1],
                  files={key: dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                         for key, path in [('robot_sdf', args.robot_file), ('scene_sdf', args.scene_file)]},
                  contact_topic_candidates=[t for t in node.topic_list() if 'contact' in t.lower()],
                  limits=['Requires installed robot SDF and local scene SDF to match the live models.',
                          'No runtime SDF edits or nested models are supported.',
                          'A positive bound establishes separation during this check only.',
                          'Overlapping bounds do not establish physical contact.',
                          'This observer was started after the trial and cannot recover past contacts.'])
    (folder/'native-poses.json').write_text(json.dumps(observed, indent=2), encoding='utf-8')
    (folder/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('结果：'+status)
    print(f'检查期间的几何间隙下界：{minimum:.6f} m（{minimum*100:.2f} cm）')
    print(f'原生位置样本：{len(observed)}；检查期间停稳：{stationary}')
    if status == 'CURRENTLY_SEPARATED':
        print('在所用碰撞模型与当前场景一致的前提下，当前车体与方块有间隙。')
    else:
        print('本次几何检查不能判定接触，请保留结果；不能把零下界当作碰撞。')
    print('这不能追溯停车前是否曾经接触；完整碰撞结论需要全程独立记录。')
    print('报告：'+str(folder/'summary.json'))


if __name__ == '__main__':
    main()
