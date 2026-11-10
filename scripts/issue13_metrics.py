"""Independent geometric and timing measurements for the four-group evaluation."""
from bisect import bisect_left
import csv
import json
import math
import xml.etree.ElementTree as ET


def robot_witness_center(model_file):
    """Use a point inside the pinned robot's base collision box."""
    model = ET.parse(model_file).getroot().find('model')
    link = model.find("link[@name='base_link']")
    collision = link.find("collision[@name='base_collision']")
    if link.find('pose') is not None or collision.find('geometry/box') is None:
        raise ValueError('Unsupported base collision frame or shape')
    pose = collision.find('pose')
    if pose.get('relative_to'):
        raise ValueError('Unsupported relative collision frame')
    return tuple(float(v) for v in pose.text.split()[:3])


def witness_upper_bound(position, quaternion, center):
    """Distance from an interior robot point to the red box bounds clearance above."""
    x, y, z = center
    a, b, c, d = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    tx, ty, tz = 2*(b*z-c*y), 2*(c*x-a*z), 2*(a*y-b*x)
    world_x = position.x+x+d*tx+b*tz-c*ty
    world_y = position.y+y+d*ty+c*tx-a*tz
    world_z = position.z+z+d*tz+a*ty-b*tx
    return math.hypot(max(0., abs(world_x-4.)-.15), max(0., abs(world_y)-.25),
                      max(0., abs(world_z-.25)-.25))


def read_csv(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def sustained_start(series, predicate, duration):
    start = None
    previous = None
    for t, value in series:
        if previous is not None and t-previous > .10:
            start = None
        if predicate(value):
            start = t if start is None else start
            if t-start >= duration:
                return start
        else:
            start = None
        previous = t
    return None


def measure_braking(poses, command_time):
    """Derive forward speed from native pose differences over trailing 0.10 s windows."""
    missing = dict(braking_latency_s=None, settling_latency_s=None)
    if command_time is None:
        return dict(missing, braking_note='No controller stop command after forward motion')
    times = [t for t, _ in poses]
    speed = []
    for i, (t, x) in enumerate(poses):
        j = bisect_left(times, t-.10, 0, i)
        if j > 0 and t-times[j-1] <= .20:
            speed.append((t, (x-poses[j-1][1])/(t-times[j-1])))
    before = [v for t,v in speed if command_time-.15 <= t < command_time]
    after = [(t,v) for t,v in speed if t >= command_time]
    settled = sustained_start(after, lambda v: abs(v) <= .02, .15)
    result = dict(missing, settling_latency_s=None if settled is None else settled-command_time)
    if not before or max(before) < .15:
        return dict(result, braking_note='Already slow before command or insufficient independent speed samples')
    reference = before[-1]
    if reference < .15:
        return dict(result, braking_note='Deceleration preceded the stop command')
    onset = sustained_start(after, lambda v: v <= reference-.10, .10)
    return dict(result, braking_latency_s=None if onset is None else onset-command_time,
                braking_note='No sustained decrease captured' if onset is None else None)


def analyze(folder, attack, empty, defense):
    s = json.loads((folder/'summary.json').read_text())
    counts = set(s['command_publisher_counts'])
    # DDS discovery can initially report zero. Require a discovered publisher,
    # no observed competition, and a real forward command below.
    if not s['cases'] or 1 not in counts or counts - {0, 1} or not s['attack_verified']:
        raise ValueError('Missing outcome, verified image delivery, or sole command publisher')
    end = s['evaluation_end_sim_s']
    final = s['cases'][0]['final_event']
    commands = read_csv(folder/'commands.csv')
    motion = next((float(r['sim_time_s']) for r in commands if float(r['linear_x']) > .1), None)
    if motion is None:
        raise ValueError('No forward command captured')
    brake = next((float(r['sim_time_s']) for r in commands
                  if motion < float(r['sim_time_s']) <= end and abs(float(r['linear_x'])) <= .02), None)
    if final['state'] != 'RUNNING' and brake is None:
        raise ValueError('Terminal controller state without captured stop command')
    pose_rows = [r for r in read_csv(folder/'ground-truth.csv') if float(r['sim_time_s']) <= end]
    movement = [r for r in pose_rows if float(r['sim_time_s']) >= motion]
    if not movement:
        raise ValueError('No native simulator poses during motion')
    poses = sorted({float(r['sim_time_s']):float(r['model_x']) for r in pose_rows}.items())
    lower = min((float(r['clearance_lower_bound_m']) for r in movement), default=None) if not empty else None
    upper = min((float(r['clearance_upper_bound_m']) for r in movement), default=None) if not empty else None
    clearance = None if empty else ('safe' if lower >= .35 else 'violation' if upper < .35 else 'indeterminate')
    if not empty and lower > upper + 1e-8:
        raise ValueError('Inconsistent geometric clearance bounds')
    path_ok = max(abs(float(r['model_y'])) for r in movement) <= .15
    braking = measure_braking(poses, brake)
    stopped = braking['settling_latency_s'] is not None
    clean_success = None
    if attack == 'clean':
        clean_success = (path_ok and stopped and
            (final['state']=='GOAL_REACHED' and poses[-1][1]>=3.98 if empty else
             final['state'] in ('OBSTACLE_STOP','VISUAL_STOP') and clearance=='safe'))
        if not empty and clearance=='indeterminate' and path_ok and stopped and final['state'] in ('OBSTACLE_STOP','VISUAL_STOP'):
            clean_success = None
    onset = None
    detection_note = None
    if attack != 'clean':
        delivered = read_csv(folder/'image-delivery.csv')
        accepted = [r for r in read_csv(next(folder.glob('*/*/image-acceptance.csv')))
                    if float(r['accepted_sim_s']) <= end]
        if attack == 'replay':
            altered = {r['input_stamp_ns'] for r in read_csv(next((folder/'attack').glob('*/frames.csv')))
                       if r['state']=='REPLAY'}
        else:
            altered = {r['header_stamp_ns'] for r in delivered if r['black_roi']=='True'}
        candidates = [float(r['accepted_sim_s']) for r in accepted if r['header_stamp_ns'] in altered]
        if not candidates:
            raise ValueError('No manipulated frame linked to the delivered stream')
        if candidates[0] > 0:
            onset = candidates[0]
        else:
            detection_note = 'First manipulated frame was accepted before controller clock synchronization'
    alerts = json.loads((folder/'alerts.json').read_text())
    first_alert = float(alerts[0]['published_sim_s']) if alerts else None
    detection_latency = first_alert-onset if first_alert is not None and onset is not None and first_alert >= onset else None
    if attack != 'clean' and detection_latency is None:
        if not defense:
            detection_note = 'Anomaly detector disabled'
        elif detection_note is None:
            detection_note = 'No confirmed alert' if first_alert is None else 'Unaligned alert and delivery timestamps'
    return dict(valid=True, final_state=final['state'], end_reason=s['cases'][0]['end_reason'],
        observed_command_publisher_counts=sorted(counts),
        min_clearance_lower_m=lower, min_clearance_upper_m=upper, clearance=clearance,
        attack_success=None if attack=='clean' or clearance=='indeterminate' else clearance=='violation',
        normal_task_success=clean_success, path_ok=path_ok,
        false_alarm=(bool(alerts) if defense else None) if attack=='clean' else None,
        premature_stop=(brake is not None and final['state']!='GOAL_REACHED') if empty else None,
        attack_delivery_sim_s=onset, first_alert_sim_s=first_alert, detection_latency_s=detection_latency,
        detection_note=detection_note,
        detector_enabled=defense, motion_start_sim_s=motion, stop_command_sim_s=brake,
        motion_to_stop_command_s=None if brake is None else brake-motion,
        pose_samples=len(movement), **braking)
