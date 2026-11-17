"""ROS-independent helpers for the four-sensor bring-up.

The pinned TurtleBot3 Waffle Pi model already provides an IMU and wheel joint
states.  This module converts wheel joint velocities to a wheel-speed reading
and keeps the latest timestamped sample of each sensor so a ROS node can emit
one synchronised CSV row.  It deliberately has no ROS/OpenCV dependency and is
unit-tested on the host.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WheelSpeed:
    """Left/right wheel speeds in rad/s and m/s."""

    left_rad_s: float
    right_rad_s: float
    left_m_s: float
    right_m_s: float


def wheel_speed_from_joint_state(names, velocities, wheel_radius,
                                 left_joint='wheel_left_joint',
                                 right_joint='wheel_right_joint'):
    """Map a JointState name/velocity pair to WheelSpeed.

    Raises KeyError if a configured wheel joint is missing, ValueError for
    duplicate names, mismatched lengths, non-finite values or a bad radius.
    """
    if not math.isfinite(wheel_radius) or wheel_radius <= 0:
        raise ValueError('wheel_radius must be finite and positive')
    if left_joint == right_joint:
        raise ValueError('left_joint and right_joint must differ')
    names = list(names)
    velocities = list(velocities)
    if len(names) != len(velocities):
        raise ValueError('JointState name and velocity lengths differ')
    lookup = {}
    for name, velocity in zip(names, velocities):
        if name in lookup:
            raise ValueError(f'Duplicate joint name: {name}')
        if not math.isfinite(velocity):
            raise ValueError(f'Non-finite velocity for {name}')
        lookup[name] = velocity
    missing = [name for name in (left_joint, right_joint) if name not in lookup]
    if missing:
        raise KeyError('Missing wheel joint(s): ' + ', '.join(missing))
    left = lookup[left_joint]
    right = lookup[right_joint]
    return WheelSpeed(left, right, left * wheel_radius, right * wheel_radius)


@dataclass(frozen=True)
class SyncSnapshot:
    """One complete four-sensor sample."""

    stamps: dict
    span_s: float
    payloads: dict


class SensorBuffer:
    """Latest sample per sensor with tolerance and staleness rules.

    ``update`` ignores duplicate and backward timestamps so a repeated timer
    tick cannot create a new sample.  ``ready`` returns a snapshot only when
    all required sensors are present, the oldest/newest stamp span is within
    ``tolerance``, and the newest sample is not older than ``staleness``.
    """

    KINDS = ('image', 'scan', 'imu', 'wheel')

    def __init__(self, required=KINDS):
        required = tuple(required)
        if not required or any(kind not in self.KINDS for kind in required):
            raise ValueError('Invalid required sensor set')
        self.required = required
        self.latest = {}

    def update(self, kind, stamp, payload):
        if kind not in self.required:
            raise ValueError(f'Unexpected sensor kind: {kind}')
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError('Invalid sensor timestamp')
        previous = self.latest.get(kind)
        if previous is not None and stamp <= previous[0]:
            # Equal stamps are duplicate deliveries; older stamps are out of order.
            return False
        self.latest[kind] = (stamp, payload)
        return True

    def ready(self, now, tolerance, staleness):
        if not math.isfinite(now) or tolerance <= 0 or staleness <= 0:
            raise ValueError('Invalid sync configuration')
        if any(kind not in self.latest for kind in self.required):
            return None
        stamps = {kind: self.latest[kind][0] for kind in self.required}
        newest = max(stamps.values())
        oldest = min(stamps.values())
        if newest < now - staleness:
            return None
        span = newest - oldest
        if span > tolerance:
            return None
        return SyncSnapshot(
            stamps=stamps,
            span_s=span,
            payloads={kind: self.latest[kind][1] for kind in self.required},
        )

    def missing(self):
        return [kind for kind in self.required if kind not in self.latest]
