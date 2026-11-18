"""ROS-independent geometry and latched control for a straight, planar route."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ScanResult:
    usable: bool
    clearance: float = math.inf
    reason: str = ''
    invalid_rays: int = 0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def stopping_distance(speed, clearance, reaction_time, deceleration):
    return clearance + speed * reaction_time + speed * speed / (2 * deceleration)


def configured_stop_distance(speed, clearance, reaction_time, deceleration,
                             mode='dynamic', fixed_distance=1.0):
    """Select a braking-model distance or an explicitly calibrated scene boundary."""
    if mode not in ('dynamic', 'fixed'):
        raise ValueError('stop_distance_mode must be dynamic or fixed')
    if not math.isfinite(fixed_distance) or fixed_distance <= 0:
        raise ValueError('fixed_stop_distance must be finite and positive')
    if mode == 'fixed':
        return fixed_distance
    return stopping_distance(speed, clearance, reaction_time, deceleration)


def corridor_scan(ranges, angle_min, angle_increment, range_min, range_max,
                  tx, ty, yaw, half_width, front_extent, lookahead,
                  allow_positive_infinity=True):
    """Transform rays into base coordinates; reject incomplete front coverage.

    +inf is a no-return ray only under the explicitly enabled driver convention.
    NaN, -inf and finite out-of-range readings in the front sector are invalid.
    A full forward semicircle is required, with no angular gap above 5 degrees.
    """
    values = (angle_min, angle_increment, range_min, range_max, tx, ty, yaw,
              half_width, front_extent, lookahead)
    if (not all(math.isfinite(v) for v in values) or len(ranges) < 2
            or not 0 < angle_increment <= math.radians(5)
            or not 0 <= range_min < range_max or half_width <= 0
            or front_extent <= 0 or lookahead <= 0):
        return ScanResult(False, reason='INVALID_SCAN_METADATA')
    required_range = math.hypot(front_extent + lookahead + abs(tx), half_width + abs(ty))
    if range_max < required_range:
        return ScanResult(False, reason='INSUFFICIENT_RANGE')
    bearings = sorted(wrap(angle_min + i * angle_increment + yaw)
                      for i in range(len(ranges)))
    # Include the samples immediately outside both sector boundaries.
    sector = [a for a in bearings
              if -math.pi / 2 - angle_increment <= a <= math.pi / 2 + angle_increment]
    if (not sector or sector[0] > -math.pi / 2 + angle_increment
            or sector[-1] < math.pi / 2 - angle_increment
            or any(b - a > 1.5 * angle_increment for a, b in zip(sector, sector[1:]))):
        return ScanResult(False, reason='INCOMPLETE_FORWARD_COVERAGE')
    nearest, invalid = math.inf, 0
    for i, distance in enumerate(ranges):
        bearing = wrap(angle_min + i * angle_increment + yaw)
        if abs(bearing) > math.pi / 2 + 1e-6:
            continue
        if distance == math.inf and allow_positive_infinity:
            continue
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            invalid += 1
            continue
        x = tx + distance * math.cos(bearing)
        y = ty + distance * math.sin(bearing)
        if x >= 0 and abs(y) <= half_width:
            nearest = min(nearest, max(0.0, x - front_extent))
    if invalid:
        return ScanResult(False, nearest, 'INVALID_FORWARD_RAYS', invalid)
    return ScanResult(True, nearest)


class Controller:
    TERMINAL = {'OBSTACLE_STOP', 'GOAL_REACHED', 'SENSOR_TIMEOUT', 'SENSOR_INVALID',
                'FAULT', 'TRIAL_TIMEOUT', 'COMMAND_CONFLICT'}

    def __init__(self):
        self.state = 'WAITING'
        self.started = False

    def stop(self, reason):
        if reason not in self.TERMINAL:
            raise ValueError(reason)
        if self.state not in self.TERMINAL:
            self.state = reason

    def step(self, enabled, ready, fresh, usable, blocked, goal_reached, speed):
        if self.state in self.TERMINAL:
            return 0.0
        if not enabled:
            self.state = 'DISABLED'
            return 0.0
        if not ready or not fresh:
            if self.started:
                self.stop('SENSOR_TIMEOUT')
            else:
                self.state = 'WAITING'
            return 0.0
        if not usable:
            if self.started:
                self.stop('SENSOR_INVALID')
            else:
                self.state = 'WAITING'
            return 0.0
        if blocked:
            self.stop('OBSTACLE_STOP')
            return 0.0
        if goal_reached:
            self.stop('GOAL_REACHED')
            return 0.0
        self.state, self.started = 'RUNNING', True
        return speed
