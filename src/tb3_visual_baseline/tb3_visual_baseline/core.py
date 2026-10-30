"""ROS-independent state machine. Stops latch until process restart."""
import math


class Trial:
    def __init__(self, goal=1.0, speed=0.05, timeout=2.0):
        if not all(math.isfinite(v) and v > 0 for v in (goal, speed, timeout)):
            raise ValueError('goal, speed and timeout must be finite and positive')
        self.goal, self.speed, self.timeout = goal, speed, timeout
        self.state = 'WAITING'
        self.started = False

    def step(self, enabled, ready, image_age, odom_age, blocked, distance):
        if self.state in ('OBSTACLE_STOP', 'GOAL_REACHED', 'SENSOR_TIMEOUT', 'FAULT'):
            return 0.0
        if not enabled:
            self.state = 'DISABLED'
            return 0.0
        if not ready:
            self.state = 'WAITING'
            return 0.0
        if not all(math.isfinite(v) for v in (image_age, odom_age, distance)):
            self.state = 'FAULT'
            return 0.0
        if max(image_age, odom_age) > self.timeout:
            self.state = 'SENSOR_TIMEOUT' if self.started else 'WAITING'
            return 0.0
        if blocked:
            self.state = 'OBSTACLE_STOP'
            return 0.0
        if distance >= self.goal:
            self.state = 'GOAL_REACHED'
            return 0.0
        self.started = True
        self.state = 'RUNNING'
        return self.speed


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))
