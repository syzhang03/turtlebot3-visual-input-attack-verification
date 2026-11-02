"""ROS-independent state machine. Stops latch until process restart."""
import math


class Trial:
    STOP_STATES = ('OBSTACLE_STOP', 'GOAL_REACHED', 'SENSOR_TIMEOUT', 'FAULT')

    def __init__(self, goal=1.0, speed=0.05, timeout=2.0, enable_goal_stop=True):
        if not all(math.isfinite(v) and v > 0 for v in (goal, speed, timeout)):
            raise ValueError('goal, speed and timeout must be finite and positive')
        self.goal, self.speed, self.timeout = goal, speed, timeout
        if not isinstance(enable_goal_stop, bool):
            raise ValueError('enable_goal_stop must be a boolean')
        self.enable_goal_stop = enable_goal_stop
        self.state = 'WAITING'
        self.started = False

    def stop(self, reason):
        """Latch the first stop reason, including stops requested by ROS callbacks."""
        if reason not in self.STOP_STATES:
            raise ValueError(f'Invalid stop reason: {reason}')
        if self.state not in self.STOP_STATES:
            self.state = reason

    def step(self, enabled, ready, image_age, odom_age, blocked, distance):
        if self.state in self.STOP_STATES:
            return 0.0
        if not enabled:
            self.state = 'DISABLED'
            return 0.0
        if not ready:
            self.state = 'WAITING'
            return 0.0
        if not all(math.isfinite(v) for v in (image_age, odom_age, distance)):
            self.stop('FAULT')
            return 0.0
        if max(image_age, odom_age) > self.timeout:
            if self.started:
                self.stop('SENSOR_TIMEOUT')
            else:
                self.state = 'WAITING'
            return 0.0
        if blocked:
            self.stop('OBSTACLE_STOP')
            return 0.0
        if self.enable_goal_stop and distance >= self.goal:
            self.stop('GOAL_REACHED')
            return 0.0
        self.started = True
        self.state = 'RUNNING'
        return self.speed


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))
