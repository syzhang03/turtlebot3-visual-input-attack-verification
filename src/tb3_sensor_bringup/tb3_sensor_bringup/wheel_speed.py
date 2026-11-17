"""Convert wheel joint velocities to a dedicated /wheel_speed topic.

The TurtleBot3 Gazebo model publishes ``sensor_msgs/msg/JointState`` on
``/joint_states`` for ``wheel_left_joint`` and ``wheel_right_joint``.  This
node keeps the latest wheel sample and republishes it at ``publish_rate`` on
``/wheel_speed`` with the original header stamp.  Velocity is in rad/s;
multiply by ``wheel_radius`` (default 0.033 m) for m/s.

The Gazebo joint-state plugin can publish at the simulation step rate, so
throttling here keeps the four-sensor recorder from receiving unnecessary
1 kHz wheel messages.  Repeating a stamp is harmless: the sync buffer ignores
duplicate stamps and treats a stopped source as stale.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .core import wheel_speed_from_joint_state


class WheelSpeed(Node):
    def __init__(self):
        super().__init__('wheel_speed')
        defaults = dict(
            joint_states_topic='/joint_states',
            wheel_speed_topic='/wheel_speed',
            left_joint='wheel_left_joint',
            right_joint='wheel_right_joint',
            wheel_radius=0.033,
            publish_rate=30.0,
        )
        self.cfg = {key: self.declare_parameter(key, value).value
                    for key, value in defaults.items()}
        if not 0 < self.cfg['wheel_radius'] < 1:
            raise ValueError('wheel_radius must be between 0 and 1 m')
        if not 0 < self.cfg['publish_rate'] <= 200:
            raise ValueError('publish_rate must be between 0 and 200 Hz')
        self.publisher = self.create_publisher(
            JointState, self.cfg['wheel_speed_topic'], 10)
        self.create_subscription(
            JointState, self.cfg['joint_states_topic'], self.on_joint_state, 10)
        self.create_timer(1.0 / self.cfg['publish_rate'], self.publish_latest)
        self.latest = None
        self.received = 0
        self.published = 0
        self.rejected = 0
        self.get_logger().info(
            f"wheel_speed: {self.cfg['joint_states_topic']} -> "
            f"{self.cfg['wheel_speed_topic']}; radius={self.cfg['wheel_radius']} m; "
            f"rate={self.cfg['publish_rate']} Hz; "
            f"joints={self.cfg['left_joint']},{self.cfg['right_joint']}")

    def on_joint_state(self, msg):
        self.received += 1
        try:
            speed = wheel_speed_from_joint_state(
                msg.name, msg.velocity, self.cfg['wheel_radius'],
                self.cfg['left_joint'], self.cfg['right_joint'])
        except (KeyError, ValueError) as exc:
            self.rejected += 1
            self.get_logger().warning(
                f'Rejected JointState: {exc}', throttle_duration_sec=2.0)
            return
        self.latest = (msg.header, speed)

    def publish_latest(self):
        if self.latest is None:
            return
        header, speed = self.latest
        out = JointState()
        out.header = header
        out.name = [self.cfg['left_joint'], self.cfg['right_joint']]
        out.velocity = [speed.left_rad_s, speed.right_rad_s]
        self.publisher.publish(out)
        self.published += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelSpeed()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.get_logger().info(
                f'wheel_speed stopped: received={node.received}, '
                f'published={node.published}, rejected={node.rejected}')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
