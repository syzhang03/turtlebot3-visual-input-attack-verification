"""ROS 2 image relay. This node has no robot command or LiDAR interfaces."""

from cv_bridge import CvBridge
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .occlusion import DEFAULT_RECT, validate_rectangle
from .relay import process_image


class CameraOcclusion(Node):
    def __init__(self, **kwargs):
        super().__init__('camera_occlusion', **kwargs)
        self.declare_parameter('input_topic', '/camera/image_raw',
                               ParameterDescriptor(read_only=True))
        self.declare_parameter('output_topic', '/camera/attacked_image',
                               ParameterDescriptor(read_only=True))
        self.declare_parameter('attack_enabled', False)
        for name, value in DEFAULT_RECT.items():
            self.declare_parameter(name, value)

        rectangle = self.rectangle()
        try:
            validate_rectangle(**rectangle)
        except ValueError as exc:
            self.get_logger().warning(f'Invalid startup mask: {exc}; using detector ROI defaults')
            self.set_parameters([
                Parameter(name, value=value)
                for name, value in DEFAULT_RECT.items()
            ])

        source = self.get_parameter('input_topic').value
        target = self.get_parameter('output_topic').value
        # Resolve remappings too: a relay feeding itself would form a feedback loop.
        if self.resolve_topic_name(source) == self.resolve_topic_name(target):
            self.destroy_node()
            raise ValueError('input_topic and output_topic must resolve to different topics')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, target, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, source, self.on_image, qos_profile_sensor_data)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.get_logger().info(
            f'{source} -> {target}; attack_enabled={self.get_parameter("attack_enabled").value}; '
            f'mask={self.rectangle()}')

    def rectangle(self):
        return {name: self.get_parameter(name).value for name in DEFAULT_RECT}

    def validate_parameters(self, parameters):
        rectangle = self.rectangle()
        for parameter in parameters:
            if parameter.name in rectangle:
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False, reason='Mask parameters require floating-point values')
                rectangle[parameter.name] = parameter.value
            elif parameter.name == 'attack_enabled':
                if parameter.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(successful=False, reason='attack_enabled requires bool')
        try:
            validate_rectangle(**rectangle)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def on_image(self, message):
        try:
            output = process_image(
                message, self.bridge,
                attack_enabled=self.get_parameter('attack_enabled').value,
                **self.rectangle())
        except Exception as exc:
            # Do not fabricate a fresh timestamp or silently substitute a clean frame.
            self.get_logger().error(f'Image processing failed; frame dropped: {exc}')
            return
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraOcclusion()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
