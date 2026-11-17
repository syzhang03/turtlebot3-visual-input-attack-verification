"""ROS 2 image relay. This node has no robot command or LiDAR interfaces."""

from cv_bridge import CvBridge
import json
from pathlib import Path
from datetime import datetime, timezone
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

from .occlusion import DEFAULT_RECT, validate_rectangle
from .relay import process_image
from .perturbation import DEFAULTS, validate_settings, active


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
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        for name, value in dict(input_reliability='best_effort', output_reliability='reliable',
                                input_history='keep_last', output_history='keep_last',
                                input_depth=5, output_depth=1, reject_competing_publishers=True,
                                log_dir='').items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))

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
        if self.resolve_topic_name(target).endswith('/cmd_vel'):
            raise ValueError('Camera attack output cannot be a robot command topic')
        # Resolve remappings too: a relay feeding itself would form a feedback loop.
        if self.resolve_topic_name(source) == self.resolve_topic_name(target):
            self.destroy_node()
            raise ValueError('input_topic and output_topic must resolve to different topics')
        self.bridge = CvBridge()
        validate_settings(self.settings())
        def qos(prefix):
            reliability = self.get_parameter(prefix + '_reliability').value
            history = self.get_parameter(prefix + '_history').value
            depth = self.get_parameter(prefix + '_depth').value
            if reliability not in ('best_effort', 'reliable') or history not in ('keep_last', 'keep_all'):
                raise ValueError('Invalid QoS reliability/history')
            if type(depth) is not int or not 1 <= depth <= 1000:
                raise ValueError('QoS depth must be in [1,1000]')
            return QoSProfile(depth=depth, reliability=(ReliabilityPolicy.RELIABLE
                if reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT),
                history=(HistoryPolicy.KEEP_LAST if history == 'keep_last' else HistoryPolicy.KEEP_ALL),
                durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Image, target, qos('output'))
        self.subscription = self.create_subscription(
            Image, source, self.on_image, qos('input'))
        self.first_stamp = None
        self.last_stamp = None
        self.seed = None
        self.rng = None
        self.frame_log = None
        directory = self.get_parameter('log_dir').value
        if directory:
            run = Path(directory).expanduser() / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
            run.mkdir(parents=True, exist_ok=False)
            (run / 'config.json').write_text(json.dumps({
                k: self.get_parameter(k).value for k in
                list(DEFAULTS) + list(DEFAULT_RECT) + ['input_topic', 'output_topic',
                'input_reliability', 'output_reliability', 'input_history', 'output_history',
                'input_depth', 'output_depth', 'attack_enabled']}, indent=2))
            self.frame_log = (run / 'frames.jsonl').open('w')
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.get_logger().info(
            f'{source} -> {target}; attack_enabled={self.get_parameter("attack_enabled").value}; '
            f'mask={self.rectangle()}')

    def rectangle(self):
        return {name: self.get_parameter(name).value for name in DEFAULT_RECT}

    def settings(self):
        return {name: self.get_parameter(name).value for name in (*DEFAULT_RECT, *DEFAULTS)}

    def validate_parameters(self, parameters):
        settings = self.settings()
        for parameter in parameters:
            if parameter.name in DEFAULT_RECT:
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False, reason='Mask parameters require floating-point values')
                settings[parameter.name] = parameter.value
            elif parameter.name in DEFAULTS:
                settings[parameter.name] = parameter.value
            elif parameter.name == 'attack_enabled':
                if parameter.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(successful=False, reason='attack_enabled requires bool')
        try:
            validate_settings(settings)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def on_image(self, message):
        try:
            if (self.get_parameter('reject_competing_publishers').value
                    and self.count_publishers(self.publisher.topic_name) > 1):
                self.get_logger().error('Competing image publisher detected; dropping frame')
                return
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            # Time origin is the first input frame, including clean frames. Restart after a scene reset.
            if self.last_stamp is not None and stamp < self.last_stamp:
                self.get_logger().error('Backward image timestamp; restart relay after scene reset')
                return
            self.last_stamp = stamp
            if self.first_stamp is None:
                self.first_stamp = stamp
            settings = self.settings()
            enabled = self.get_parameter('attack_enabled').value and active(
                settings, stamp - self.first_stamp)
            if self.seed != settings['random_seed']:
                self.seed = settings['random_seed']
                self.rng = np.random.default_rng(self.seed)
            output = process_image(
                message, self.bridge,
                attack_enabled=enabled, settings=settings, rng=self.rng)
        except Exception as exc:
            # Do not fabricate a fresh timestamp or silently substitute a clean frame.
            self.get_logger().error(f'Image processing failed; frame dropped: {exc}')
            return
        self.publisher.publish(output)
        if self.frame_log:
            self.frame_log.write(json.dumps(dict(stamp_sec=message.header.stamp.sec,
                stamp_nanosec=message.header.stamp.nanosec, frame_id=message.header.frame_id,
                attack_active=enabled, settings=settings, encoding=output.encoding)) + '\n')
            self.frame_log.flush()

    def destroy_node(self):
        if getattr(self, 'frame_log', None):
            self.frame_log.close()
        return super().destroy_node()


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
