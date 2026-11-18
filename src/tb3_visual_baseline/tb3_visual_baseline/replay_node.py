"""Camera-only delayed replay relay for controlled simulation trials."""
import copy
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image

from .replay_core import DelayBuffer


class DelayReplay(Node):
    def __init__(self):
        super().__init__('delay_replay')
        defaults = dict(input_topic='/camera/image_raw', output_topic='/attack/image_raw',
                        input_reliability='best_effort', output_reliability='reliable',
                        delay_s=1.0, start_s=0.0, duration_s=60.0,
                        max_frames=300, max_buffer_bytes=268435456,
                        log_dir='~/replay-results')
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for key in ('input_reliability', 'output_reliability'):
            if self.cfg[key] not in ('best_effort', 'reliable'):
                raise ValueError(f'{key} must be best_effort or reliable')
        input_qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
                     if self.cfg['input_reliability'] == 'reliable' else qos_profile_sensor_data)
        output_qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
                      if self.cfg['output_reliability'] == 'reliable' else qos_profile_sensor_data)
        self.buffer = DelayBuffer(self.cfg['delay_s'], self.cfg['start_s'],
                                  self.cfg['duration_s'], self.cfg['max_frames'],
                                  self.cfg['max_buffer_bytes'])
        if self.resolve_topic_name(self.cfg['input_topic']) == self.resolve_topic_name(self.cfg['output_topic']):
            raise ValueError('Input and output must resolve to distinct topics')
        self.publisher = self.create_publisher(Image, self.cfg['output_topic'], output_qos)
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        run_dir = Path(self.cfg['log_dir']).expanduser() / run_id
        run_dir.mkdir(parents=True, exist_ok=False)
        (run_dir / 'config.json').write_text(json.dumps(dict(
            self.cfg, run_id=run_id, use_sim_time=self.get_parameter('use_sim_time').value,
            transport_revision='configurable-reliable-input-v2',
            time_basis='input image header stamps; schedule starts at first frame'), indent=2), encoding='utf-8')
        self.log = (run_dir / 'frames.csv').open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.log)
        self.writer.writerow(['input_stamp_ns', 'selected_source_stamp_ns', 'actual_delay_s',
                              'elapsed_s', 'state', 'buffer_reset', 'buffer_frames', 'buffer_bytes',
                              'callback_monotonic_ns', 'publish_return_monotonic_ns',
                              'input_gap_wall_s', 'prepare_s', 'publish_call_s',
                              'previous_log_write_s', 'output_subscribers'])
        self.previous_state = None
        self.previous_callback_ns = None
        self.previous_log_write_s = 0.0
        self.create_subscription(Image, self.cfg['input_topic'], self.on_image, input_qos)
        self.get_logger().info(f'Replay logs: {run_dir}')
        self.get_logger().info(f'Input reliability={self.cfg["input_reliability"]}; queue depth={input_qos.depth}')
        self.get_logger().info(f'Output reliability={self.cfg["output_reliability"]}; reliable queue depth=1')

    def on_image(self, msg):
        callback_ns = time.monotonic_ns()
        gap = ((callback_ns - self.previous_callback_ns) / 1e9
               if self.previous_callback_ns is not None else '')
        self.previous_callback_ns = callback_ns
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        layout = (msg.height, msg.width, msg.encoding, msg.is_bigendian, msg.step, msg.header.frame_id)
        result = self.buffer.push(stamp, msg, len(msg.data), layout)
        if result is None:
            return
        output = result.frame
        if result.state == 'REPLAY':
            # Never mutate a buffered message. Payload geometry/encoding are unchanged.
            output = Image()
            output.header = copy.deepcopy(msg.header)
            output.height, output.width = result.frame.height, result.frame.width
            output.encoding, output.is_bigendian = result.frame.encoding, result.frame.is_bigendian
            output.step, output.data = result.frame.step, result.frame.data
        before_publish_ns = time.monotonic_ns()
        self.publisher.publish(output)
        after_publish_ns = time.monotonic_ns()
        log_start_ns = time.monotonic_ns()
        self.writer.writerow([stamp, result.source_ns, (stamp - result.source_ns) / 1e9,
                              result.elapsed_ns / 1e9, result.state, int(result.reset),
                              len(self.buffer.frames), self.buffer.bytes,
                              callback_ns, after_publish_ns, gap,
                              (before_publish_ns - callback_ns) / 1e9,
                              (after_publish_ns - before_publish_ns) / 1e9,
                              self.previous_log_write_s, self.publisher.get_subscription_count()])
        self.log.flush()
        self.previous_log_write_s = (time.monotonic_ns() - log_start_ns) / 1e9
        if result.state != self.previous_state or result.reset:
            self.get_logger().info(f'{result.state}; reset={result.reset}; elapsed={result.elapsed_ns / 1e9:.3f}s')
            self.previous_state = result.state

    def destroy_node(self):
        if hasattr(self, 'log'):
            self.log.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DelayReplay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
