"""Integrated camera nominal decisions, consistency monitor and LiDAR guard."""
import csv
import json
import math
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from tb3_lidar_baseline.node import LidarBaseline
from tb3_lidar_baseline.core import Controller
from tb3_visual_baseline.vision import detect_red
from .core import ConsistencyMonitor


class FusionController(Controller):
    TERMINAL = Controller.TERMINAL | {'ANOMALY_STOP', 'VISUAL_STOP'}


class CameraLidarMonitor(LidarBaseline):
    def __init__(self):
        super().__init__('camera_lidar_monitor')
        defaults = dict(image_topic='/attack/image_raw', image_reliability='best_effort', visual_stop_threshold=.4,
                        visual_verify_threshold=.06, roi_top=.25, verify_near=2.57,
                        verify_far=2.91, pair_tolerance=.12, pair_timeout=.6,
                        confirmations=3, calibration_mode=False, record_image_acceptance=False)
        self.visual_cfg = {k: self.declare_parameter(k, v, ParameterDescriptor(read_only=True)).value
                           for k, v in defaults.items()}
        for key in ('visual_stop_threshold', 'visual_verify_threshold', 'roi_top'):
            if not 0 < self.visual_cfg[key] < 1:
                raise ValueError(key)
        if self.visual_cfg['pair_timeout'] <= 0:
            raise ValueError('pair_timeout')
        if self.visual_cfg['image_reliability'] not in ('best_effort', 'reliable'):
            raise ValueError('image_reliability must be best_effort or reliable')
        self.monitor = ConsistencyMonitor(self.visual_cfg['verify_near'], self.visual_cfg['verify_far'],
                                          self.visual_cfg['pair_tolerance'],
                                          self.visual_cfg['confirmations'], self.visual_cfg['pair_timeout'])
        self.controller = FusionController()
        self.bridge = CvBridge()
        self.image_stamp = self.image_at = self.pair_at = None
        self.alerts = self.create_publisher(String, '/defense/consistency', 10)
        image_qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
                     if self.visual_cfg['image_reliability'] == 'reliable' else qos_profile_sensor_data)
        self.create_subscription(Image, self.visual_cfg['image_topic'], self.on_image, image_qos)
        self.timeout_diagnostic_logged = False
        self.get_logger().info(f'Image input: {self.visual_cfg["image_topic"]}; '
                               f'reliability={self.visual_cfg["image_reliability"]}')
        (self.run_dir/'visual-config.json').write_text(json.dumps(self.visual_cfg, indent=2))
        self.pair_log = (self.run_dir/'pairs.csv').open('w', newline='')
        self.pair_writer = csv.DictWriter(self.pair_log, fieldnames=[
            'image_stamp', 'scan_stamp', 'visual_present', 'clearance', 'lidar_present',
            'direction', 'count', 'alert', 'red_ratio'])
        self.pair_writer.writeheader()
        self.image_trace = ((self.run_dir/'image-acceptance.csv').open('w', newline='')
                            if self.visual_cfg['record_image_acceptance'] else None)
        if self.image_trace is not None:
            csv.writer(self.image_trace).writerow(['accepted_sim_s', 'header_stamp_ns'])
        self.ratios = {}

    def on_image(self, msg):
        stamp = self.accept_stamp(msg, self.image_stamp)
        accepted_sim_s = self.get_clock().now().nanoseconds / 1e9 if self.image_trace is not None else None
        if stamp is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            blocked, ratio, _ = detect_red(frame, self.visual_cfg['visual_stop_threshold'],
                                            self.visual_cfg['roi_top'])
        except Exception as exc:
            self.controller.stop('SENSOR_INVALID')
            self.get_logger().error(f'Image conversion/detection failed: {exc}')
            return
        self.image_stamp, self.image_at = stamp, time.monotonic()
        if self.image_trace is not None:
            source_stamp = msg.header.stamp
            csv.writer(self.image_trace).writerow([accepted_sim_s, source_stamp.sec*10**9+source_stamp.nanosec])
            self.image_trace.flush()
        self.ratios[stamp] = ratio
        if len(self.ratios) > 100:
            del self.ratios[min(self.ratios)]
        if blocked and self.cfg['enabled']:
            self.controller.stop('VISUAL_STOP')
        self.record(self.monitor.add('image', stamp, ratio >= self.visual_cfg['visual_verify_threshold']))

    def on_scan(self, msg):
        previous = self.scan_stamp
        super().on_scan(msg)
        if self.scan_stamp != previous and self.scan_result.usable:
            if msg.range_max < self.visual_cfg['verify_far'] + self.cfg['front_extent'] + .1:
                return  # Insufficient verification horizon is not agreement.
            self.record(self.monitor.add('scan', self.scan_stamp, self.scan_result.clearance))

    def record(self, pairs):
        for pair in pairs:
            self.pair_at = time.monotonic()
            pair['red_ratio'] = self.ratios.get(pair['image_stamp'])
            self.pair_writer.writerow(pair)
            self.pair_log.flush()
            message = String()
            # JSON uses null for a no-return distance, not nonstandard Infinity.
            wire = dict(pair)
            if self.image_trace is not None:
                wire['published_sim_s'] = self.get_clock().now().nanoseconds / 1e9
            if not math.isfinite(wire['clearance']):
                wire['clearance'] = None
            message.data = json.dumps(wire)
            self.alerts.publish(message)
            if pair['alert'] and self.cfg['enabled'] and not self.visual_cfg['calibration_mode']:
                self.controller.stop('ANOMALY_STOP')

    def extra_input_status(self, wall, sim):
        ready = self.image_at is not None and self.pair_at is not None
        fresh = (ready and wall-self.image_at <= self.cfg['sensor_timeout_wall']
                 and wall-self.pair_at <= self.cfg['sensor_timeout_wall']
                 and -.05 <= sim-self.image_stamp <= self.cfg['max_stamp_age_sim']
                 and self.monitor.last_pair is not None
                 and -.05 <= sim-self.monitor.last_pair <= self.visual_cfg['pair_timeout'])
        if self.controller.started and not fresh and not self.timeout_diagnostic_logged:
            snapshot = dict(sim_time_s=sim,
                image_age_wall_s=None if self.image_at is None else wall-self.image_at,
                image_age_sim_s=None if self.image_stamp is None else sim-self.image_stamp,
                pair_age_wall_s=None if self.pair_at is None else wall-self.pair_at,
                pair_age_sim_s=None if self.monitor.last_pair is None else sim-self.monitor.last_pair,
                sensor_timeout_wall=self.cfg['sensor_timeout_wall'],
                max_stamp_age_sim=self.cfg['max_stamp_age_sim'], pair_timeout=self.visual_cfg['pair_timeout'])
            self.get_logger().warning('INPUT_FRESHNESS_FAILED: ' + json.dumps(snapshot))
            self.timeout_diagnostic_logged = True
        return ready, fresh


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = CameraLidarMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                for _ in range(5):
                    node.publish()
                    time.sleep(.05)
            node.pair_log.close()
            if node.image_trace is not None:
                node.image_trace.close()
            node.log.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
