"""Write one synchronised CSV row from camera, LiDAR, IMU and wheel speed.

The node keeps the latest sample of each sensor and emits a row when all four
stamps are within ``sync_tolerance`` and the newest sample is not older than
``staleness``.  It stores image metadata only; use ``ros2 bag record`` for the
full image stream.  This is a recording/diagnostic node, not a controller.
"""
from datetime import datetime, timezone
import csv
import json
import math
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from std_msgs.msg import String

from .core import SensorBuffer, wheel_speed_from_joint_state


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SensorSync(Node):
    FIELDS = [
        'stamp', 'image_stamp', 'image_width', 'image_height', 'image_encoding',
        'scan_stamp', 'scan_min_forward', 'scan_range_max',
        'imu_stamp', 'imu_ax', 'imu_ay', 'imu_az', 'imu_gx', 'imu_gy', 'imu_gz',
        'imu_qx', 'imu_qy', 'imu_qz', 'imu_qw',
        'wheel_stamp', 'left_rad_s', 'right_rad_s', 'left_m_s', 'right_m_s',
        'sync_span_s',
    ]

    def __init__(self):
        super().__init__('sensor_sync')
        defaults = dict(
            image_topic='/camera/image_raw',
            scan_topic='/scan',
            imu_topic='/imu',
            wheel_speed_topic='/wheel_speed',
            status_topic='/sensor_sync/status',
            left_joint='wheel_left_joint',
            right_joint='wheel_right_joint',
            wheel_radius=0.033,
            sync_tolerance=0.20,
            staleness=0.50,
            rate=20.0,
            output_dir='~/sensor-results',
        )
        self.cfg = {key: self.declare_parameter(key, value).value
                    for key, value in defaults.items()}
        if not 0 < self.cfg['wheel_radius'] < 1:
            raise ValueError('wheel_radius must be between 0 and 1 m')
        if self.cfg['sync_tolerance'] <= 0 or self.cfg['staleness'] <= 0:
            raise ValueError('sync_tolerance and staleness must be positive')
        if self.cfg['rate'] <= 0:
            raise ValueError('rate must be positive')

        self.buffer = SensorBuffer()
        self.rows = 0
        self.skipped = 0
        self.last_status = 0.0
        self.last_span = None
        self.run_dir = (Path(os.path.expanduser(self.cfg['output_dir']))
                        / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
        self.run_dir.mkdir(parents=True, exist_ok=False)
        (self.run_dir / 'config.json').write_text(
            json.dumps(self.cfg, indent=2) + '\n')
        self.csv = (self.run_dir / 'sensors.csv').open('w', newline='')
        self.writer = csv.DictWriter(self.csv, fieldnames=self.FIELDS)
        self.writer.writeheader()

        self.status = self.create_publisher(String, self.cfg['status_topic'], 10)
        self.create_subscription(Image, self.cfg['image_topic'], self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.cfg['scan_topic'], self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, self.cfg['imu_topic'], self.on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, self.cfg['wheel_speed_topic'],
                                 self.on_wheel, 10)
        self.timer = self.create_timer(1.0 / self.cfg['rate'], self.on_timer)
        self.get_logger().info(
            f"Sensor sync -> {self.run_dir}/sensors.csv; "
            f"tolerance={self.cfg['sync_tolerance']} s; "
            f"staleness={self.cfg['staleness']} s; rate={self.cfg['rate']} Hz")

    def on_image(self, msg):
        self.buffer.update('image', self.stamp(msg), {
            'width': msg.width, 'height': msg.height, 'encoding': msg.encoding})

    def on_scan(self, msg):
        minimum = math.inf
        if len(msg.ranges) >= 2 and math.isfinite(msg.angle_increment):
            for index, distance in enumerate(msg.ranges):
                angle = wrap(msg.angle_min + index * msg.angle_increment)
                if abs(angle) > math.radians(30):
                    continue
                if math.isfinite(distance) and distance >= msg.range_min:
                    minimum = min(minimum, distance)
        self.buffer.update('scan', self.stamp(msg), {
            'min_forward': minimum, 'range_max': msg.range_max})

    def on_imu(self, msg):
        self.buffer.update('imu', self.stamp(msg), {
            'ax': msg.linear_acceleration.x, 'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z, 'gx': msg.angular_velocity.x,
            'gy': msg.angular_velocity.y, 'gz': msg.angular_velocity.z,
            'qx': msg.orientation.x, 'qy': msg.orientation.y,
            'qz': msg.orientation.z, 'qw': msg.orientation.w})

    def on_wheel(self, msg):
        try:
            speed = wheel_speed_from_joint_state(
                msg.name, msg.velocity, self.cfg['wheel_radius'],
                self.cfg['left_joint'], self.cfg['right_joint'])
        except (KeyError, ValueError):
            return
        self.buffer.update('wheel', self.stamp(msg), {
            'left_rad_s': speed.left_rad_s, 'right_rad_s': speed.right_rad_s,
            'left_m_s': speed.left_m_s, 'right_m_s': speed.right_m_s})

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        snapshot = self.buffer.ready(
            now, self.cfg['sync_tolerance'], self.cfg['staleness'])
        if snapshot is None:
            self.skipped += 1
        else:
            image = snapshot.payloads['image']
            scan = snapshot.payloads['scan']
            imu = snapshot.payloads['imu']
            wheel = snapshot.payloads['wheel']
            row = {
                'stamp': now,
                'image_stamp': snapshot.stamps['image'],
                'image_width': image['width'], 'image_height': image['height'],
                'image_encoding': image['encoding'],
                'scan_stamp': snapshot.stamps['scan'],
                'scan_min_forward': scan['min_forward'] if math.isfinite(scan['min_forward']) else None,
                'scan_range_max': scan['range_max'],
                'imu_stamp': snapshot.stamps['imu'],
                'imu_ax': imu['ax'], 'imu_ay': imu['ay'], 'imu_az': imu['az'],
                'imu_gx': imu['gx'], 'imu_gy': imu['gy'], 'imu_gz': imu['gz'],
                'imu_qx': imu['qx'], 'imu_qy': imu['qy'], 'imu_qz': imu['qz'],
                'imu_qw': imu['qw'],
                'wheel_stamp': snapshot.stamps['wheel'],
                'left_rad_s': wheel['left_rad_s'], 'right_rad_s': wheel['right_rad_s'],
                'left_m_s': wheel['left_m_s'], 'right_m_s': wheel['right_m_s'],
                'sync_span_s': snapshot.span_s,
            }
            self.writer.writerow(row)
            self.csv.flush()
            self.rows += 1
            self.last_span = snapshot.span_s
        if now - self.last_status >= 1.0 or now < self.last_status:
            self.last_status = now
            status = String()
            status.data = json.dumps({
                'rows': self.rows, 'skipped': self.skipped,
                'missing': self.buffer.missing(), 'span_s': self.last_span,
                'stamps': snapshot.stamps if snapshot is not None else None,
            })
            self.status.publish(status)

    def close(self):
        self.csv.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SensorSync()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.get_logger().info(
                f'Sensor sync stopped: rows={node.rows}, skipped={node.skipped}, '
                f'dir={node.run_dir}')
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
