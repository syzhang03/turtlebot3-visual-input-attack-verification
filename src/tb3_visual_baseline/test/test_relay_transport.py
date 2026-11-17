"""ROS message boundary tests; no simulator or velocity publisher is started."""
import csv
import io
from types import SimpleNamespace
import unittest

from tb3_visual_baseline.replay_core import DelayBuffer

try:
    from sensor_msgs.msg import Image
    from tb3_visual_baseline.replay_node import DelayReplay
except ModuleNotFoundError:
    Image = DelayReplay = None


@unittest.skipIf(Image is None, 'ROS 2 message libraries are not available')
class RelayTransportTests(unittest.TestCase):
    def fake_relay(self, delay):
        published = []
        log = io.StringIO()
        relay = SimpleNamespace(
            buffer=DelayBuffer(delay_s=delay), previous_callback_ns=None,
            previous_log_write_s=0.0, previous_state=None,
            publisher=SimpleNamespace(publish=published.append, get_subscription_count=lambda: 1),
            log=log, writer=csv.writer(log),
            get_logger=lambda: SimpleNamespace(info=lambda _: None),
        )
        return relay, published, log

    def frame(self, seconds, color):
        msg = Image()
        msg.header.stamp.sec = seconds
        msg.header.frame_id = 'camera_frame'
        msg.height, msg.width, msg.step = 480, 640, 1920
        msg.encoding = 'rgb8'
        msg.data = bytes([color]) * (480 * 1920)
        return msg

    def test_zero_delay_preserves_full_image_and_header(self):
        relay, published, log = self.fake_relay(0)
        msg = self.frame(10, 25)
        DelayReplay.on_image(relay, msg)
        self.assertIs(published[0], msg)
        self.assertEqual(bytes(published[0].data), bytes([25]) * 921600)
        fields = next(csv.reader(io.StringIO(log.getvalue())))
        self.assertEqual(fields[4], 'ZERO_DELAY')
        self.assertGreaterEqual(int(fields[9]), int(fields[8]))
        self.assertGreaterEqual(float(fields[12]), 0)

    def test_delayed_output_uses_current_header_without_mutating_history(self):
        relay, published, _ = self.fake_relay(1)
        old, current = self.frame(10, 10), self.frame(11, 20)
        DelayReplay.on_image(relay, old)
        DelayReplay.on_image(relay, current)
        output = published[-1]
        self.assertEqual(output.header.stamp.sec, 11)
        self.assertEqual(output.header.frame_id, 'camera_frame')
        self.assertEqual(bytes(output.data), bytes(old.data))
        self.assertEqual(old.header.stamp.sec, 10)
        self.assertEqual(bytes(current.data), bytes([20]) * 921600)
        self.assertIsNot(output.header, current.header)


if __name__ == '__main__':
    unittest.main()
