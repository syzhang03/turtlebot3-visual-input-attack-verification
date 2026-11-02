import unittest

from tb3_visual_baseline.replay_core import DelayBuffer


class ReplayTests(unittest.TestCase):
    def push(self, buffer, seconds, size=10, layout='rgb'):
        return buffer.push(round(seconds * 1e9), f'frame-{seconds}', size, layout)

    def test_zero_delay_is_identity(self):
        b = DelayBuffer(delay_s=0)
        r = self.push(b, 10)
        self.assertEqual((r.state, r.frame, r.source_ns), ('ZERO_DELAY', 'frame-10', 10000000000))

    def test_warmup_and_exact_delay(self):
        b = DelayBuffer(delay_s=1)
        self.assertEqual(self.push(b, 5).state, 'BUFFER_NOT_READY')
        self.push(b, 5.5)
        r = self.push(b, 6)
        self.assertEqual((r.state, r.frame), ('REPLAY', 'frame-5'))

    def test_irregular_sampling_uses_latest_not_after_target(self):
        b = DelayBuffer(delay_s=1)
        for t in (0, .4, .9):
            self.push(b, t)
        r = self.push(b, 1.6)
        self.assertEqual(r.source_ns, 400000000)
        self.assertAlmostEqual((1600000000 - r.source_ns) / 1e9, 1.2)

    def test_schedule_start_inclusive_end_exclusive(self):
        b = DelayBuffer(delay_s=1, start_s=2, duration_s=2)
        self.assertEqual(self.push(b, 10).state, 'BEFORE_ATTACK')
        self.push(b, 11)
        self.assertEqual(self.push(b, 12).state, 'REPLAY')
        r = self.push(b, 14)
        self.assertEqual((r.state, r.frame), ('AFTER_ATTACK', 'frame-14'))

    def test_duplicate_does_not_publish_or_accumulate(self):
        b = DelayBuffer()
        self.push(b, 1)
        self.assertIsNone(self.push(b, 1))
        self.assertEqual(len(b.frames), 1)

    def test_time_reset_clears_history_and_schedule(self):
        b = DelayBuffer()
        self.push(b, 10)
        self.push(b, 11)
        r = self.push(b, 0)
        self.assertTrue(r.reset)
        self.assertEqual((r.elapsed_ns, r.state, len(b.frames)), (0, 'BUFFER_NOT_READY', 1))

    def test_format_change_clears_incompatible_frames(self):
        b = DelayBuffer()
        self.push(b, 0)
        r = self.push(b, 1, layout='mono')
        self.assertTrue(r.reset)
        self.assertEqual(r.state, 'BUFFER_NOT_READY')

    def test_frame_limit_falls_back_honestly(self):
        b = DelayBuffer(delay_s=2, max_frames=2)
        for t in (0, .5, 1, 1.5, 2):
            r = self.push(b, t)
        self.assertEqual(r.state, 'BUFFER_NOT_READY')
        self.assertEqual(len(b.frames), 2)

    def test_byte_limit_and_oversized_frame(self):
        b = DelayBuffer(max_bytes=15)
        self.push(b, 0)
        r = self.push(b, 1)
        self.assertEqual(r.state, 'BUFFER_NOT_READY')
        self.assertLessEqual(b.bytes, 15)
        r = self.push(b, 2, size=20)
        self.assertEqual((r.state, b.bytes), ('BUFFER_NOT_READY', 0))

    def test_history_is_bounded_over_long_run(self):
        b = DelayBuffer(delay_s=1)
        for i in range(10000):
            self.push(b, i / 10)
        self.assertLessEqual(len(b.frames), 11)
        self.assertEqual(b.bytes, len(b.frames) * 10)

    def test_invalid_parameters(self):
        for kwargs in ({'delay_s': -1}, {'delay_s': float('nan')},
                       {'start_s': float('inf')}, {'duration_s': 0},
                       {'max_frames': 0}, {'max_bytes': 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                DelayBuffer(**kwargs)


if __name__ == '__main__':
    unittest.main()
