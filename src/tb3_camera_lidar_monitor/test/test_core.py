import math
import unittest
from tb3_camera_lidar_monitor.core import ConsistencyMonitor


class PairTests(unittest.TestCase):
    def setUp(self):
        self.m = ConsistencyMonitor(2.3, 2.7)

    def pair(self, t, visual=False, distance=2.0):
        self.m.add('scan', t, distance)
        return self.m.add('image', t+.01, visual)[0]

    def test_three_distinct_pairs_required(self):
        self.assertFalse(self.pair(1)['alert'])
        for _ in range(20):
            self.assertEqual(self.m.add('image', 1.01, False), [])
        self.assertEqual(self.m.count, 1)
        self.assertFalse(self.pair(1.2)['alert'])
        self.assertTrue(self.pair(1.4)['alert'])

    def test_ambiguity_and_agreement_reset_evidence(self):
        self.pair(1)
        self.assertEqual(self.pair(1.2, distance=2.5)['direction'], 'AMBIGUOUS')
        self.assertEqual(self.pair(1.4)['count'], 1)
        self.assertEqual(self.pair(1.6, visual=True)['direction'], 'AGREE')
        self.assertEqual(self.m.count, 0)

    def test_direction_flip_and_long_gap_restart(self):
        self.pair(1)
        self.assertEqual(self.pair(1.2, visual=True, distance=math.inf)['direction'], 'VISUAL_EXTRA')
        self.assertEqual(self.m.count, 1)
        self.assertEqual(self.pair(2, visual=True, distance=math.inf)['count'], 1)

    def test_unmatched_and_out_of_order_data(self):
        self.m.add('scan', 1, 2.)
        self.assertEqual(self.m.add('image', 2, False), [])
        self.assertEqual(self.m.count, 0)
        with self.assertRaises(ValueError):
            self.m.add('image', 1, False)

    def test_pair_is_consumed_once_and_alert_latches(self):
        for t in (1,1.2,1.4): self.pair(t)
        self.assertTrue(self.pair(1.6, visual=True)['alert'])
        self.assertEqual(self.m.add('scan', 1.6, 2.), [])

    def test_invalid_configuration_and_scan(self):
        with self.assertRaises(ValueError): ConsistencyMonitor(3,2)
        with self.assertRaises(ValueError): self.m.add('scan', 1, math.nan)
