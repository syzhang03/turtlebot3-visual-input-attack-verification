"""Exercise the actual defense cores with attacked data, without a simulator.

These constructed inputs check plumbing/semantics, not physical attack success.
"""
import math
import unittest

from tb3_lidar_attack.core import RangeBias
from tb3_lidar_baseline.core import Controller, corridor_scan, stopping_distance
from tb3_camera_lidar_monitor.core import ConsistencyMonitor


class DefenseIntegrationTests(unittest.TestCase):
    def clearance(self, distance, bias):
        ranges = [math.inf]*361
        ranges[180] = distance
        result = RangeBias(bias_m=bias).process(1_000_000_000, ranges, -math.pi, math.pi/180, .1, 10.)
        return corridor_scan(result.ranges, -math.pi, math.pi/180, .1, 10.,
                             0., 0., 0., .2, .15, 1.15)

    def test_positive_bias_reaches_existing_corridor_computation(self):
        clean, biased = self.clearance(2.5, 0.), self.clearance(2.5, 1.)
        self.assertTrue(clean.usable and biased.usable)
        self.assertAlmostEqual(clean.clearance, 2.35)
        self.assertAlmostEqual(biased.clearance, 3.35)

    def test_unchanged_guard_still_stops_when_biased_range_is_close(self):
        reading = self.clearance(.5, .5)
        threshold = stopping_distance(1., .35, .3, 1.)
        controller = Controller()
        speed = controller.step(True, True, True, reading.usable,
                                reading.clearance <= threshold, False, 1.)
        self.assertEqual((controller.state, speed), ('OBSTACLE_STOP', 0.))

    def test_lidar_only_disagreement_can_still_be_detected(self):
        monitor = ConsistencyMonitor(2.57, 2.91)
        clearance = self.clearance(2.5, 1.).clearance
        for t in (1., 1.2, 1.4):
            monitor.add('scan', t, clearance)
            pair = monitor.add('image', t, True)[0]
        self.assertTrue(pair['alert'])
        self.assertEqual(pair['direction'], 'VISUAL_EXTRA')


if __name__ == '__main__':
    unittest.main()
