"""Scientific invariants for the range-bias injection (no ROS required)."""
import math
import unittest

from tb3_lidar_attack.core import RangeBias
from tb3_lidar_attack.experiment import condition


class RangeBiasTests(unittest.TestCase):
    def scan(self, attack, seconds=0, values=None, **geometry):
        defaults = dict(angle_min=-math.pi/2, angle_increment=math.pi/6, range_min=.1, range_max=10.)
        defaults.update(geometry)
        return attack.process(round(seconds*1e9), values if values is not None else [2.]*7, **defaults)

    def test_zero_bias_identity_including_invalid_and_no_returns(self):
        source = [math.nan, math.inf, -math.inf, -.1, 0., 2., 11.]
        result = self.scan(RangeBias(bias_m=0.), values=source)
        self.assertEqual(result.state, 'PASSTHROUGH')
        self.assertEqual(result.changed_rays, 0)
        self.assertTrue(math.isnan(result.ranges[0]))
        self.assertEqual(result.ranges[1:], tuple(source[1:]))

    def test_only_selected_sector_changes_and_input_is_untouched(self):
        source = [2.]*7
        result = self.scan(RangeBias(), values=source)
        self.assertEqual(result.ranges, (2., 2., 3., 3., 3., 2., 2.))
        self.assertEqual(source, [2.]*7)
        self.assertEqual((result.eligible_rays, result.changed_rays, result.clipped_rays), (3, 3, 0))
        self.assertEqual((result.min_delta_m, result.max_delta_m), (1., 1.))

    def test_wraparound_and_reversed_scan_have_same_sector(self):
        for angle, increment in ((0., math.pi/6), (2*math.pi, -math.pi/6)):
            result = self.scan(RangeBias(), values=[2.]*12, angle_min=angle, angle_increment=increment)
            changed = [i for i, value in enumerate(result.ranges) if value == 3.]
            self.assertEqual(changed, [0, 1, 11])

    def test_off_axis_sector_is_explicitly_in_scan_frame(self):
        result = self.scan(RangeBias(sector_center_deg=90, sector_half_width_deg=15))
        self.assertEqual(result.ranges, (2., 2., 2., 2., 2., 2., 3.))

    def test_clamping_preserves_max_range_without_creating_infinity(self):
        result = self.scan(RangeBias(bias_m=2., sector_half_width_deg=180), values=[.1, 9., 10.],
                           angle_increment=math.pi/2)
        self.assertEqual(result.ranges, (2.1, 10., 10.))
        self.assertEqual((result.changed_rays, result.clipped_rays), (2, 2))
        self.assertEqual((result.min_delta_m, result.max_delta_m), (1., 2.))

    def test_active_attack_does_not_sanitize_invalid_measurements(self):
        source = [math.nan, math.inf, -math.inf, -.1, 0., 2., 11.]
        result = self.scan(RangeBias(sector_half_width_deg=180), values=source)
        self.assertTrue(math.isnan(result.ranges[0]))
        self.assertEqual(result.ranges[1:], (math.inf, -math.inf, -.1, 0., 3., 11.))
        self.assertEqual(result.invalid_rays, 5)

    def test_schedule_start_inclusive_end_exclusive(self):
        attack = RangeBias(start_s=1., duration_s=2.)
        self.assertEqual(self.scan(attack, 10).state, 'BEFORE_WINDOW')
        self.assertEqual(self.scan(attack, 11).state, 'BIAS_ACTIVE')
        self.assertEqual(self.scan(attack, 12.999).state, 'BIAS_ACTIVE')
        result = self.scan(attack, 13)
        self.assertEqual(result.state, 'AFTER_WINDOW')
        self.assertEqual(result.changed_rays, 0)

    def test_duplicates_ignored_and_time_reset_restarts_schedule(self):
        attack = RangeBias(start_s=1.)
        self.scan(attack, 10)
        self.assertIsNone(self.scan(attack, 10))
        self.assertEqual(attack.duplicates, 1)
        self.assertEqual(self.scan(attack, 12).state, 'BIAS_ACTIVE')
        result = self.scan(attack, 1)
        self.assertTrue(result.reset)
        self.assertEqual((result.elapsed_s, result.state), (0., 'BEFORE_WINDOW'))

    def test_invalid_metadata_is_passed_through_for_controller_to_reject(self):
        for geometry in (dict(angle_increment=0.), dict(angle_min=math.nan),
                         dict(range_min=10.), dict(range_max=math.inf)):
            with self.subTest(geometry=geometry):
                result = self.scan(RangeBias(), **geometry)
                self.assertEqual(result.state, 'INVALID_METADATA')
                self.assertEqual(result.ranges, (2.,)*7)

    def test_no_eligible_returns_does_not_claim_modified_rays(self):
        result = self.scan(RangeBias(), values=[math.inf]*7)
        self.assertEqual(result.state, 'BIAS_ACTIVE')
        self.assertEqual(result.changed_rays, 0)
        self.assertIsNone(result.input_sector_min_m)

    def test_invalid_parameters_are_rejected(self):
        for kwargs in (dict(bias_m=-1), dict(bias_m=math.nan), dict(sector_center_deg=math.inf),
                       dict(sector_half_width_deg=0), dict(sector_half_width_deg=181),
                       dict(start_s=-1), dict(duration_s=0), dict(duration_s=1e-12)):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                RangeBias(**kwargs)


class ConditionTests(unittest.TestCase):
    def test_four_conditions_hold_attack_strength_constant(self):
        for mode, expected in [('clean', (0., 0.)), ('camera', (1., 0.)),
                               ('lidar', (0., 1.)), ('joint', (1., 1.))]:
            cfg = condition(mode)['effective']
            self.assertEqual((cfg['camera_delay_s'], cfg['lidar_bias_m']), expected)

    def test_shell_numbers_are_explicitly_converted_to_doubles(self):
        cfg = condition('clean', '0', '0')['effective']
        self.assertIsInstance(cfg['camera_delay_s'], float)
        self.assertIsInstance(cfg['lidar_bias_m'], float)

    def test_reject_mislabeled_or_mistyped_experiments(self):
        for args in [('joint', '1.0.0'), ('joint', 0.), ('joint', 1., 0.),
                     ('camera', math.inf), ('typo',), ('clean', -1.)]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                condition(*args)


if __name__ == '__main__':
    unittest.main()
