import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('calibrate_stop_point',
    Path(__file__).resolve().parents[1]/'scripts/calibrate_stop_point.py')
calibration = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibration)


def pair(r, d, t, offset=.03):
    return dict(red_ratio=r, clearance=d, image_stamp=t, scan_stamp=t+offset)


class CalibrationTests(unittest.TestCase):
    def test_interpolates_crossing_not_final_stationary_distance(self):
        fit = calibration.fit_crossing([pair(.38, .8, 10.), pair(.42, .76, 10.1),
                                        pair(.6, .5, 10.2)], .4, .3, .06)
        self.assertEqual(fit['critical_distance_m'], .78)
        self.assertLess(fit['verify_near'], .76)
        self.assertGreater(fit['verify_far'], .8)

    def test_rejects_missing_bracket_gaps_bad_pairs_and_nonapproach(self):
        cases = [[pair(.42, .76, 10.)], [pair(.38, .8, 10.)],
                 [pair(.38, .8, 10.), pair(.42, .76, 11.)],
                 [pair(.38, .8, 10., .1), pair(.42, .76, 10.1)],
                 [pair(.38, .7, 10.), pair(.42, .76, 10.1)],
                 [pair(.38, float('inf'), 10.), pair(.42, .76, 10.1)],
                 [pair(.38, .8, 10.), pair(.42, .76, 10.)]]
        for case in cases:
            with self.subTest(case=case), self.assertRaises(ValueError):
                calibration.fit_crossing(case, .4, .3, .06)

    def write_trial(self, root, *, scan='/scan', stop='VISUAL_STOP', stopped=True):
        folder = root/'controller/run'
        folder.mkdir(parents=True, exist_ok=True)
        cfg = dict(enabled=True, scan_topic=scan, speed=.3, enable_goal_stop=False,
                   stop_distance_mode='dynamic', front_extent=.15, corridor_half_width=.2,
                   base_frame='base_footprint')
        visual = dict(image_topic='/camera/image_raw', calibration_mode=True,
                      visual_stop_threshold=.4, visual_verify_threshold=.4, pair_tolerance=.06, roi_top=.25)
        (folder/'config.json').write_text(json.dumps(cfg))
        (folder/'visual-config.json').write_text(json.dumps(visual))
        events = [dict(cmd_v=.3, odom_v=.3, state='RUNNING', sim_time_s=10.),
                  dict(cmd_v=0., odom_v=.3, state=stop, sim_time_s=10.1),
                  dict(cmd_v=0., odom_v=0. if stopped else .2, state=stop, sim_time_s=10.2)]
        pairs = [pair(.38, .8, 10.), pair(.42, .76, 10.1)]
        for name, records in [('events.csv', events), ('pairs.csv', pairs)]:
            with (folder/name).open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(records[0]))
                writer.writeheader()
                writer.writerows(records)

    def test_profile_preserves_formal_speed_and_accounts_for_formal_pairing(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            self.write_trial(root)
            report = calibration.calibrate(root)
        p = report['parameters']
        self.assertEqual(p['speed'], 1.)
        self.assertEqual(p['stop_distance_mode'], 'fixed')
        self.assertEqual(p['fixed_stop_distance'], .78)
        self.assertEqual(p['visual_verify_threshold'], p['visual_stop_threshold'])
        self.assertLessEqual(p['verify_near'], .76-.06)
        self.assertGreaterEqual(p['verify_far'], .8+.06)
        self.assertFalse(p['calibration_mode'])

    def test_attacked_input_timeout_and_no_stop_cannot_generate_profile(self):
        for changes in (dict(scan='/attack/scan'), dict(stop='SENSOR_TIMEOUT'), dict(stopped=False)):
            with self.subTest(changes=changes), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                self.write_trial(root, **changes)
                with self.assertRaises(ValueError):
                    calibration.calibrate(root)


if __name__ == '__main__':
    unittest.main()
