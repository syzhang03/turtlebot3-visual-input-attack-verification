"""Check that log auditing distinguishes accepted frames from relay-only output."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / 'summarize_joint_trial.py'
spec = importlib.util.spec_from_file_location('joint_summary', SCRIPT)
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)


class SummaryTests(unittest.TestCase):
    def write_json(self, name, data):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data), encoding='utf-8')

    def write_csv(self, name, rows):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.write_json('manifest.json', dict(mode='joint', effective=dict(camera_delay_s=1., lidar_bias_m=1.),
                        camera_transport=dict(input_reliability='reliable', output_reliability='reliable')))
        self.write_json('camera/c/config.json', dict(input_reliability='reliable', output_reliability='reliable',
                                                    transport_revision='configurable-reliable-input-v2'))
        self.write_json('controller/c/config.json', dict(enabled=True, speed=1., enable_goal_stop=False,
                                                        scan_topic='/attack/scan', sensor_timeout_wall=2.,
                                                        stop_distance_mode='fixed', fixed_stop_distance=.788))
        self.write_json('controller/c/visual-config.json', dict(visual_stop_threshold=.4, calibration_mode=False,
                        visual_verify_threshold=.4, verify_near=.71, verify_far=.87))
        self.write_csv('controller/c/events.csv', [dict(sim_time_s=t, cmd_v=v, cmd_w=0., odom_v=v,
                                                       state=state, distance_m=t-10.,
                                                       corridor_clearance_m=3.1-(t-10.), stop_threshold_m=.788)
            for t, v, state in [(10., 1., 'RUNNING'), (11., 1., 'RUNNING'), (12., 0., 'OBSTACLE_STOP')]])
        self.write_csv('controller/c/image-acceptance.csv', [dict(accepted_sim_s=11., header_stamp_ns=11*10**9)])
        self.write_csv('camera/c/frames.csv', [dict(input_stamp_ns=t*10**9, actual_delay_s=delay,
                                                   state=state, buffer_reset=0)
            for t, delay, state in [(10, 0., 'BUFFER_NOT_READY'), (11, 1.1, 'REPLAY'), (20, 0., 'AFTER_WINDOW')]])
        self.write_csv('lidar/l/scans.csv', [dict(input_stamp_ns=11*10**9, state='BIAS_ACTIVE',
                                                changed_rays=5, max_delta_m=1., reset=False)])

    def test_uses_only_accepted_camera_frames_and_does_not_infer_contact(self):
        result = summary.summarize(self.root)
        self.assertEqual(result['issues'], [])
        self.assertEqual(result['accepted_camera_delay_s']['count'], 1)
        self.assertAlmostEqual(result['accepted_camera_delay_s']['mean'], 1.1)
        self.assertEqual(result['first_zero_after_motion_sim_s'], 12.)
        self.assertEqual(result['stop_command_excerpt'][-1]['cmd_v'], '0.0')
        self.assertIn('not determined', result['contact'])

    def test_flags_lidar_configuration_without_actual_modification(self):
        self.write_csv('lidar/l/scans.csv', [dict(input_stamp_ns=11*10**9, state='BIAS_ACTIVE',
                                                changed_rays=0, max_delta_m=0., reset=False)])
        result = summary.summarize(self.root)
        self.assertTrue(any('no ranges changed' in issue for issue in result['issues']))

    def test_refuses_to_mix_multiple_enabled_controller_runs(self):
        self.write_json('controller/other/config.json', dict(enabled=True))
        with self.assertRaisesRegex(ValueError, 'one enabled controller'):
            summary.summarize(self.root)

    def test_flags_stale_built_camera_transport(self):
        self.write_json('camera/c/config.json', dict(input_reliability='best_effort', output_reliability='reliable'))
        result = summary.summarize(self.root)
        self.assertTrue(any('transport does not match' in issue for issue in result['issues']))

    def test_flags_changed_logged_stop_threshold(self):
        events = summary.read_rows(self.root/'controller/c/events.csv')
        events[-1]['stop_threshold_m'] = '1.15'
        self.write_csv('controller/c/events.csv', events)
        result = summary.summarize(self.root)
        self.assertTrue(any('threshold differs' in issue for issue in result['issues']))


if __name__ == '__main__':
    unittest.main()
