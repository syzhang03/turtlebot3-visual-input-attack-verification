import unittest
from tb3_visual_baseline.core import Trial, wrap_angle


class TrialTests(unittest.TestCase):
    def test_wait_for_both_sensors(self):
        trial = Trial()
        self.assertEqual(trial.step(True, False, 0, 0, False, 0), 0)
        self.assertEqual(trial.state, 'WAITING')

    def test_requires_explicit_enable(self):
        trial = Trial()
        self.assertEqual(trial.step(False, True, 0, 0, False, 0), 0)

    def test_goal_latches(self):
        trial = Trial()
        self.assertGreater(trial.step(True, True, 0, 0, False, 0), 0)
        self.assertEqual(trial.step(True, True, 0, 0, False, 1.1), 0)
        self.assertEqual(trial.state, 'GOAL_REACHED')
        self.assertEqual(trial.step(True, True, 0, 0, False, 0), 0)

    def test_obstacle_latches_after_disappearing(self):
        trial = Trial()
        self.assertEqual(trial.step(True, True, 0, 0, True, 0.2), 0)
        self.assertEqual(trial.step(True, True, 0, 0, False, 0.2), 0)
        self.assertEqual(trial.state, 'OBSTACLE_STOP')

    def test_stale_stream_stops_running_robot(self):
        for ages in ((2.1, 0), (0, 2.1)):
            trial = Trial()
            trial.step(True, True, 0, 0, False, 0)
            self.assertEqual(trial.step(True, True, *ages, False, 0.1), 0)
            self.assertEqual(trial.state, 'SENSOR_TIMEOUT')
            self.assertEqual(trial.step(True, True, 0, 0, False, 0.1), 0)

    def test_bad_parameters_and_nan(self):
        for value in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                Trial(speed=value)
        trial = Trial()
        self.assertEqual(trial.step(True, True, 0, 0, False, float('nan')), 0)
        self.assertEqual(trial.state, 'FAULT')

    def test_wrap_heading_at_pi_boundary(self):
        self.assertAlmostEqual(wrap_angle(6.2), -0.083185307, places=7)


if __name__ == '__main__':
    unittest.main()
