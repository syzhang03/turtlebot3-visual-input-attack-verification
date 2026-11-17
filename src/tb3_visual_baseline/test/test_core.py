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

    def test_disabled_goal_does_not_stop_at_or_beyond_endpoint(self):
        trial = Trial(goal=4.0, speed=1.0, enable_goal_stop=False)
        for distance in (0, 3.99, 4.0, 4.016, 5.0):
            self.assertEqual(trial.step(True, True, 0, 0, False, distance), 1.0)
            self.assertEqual(trial.state, 'RUNNING')

    def test_visual_stop_still_latches_when_goal_disabled(self):
        trial = Trial(goal=4.0, speed=1.0, enable_goal_stop=False)
        trial.step(True, True, 0, 0, False, 4.016)
        self.assertEqual(trial.step(True, True, 0, 0, True, 4.1), 0.0)
        self.assertEqual(trial.state, 'OBSTACLE_STOP')
        self.assertEqual(trial.step(True, True, 0, 0, False, 4.2), 0.0)

    def test_timeout_and_fault_still_stop_with_goal_disabled(self):
        for image_age, odom_age, distance, reason in (
            (2.1, 0, 4.1, 'SENSOR_TIMEOUT'),
            (0, 2.1, 4.1, 'SENSOR_TIMEOUT'),
            (0, 0, float('nan'), 'FAULT'),
        ):
            trial = Trial(goal=4.0, enable_goal_stop=False)
            trial.step(True, True, 0, 0, False, 4.0)
            self.assertEqual(trial.step(True, True, image_age, odom_age, False, distance), 0.0)
            self.assertEqual(trial.state, reason)

    def test_later_visual_callback_does_not_replace_first_stop_reason(self):
        for initial in Trial.STOP_STATES:
            trial = Trial()
            trial.stop(initial)
            trial.stop('OBSTACLE_STOP')
            self.assertEqual(trial.state, initial)
            self.assertEqual(trial.step(True, True, 0, 0, True, 0), 0.0)

    def test_visual_callback_between_ticks_is_latched(self):
        trial = Trial(enable_goal_stop=False)
        trial.step(True, True, 0, 0, False, 0)
        trial.stop('OBSTACLE_STOP')
        self.assertEqual(trial.step(True, True, 0, 0, False, 0.1), 0.0)
        trial.stop('FAULT')
        self.assertEqual(trial.state, 'OBSTACLE_STOP')

    def test_goal_stop_requires_boolean(self):
        for value in ('false', 0, 1, None):
            with self.assertRaises(ValueError):
                Trial(enable_goal_stop=value)


if __name__ == '__main__':
    unittest.main()
