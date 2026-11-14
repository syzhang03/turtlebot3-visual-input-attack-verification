import math
import unittest

from tb3_lidar_baseline.core import Controller, corridor_scan, stopping_distance, configured_stop_distance


class GeometryTests(unittest.TestCase):
    def scan(self, values=None, **overrides):
        args = dict(ranges=[math.inf]*360 if values is None else values,
                    angle_min=0.0, angle_increment=math.pi/180,
                    range_min=0.12, range_max=3.5, tx=-0.064, ty=0.0, yaw=0.0,
                    half_width=0.20, front_extent=0.15, lookahead=1.15)
        args.update(overrides)
        return corridor_scan(**args)

    def test_front_distance_accounts_for_sensor_offset_and_robot_front(self):
        rays = [math.inf]*360
        rays[0] = 1.5
        result = self.scan(rays)
        self.assertTrue(result.usable)
        self.assertAlmostEqual(result.clearance, 1.286)

    def test_side_walls_and_rear_obstacles_are_not_forward_hazards(self):
        rays = [math.inf]*360
        rays[45], rays[180], rays[315] = 1.0, 0.2, 1.0
        self.assertEqual(self.scan(rays).clearance, math.inf)

    def test_rotated_sensor_and_reversed_angle_origin(self):
        rays = [math.inf]*360
        rays[270] = 1.5  # Sensor rotated left 90 degrees.
        self.assertAlmostEqual(self.scan(rays, yaw=math.pi/2).clearance, 1.286)
        rays = [math.inf]*360
        rays[180] = 1.5
        self.assertAlmostEqual(self.scan(rays, angle_min=-math.pi).clearance, 1.286)

    def test_no_return_convention_is_explicit(self):
        self.assertTrue(self.scan().usable)
        self.assertFalse(self.scan(allow_positive_infinity=False).usable)

    def test_bad_front_readings_cannot_mean_clear(self):
        for value in (math.nan, -math.inf, 0.0, 0.05, 4.0):
            with self.subTest(value=value):
                rays = [math.inf]*360
                rays[0] = value
                result = self.scan(rays)
                self.assertFalse(result.usable)
                self.assertEqual(result.invalid_rays, 1)

    def test_unobserved_front_or_insufficient_range_rejected(self):
        self.assertFalse(self.scan(ranges=[]).usable)
        self.assertFalse(self.scan(ranges=[math.inf]*60).usable)
        self.assertFalse(self.scan(range_max=0.7).usable)
        self.assertFalse(self.scan(angle_increment=math.nan).usable)

    def test_larger_speed_requires_more_stopping_distance(self):
        self.assertAlmostEqual(stopping_distance(1.0, .35, .3, 1.0), 1.15)
        self.assertGreater(stopping_distance(1.2, .35, .3, 1.0), 1.15)

    def test_fixed_boundary_is_not_added_to_dynamic_braking_distance(self):
        for speed in (.3, 1., 1.2):
            distance = configured_stop_distance(speed, .35, .3, 1., 'fixed', .77)
            self.assertEqual(distance, .77)
            c = Controller()
            self.assertEqual(c.step(True, True, True, True, .78 <= distance, False, speed), speed)
            self.assertEqual(c.step(True, True, True, True, .77 <= distance, False, speed), 0.)
            self.assertEqual(c.state, 'OBSTACLE_STOP')

    def test_default_remains_dynamic_and_bad_fixed_configuration_fails(self):
        self.assertAlmostEqual(configured_stop_distance(1., .35, .3, 1.), 1.15)
        for value in (0., -1., math.nan, math.inf):
            with self.assertRaises(ValueError):
                configured_stop_distance(1., .35, .3, 1., 'fixed', value)
        with self.assertRaises(ValueError):
            configured_stop_distance(1., .35, .3, 1., 'typo', .77)


class ControlTests(unittest.TestCase):
    def step(self, controller, **changes):
        args = dict(enabled=True, ready=True, fresh=True, usable=True,
                    blocked=False, goal_reached=False, speed=1.0)
        args.update(changes)
        return controller.step(**args)

    def test_no_motion_until_ready_fresh_and_usable(self):
        for changes in (dict(enabled=False), dict(ready=False), dict(fresh=False), dict(usable=False)):
            self.assertEqual(self.step(Controller(), **changes), 0.0)

    def test_obstacle_stop_latches_even_if_obstacle_disappears(self):
        c = Controller()
        self.assertEqual(self.step(c), 1.0)
        self.assertEqual(self.step(c, blocked=True), 0.0)
        self.assertEqual(self.step(c), 0.0)
        self.assertEqual(c.state, 'OBSTACLE_STOP')

    def test_timeout_and_invalid_data_stop_a_running_robot(self):
        for changes, state in ((dict(fresh=False), 'SENSOR_TIMEOUT'),
                               (dict(usable=False), 'SENSOR_INVALID')):
            c = Controller()
            self.step(c)
            self.assertEqual(self.step(c, **changes), 0.0)
            self.assertEqual(c.state, state)
            self.assertEqual(self.step(c), 0.0)

    def test_goal_and_first_stop_reason_latch(self):
        c = Controller()
        self.assertEqual(self.step(c, goal_reached=True), 0.0)
        c.stop('FAULT')
        self.assertEqual(c.state, 'GOAL_REACHED')


if __name__ == '__main__':
    unittest.main()
