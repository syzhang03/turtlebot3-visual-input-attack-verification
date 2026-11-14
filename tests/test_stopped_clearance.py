import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/check_stopped_clearance.py'
spec = importlib.util.spec_from_file_location('stopped_clearance', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def native(position, angle=0.):
    return dict(position=list(position), quaternion_xyzw=[0., 0., math.sin(angle/2), math.cos(angle/2)])


class ClearanceTests(unittest.TestCase):
    def setUp(self):
        self.box = dict(link='base', name='body', pose=((-0.064, 0., .048), module.IDENTITY),
                        shape=('box', (.265, .265, .089)))
        self.target = dict(model='scene', link='static', pose=((4., 0., .25), module.IDENTITY),
                           half=(.15, .25, .25))

    def sample(self, x):
        return dict(sim_time_s=10., poses=dict(robot=native((x, 0., .01)),
                    base=native((0., 0., 0.)), scene=native((0., 0., 0.)), static=native((0., 0., 0.))))

    def test_body_offset_not_controller_front_extent(self):
        result = module.measure(self.sample(3.708), [self.box], self.target, 'robot')
        # Obstacle face 3.85 minus body front (3.708 - .064 + .1325).
        self.assertAlmostEqual(result['clearance_lower_bound_m'], .0735)

    def test_intersecting_bounds_do_not_produce_positive_gap(self):
        result = module.measure(self.sample(3.8), [self.box], self.target, 'robot')
        self.assertEqual(result['clearance_lower_bound_m'], 0.)

    def test_all_shapes_including_protrusion_are_checked(self):
        sphere = dict(link='base', name='protrusion', pose=((.2, 0., .1), module.IDENTITY),
                      shape=('sphere', (.03,)))
        result = module.measure(self.sample(3.708), [self.box, sphere], self.target, 'robot')
        self.assertEqual(result['clearance_lower_bound_m'], 0.)

    def test_current_link_pose_is_composed(self):
        sample = self.sample(3.708)
        sample['poses']['base'] = native((.01, 0., 0.))
        result = module.measure(sample, [self.box], self.target, 'robot')
        self.assertAlmostEqual(result['clearance_lower_bound_m'], .0635)

    def test_common_world_rotation_preserves_separation(self):
        original = self.sample(3.708)
        transformed = self.sample(3.708)
        transformed['poses']['robot'] = native((1., 5.708, .01), math.pi/2)
        transformed['poses']['scene'] = native((1., 2., 0.), math.pi/2)
        a = module.measure(original, [self.box], self.target, 'robot')
        b = module.measure(transformed, [self.box], self.target, 'robot')
        self.assertAlmostEqual(a['clearance_lower_bound_m'], b['clearance_lower_bound_m'])

    def test_rotated_cylinder_projection(self):
        pose = module.xml_pose(ET.fromstring('<collision><pose>0 0 0 0 1.5707963267948966 0</pose></collision>'))
        lo, hi = module.aabb(pose, ('cylinder', (.03, .2)))
        for actual, expected in zip(hi, (.1, .03, .03)):
            self.assertAlmostEqual(actual, expected)
        for a, b in zip(lo, hi):
            self.assertAlmostEqual(a, -b)

    def test_unknown_geometry_and_frames_fail_closed(self):
        with self.assertRaises(ValueError):
            module.xml_pose(ET.fromstring('<collision><pose relative_to="other">0 0 0 0 0 0</pose></collision>'))
        with self.assertRaises(ValueError):
            module.shape(ET.fromstring('<collision><geometry><mesh/></geometry></collision>'))


if __name__ == '__main__':
    unittest.main()
