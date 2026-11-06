"""Independent evaluation geometry checks, separate from LiDAR projection."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

path = Path(__file__).resolve().parents[3]/'scripts/ground_truth.py'
spec = importlib.util.spec_from_file_location('evaluation_geometry', path)
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


class GroundTruthTests(unittest.TestCase):
    def test_front_and_side_clearance_are_euclidean_lower_bounds(self):
        self.assertAlmostEqual(geometry.box_clearance_lower_bound(3, 0, .3), .55)
        self.assertAlmostEqual(geometry.box_clearance_lower_bound(4, 1, .3), .45)
        self.assertLess(geometry.box_clearance_lower_bound(4, 0, .3), 0)

    def test_robot_collision_offsets_are_included(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'model.sdf'
            p.write_text('<sdf><model><link><pose>1 0 0 0 0 0</pose>'
                         '<collision><pose>0 2 0 0 0 0</pose><geometry>'
                         '<sphere><radius>0.5</radius></sphere></geometry>'
                         '</collision></link></model></sdf>')
            self.assertAlmostEqual(geometry.collision_radius(p), 3.5)

    def test_unknown_collision_shapes_are_not_ignored(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'model.sdf'
            p.write_text('<sdf><model><link><collision><geometry><mesh/></geometry>'
                         '</collision></link></model></sdf>')
            with self.assertRaises(ValueError):
                geometry.collision_radius(p)
