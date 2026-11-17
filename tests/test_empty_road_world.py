"""Static checks for the full and empty experiment worlds.

These tests run on any host with Python 3 and do not require ROS or Gazebo.
They ensure the empty scene differs from the full scene only by the red
obstacle and that the launch/config wiring exposes both scenes.
"""
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FULL = ROOT / 'worlds' / 'intersection-white-line.sdf'
EMPTY = ROOT / 'worlds' / 'empty-road-white-line.sdf'


def parse(path):
    return ET.parse(path).getroot()


def pose(element):
    node = element.find('pose')
    if node is None or node.text is None:
        return None
    return [float(value) for value in node.text.split()]


class WorldTests(unittest.TestCase):
    def test_scene_files_exist_and_share_model_name(self):
        self.assertTrue(FULL.is_file())
        self.assertTrue(EMPTY.is_file())
        for path in (FULL, EMPTY):
            model = parse(path).find('model')
            self.assertIsNotNone(model, path)
            self.assertEqual(model.get('name'), 'boulder_intersection')

    def test_full_scene_has_red_box_at_expected_pose(self):
        scene = parse(FULL)
        visual = scene.find(".//visual[@name='red_box_visual']")
        collision = scene.find(".//collision[@name='red_box_collision']")
        self.assertIsNotNone(visual)
        self.assertIsNotNone(collision)
        self.assertEqual(pose(visual), [4, 0, .25, 0, 0, 0])
        self.assertEqual(pose(collision), [4, 0, .25, 0, 0, 0])

    def test_empty_scene_has_no_red_box(self):
        scene = parse(EMPTY)
        self.assertIsNone(scene.find(".//visual[@name='red_box_visual']"))
        self.assertIsNone(scene.find(".//collision[@name='red_box_collision']"))
        self.assertNotIn('red_box', EMPTY.read_text(encoding='utf-8'))

    def test_both_scenes_keep_road_and_stop_line(self):
        for path in (FULL, EMPTY):
            scene = parse(path)
            road = scene.find(".//visual[@name='road_x_visual']")
            stop = scene.find(".//visual[@name='stop_line_visual']")
            self.assertIsNotNone(road, path)
            self.assertIsNotNone(stop, path)
            self.assertEqual(pose(stop), [3.15, 0, .012, 0, 0, 0])
            self.assertEqual(road.findtext('geometry/box/size'), '14 2.4 0.004')
            self.assertEqual(stop.findtext('geometry/box/size'), '0.06 2.3 0.004')

    def test_launch_and_configs_expose_both_scenes(self):
        launch = (ROOT / 'scripts' / 'start_scene.launch.py').read_text(encoding='utf-8')
        self.assertIn("'full': ('intersection-white-line.sdf', True)", launch)
        self.assertIn("'empty': ('empty-road-white-line.sdf', False)", launch)
        self.assertIn("DeclareLaunchArgument(", launch)
        self.assertIn("'scene'", launch)
        helper = (ROOT / 'scripts' / 'start_scene.sh').read_text(encoding='utf-8')
        self.assertIn('full|empty', helper)
        self.assertIn('empty-road-white-line.sdf', helper)
        for name in ('baseline_empty_road.yaml', 'lidar_empty_road.yaml'):
            config = (ROOT / 'configs' / name).read_text(encoding='utf-8')
            self.assertIn('enable_goal_stop: true', config)
            self.assertIn('goal_distance: 4.0', config)


if __name__ == '__main__':
    unittest.main()
