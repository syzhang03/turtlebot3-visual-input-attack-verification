import unittest
import numpy as np
from tb3_visual_baseline.vision import detect_red


class VisionTests(unittest.TestCase):
    def test_clear_image(self):
        blocked, ratio, _ = detect_red(np.zeros((240, 320, 3), dtype=np.uint8), 0.06, 0.25)
        self.assertFalse(blocked)
        self.assertEqual(ratio, 0)

    def test_large_red_obstacle(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        frame[90:220, 110:210] = [0, 0, 255]
        self.assertTrue(detect_red(frame, 0.06, 0.25)[0])

    def test_side_object_is_outside_route(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        frame[:, :40] = [0, 0, 255]
        self.assertFalse(detect_red(frame, 0.06, 0.25)[0])


if __name__ == '__main__':
    unittest.main()
