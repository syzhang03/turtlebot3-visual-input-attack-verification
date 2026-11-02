import unittest

import numpy as np

from tb3_camera_attack.occlusion import DEFAULT_RECT, black_rectangle, validate_rectangle

try:
    from tb3_visual_baseline.vision import detect_red
except ModuleNotFoundError:
    detect_red = None


class OcclusionTests(unittest.TestCase):
    def setUp(self):
        self.frame = np.random.default_rng(42).integers(
            1, 256, size=(240, 320, 3), dtype=np.uint8)

    def test_requested_rectangle_black_and_outside_unchanged(self):
        original = self.frame.copy()
        result = black_rectangle(self.frame, 0.1, 0.2, 0.3, 0.4)
        x0, x1 = int(320 * 0.1), int(320 * (0.1 + 0.3))
        y0, y1 = int(240 * 0.2), int(240 * (0.2 + 0.4))
        expected = original.copy()
        expected[y0:y1, x0:x1] = 0
        np.testing.assert_array_equal(result, expected)
        self.assertTrue(np.all(result[y0:y1, x0:x1] == 0))
        np.testing.assert_array_equal(self.frame, original)
        self.assertFalse(np.shares_memory(result, self.frame))
        self.assertEqual(result.shape, original.shape)
        self.assertEqual(result.dtype, original.dtype)

    def test_default_matches_detector_roi_at_multiple_resolutions(self):
        for height, width in ((240, 320), (480, 640), (243, 321), (5, 7)):
            with self.subTest(size=(height, width)):
                frame = np.full((height, width, 3), 255, dtype=np.uint8)
                expected = frame.copy()
                expected[int(height * 0.25):, int(width * 0.2):int(width * 0.8)] = 0
                np.testing.assert_array_equal(black_rectangle(frame), expected)

    def test_full_image_including_last_row_and_column(self):
        self.assertFalse(black_rectangle(self.frame, 0.0, 0.0, 1.0, 1.0).any())

    def test_subpixel_mask_is_noop(self):
        np.testing.assert_array_equal(
            black_rectangle(self.frame, 0.0, 0.0, 1e-8, 1e-8), self.frame)

    def test_invalid_coordinates_rejected_without_mutation(self):
        original = self.frame.copy()
        invalid = (
            (-0.1, 0.0, 0.5, 0.5), (0.0, -0.1, 0.5, 0.5),
            (1.0, 0.0, 0.5, 0.5), (0.0, 1.0, 0.5, 0.5),
            (0.0, 0.0, 0.0, 0.5), (0.0, 0.0, 0.5, -0.5),
            (0.8, 0.0, 0.3, 0.5), (0.0, 0.8, 0.5, 0.3),
            (float('nan'), 0.0, 0.5, 0.5), (0.0, float('inf'), 0.5, 0.5),
            (True, 0.0, 0.5, 0.5), ('0.2', 0.0, 0.5, 0.5),
        )
        for rectangle in invalid:
            with self.subTest(rectangle=rectangle), self.assertRaises(ValueError):
                black_rectangle(self.frame, *rectangle)
        np.testing.assert_array_equal(self.frame, original)

    def test_invalid_images_rejected(self):
        for frame in (None, np.zeros((0, 10, 3), dtype=np.uint8),
                      np.zeros((10, 10), dtype=np.uint8),
                      np.zeros((10, 10, 4), dtype=np.uint8),
                      np.zeros((10, 10, 3), dtype=np.float64)):
            with self.subTest(frame_type=type(frame)), self.assertRaises(ValueError):
                black_rectangle(frame)

    def test_default_rectangle_is_valid(self):
        validate_rectangle(**DEFAULT_RECT)

    @unittest.skipIf(detect_red is None, 'Baseline package not on PYTHONPATH')
    def test_red_obstacles_at_different_positions_inside_actual_detector_roi(self):
        # Each obstacle exceeds 6% of the actual ROI, at different image positions.
        for x, y in ((65, 61), (170, 61), (65, 150), (170, 150)):
            with self.subTest(position=(x, y)):
                frame = np.zeros((240, 320, 3), dtype=np.uint8)
                frame[y:y + 80, x:x + 80] = (0, 0, 255)
                self.assertTrue(detect_red(frame, 0.06, 0.25)[0])
                self.assertFalse(detect_red(black_rectangle(frame), 0.06, 0.25)[0])


if __name__ == '__main__':
    unittest.main()
