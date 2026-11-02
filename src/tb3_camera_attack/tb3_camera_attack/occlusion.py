"""Image operations independent of ROS; coordinates are fractions of the image."""

import math

import cv2
import numpy as np


# Matches vision.py with the baseline YAML's roi_top=0.25.
# This covers the detector's search region, not an assumed obstacle position.
DEFAULT_RECT = dict(mask_x=0.2, mask_y=0.25, mask_width=0.6, mask_height=0.75)


def validate_rectangle(mask_x, mask_y, mask_width, mask_height):
    values = (mask_x, mask_y, mask_width, mask_height)
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in values):
        raise ValueError('Mask coordinates must be finite numbers')
    if not (0 <= mask_x < 1 and 0 <= mask_y < 1):
        raise ValueError('mask_x and mask_y must be in [0, 1)')
    if not (0 < mask_width <= 1 and 0 < mask_height <= 1):
        raise ValueError('mask_width and mask_height must be in (0, 1]')
    if mask_x + mask_width > 1 + 1e-12 or mask_y + mask_height > 1 + 1e-12:
        raise ValueError('The mask rectangle must fit inside the image')


def black_rectangle(bgr, mask_x=0.2, mask_y=0.25, mask_width=0.6, mask_height=0.75):
    """Copy a uint8 BGR image and blacken the half-open rectangle [x0:x1, y0:y1]."""
    validate_rectangle(mask_x, mask_y, mask_width, mask_height)
    if (not isinstance(bgr, np.ndarray) or bgr.dtype != np.uint8
            or bgr.ndim != 3 or bgr.shape[2] != 3 or 0 in bgr.shape[:2]):
        raise ValueError('Expected a nonempty uint8 BGR image with three channels')
    height, width = bgr.shape[:2]
    x0, y0 = int(width * mask_x), int(height * mask_y)
    x1 = min(width, int(width * (mask_x + mask_width)))
    y1 = min(height, int(height * (mask_y + mask_height)))
    result = bgr.copy()
    # Very small normalized rectangles can cover no pixels after rounding.
    if x1 > x0 and y1 > y0:
        cv2.rectangle(result, (x0, y0), (x1 - 1, y1 - 1), (0, 0, 0), cv2.FILLED)
    return result
