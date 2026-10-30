"""HSV red-obstacle detector; area is a visual cue, not a distance estimate."""
import cv2
import numpy as np


def detect_red(bgr, threshold, roi_top):
    height, width = bgr.shape[:2]
    if not height or not width:
        raise ValueError('Empty image')
    # Restrict comparison to the central forward image region.
    x0, x1 = int(width * 0.2), int(width * 0.8)
    y0 = int(height * roi_top)
    roi = bgr[y0:, x0:x1]
    if roi.size == 0:
        raise ValueError('Empty ROI')
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 100, 70]), np.array([10, 255, 255]))
    mask |= cv2.inRange(hsv, np.array([170, 100, 70]), np.array([179, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea) if contours else None
    ratio = cv2.contourArea(contour) / mask.size if contour is not None else 0.0
    debug = bgr.copy()
    cv2.rectangle(debug, (x0, y0), (x1 - 1, height - 1), (255, 255, 0), 2)
    if contour is not None:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(debug, (x + x0, y + y0), (x + x0 + w, y + y0 + h), (0, 255, 0), 2)
    cv2.putText(debug, f'red_area={ratio:.4f} threshold={threshold:.4f}',
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return ratio >= threshold, ratio, debug
