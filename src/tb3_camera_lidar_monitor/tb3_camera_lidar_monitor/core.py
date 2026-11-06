"""One-to-one approximate pairing and direction-specific confirmation."""
from collections import deque
import math


class ConsistencyMonitor:
    def __init__(self, near, far, tolerance=.12, confirmations=3, max_gap=.5):
        if not all(math.isfinite(x) for x in (near, far, tolerance, max_gap)):
            raise ValueError('Nonfinite configuration')
        if not 0 < near < far or tolerance <= 0 or max_gap <= 0 or confirmations < 1:
            raise ValueError('Invalid configuration')
        self.near, self.far, self.tolerance = near, far, tolerance
        self.confirmations, self.max_gap = confirmations, max_gap
        self.images, self.scans = deque(maxlen=30), deque(maxlen=30)
        self.last_image = self.last_scan = None
        self.last_pair = None
        self.direction, self.count, self.alert = '', 0, False

    def add(self, kind, stamp, value):
        if kind not in ('image', 'scan') or not math.isfinite(stamp) or stamp < 0:
            raise ValueError('Invalid observation')
        previous = self.last_image if kind == 'image' else self.last_scan
        if previous is not None and stamp < previous:
            raise ValueError('Backward timestamp')
        if previous == stamp:
            return []
        if kind == 'image':
            self.last_image = stamp
            self.images.append((stamp, bool(value)))
        else:
            if math.isnan(value) or value < 0:
                raise ValueError('Invalid scan distance')
            self.last_scan = stamp
            self.scans.append((stamp, value))
        results = []
        while self.images and self.scans:
            image, scan = self.images[0], self.scans[0]
            if abs(image[0]-scan[0]) > self.tolerance:
                (self.images if image[0] < scan[0] else self.scans).popleft()
                continue
            self.images.popleft()
            self.scans.popleft()
            paired_at = max(image[0], scan[0])
            if self.last_pair is not None and paired_at-self.last_pair > self.max_gap:
                self.count, self.direction = 0, ''
            self.last_pair = paired_at
            if scan[1] <= self.near:
                lidar = True
            elif scan[1] >= self.far:
                lidar = False
            else:
                lidar = None
            direction = ('AMBIGUOUS' if lidar is None else 'AGREE' if image[1] == lidar
                         else 'VISUAL_MISS' if lidar else 'VISUAL_EXTRA')
            if direction in ('AGREE', 'AMBIGUOUS'):
                self.count, self.direction = 0, ''
            else:
                self.count = self.count+1 if direction == self.direction else 1
                self.direction = direction
                self.alert = self.alert or self.count >= self.confirmations
            results.append(dict(image_stamp=image[0], scan_stamp=scan[0],
                                visual_present=image[1], clearance=scan[1], lidar_present=lidar,
                                direction=direction, count=self.count, alert=self.alert))
        return results
