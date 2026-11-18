"""Pure range-bias transform, scheduled using original scan timestamps."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BiasResult:
    ranges: tuple
    state: str
    elapsed_s: float
    reset: bool
    eligible_rays: int = 0
    changed_rays: int = 0
    clipped_rays: int = 0
    invalid_rays: int = 0
    min_delta_m: float = 0.0
    max_delta_m: float = 0.0
    input_sector_min_m: float | None = None
    output_sector_min_m: float | None = None


class RangeBias:
    def __init__(self, bias_m=1.0, sector_center_deg=0.0, sector_half_width_deg=30.0,
                 start_s=0.0, duration_s=600.0):
        values = (bias_m, sector_center_deg, sector_half_width_deg, start_s, duration_s)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('All attack parameters must be finite')
        if bias_m < 0 or not 0 < sector_half_width_deg <= 180 or start_s < 0 or duration_s <= 0:
            raise ValueError('Require bias >= 0, 0 < sector half width <= 180, start >= 0, duration > 0')
        self.bias_m = bias_m
        self.center = math.radians(sector_center_deg % 360)
        self.half_width = math.radians(sector_half_width_deg)
        self.start_ns, self.duration_ns = round(start_s * 1e9), round(duration_s * 1e9)
        if self.duration_ns < 1:
            raise ValueError('Duration must be at least one nanosecond')
        self.origin_ns = self.previous_ns = None
        self.duplicates = 0

    def process(self, stamp_ns, ranges, angle_min, angle_increment, range_min, range_max):
        if stamp_ns == self.previous_ns:
            self.duplicates += 1
            return None
        reset = self.previous_ns is not None and stamp_ns < self.previous_ns
        if self.origin_ns is None or reset:
            self.origin_ns = stamp_ns
        self.previous_ns = stamp_ns
        elapsed_ns = stamp_ns - self.origin_ns
        original = tuple(ranges)
        valid_geometry = (
            len(original) >= 2
            and all(math.isfinite(v) for v in (angle_min, angle_increment, range_min, range_max))
            and angle_increment != 0 and 0 <= range_min < range_max
            and math.isfinite(angle_min + (len(original) - 1) * angle_increment)
        )
        if not valid_geometry:
            return BiasResult(original, 'INVALID_METADATA', elapsed_ns / 1e9, reset)
        active = self.start_ns <= elapsed_ns < self.start_ns + self.duration_ns
        state = ('PASSTHROUGH' if self.bias_m == 0 else
                 'BIAS_ACTIVE' if active else 'BEFORE_WINDOW' if elapsed_ns < self.start_ns else 'AFTER_WINDOW')
        output = list(original)
        eligible = clipped = invalid = 0
        deltas, before, after = [], [], []
        for i, distance in enumerate(original):
            angle = angle_min + i * angle_increment
            relative = math.atan2(math.sin(angle - self.center), math.cos(angle - self.center))
            if abs(relative) > self.half_width + 1e-12:
                continue
            if not math.isfinite(distance) or not range_min <= distance <= range_max:
                # Preserve invalid readings and no-returns. Do not turn faults into free space.
                if distance != math.inf:
                    invalid += 1
                continue
            eligible += 1
            before.append(distance)
            if active and self.bias_m > 0:
                output[i] = min(range_max, distance + self.bias_m)
                clipped += int(distance + self.bias_m > range_max)
            after.append(output[i])
            if output[i] != distance:
                deltas.append(output[i] - distance)
        return BiasResult(
            tuple(output), state, elapsed_ns / 1e9, reset, eligible, len(deltas), clipped, invalid,
            min(deltas, default=0.0), max(deltas, default=0.0),
            min(before, default=None), min(after, default=None),
        )
