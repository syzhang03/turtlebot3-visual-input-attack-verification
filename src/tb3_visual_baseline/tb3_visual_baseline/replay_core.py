"""ROS-independent, bounded source-timestamp delay selection."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass
class Selection:
    frame: object
    source_ns: int
    state: str
    elapsed_ns: int
    reset: bool = False


class DelayBuffer:
    def __init__(self, delay_s=1.0, start_s=0.0, duration_s=60.0,
                 max_frames=300, max_bytes=268435456):
        for value in (delay_s, start_s, duration_s):
            if not math.isfinite(value) or value < 0:
                raise ValueError('Times must be finite and nonnegative')
        if duration_s <= 0 or max_frames < 1 or max_bytes < 1:
            raise ValueError('Duration and buffer limits must be positive')
        self.delay_ns = round(delay_s * 1e9)
        self.start_ns = round(start_s * 1e9)
        self.duration_ns = round(duration_s * 1e9)
        self.max_frames, self.max_bytes = max_frames, max_bytes
        self.clear()

    def clear(self):
        self.frames = deque()
        self.bytes = 0
        self.first_ns = self.last_ns = None
        self.layout = None

    def _pop(self):
        self.bytes -= self.frames.popleft()[2]

    def push(self, stamp_ns, frame, size, layout=None):
        if stamp_ns < 0 or size < 0:
            raise ValueError('Negative timestamp or payload size')
        reset = self.last_ns is not None and (
            stamp_ns < self.last_ns or layout != self.layout)
        if reset:
            self.clear()
        if stamp_ns == self.last_ns:
            return None  # Do not turn duplicate source messages into fresh inputs.
        if self.first_ns is None:
            self.first_ns = stamp_ns
        self.last_ns, self.layout = stamp_ns, layout
        elapsed = stamp_ns - self.first_ns
        target = stamp_ns - self.delay_ns
        self.frames.append((stamp_ns, frame, size))
        self.bytes += size
        # Keep the newest sample <= target and all samples newer than it.
        while len(self.frames) > 1 and self.frames[1][0] <= target:
            self._pop()
        while len(self.frames) > self.max_frames or self.bytes > self.max_bytes:
            self._pop()
        state = 'BEFORE_ATTACK'
        selected_ns, selected = stamp_ns, frame
        if elapsed >= self.start_ns + self.duration_ns:
            state = 'AFTER_ATTACK'
        elif elapsed >= self.start_ns:
            if self.delay_ns == 0:
                state = 'ZERO_DELAY'
            elif self.frames and self.frames[0][0] <= target:
                selected_ns, selected, _ = self.frames[0]
                state = 'REPLAY'
            else:
                state = 'BUFFER_NOT_READY'
        return Selection(selected, selected_ns, state, elapsed, reset)
