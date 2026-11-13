"""Position-only arrival confirmation for the taught mime runtime."""

import math

from box_motion_plan import delta, validate_pose

POSITION_TOLERANCE_DEG = 2.
ARRIVAL_SAMPLE_INTERVAL_SECONDS = .1
ARRIVAL_WINDOW_SECONDS = 1.


class PositionAcceptance:
    """Two fresh qualifying frames, separated by 0.1 to 1 second."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.first_time = None
        self.first_serial = None
        self.last_time = None

    def update(self, actual, target, serial, now):
        validate_pose(actual)
        validate_pose(target)
        if not math.isfinite(now) or (self.last_time is not None and now < self.last_time):
            raise ValueError("Invalid arrival feedback time")
        self.last_time = now
        if delta(actual, target) > POSITION_TOLERANCE_DEG+1e-9:
            self.first_time = self.first_serial = None
            return False
        if self.first_time is None or now-self.first_time > ARRIVAL_WINDOW_SECONDS+1e-9:
            self.first_time, self.first_serial = now, serial
            return False
        return (serial != self.first_serial
                and now-self.first_time >= ARRIVAL_SAMPLE_INTERVAL_SECONDS-1e-9)
