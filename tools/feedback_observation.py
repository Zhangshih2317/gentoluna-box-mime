"""Bound displacement using feedback observation windows, not return timestamps."""

import copy
import math


class FeedbackObservation:
    def __init__(self):
        self.previous = None
        self.started = None
        self.finished = None
        self.serial = None

    def reset(self):
        self.__init__()

    def check(self, pose, serial, started, finished, limits):
        if (not all(math.isfinite(t) for t in (started, finished))
                or finished < started
                or (self.finished is not None and started < self.finished)):
            raise RuntimeError("Invalid feedback observation time")
        if self.previous is not None:
            # The previous RT snapshot may have been copied before a slow SG
            # read. Its return time must not shorten the next displacement window.
            interval = finished-self.started
            for key, values in pose.items():
                for joint, (value, prior, speed) in enumerate(
                        zip(values, self.previous[key], limits[key])):
                    allowed = .04 if serial == self.serial else speed*interval+.04
                    if abs(value-prior) > allowed:
                        return {
                            "group": key, "joint": joint+1, "delta_deg": abs(value-prior),
                            "allowed_delta_deg": allowed, "velocity_limit_deg_s": speed,
                            "previous_frame": self.serial, "frame_serial": serial,
                            "previous_read_started": self.started,
                            "previous_read_finished": self.finished,
                            "read_started": started, "read_finished": finished,
                            "observation_window_seconds": interval,
                            "same_frame": serial == self.serial}
        # Repeated frames must not move the time associated with an old pose.
        if self.previous is None or serial != self.serial:
            self.previous = copy.deepcopy(pose)
            self.started, self.finished, self.serial = started, finished, serial
        return None
