"""Common-phase timing with explicit per-axis command limits.

This module never connects to hardware. Limits may be conservative software
budgets; they are not evidence of motor capability or dynamic calibration.
"""

from dataclasses import dataclass
import math

from box_motion_plan import blend


def positive(value, name):
    if isinstance(value, bool):
        raise ValueError(name+" must be finite and positive")
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(name+" must be finite and positive")
    return result


@dataclass(frozen=True)
class AxisLimits:
    reference_velocity_deg_s: float
    maximum_velocity_deg_s: float
    acceleration_deg_s2: float
    jerk_deg_s3: float

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, positive(getattr(self, name), name))
        if self.reference_velocity_deg_s > self.maximum_velocity_deg_s:
            raise ValueError("Reference velocity exceeds the configured maximum")


def plan_timing(start, target, limits, speed_percent, reference_percent=30., rate_hz=50.):
    """Keep one phase for every axis; scale velocity requests, cap acceleration/jerk."""
    if not start or len(start) != len(target) or len(start) != len(limits):
        raise ValueError("Start, target and axis limits must have the same nonzero length")
    start, target = tuple(map(float, start)), tuple(map(float, target))
    if not all(math.isfinite(q) for q in start+target):
        raise ValueError("Non-finite joint position")
    speed = positive(speed_percent, "speed_percent")
    reference = positive(reference_percent, "reference_percent")
    rate = positive(rate_hz, "rate_hz")
    if max(speed, reference) > 100:
        raise ValueError("Speed percentages must not exceed 100")
    seconds, constraints = 0., []
    for index, (a, b, bound) in enumerate(zip(start, target, limits)):
        distance = abs(b-a)
        if distance == 0:
            continue
        if not isinstance(bound, AxisLimits):
            raise ValueError("Missing configured limits for moving axis "+str(index+1))
        velocity = min(bound.maximum_velocity_deg_s,
                       bound.reference_velocity_deg_s*speed/reference)
        times = {
            "velocity": 1.875*distance/velocity,
            "acceleration": math.sqrt((10/math.sqrt(3))*distance/bound.acceleration_deg_s2),
            "jerk": (60*distance/bound.jerk_deg_s3)**(1/3),
        }
        seconds = max(seconds, *times.values())
        constraints.append({"axis": index+1, "velocity_limit_deg_s": velocity,
                            "duration_bounds_s": times})
    # Round upwards so discrete samples cannot violate the continuous limits.
    ticks = max(1, math.ceil(seconds*rate))
    seconds = ticks/rate
    return {
        "seconds": seconds, "ticks": ticks, "rate_hz": rate,
        "start": start, "target": target,
        "signed_peak_velocity_deg_s": [(b-a)*1.875/seconds for a, b in zip(start, target)],
        "peak_acceleration_deg_s2": [abs(b-a)*(10/math.sqrt(3))/seconds**2
                                   for a, b in zip(start, target)],
        "peak_jerk_deg_s3": [60*abs(b-a)/seconds**3 for a, b in zip(start, target)],
        "constraints": constraints, "hardware_execution_verified": False,
    }


def sample(timing, elapsed_seconds):
    elapsed = float(elapsed_seconds)
    if not math.isfinite(elapsed):
        raise ValueError("Non-finite sample time")
    phase = blend(max(0., min(1., elapsed/timing["seconds"])))
    return [a+(b-a)*phase for a, b in zip(timing["start"], timing["target"])]
