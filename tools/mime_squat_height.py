"""Resolve a physical squat depth along the taught joint path, offline only."""

import copy
import math

import numpy as np

from box_motion_plan import validate_pose
from stand_up_plan import BodyHeadKinematics, body_transform


def solve_squat_depth(home, taught, depth_cm):
    if isinstance(depth_cm, bool):
        raise ValueError("Squat depth must be a finite number in (0, 30] cm")
    depth_mm = float(depth_cm)*10.
    if not math.isfinite(depth_mm) or not 0 < depth_mm <= 300.:
        raise ValueError("Squat depth must be a finite number in (0, 30] cm")
    validate_pose(home)
    hip = taught["body"][1]-home["body"][1]
    knee = home["body"][2]-taught["body"][2]
    if not 0 < hip < knee:
        raise ValueError("Invalid taught squat coordination")
    kin = BodyHeadKinematics()
    origin = body_transform(kin, home["body"])

    def target(degrees):
        state = copy.deepcopy(home)
        state["body"][1] += degrees*hip/knee
        state["body"][2] -= degrees
        state["body"][3] = sum(home["body"][1:4])-sum(state["body"][1:3])
        return state

    def transform(degrees):
        return body_transform(kin, target(degrees)["body"])

    if origin[2, 3]-transform(knee)[2, 3] <= depth_mm:
        raise ValueError("Requested squat depth exceeds the taught branch")
    low, high = 0., knee
    for _ in range(60):
        middle = (low+high)/2.
        if origin[2, 3]-transform(middle)[2, 3] < depth_mm:
            low = middle
        else:
            high = middle
    degrees = (low+high)/2.
    previous_drop = 0.
    # Validate the same straight joint-space path the runner will stream.
    for angle in np.linspace(0., degrees, math.ceil(degrees/.1)+1):
        state = target(float(angle))
        validate_pose(state)
        actual = body_transform(kin, state["body"])
        drop = origin[2, 3]-actual[2, 3]
        if drop < previous_drop-1e-8:
            raise ValueError("Squat path is not monotonically downward")
        if np.max(np.abs(actual[:3, :3]-origin[:3, :3])) > 1e-7:
            raise ValueError("Squat path changes torso orientation")
        if abs(actual[0, 3]-origin[0, 3]) > 30 or abs(actual[1, 3]-origin[1, 3]) > 5:
            raise ValueError("Squat path leaves the existing horizontal corridor")
        previous_drop = drop
    if abs(previous_drop-depth_mm) > 1e-5:
        raise ValueError("Squat depth solve did not reach the requested height")
    return {
        "depth_mm": depth_mm,
        "knee_delta_deg": degrees,
        "target_body": target(degrees)["body"],
        "displacement_mm": (transform(degrees)[:3, 3]-origin[:3, 3]).tolist(),
    }
