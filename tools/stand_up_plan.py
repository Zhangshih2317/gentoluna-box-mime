"""Solve a fixed 100 mm rise from a captured squat; never access the robot."""

import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

from box_motion_plan import delta, interpolate, pose
from recover_box_start_robot import validate_limits

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"reference/robot_frames"))
from calibration_transforms.body_head_fk import BODY_JOINTS, BodyHeadKinematics

SOURCE = "points/taught/SQUAT_TAUGHT-20261111-150201.json"
DESTINATION = "points/STAND_UP_100MM.json"


def body_transform(kin, body):
    return kin._chain(BODY_JOINTS, kin._angles_rad(np.asarray(body), np.zeros(2)))


def solve_height(kin, start_body, height_mm, previous_body=None):
    """Continue the existing negative-B3 branch, holding base X and torso rotation."""
    if not math.isfinite(height_mm) or not 0 <= height_mm <= 100:
        raise ValueError("Only a rise in [0, 100] mm is allowed")
    start = np.asarray(start_body, dtype=float)
    if start.shape != (6,) or not np.isfinite(start).all():
        raise ValueError("Expected six finite BODY angles")
    origin = body_transform(kin, start)
    desired = origin[[0, 2], 3]+[0, height_mm]
    q = np.array(previous_body if previous_body is not None else start, dtype=float)
    q[[0, 4, 5]] = start[[0, 4, 5]]
    angle_sum = float(sum(start[1:4]))
    for _ in range(30):
        q[3] = angle_sum-q[1]-q[2]
        actual = body_transform(kin, q)[[0, 2], 3]
        residual = desired-actual
        if max(abs(residual)) < 1e-7:
            if q[2] >= -5 or max(abs(q-start)) > 30:
                raise ValueError("Inverse kinematics left the intended branch")
            return q.tolist()
        jacobian = np.zeros((2, 2))
        for column, joint in enumerate((1, 2)):
            shifted = q.copy()
            shifted[joint] += .001
            shifted[3] = angle_sum-shifted[1]-shifted[2]
            jacobian[:, column] = (
                body_transform(kin, shifted)[[0, 2], 3]-actual)/.001
        if np.linalg.cond(jacobian) > 100:
            raise ValueError("Near-singular height solve")
        step = np.linalg.solve(jacobian, residual)
        step *= min(1., 2/max(abs(step)))
        q[1:3] += step
    raise ValueError("Height solve did not converge")


def build_plan(source):
    start = pose(**source["pose"])
    validate_limits(start)
    kin = BodyHeadKinematics()
    initial = body_transform(kin, start["body"])
    points = []
    previous, previous_height = start, 0.
    # First increment is small enough to prove tracking before a larger rise.
    heights = [.5]+list(np.arange(2., 100.1, 2.))
    for height in heights:
        target = copy.deepcopy(start)
        target["body"] = solve_height(kin, start["body"], float(height), previous["body"])
        validate_limits(target)
        subdivisions = max(1, math.ceil(delta(previous, target, ("body",))/.45))
        section_start, section_height = previous, previous_height
        for index in range(1, subdivisions+1):
            subheight = section_height+(height-section_height)*index/subdivisions
            waypoint = copy.deepcopy(start)
            waypoint["body"] = solve_height(kin, start["body"], float(subheight), previous["body"])
            validate_limits(waypoint)
            if delta(previous, waypoint, ("body",)) > .5:
                raise ValueError("Waypoint joint increment exceeds 0.5 degrees")
            for sample in (interpolate(previous, waypoint, i/10) for i in range(11)):
                transform = body_transform(kin, sample["body"])
                if abs(transform[0, 3]-initial[0, 3]) > .02:
                    raise ValueError("Segment leaves the specified base-X corridor")
                if np.max(abs(transform[:3, :3]-initial[:3, :3])) > 1e-7:
                    raise ValueError("Segment changes torso orientation")
                if abs(transform[1, 3]-initial[1, 3]) > 2:
                    raise ValueError("Lateral displacement exceeds 2 mm")
            # All three commanded axes must move inward from the deep squat.
            if (waypoint["body"][1] > previous["body"][1]+1e-8
                    or waypoint["body"][2] < previous["body"][2]-1e-8
                    or waypoint["body"][3] > previous["body"][3]+1e-8):
                raise ValueError("Unexpected joint direction during rise")
            points.append({"height_mm": float(subheight), "pose": waypoint, "seconds": 2.})
            previous, previous_height = waypoint, float(subheight)
    if delta(start, points[0]["pose"], ("body",)) > .2:
        raise ValueError("First proof step exceeds 0.2 degrees")
    final = body_transform(kin, previous["body"])
    if abs(final[2, 3]-initial[2, 3]-100) > 1e-5:
        raise ValueError("Height target is not 100 mm")
    return {
        "schema": "luna_stand_up_100mm_v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_point": SOURCE,
        "source_evidence": source["evidence"],
        "model": str(kin.urdf_path.relative_to(ROOT)),
        "model_sha256": hashlib.sha256(kin.urdf_path.read_bytes()).hexdigest(),
        "distance_definition": "BODY J6 origin +100 mm in fixed-base Z; torso orientation and base X preserved",
        "rise_mm": 100.,
        "start": start, "target": previous, "waypoints": points,
        "initial_transform": initial.tolist(), "target_transform": final.tolist(),
        "predicted_displacement_mm": (final[:3, 3]-initial[:3, 3]).tolist(),
        "controller_ratio_percent": 3.,
        "maximum_joint_step_deg": .5,
        "first_step_maximum_joint_deg": .2,
        "minimum_streaming_duration_seconds": sum(p["seconds"] for p in points),
        "execution_start_tolerance_deg": .03,
        "end_action": "hold raised pose; BODY IDLE after arrival; disconnect SDK",
        "automatic_home": False,
        "hardware_execution_verified": False,
        "external_obstacles_checked": False,
        "motion_commands_sent": 0,
    }


def main():
    source = json.loads((ROOT/SOURCE).read_text("utf-8"))
    plan = build_plan(source)
    destination = ROOT/DESTINATION
    destination.write_text(json.dumps(plan, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in plan.items()
                      if k not in ("waypoints", "initial_transform", "target_transform")},
                     indent=2))
    print("Saved: "+str(destination))


if __name__ == "__main__":
    main()
