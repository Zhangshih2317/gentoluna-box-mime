"""Empty-handed motion using taught squat coordination and negative B6 yaw."""

import copy
import math

from box_motion_plan import FIELDS, delta, interpolate, pose, validate_pose
from coordinated_joint_timing import AxisLimits, plan_timing

SPEED_PERCENT = 20.
ACCELERATION_PERCENT = 5.
SQUAT_DEG = 30.
SQUAT_DROP_MM = None
TURN_DEG = -45.
STREAM_RATE = 50
STREAM_SPEED = .9*(SPEED_PERCENT/5.)
STREAM_ACCELERATION = .45
BODY_STREAM_SPEED = 7.2
BODY_STREAM_ACCELERATION = 1.8
COORDINATED_TIMING = False
NATIVE_ARM_POSITION = False
TIMING_PROFILE = "legacy_quintic"
CONTROLLER_RATIOS = {key: (SPEED_PERCENT, ACCELERATION_PERCENT)
                     for key in ("arm0", "arm1", "body")}


def validate_speed_percent(value):
    if isinstance(value, bool):
        raise ValueError("Motion speed must be a finite percentage from 1 to 100")
    value = float(value)
    if not math.isfinite(value) or not 1. <= value <= 100.:
        raise ValueError("Motion speed must be a finite percentage from 1 to 100")
    return value


def configure_speed(percent, acceleration_percent=None):
    """Apply before importing the runner; preserve the 20% baseline at scale 1."""
    global SPEED_PERCENT, ACCELERATION_PERCENT, STREAM_SPEED, STREAM_ACCELERATION
    global BODY_STREAM_SPEED, BODY_STREAM_ACCELERATION, COORDINATED_TIMING, TIMING_PROFILE
    global CONTROLLER_RATIOS, NATIVE_ARM_POSITION
    NATIVE_ARM_POSITION = False
    COORDINATED_TIMING, TIMING_PROFILE = False, "legacy_quintic"
    SPEED_PERCENT = validate_speed_percent(percent)
    ACCELERATION_PERCENT = (5. if acceleration_percent is None
                            else validate_speed_percent(acceleration_percent))
    CONTROLLER_RATIOS = {key: (SPEED_PERCENT, ACCELERATION_PERCENT)
                         for key in ("arm0", "arm1", "body")}
    scale = SPEED_PERCENT/20.
    STREAM_SPEED = 3.6*scale
    BODY_STREAM_SPEED = 7.2*scale
    # Legacy callers retain speed-scaled ramps; HOME supplies an independent
    # acceleration ratio against the original 5% command budget.
    accel_scale = scale if acceleration_percent is None else ACCELERATION_PERCENT/5.
    STREAM_ACCELERATION = .45*accel_scale
    BODY_STREAM_ACCELERATION = 1.8*accel_scale


def configure_coordination():
    """Opt in only for the mime entry; the separate HOME entry keeps its timing."""
    global COORDINATED_TIMING, TIMING_PROFILE
    global STREAM_ACCELERATION, BODY_STREAM_ACCELERATION
    COORDINATED_TIMING, TIMING_PROFILE = True, "common_phase_command_budget_v1"
    # These are software budgets from the existing 20% trajectory, not SDK
    # motor limits. Below 20% reduce them too; above 20% never increase them.
    scale = min(1., SPEED_PERCENT/20.)
    STREAM_ACCELERATION = .45*scale
    BODY_STREAM_ACCELERATION = 1.8*scale


def configure_groups(arm_speed, arm_acceleration, body_speed, body_acceleration):
    """Independent user controls; legacy configure_speed is HOME-only."""
    global CONTROLLER_RATIOS, STREAM_SPEED, STREAM_ACCELERATION, BODY_STREAM_ACCELERATION
    global TIMING_PROFILE
    arm_speed, arm_acceleration, body_speed, body_acceleration = map(
        validate_speed_percent, (arm_speed, arm_acceleration, body_speed, body_acceleration))
    configure_speed(body_speed)
    configure_coordination()
    CONTROLLER_RATIOS = {
        "arm0": (arm_speed, arm_acceleration),
        "arm1": (arm_speed, arm_acceleration),
        "body": (body_speed, body_acceleration),
    }
    STREAM_SPEED = 3.6*arm_speed/20.
    # Explicit acceleration control scales the old 5% software budget.
    # These are command limits, not calibrated physical actuator maxima.
    STREAM_ACCELERATION = .45*arm_acceleration/5.
    BODY_STREAM_ACCELERATION *= body_acceleration/5.
    TIMING_PROFILE = "common_phase_group_ratios_v2"


def controller_ratios(key):
    return CONTROLLER_RATIOS[key]


def configure_native_arm_position():
    global NATIVE_ARM_POSITION
    NATIVE_ARM_POSITION = True


def controller_settings():
    return {key: {"speed_percent": values[0], "acceleration_percent": values[1]}
            for key, values in CONTROLLER_RATIOS.items()}


def coordinated_timing(stage):
    """Allocate each group's command budget by displacement, with one phase."""
    start, target, limits, axes = [], [], [], []
    for key in FIELDS:
        distances = [abs(b-a) for a, b in zip(stage["start"][key], stage["target"][key])]
        largest = max(distances)
        if key == "head" and largest > 0:
            raise ValueError("Coordinated mime must hold HEAD")
        reference_speed = BODY_STREAM_SPEED if key == "body" else STREAM_SPEED
        acceleration = BODY_STREAM_ACCELERATION if key == "body" else STREAM_ACCELERATION
        for i, (a, b, distance) in enumerate(zip(stage["start"][key], stage["target"][key], distances)):
            axes.append(key+":"+str(i+1))
            start.append(a)
            target.append(b)
            weight = distance/largest if largest else 0.
            # A one-second acceleration-change budget bounds quintic jerk.
            # No per-axis delay advance or guessed motor gain is introduced.
            limits.append(AxisLimits(reference_speed*weight, reference_speed*weight,
                                     acceleration*weight, acceleration*weight)
                          if weight else None)
    # The group-specific percentage has already been applied to each limit.
    result = plan_timing(start, target, limits, 100.,
                         reference_percent=100., rate_hz=STREAM_RATE)
    # Keep the existing minimum segment duration, including tiny recoveries.
    if result["seconds"] < 2.:
        ratio = result["seconds"]/2.
        result["seconds"], result["ticks"] = 2., 2*STREAM_RATE
        for name, exponent in (("signed_peak_velocity_deg_s", 1),
                               ("peak_acceleration_deg_s2", 2), ("peak_jerk_deg_s3", 3)):
            result[name] = [value*ratio**exponent for value in result[name]]
    result.update(axes=axes, profile=TIMING_PROFILE,
                  limit_basis="software_command_budgets_not_motor_calibration",
                  dynamic_delay_compensation=False)
    return result


def configure_squat(knee_delta_deg, depth_mm):
    """Use an offline FK solution; the model preflight checks its physical drop."""
    global SQUAT_DEG, SQUAT_DROP_MM
    degrees, depth = float(knee_delta_deg), float(depth_mm)
    if (not math.isfinite(degrees) or not 0 < degrees < 140
            or not math.isfinite(depth) or not 0 < depth <= 300):
        raise ValueError("Invalid height-based squat configuration")
    SQUAT_DEG, SQUAT_DROP_MM = degrees, depth


def continuous_limits(stage):
    body_only = (delta(stage["start"], stage["target"], ("body",)) > 1e-9
                 and delta(stage["start"], stage["target"], ("arm0", "arm1")) <= 1e-9)
    return ((BODY_STREAM_SPEED, BODY_STREAM_ACCELERATION) if body_only
            else (STREAM_SPEED, STREAM_ACCELERATION))


def continuous_seconds(stage):
    if COORDINATED_TIMING:
        return coordinated_timing(stage)["seconds"]
    distance = delta(stage["start"], stage["target"])
    speed, acceleration = continuous_limits(stage)
    return max(2., 1.875*distance/speed,
               math.sqrt((10/math.sqrt(3))*distance/acceleration))


def build_stages(start, home, squat, waist):
    start, home, squat, waist = [pose(**p) for p in (start, home, squat, waist)]
    for state in (start, home, waist):
        validate_pose(state)
    if delta(start, home, ("head",)) > .05:
        raise ValueError("Head must be near saved HOME")
    interrupted_crouch = (
        0 < home["body"][2]-start["body"][2] <= 3.
        and abs(start["body"][5]-home["body"][5]) <= .05
        and abs(start["arm0"][2]-home["arm0"][2]+18) <= .05
        and abs(start["arm1"][2]-home["arm1"][2]-18) <= .05)
    interrupted_turn = (
        TURN_DEG <= start["body"][5]-home["body"][5] <= 0
        and max(abs(start["body"][i]-home["body"][i]) for i in range(5)) <= .05
        and all(abs(start[key][2]-home[key][2]-sign*12) <= .05
                and abs(start[key][3]-home[key][3]-10) <= .05
                for key, sign in (("arm0", 1), ("arm1", -1))))
    for key, direction in (("arm0", -1), ("arm1", 1)):
        ignored = (2, 3) if interrupted_turn else (2,)
        if any(abs(start[key][i]-home[key][i]) > .05 for i in range(7) if i not in ignored):
            raise ValueError("Unknown arm pose; manual HOME recovery required")
        opening = direction*(start[key][2]-home[key][2])
        if not interrupted_turn and not -.05 <= opening <= (18.05 if interrupted_crouch else 2.):
            raise ValueError("Arm is outside the small interrupted-opening recovery region")
    if not interrupted_crouch and max(abs(start["body"][i]-home["body"][i]) for i in range(5)) > .05:
        raise ValueError("Start must be the taught standing posture")
    yaw = start["body"][5]
    if not interrupted_turn and min(abs(yaw-home["body"][5])/2., abs(yaw-waist["body"][5])/.05) > 1:
        raise ValueError("Start yaw must be near HOME or the recorded waist endpoint")
    if max(abs(waist["body"][i]-home["body"][i]) for i in range(5)) > .05:
        raise ValueError("Waist teaching changed other BODY axes")
    if not -46 <= waist["body"][5]-home["body"][5] <= -43:
        raise ValueError("Expected the demonstrated negative B6 turn")
    hip = squat["body"][1]-home["body"][1]
    knee = squat["body"][2]-home["body"][2]
    if not 0 < hip < -knee or knee >= -SQUAT_DEG:
        raise ValueError("Invalid taught squat coordination")

    home = copy.deepcopy(home)
    home["head"] = start["head"].copy()
    crouch = home["body"].copy()
    crouch[1] += SQUAT_DEG*hip/(-knee)
    crouch[2] -= SQUAT_DEG
    crouch[3] = sum(home["body"][1:4])-sum(crouch[1:3])
    if interrupted_crouch:
        fraction = (home["body"][2]-start["body"][2])/SQUAT_DEG
        expected = [a+(b-a)*fraction for a, b in zip(home["body"], crouch)]
        if max(abs(a-b) for a, b in zip(expected, start["body"])) > .05:
            raise ValueError("Interrupted crouch is outside the coordinated path")
    if max(abs(a-b) for a, b in zip(crouch, home["body"])) > SQUAT_DEG+1e-9:
        raise ValueError("Squat exceeds the configured joint excursion")
    turned = home["body"].copy()
    turned[5] += TURN_DEG
    opened, embrace = {}, {}
    for key, sign in (("arm0", 1), ("arm1", -1)):
        opened[key] = home[key].copy()
        opened[key][2] -= sign*18
        embrace[key] = home[key].copy()
        embrace[key][2] += sign*12
        embrace[key][3] += 10

    stages = []
    current = copy.deepcopy(start)

    def add(name, groups, body=None, arms=None):
        nonlocal current
        target = copy.deepcopy(current)
        if body is not None:
            target["body"] = body.copy()
        if arms is not None:
            for key in ("arm0", "arm1"):
                target[key] = arms[key].copy()
        validate_pose(target)
        if delta(current, target) > 1e-9:
            stages.append({"name": name, "moving": list(groups),
                           "start": current, "target": target})
        current = target

    if interrupted_crouch:
        add("align_body_from_crouch", ("body",), body=home["body"])
    if interrupted_turn:
        add("align_waist_from_turn", ("body",), body=home["body"])
    add("align_arms_home", ("arm0", "arm1"), arms=home)
    add("align_standing_home", ("body", "arm0", "arm1"),
        body=home["body"], arms=home)
    add("open_before_crouch", ("arm0", "arm1"), arms=opened)
    add("crouch", ("body",), body=crouch)
    add("embrace_empty_space", ("arm0", "arm1"), arms=embrace)
    add("stand", ("body",), body=home["body"])
    add("turn_waist_negative_45", ("body",), body=turned)
    add("open_arms_standing", ("arm0", "arm1"), arms=opened)
    add("arms_home", ("arm0", "arm1"), arms=home)
    add("body_home", ("body",), body=home["body"])
    return stages


def waypoints(stage):
    start, target = stage["start"], stage["target"]
    distance = delta(start, target)
    proof = interpolate(start, target, min(1., .2/max(distance, 1e-9)))
    points = [{"pose": proof, "seconds": 2.}]
    if delta(proof, target) > 1e-9:
        count = math.ceil(delta(proof, target)/.49)
        points.extend({"pose": interpolate(proof, target, i/count), "seconds": 1.}
                      for i in range(1, count+1))
    return points
