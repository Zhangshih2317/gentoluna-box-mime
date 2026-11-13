"""Native Position targets with independent-axis envelope and arrival monitoring."""

import copy
import itertools
import math
import time
from collections import deque

from box_motion_plan import FIELDS, delta, validate_pose
from box_motion_robot import emit, require_zero
from box_mime_taught_plan import controller_ratios
from position_acceptance import PositionAcceptance, POSITION_TOLERANCE_DEG

NATIVE_ARM_NAMES = frozenset((
    "open_before_crouch", "embrace_empty_space", "open_arms_standing", "arms_home"))
ENVELOPE_GRID_DEG = 3.
ENVELOPE_MODEL = "native_position_independent_joint_grid_v1"


def is_native_arm_stage(stage):
    if stage["name"] not in NATIVE_ARM_NAMES:
        return False
    if (set(stage["moving"]) != {"arm0", "arm1"}
            or delta(stage["start"], stage["target"], ("body", "head")) > 1e-9):
        raise ValueError("Native arm stage must hold BODY and HEAD")
    return True


def envelope_axes(stage):
    if not is_native_arm_stage(stage):
        raise ValueError("Not a native arm stage")
    validate_pose(stage["start"])
    validate_pose(stage["target"])
    axes = [(key, i, a, b) for key in ("arm0", "arm1")
            for i, (a, b) in enumerate(zip(stage["start"][key], stage["target"][key]))
            if abs(a-b) > 1e-9]
    # The taught gestures move J3/J4 only. Recovery remains in the old runner.
    if any(i not in (2, 3) for _, i, _, _ in axes) or len(axes) > 4:
        raise ValueError("Native arm envelope is outside taught J3/J4 gestures")
    return axes


def envelope_sample_count(stage):
    return math.prod(math.ceil(abs(b-a)/ENVELOPE_GRID_DEG)+1
                     for _, _, a, b in envelope_axes(stage))


def envelope_samples(stage):
    axes = envelope_axes(stage)
    grids = []
    for _, _, a, b in axes:
        count = math.ceil(abs(b-a)/ENVELOPE_GRID_DEG)
        grids.append([a+(b-a)*i/count for i in range(count+1)])
    for values in itertools.product(*grids):
        result = copy.deepcopy(stage["start"])
        for (key, joint, _, _), value in zip(axes, values):
            result[key][joint] = value
        yield result


def read_native_limits(robot):
    result = {}
    for arm in (0, 1):
        rows = []
        for joint in range(7):
            row = {}
            for field in ("VelMax", "AccMax"):
                name = "R.A%d.L%d.BASIC.%s" % (arm, joint, field)
                code, value = robot.param_get_float(name)
                require_zero(code, "Read "+name)
                if not math.isfinite(value) or value <= 0:
                    raise RuntimeError("Invalid native arm parameter: "+name)
                row[field] = value
            rows.append(row)
        result["arm%d" % arm] = rows
    emit("native_arm_limits_read", limits=result, source="live_controller_parameters")
    return result


def estimated_seconds(stage, limits):
    seconds = 0.
    for key in ("arm0", "arm1"):
        velocity_ratio, acceleration_ratio = controller_ratios(key)
        for a, b, bound in zip(stage["start"][key], stage["target"][key], limits[key]):
            distance = abs(b-a)
            velocity = bound["VelMax"]*velocity_ratio/100.
            acceleration = bound["AccMax"]*acceleration_ratio/100.
            duration = (2*math.sqrt(distance/acceleration) if distance <= velocity**2/acceleration
                        else distance/velocity+velocity/acceleration)
            seconds = max(seconds, duration)
    return seconds


class NativeArrival:
    def __init__(self, stage, deadline):
        envelope_axes(stage)
        self.stage = stage
        self.deadline = deadline
        self.arrival = PositionAcceptance()
        self.guard_details = None

    def update(self, actual, velocities, serial, now):
        validate_pose(actual)
        self.guard_details = None
        for key in FIELDS:
            for joint, (q, a, b) in enumerate(zip(
                    actual[key], self.stage["start"][key], self.stage["target"][key])):
                if abs(a-b) < 1e-9:
                    error = abs(q-a)
                    if error > POSITION_TOLERANCE_DEG+1e-9:
                        self.guard_details = {
                            "group": key, "joint": joint+1, "expected_deg": a,
                            "actual_deg": q, "error_deg": error,
                            "hard_limit_deg": POSITION_TOLERANCE_DEG}
                        raise RuntimeError("Uncommanded native stationary axis: "+key)
                elif (q < min(a, b)-POSITION_TOLERANCE_DEG-1e-9
                      or q > max(a, b)+POSITION_TOLERANCE_DEG+1e-9):
                    self.guard_details = {
                        "group": key, "joint": joint+1, "actual_deg": q,
                        "minimum_deg": min(a, b)-POSITION_TOLERANCE_DEG,
                        "maximum_deg": max(a, b)+POSITION_TOLERANCE_DEG}
                    raise RuntimeError("Native arm outside endpoint envelope: "+key)
            if (len(velocities[key]) != len(actual[key])
                    or not all(math.isfinite(v) for v in velocities[key])):
                raise RuntimeError("Invalid native velocity feedback: "+key)
        arrived = self.arrival.update(actual, self.stage["target"], serial, now)
        if arrived:
            return "complete"
        return "waiting_for_arrival_overdue" if now > self.deadline else "waiting_for_arrival"


def execute_native_stage(runner, stage):
    if runner.native_limits is None:
        raise RuntimeError("Native arm limits have not been read")
    if delta(runner.command, stage["start"]) > .03:
        raise RuntimeError("Native arm stage start mismatch")
    actual, _, _ = runner.read()
    if delta(actual, stage["start"]) > POSITION_TOLERANCE_DEG+1e-9:
        raise RuntimeError("Native arm actual start mismatch")
    began = time.monotonic()
    estimate = estimated_seconds(stage, runner.native_limits)
    policy = NativeArrival(stage, began+max(10., 3*estimate+5.))
    runner.stage = stage
    runner.native_stage = stage
    runner.stream_history = deque([(began, copy.deepcopy(runner.command))])
    previous_speed_bound = runner.max_feedback_speed
    runner.max_feedback_speed = max(
        runner.max_feedback_speed,
        *(row["VelMax"]*controller_ratios(key)[0]/100.+.6
          for key in ("arm0", "arm1") for row in runner.native_limits[key]))
    emit("stage_start", name=stage["name"], target=stage["target"],
         motion_style="native_position", native_controller_time_estimate_seconds=estimate,
         estimate_is_hardware_verified=False, overdue_warning_at=policy.deadline)
    try:
        # Each moving arm receives one complete target. No interpolated points
        # or BODY commands are written by this branch.
        for key, obj in (("arm0", 0), ("arm1", 1)):
            runner.heartbeat()
            target = stage["target"][key]
            require_zero(runner.robot.runtime_set_joint_pos_cmd(obj, target),
                         "Native target "+key)
            runner.command[key] = list(target)
            runner.stream_history.append((time.monotonic(), copy.deepcopy(runner.command)))
        last_poll, last_log, previous_action = time.monotonic(), began, None
        while True:
            time.sleep(.02)
            now = time.monotonic()
            if now-last_poll > .2:
                raise RuntimeError("Native feedback monitoring gap")
            actual, rt, sg = runner.read()
            checked = time.monotonic()
            if checked-now > .2:
                raise RuntimeError("Native feedback read gap")
            last_poll = now
            runner.check_echo(rt)
            velocities = {
                "arm0": rt["arms"][0]["fb"]["fb_vel"],
                "arm1": rt["arms"][1]["fb"]["fb_vel"],
                "body": rt["body"]["fb_vel"], "head": [0.]*len(actual["head"])}
            sample = {
                "observed_monotonic": checked, "frame_serial": rt["frame_serial"],
                "stage": stage["name"], "motion_style": "native_position",
                "actual": copy.deepcopy(actual), "velocity_deg_s": copy.deepcopy(velocities),
                "command_before_read": copy.deepcopy(runner.command),
                "command_after_tick": copy.deepcopy(runner.command),
                "input_echo": {key: list(rt["arms"][i]["cmd"]["joint_pos"])
                               for i, key in enumerate(("arm0", "arm1"))},
                "controller_ratios_observed": {
                    key: {f: sg["arms"][i]["set"][f] for f in ("vel_ratio", "acc_ratio")}
                    for i, key in enumerate(("arm0", "arm1"))},
                "read_started": runner.last_read_started,
                "read_finished": runner.last_read_finished,
                "action": "observation", "elapsed_seconds": checked-began}
            # Save before policy evaluation so the rejecting frame is retained.
            if runner.response_samples is not None:
                runner.response_sample_count += 1
                runner.response_samples.append(sample)
            try:
                action = policy.update(actual, velocities, rt["frame_serial"], checked)
            except RuntimeError as error:
                sample.update(action="guard_rejected", error=str(error),
                              guard_details=policy.guard_details)
                emit("native_guard_rejected", name=stage["name"], error=str(error),
                     guard_details=policy.guard_details, actual=actual,
                     velocities=velocities, frame_serial=rt["frame_serial"])
                raise
            sample.update(action=action)
            if action != previous_action or checked-last_log >= 5.:
                emit("native_arm_status", name=stage["name"], action=action,
                     elapsed_seconds=checked-began,
                     target_error_deg=delta(actual, stage["target"]),
                     peak_feedback_velocity_deg_s=max(
                         abs(v) for key in ("arm0", "arm1") for v in velocities[key]))
                previous_action, last_log = action, checked
            if action == "complete":
                if runner.geometry is not None:
                    runner.geometry.check(actual)
                emit("stage_complete", name=stage["name"], pose=actual,
                     motion_style="native_position", elapsed_seconds=checked-began)
                return actual
    finally:
        runner.max_feedback_speed = previous_speed_bound
        runner.native_stage = None
        runner.stream_history = None
