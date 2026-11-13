"""Checked BODY position streaming for a saved one-shot 100 mm rise."""

import math
import re
import sys
import time

from box_motion_plan import FIELDS, delta, interpolate, pose
from box_motion_robot import Geometry, SDK_ROOT, emit, require_zero
from recover_box_start_robot import Recovery, validate_limits


def model_transform(geometry, state):
    js = geometry.state_type(tuple(state["arm0"]), tuple(state["arm1"]),
                             tuple(state["body"]), (state["head"][1], state["head"][0]))
    return geometry.checker.model.link_transforms(js.radians_by_name())["Leg_J6_Link"]


def displacement_mm(a, b):
    return [(b[i][3]-a[i][3])*1000 for i in range(3)]


def verify_plan_structure(plan):
    if plan["schema"] != "luna_stand_up_100mm_v1" or plan["rise_mm"] != 100.:
        raise ValueError("Only the fixed 100 mm rise plan is supported")
    start, target = pose(**plan["start"]), pose(**plan["target"])
    validate_limits(start)
    validate_limits(target)
    waypoints = plan["waypoints"]
    if not 2 <= len(waypoints) <= 150:
        raise ValueError("Invalid number of waypoints")
    prior, height = start, 0.
    for index, point in enumerate(waypoints):
        state = pose(**point["pose"])
        validate_limits(state)
        if not math.isfinite(point["height_mm"]) or not height < point["height_mm"] <= 100:
            raise ValueError("Height must increase monotonically to 100 mm")
        if point["seconds"] != 2.:
            raise ValueError("Expected two-second small segments")
        if delta(prior, state, ("body",)) > (.2 if index == 0 else .5):
            raise ValueError("Excessive joint step")
        if delta(start, state, ("arm0", "arm1", "head")) > 1e-9:
            raise ValueError("Rise may not command arms or head")
        if any(abs(state["body"][i]-start["body"][i]) > 1e-9 for i in (0, 4, 5)):
            raise ValueError("B1/B5/B6 must remain fixed")
        if abs(sum(state["body"][1:4])-sum(start["body"][1:4])) > 1e-7:
            raise ValueError("Torso pitch compensation mismatch")
        if (state["body"][1] > prior["body"][1]+1e-8
                or state["body"][2] < prior["body"][2]-1e-8
                or state["body"][3] > prior["body"][3]+1e-8):
            raise ValueError("Wrong rise direction")
        prior, height = state, point["height_mm"]
    if height != 100. or delta(prior, target) > 1e-9:
        raise ValueError("Final waypoint mismatch")


class RiseRunner(Recovery):
    def __init__(self, robot):
        super().__init__(robot)
        self.task_start = None
        self.previous_time = None
        self.previous_pose = None

    def read(self, allow_disabled=False):
        state, rt, sg = super().read(allow_disabled=False)
        if self.task_start:
            if delta(state, self.task_start, ("arm0", "arm1", "head")) > .05:
                raise RuntimeError("Arm/head movement during BODY-only task")
            if any(abs(state["body"][i]-self.task_start["body"][i]) > .05 for i in (0, 4, 5)):
                raise RuntimeError("Uncommanded B1/B5/B6 movement")
            if abs(sum(state["body"][1:4])-sum(self.task_start["body"][1:4])) > .2:
                raise RuntimeError("Torso pitch tracking mismatch")
        if self.enabled:
            if any(abs(sg["body"]["set"][name]-3.) > .1
                   for name in ("vel_ratio", "acc_ratio")):
                raise RuntimeError("BODY controller ratio changed")
        now = time.monotonic()
        if self.previous_time is not None:
            duration = now-self.previous_time
            if duration > 0 and delta(state, self.previous_pose) > 1.5*duration+.04:
                raise RuntimeError("Measured joint jump")
        self.previous_time, self.previous_pose = now, state
        return state, rt, sg

    def enable_body(self):
        before = self.stationary()
        _, rt, _ = self.read()
        self.changed = True
        if rt["body"]["state"]["cur"] == 0:
            require_zero(self.robot.switch_to_position_mode(3, 2000, 3., 3.), "BODY Position")
        elif rt["body"]["state"]["cur"] == 1:
            require_zero(self.robot.runtime_set_speed_ratio(3, 3., 3.), "BODY ratios")
        else:
            raise RuntimeError("BODY is not IDLE or Position; automatic reset is prohibited")
        deadline = time.monotonic()+1.
        while True:
            current, _, sg = self.read()
            if delta(before, current) > .02:
                raise RuntimeError("Pose drift during BODY preparation")
            if all(abs(sg["body"]["set"][name]-3.) <= .1 for name in ("vel_ratio", "acc_ratio")):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("BODY ratio update not confirmed")
            time.sleep(.02)
        self.enabled.add("body")
        self.task_start = before
        emit("body_ready", pose=before, velocity_ratio=3., acceleration_ratio=3.)


def validate_geometry(geometry, plan, robot):
    stages = [{"start": plan["start"], "target": plan["target"]}]
    geometry.verify_sdk(robot, stages)
    initial = model_transform(geometry, plan["start"])
    # Independently compare the planning model with the on-robot collision model.
    if max(abs(initial[i][3]*1000-plan["initial_transform"][i][3]) for i in range(3)) > .05:
        raise RuntimeError("Planning/collision BODY origins disagree")
    if max(abs(initial[i][j]-plan["initial_transform"][i][j])
           for i in range(3) for j in range(3)) > 1e-5:
        raise RuntimeError("Planning/collision BODY orientations disagree")
    previous, last_z = plan["start"], initial[2][3]
    count, minimum = 0, math.inf
    for point in plan["waypoints"]:
        # Samples cover the same piecewise joint path used by the streaming code.
        count_in_segment = max(1, math.ceil(delta(previous, point["pose"])/.1))
        for index in range(count_in_segment+1):
            sample = interpolate(previous, point["pose"], index/count_in_segment)
            spacing, _ = geometry.check(sample)
            transform = model_transform(geometry, sample)
            change = displacement_mm(initial, transform)
            if abs(change[0]) > .05 or abs(change[1]) > 2:
                raise RuntimeError("Rise leaves the X/Y corridor")
            if max(abs(transform[i][j]-initial[i][j])
                   for i in range(3) for j in range(3)) > 1e-5:
                raise RuntimeError("Rise changes torso rotation")
            if transform[2][3] < last_z-1e-8:
                raise RuntimeError("Rise height is not monotonic")
            last_z = transform[2][3]
            minimum, count = min(minimum, spacing), count+1
        if abs(change[2]-point["height_mm"]) > .05:
            raise RuntimeError("Waypoint height mismatch")
        previous = point["pose"]
    if abs(change[2]-100.) > .05:
        raise RuntimeError("Plan does not rise 100 mm")
    emit("geometry_verified", samples=count, minimum_flange_spacing_mm=minimum,
         displacement_mm=change, continuous_collision_detection=False,
         external_obstacles_checked=False)
    return initial


def run(plan, execute):
    import fcntl
    verify_plan_structure(plan)
    sys.path.insert(0, SDK_ROOT+"/PYTHON_SDK")
    from GentoRobot import GentoRobot
    robot, runner, geometry = GentoRobot(), None, None
    with open("/tmp/gento_sdk_6_6_7_190.lock", "a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            robot.link(*map(int, __import__("os").environ["LUNA_CONTROLLER_IP"].split(".")))
            if not robot._connected:
                raise RuntimeError("Disconnect the teaching client before running this program")
            runner = RiseRunner(robot)
            start = runner.stationary()
            if delta(start, plan["start"]) > .03:
                raise RuntimeError("Current pose differs from captured squat; refusing repeat or stale execution")
            for key, obj, size in (("arm0", 0, 7), ("arm1", 1, 7), ("body", 3, 6)):
                code, text = robot.get_servo_error_codes(obj)
                codes = re.findall(r"0x([0-9A-Fa-f]{4})", text)
                if code or len(codes) != size or any(int(c, 16) for c in codes):
                    raise RuntimeError("Servo error: "+key)
            emit("initial_feedback", pose=start, target=plan["target"], execute=execute)
            geometry = Geometry()
            initial = validate_geometry(geometry, plan, robot)
            if delta(start, runner.stationary()) > .02:
                raise RuntimeError("Pose changed during preflight")
            if not execute:
                emit("preflight_complete", motion_commands_sent=0, mode_changes_sent=0)
                return
            runner.enable_body()
            current = runner.stationary()
            runner.segment(current, current, ("body",), .5)
            for index, point in enumerate(plan["waypoints"], start=1):
                current = runner.segment(current, point["pose"], ("body",), point["seconds"])
                actual_transform = model_transform(geometry, current)
                displacement = displacement_mm(initial, actual_transform)
                if abs(displacement[2]-point["height_mm"]) > 1. or abs(displacement[0]) > 1.:
                    raise RuntimeError("Measured BODY pose misses height/position waypoint")
                if index == 1 or index % 5 == 0 or index == len(plan["waypoints"]):
                    emit("rise_progress", waypoint=index, total=len(plan["waypoints"]),
                         pose=current, model_displacement_mm=displacement)
            final = runner.stationary()
            if delta(final, plan["target"]) > .03:
                raise RuntimeError("Final joint target not reached")
            require_zero(robot.switch_to_idle(3, 2000), "BODY IDLE after arrival")
            runner.enabled.remove("body")
            final = runner.stationary()
            if delta(final, plan["target"]) > .03:
                raise RuntimeError("Pose drift after IDLE")
            displacement = displacement_mm(initial, model_transform(geometry, final))
            if abs(displacement[2]-100.) > 1.:
                raise RuntimeError("Final model height not within 1 mm")
            emit("rise_complete", pose=final, model_displacement_mm=displacement,
                 measured_height_sensor_used=False, automatic_home=False)
        except BaseException as error:
            if runner:
                runner.emergency(error)
            raise
        finally:
            if robot._connected:
                robot.unlink()
                emit("sdk_disconnected")
            if geometry:
                geometry.close()
