"""Robot-side recovery; TARGET and EXECUTE are supplied by the dispatcher."""

import copy
import math
import re
import signal
import sys
import time

from box_motion_plan import (ARM_LIMITS, BODY_LIMITS, FIELDS, blend,
                             collision_samples, delta, interpolate, pose)
from box_motion_robot import Geometry, SDK_ROOT, emit, require_zero

OBJECTS = {"arm0": 0, "arm1": 1, "body": 3}


def validate_limits(state):
    pose(**state)
    for key, limits in (("arm0", ARM_LIMITS), ("arm1", ARM_LIMITS), ("body", BODY_LIMITS)):
        if any(not low <= q <= high for q, (low, high) in zip(state[key], limits)):
            raise RuntimeError("Hard joint limit: "+key)


def guard_pose(actual, command, start, target, moving):
    validate_limits(actual)
    for key in FIELDS:
        tolerance = .25 if key in moving else .05
        if max(abs(a-b) for a, b in zip(actual[key], command[key])) > tolerance:
            raise RuntimeError("Tracking/uncommanded-motion violation: "+key)
        if key in moving:
            for q, a, b in zip(actual[key], start[key], target[key]):
                if not min(a, b)-.12 <= q <= max(a, b)+.12:
                    raise RuntimeError("Segment envelope violation: "+key)


class Recovery:
    max_feedback_speed = 1.5

    def __init__(self, robot):
        self.robot = robot
        self.serial = None
        self.fresh_at = time.monotonic()
        self.enabled = set()
        self.changed = False

    def read(self, allow_disabled=True):
        if self.robot.check_link_state() != 1:
            raise RuntimeError("SDK link lost")
        rt, sg = self.robot.get_rt_dict(), self.robot.get_sg_dict()
        now = time.monotonic()
        if rt["frame_serial"] != self.serial:
            self.serial, self.fresh_at = rt["frame_serial"], now
        elif now-self.fresh_at > .25:
            raise RuntimeError("Stale RT feedback")
        state = pose(rt["arms"][0]["fb"]["fb_pos"], rt["arms"][1]["fb"]["fb_pos"],
                     rt["body"]["fb_pos"], rt["head"]["fb_pos"])
        validate_limits(state)
        for key, obj in OBJECTS.items():
            raw = rt["body"] if key == "body" else rt["arms"][obj]
            config = sg["body"] if key == "body" else sg["arms"][obj]
            err, cur = raw["state"]["err"], raw["state"]["cur"]
            allowed = ((100, 110), (0, 0), (1, 0)) if allow_disabled else ((0, 0), (1, 0))
            if (cur, err) not in allowed or (key in self.enabled and (cur, err) != (1, 0)):
                raise RuntimeError("Unexpected state "+key+": "+str(raw["state"]))
            if config["get"]["traj_state"] not in (0, 1, 2):
                raise RuntimeError("Buffered trajectory active: "+key)
            fb = raw if key == "body" else raw["fb"]
            if (len(fb["fb_vel"]) != len(state[key])
                    or any(not math.isfinite(v) or abs(v) > self.max_feedback_speed for v in fb["fb_vel"])):
                raise RuntimeError("Unexpected joint velocity: "+key)
        if rt["head"]["state"]["err"]:
            raise RuntimeError("HEAD error")
        return state, rt, sg

    def stationary(self):
        first, rt, _ = self.read()
        serials = {rt["frame_serial"]}
        for _ in range(4):
            time.sleep(.1)
            state, rt, _ = self.read()
            serials.add(rt["frame_serial"])
            if delta(first, state) > .02:
                raise RuntimeError("Not stationary")
        if len(serials) != 5:
            raise RuntimeError("Stationary check needs five distinct frames")
        return state

    def enable(self, key):
        before = self.stationary()
        _, rt, _ = self.read()
        obj = OBJECTS[key]
        raw = rt["body"] if key == "body" else rt["arms"][obj]
        self.changed = True
        if raw["state"]["err"] == 110:
            result = self.robot.reset_error(obj, 2000)
            emit("reset_known_disable", group=key, result=result)
            require_zero(result[0], "Reset known disable")
        elif raw["state"]["cur"] != 0:
            require_zero(self.robot.switch_to_idle(obj, 2000), "Enter IDLE")
        if delta(before, self.stationary()) > .02:
            raise RuntimeError("Pose drift during reset")
        require_zero(self.robot.switch_to_position_mode(obj, 2000, 3., 3.), "Position mode")
        self.enabled.add(key)
        after, _, sg = self.read()
        if delta(before, after) > .02:
            raise RuntimeError("Pose drift during position entry")
        config = sg["body"] if key == "body" else sg["arms"][obj]
        if any(abs(config["set"][name]-3.) > .1 for name in ("vel_ratio", "acc_ratio")):
            raise RuntimeError("Speed/acceleration ratio mismatch")
        emit("position_ready", group=key, pose=after)

    def send(self, command):
        for key in sorted(self.enabled):
            require_zero(self.robot.runtime_set_joint_pos_cmd(OBJECTS[key], command[key]),
                         "Stream "+key)

    def segment(self, start, target, moving, seconds=2.):
        began, last_tick = time.monotonic(), time.monotonic()
        command = copy.deepcopy(start)
        while True:
            now = time.monotonic()
            if now-last_tick > .2:
                raise RuntimeError("Position stream scheduling gap")
            last_tick = now
            actual, _, _ = self.read()
            guard_pose(actual, command, start, target, moving)
            fraction = min(1., (now-began)/seconds)
            command = interpolate(start, target, blend(fraction))
            self.send(command)
            if fraction == 1.:
                break
            time.sleep(.02)
        deadline = time.monotonic()+3.
        stable = 0
        while time.monotonic() < deadline:
            self.send(target)
            time.sleep(.02)
            actual, rt, _ = self.read()
            guard_pose(actual, target, start, target, moving)
            for key in self.enabled:
                fb = rt["body"] if key == "body" else rt["arms"][OBJECTS[key]]["fb"]
                if any(not math.isfinite(v) for v in fb["cmd_pos"]):
                    raise RuntimeError("Non-finite command echo")
                if max(abs(a-b) for a, b in zip(fb["cmd_pos"], target[key])) > .05:
                    raise RuntimeError("Position command echo mismatch: "+key)
            if delta(actual, target, moving) <= .025:
                stable += 1
                if stable >= 10:
                    return actual
            else:
                stable = 0
        raise RuntimeError("Segment did not settle")

    def move(self, target, moving, name):
        start = self.stationary()
        distance = delta(start, target, moving)
        emit("stage_start", name=name, start=start, target=target)
        # Validate a 0.2-degree first increment before longer 0.5-degree segments.
        proof = interpolate(start, target, min(1., .2/max(distance, 1e-9)))
        current = self.segment(start, proof, moving)
        emit("small_step_verified", name=name, pose=current)
        count = max(1, math.ceil(delta(proof, target, moving)/.5))
        for index in range(1, count+1):
            next_target = interpolate(proof, target, index/count)
            current = self.segment(current, next_target, moving)
            if index == 1 or index % 5 == 0 or index == count:
                emit("recovery_progress", name=name, segment=index, total=count, pose=current)
        emit("stage_complete", name=name, pose=current)
        return current

    def emergency(self, error):
        emit("recovery_failed", error=str(error))
        if not self.changed:
            return
        returned = self.robot.emergency_stop(11)
        emit("emergency_stop_requested", returned_mask=returned)
        previous, serial, stable = None, None, 0
        for _ in range(20):
            time.sleep(.1)
            rt = self.robot.get_rt_dict()
            current = pose(rt["arms"][0]["fb"]["fb_pos"], rt["arms"][1]["fb"]["fb_pos"],
                           rt["body"]["fb_pos"], rt["head"]["fb_pos"])
            fresh = serial is not None and rt["frame_serial"] != serial
            stable = stable+1 if fresh and previous and delta(current, previous) < .02 else 0
            previous, serial = current, rt["frame_serial"]
            if stable >= 5:
                emit("stationary_after_stop", pose=current,
                     states=[rt["arms"][0]["state"], rt["arms"][1]["state"], rt["body"]["state"]])
                return
        emit("stop_unconfirmed", message="Use physical emergency stop")


def run(target, execute):
    import fcntl
    sys.path.insert(0, SDK_ROOT+"/PYTHON_SDK")
    from GentoRobot import GentoRobot
    geometry, runner = None, None
    robot = GentoRobot()
    with open("/tmp/gento_sdk_6_6_7_190.lock", "a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            robot.link(*map(int, __import__("os").environ["LUNA_CONTROLLER_IP"].split(".")))
            if not robot._connected:
                raise RuntimeError("SDK unavailable")
            runner = Recovery(robot)
            start = runner.stationary()
            if delta(start, target, ("head",)) > .05:
                raise RuntimeError("HEAD differs from saved start")
            target = copy.deepcopy(target)
            target["head"] = start["head"]
            validate_limits(target)
            for key, obj in OBJECTS.items():
                result, text = robot.get_servo_error_codes(obj)
                codes = re.findall(r"0x([0-9A-Fa-f]{4})", text)
                if result or len(codes) != len(start[key]) or any(int(c, 16) for c in codes):
                    raise RuntimeError("Servo diagnostic rejection: "+key)
            emit("initial_feedback", pose=start, target=target)
            body_target = copy.deepcopy(start)
            body_target["body"] = target["body"]
            stages = [{"start": start, "target": body_target},
                      {"start": body_target, "target": target}]
            geometry = Geometry()
            geometry.verify_sdk(robot, stages)
            minimum, count = math.inf, 0
            for stage in stages:
                for sample in collision_samples(stage, .25):
                    spacing, _ = geometry.check(sample)
                    minimum, count = min(minimum, spacing), count+1
            emit("recovery_geometry_verified", samples=count,
                 minimum_flange_spacing_mm=minimum, maximum_sample_step_deg=.25)
            if delta(start, runner.stationary()) > .02:
                raise RuntimeError("Pose changed during preflight")
            if not execute:
                emit("preflight_complete", motion_commands_sent=0)
                return
            runner.enable("body")
            runner.segment(start, start, ("body",), .5)
            runner.move(body_target, ("body",), "body_to_saved_start")
            runner.enable("arm0")
            runner.enable("arm1")
            current = runner.stationary()
            runner.segment(current, current, ("arm0", "arm1"), .5)
            runner.move(target, ("arm0", "arm1"), "arms_to_home")
            final = runner.stationary()
            if delta(final, target) > .05:
                raise RuntimeError("Final pose mismatch")
            # The existing BODY position-stream wrapper returns to IDLE after arrival.
            require_zero(robot.switch_to_idle(3, 2000), "BODY IDLE after arrival")
            runner.enabled.remove("body")
            final = runner.stationary()
            if delta(final, target) > .05:
                raise RuntimeError("Final pose drift after BODY IDLE")
            emit("recovery_complete", pose=final, maximum_error_deg=delta(final, target))
        except BaseException as exc:
            if runner:
                runner.emergency(exc)
            raise
        finally:
            if robot._connected:
                robot.unlink()
                emit("sdk_disconnected")
            if geometry:
                geometry.close()


if __name__ == "__main__":
    def interrupted(signum, frame):
        raise RuntimeError("Recovery interrupted")
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    run(TARGET, EXECUTE)
