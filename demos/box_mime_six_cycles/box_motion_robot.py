"""Robot-side preflight and staged empty-handed gesture; never runs on import."""

import argparse
import ctypes
from datetime import datetime, timezone
import json
import math
import signal
import sys
import time

from box_motion_plan import (HOME, RATE, FIELDS, build_plan, collision_samples,
                             delta, pose, validate_pose)

SDK_ROOT = __import__("os").environ.get("LUNA_SDK_ROOT", "/opt/gento-sdk")
MODEL = __import__("os").environ.get("LUNA_MODEL_PATH", "/opt/gento-model/gento_luna_full.urdf")
RATIO = 5.
NAMES = ("Arm0", "Arm1", "Body")
KEYS = ("arm0", "arm1", "body")
OBJECTS = (0, 1, 3)


def emit(event, **data):
    print(json.dumps({"event": event, "time": datetime.now(timezone.utc).isoformat(),
                      **data}, ensure_ascii=True), flush=True)


def require_zero(code, operation):
    if code != 0:
        raise RuntimeError("%s failed: %s" % (operation, code))


class Geometry:
    def __init__(self):
        sys.path.insert(0, SDK_ROOT+"/robot")
        from collision_guard import CollisionChecker, JointState
        from collision_guard.geometry import boxes_intersect, world_box
        self.checker = CollisionChecker(MODEL)
        self.state_type = JointState
        self.boxes_intersect = boxes_intersect
        self.world_box = world_box
        self.flange_transforms = {}

    def verify_sdk(self, robot, stages):
        from collision_guard.math3d import matmul
        checked = set()
        position_error, rotation_error = 0., 0.
        parameters = {}
        for arm in (0, 1):
            require_zero(robot.init_single_arm_config(arm), "Initialize arm kinematics")
            values = {}
            for part in ("L0.DH", "FLANGE"):
                for parameter in ("Alpha", "A", "D", "Theta"):
                    name = "R.A%d.%s.%s" % (arm, part, parameter)
                    code, value = robot.param_get_float(name)
                    require_zero(code, "Read "+name)
                    if not math.isfinite(value):
                        raise RuntimeError("Non-finite DH parameter")
                    values[part+"."+parameter] = value
            if any(abs(values["L0.DH."+key]) > 1e-6 for key in ("Alpha", "A", "Theta")):
                raise RuntimeError("Unrecognized SDK arm-base convention")
            parameters[arm] = values
            alpha, theta = (math.radians(values["FLANGE."+key]) for key in ("Alpha", "Theta"))
            ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
            a, d = values["FLANGE.A"], values["FLANGE.D"]
            # Modified DH: Tx(a) Rx(alpha) Rz(theta) Tz(d), units mm.
            self.flange_transforms[arm] = [
                [ct, -st, 0, a], [ca*st, ca*ct, -sa, -d*sa],
                [sa*st, sa*ct, ca, d*ca], [0, 0, 0, 1]]
        for state in [stages[0]["start"]]+[s["target"] for s in stages]:
            js = self.state_type(tuple(state["arm0"]), tuple(state["arm1"]),
                                 tuple(state["body"]), (state["head"][1], state["head"][0]))
            model = self.checker.model
            transforms = model.link_transforms(js.radians_by_name())
            for arm, side in ((0, "L"), (1, "R")):
                q = state["arm%d" % arm]
                key = (arm, tuple(q))
                if key in checked:
                    continue
                checked.add(key)
                joint = model.joints["Arm_%s1_Joint" % side]
                base = matmul(transforms[joint.parent], joint.origin)
                tip = transforms["Arm_%s7_Link" % side]
                sdk = robot.forward_kinematics(arm, q)
                if (not isinstance(sdk, list) or len(sdk) != 4
                        or any(len(row) != 4 or not all(math.isfinite(v) for v in row)
                               for row in sdk)):
                    raise RuntimeError("Vendor FK failed")
                wrist = [[0.]*4 for _ in range(4)]
                wrist[3][3] = 1.
                for i in range(3):
                    wrist[i][3] = sum(base[j][i]*(tip[j][3]-base[j][3]) for j in range(3))*1000
                    for k in range(3):
                        wrist[i][k] = sum(base[j][i]*tip[j][k] for j in range(3))
                flange = [list(row) for row in matmul(wrist, self.flange_transforms[arm])]
                # URDF starts at J1; SDK also includes the fixed proximal DH D.
                flange[2][3] += parameters[arm]["L0.DH.D"]
                for i in range(3):
                    position_error = max(position_error, abs(flange[i][3]-sdk[i][3]))
                    for k in range(3):
                        rotation_error = max(rotation_error, abs(flange[i][k]-sdk[i][k]))
        if position_error > 2 or rotation_error > .005:
            raise RuntimeError("URDF/vendor FK disagree: %.3f mm, %.6f rotation element" %
                               (position_error, rotation_error))
        emit("vendor_fk_verified", configurations=len(checked),
             maximum_position_component_error_mm=position_error,
             maximum_rotation_element_error=rotation_error, frame_parameters=parameters)

    def check(self, state):
        # SDK head slots are yaw,pitch; the web URDF uses pitch,yaw.
        js = self.state_type(tuple(state["arm0"]), tuple(state["arm1"]),
                             tuple(state["body"]), (state["head"][1], state["head"][0]))
        report = self.checker.check_state(js)
        if not report.safe:
            raise RuntimeError("Self-collision/limit rejection: "+json.dumps(report.to_dict()))
        model = self.checker.model
        transforms = model.link_transforms(js.radians_by_name())
        groups = []
        for prefix in ("Arm_L", "Arm_R"):
            groups.append([self.world_box(g, transforms[g.link_name], .03)
                           for g in model.geometries if g.link_name.startswith(prefix)])
        if any(len(group) != 7 for group in groups):
            raise RuntimeError("Expected collision geometry for all seven links of each arm")
        if any(self.boxes_intersect(a, b) for a in groups[0] for b in groups[1]):
            raise RuntimeError("Opposite-arm expanded bounding boxes overlap (30mm per arm)")
        endpoints = []
        for arm, side in ((0, "L"), (1, "R")):
            wrist = transforms["Arm_%s7_Link" % side]
            local = self.flange_transforms[arm]
            endpoints.append([wrist[i][3]*1000+sum(wrist[i][j]*local[j][3] for j in range(3))
                              for i in range(3)])
        separation = math.sqrt(sum((a-b)**2 for a, b in zip(*endpoints)))
        if separation < 250:
            raise RuntimeError("Bare flange spacing below 250mm")
        return separation, transforms["Leg_J6_Link"][2][3]*1000

    def check_plan(self, stages):
        minimum, count = math.inf, 0
        heights = {}
        for stage in stages:
            for sample in collision_samples(stage):
                separation, height = self.check(sample)
                minimum = min(minimum, separation)
                count += 1
            heights[stage["name"]] = height
            emit("geometry_stage_checked", name=stage["name"], checked_samples=count)
        report = {"samples": count, "max_joint_step_deg": .5,
                  "minimum_bare_flange_distance_mm": minimum,
                  "opposite_arm_obb_padding_per_arm_mm": 30,
                  "body_drop_mm": heights["open_before_crouch"]-heights["crouch"],
                  "backend": "exact_mesh_bvh",
                  "continuous_collision_detection": False,
                  "external_obstacles_checked": False}
        if report["body_drop_mm"] <= 1:
            raise RuntimeError("Configured crouch does not lower this starting body pose by >1mm")
        for name in ("stand", "turn_waist", "open_arms_standing", "arms_home", "body_home"):
            if abs(heights[name]-heights["open_before_crouch"]) > .1:
                raise RuntimeError("Standing body height was not preserved")
        emit("geometry_complete", **report)
        return report

    def close(self):
        self.checker.close()


class Runner:
    def __init__(self, robot):
        self.robot = robot
        self.serial = None
        self.fresh_at = time.monotonic()
        self.initial_pose = None
        self.active = False
        self.enabled = False
        self.api = {}
        for index, name in enumerate(NAMES):
            for action, types in (
                ("InitTraj", [ctypes.c_uint32, ctypes.c_int32]),
                ("SetTraj", [ctypes.c_uint32, ctypes.c_int32, ctypes.c_int32,
                             ctypes.POINTER(ctypes.c_double)]),
                ("RunTraj", [ctypes.c_uint32]),
                ("StopTraj", [ctypes.c_uint32]),
            ):
                function = getattr(robot.dll, "FX_L0_%s_Runtime_%s" % (name, action))
                function.argtypes, function.restype = types, ctypes.c_int32
                self.api[index, action] = function

    def call(self, index, action, *args):
        with self.robot._lock:
            return self.api[index, action](0, *args)

    def snapshot(self, ratios=False):
        if self.robot.check_link_state() != 1:
            raise RuntimeError("SDK link lost")
        rt, sg = self.robot.get_rt_dict(), self.robot.get_sg_dict()
        serial = rt["frame_serial"]
        if serial != self.serial:
            self.serial, self.fresh_at = serial, time.monotonic()
        elif time.monotonic()-self.fresh_at > .5:
            raise RuntimeError("Feedback stale for 0.5 seconds")
        state = pose(rt["arms"][0]["fb"]["fb_pos"], rt["arms"][1]["fb"]["fb_pos"],
                     rt["body"]["fb_pos"], rt["head"]["fb_pos"])
        validate_pose(state)
        components = []
        for i, key in enumerate(KEYS):
            raw = rt["arms"][i] if i < 2 else rt["body"]
            config = sg["arms"][i] if i < 2 else sg["body"]
            allowed = (1,) if i < 2 or self.enabled else (0, 1)
            if raw["state"]["err"] != 0 or raw["state"]["cur"] not in allowed:
                raise RuntimeError("%s mode/error invalid: %s" % (key, raw["state"]))
            velocity = (raw["fb"] if i < 2 else raw)["fb_vel"]
            if not all(math.isfinite(v) for v in velocity):
                raise RuntimeError("Non-finite velocity")
            values = [float(config["set"][s]) for s in ("vel_ratio", "acc_ratio")]
            if ratios and any(not math.isfinite(v) or abs(v-RATIO) > .1 for v in values):
                raise RuntimeError("Controller ratio mismatch")
            components.append({"mode": raw["state"]["cur"],
                               "trajectory": config["get"]["traj_state"],
                               "velocity": velocity, "ratios": values})
        if rt["head"]["state"]["err"]:
            raise RuntimeError("HEAD error")
        if self.initial_pose and delta(state, self.initial_pose, ("head",)) > .3:
            raise RuntimeError("Uncommanded HEAD movement")
        return {"pose": state, "components": components, "serial": serial}

    def initial(self):
        first = self.snapshot()
        time.sleep(.15)
        state = self.snapshot()
        if first["serial"] == state["serial"]:
            raise RuntimeError("No fresh feedback")
        if any(c["trajectory"] not in (0, 1, 2) or max(map(abs, c["velocity"])) > .5
               for c in state["components"]):
            raise RuntimeError("Robot is moving or trajectory state is invalid")
        self.initial_pose = state["pose"]
        emit("initial_feedback", **state)
        return state["pose"]

    def stationary(self, expected, ratios=True):
        state = self.snapshot(ratios)
        if delta(state["pose"], expected) > .3:
            raise RuntimeError("Position changed from validated stage start")
        if any(c["trajectory"] not in (0, 1, 2) or max(map(abs, c["velocity"])) > .5
               for c in state["components"]):
            raise RuntimeError("Unexpected movement")
        return state

    def enable(self):
        self.stationary(self.initial_pose, ratios=False)
        # Mark before the first mutation so exceptions trigger a stop.
        self.active = True
        for obj in OBJECTS:
            require_zero(self.robot.runtime_set_speed_ratio(obj, RATIO, RATIO), "Set ratios")
        current = self.snapshot()
        if current["components"][2]["mode"] == 0:
            require_zero(self.robot.switch_to_position_mode(3, 2000, RATIO, RATIO),
                         "BODY Position entry")
        self.enabled = True
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            state = self.stationary(self.initial_pose, ratios=False)
            if all(all(math.isfinite(v) and abs(v-RATIO) <= .1 for v in c["ratios"])
                   for c in state["components"]):
                self.stationary(self.initial_pose)
                return
            time.sleep(.05)
        raise RuntimeError("Ratios not acknowledged")

    def stop(self):
        results = []
        for i in range(3):
            try:
                code = self.call(i, "StopTraj")
                results.append(code)
            except Exception as exc:
                results.append(str(exc))
        emit("stop_requested", arm0_arm1_body_results=results)
        # Enqueue success alone does not prove the hardware has stopped.
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            try:
                state = self.snapshot()
                if all(c["trajectory"] < 3 and max(map(abs, c["velocity"])) <= .5
                       for c in state["components"]):
                    emit("stop_feedback_confirmed", feedback=state)
                    return
            except Exception:
                break
            time.sleep(.05)
        try:
            emit("emergency_stop_requested", returned_mask=self.robot.emergency_stop(11))
        finally:
            emit("stop_unconfirmed", message="Use the physical emergency stop")

    def upload(self, index, rows, expected):
        key, width = KEYS[index], 7 if index < 2 else 6
        self.stationary(expected)
        require_zero(self.call(index, "InitTraj", len(rows)), "InitTraj "+key)
        deadline = time.monotonic()+2
        while True:
            time.sleep(.02)
            status = self.stationary(expected)["components"][index]["trajectory"]
            if status == 1:
                break
            if time.monotonic() > deadline:
                raise RuntimeError("New buffer initialization unconfirmed")
        for serial, begin in enumerate(range(0, len(rows), 50)):
            if self.stationary(expected)["components"][index]["trajectory"] != 1:
                raise RuntimeError("Buffer changed during upload")
            chunk = rows[begin:begin+50]
            flat = [q for row in chunk for q in row[key]]
            raw = (ctypes.c_double*(len(chunk)*width))(*flat)
            require_zero(self.call(index, "SetTraj", serial, len(chunk), raw), "SetTraj "+key)
            time.sleep(.02)
        deadline = time.monotonic()+2
        while True:
            status = self.stationary(expected)["components"][index]["trajectory"]
            if status == 2:
                return
            if status != 1 or time.monotonic() > deadline:
                raise RuntimeError("New complete buffer not acknowledged")
            time.sleep(.02)

    def execute_stage(self, stage):
        indices = (0, 1) if stage["active"] == "arms" else (2,)
        moving_keys = tuple(KEYS[i] for i in indices)
        inactive_keys = tuple(k for k in FIELDS if k not in moving_keys)
        self.stationary(stage["start"])
        for index in indices:
            self.upload(index, stage["rows"], stage["start"])
        ready = self.stationary(stage["start"])
        if any(ready["components"][i]["trajectory"] != 2 for i in indices):
            raise RuntimeError("Not all stage buffers are ready")
        emit("stage_start", name=stage["name"], target=stage["target"])
        if indices == (2,):
            # L1 RunTraj/StopTraj omit BODY in this installed SDK source.
            require_zero(self.call(2, "RunTraj"), "Start BODY")
        elif self.robot.runtime_run_traj(3) != 3:
            raise RuntimeError("Dual-arm start mask mismatch")
        began = time.monotonic()
        deadline = began+max(45, stage["seconds"]*2+15)
        observed, stable, last_log = set(), 0, 0.
        while time.monotonic() < deadline:
            state = self.snapshot(ratios=True)
            actual = state["pose"]
            if delta(actual, stage["start"], inactive_keys) > .3:
                raise RuntimeError("Inactive component moved")
            for key in moving_keys:
                for value, low, high in zip(actual[key], stage["start"][key], stage["target"][key]):
                    if not min(low, high)-1 <= value <= max(low, high)+1:
                        raise RuntimeError("Measured joint left validated stage envelope")
            # All moving joints follow a common blend; bound differential lag.
            changes = [(key, j, b-a) for key in moving_keys
                       for j, (a, b) in enumerate(zip(stage["start"][key], stage["target"][key]))
                       if abs(b-a) > 1]
            if changes:
                key, j, difference = max(changes, key=lambda v: abs(v[2]))
                progress = (actual[key][j]-stage["start"][key][j])/difference
                for key, j, difference in changes:
                    predicted = stage["start"][key][j]+difference*progress
                    if abs(actual[key][j]-predicted) > 1:
                        raise RuntimeError("Joint coordination deviated by more than 1 degree")
            for i in indices:
                if state["components"][i]["trajectory"] >= 3:
                    observed.add(i)
            arrived = (delta(actual, stage["target"]) <= .2
                       and all(state["components"][i]["trajectory"] == 0
                               and max(map(abs, state["components"][i]["velocity"])) <= .5
                               for i in indices))
            stable = stable+1 if arrived else 0
            if stable >= 3 and observed == set(indices):
                emit("stage_complete", name=stage["name"], feedback=state)
                return
            if time.monotonic()-last_log >= 2:
                emit("progress", name=stage["name"], feedback=state)
                last_log = time.monotonic()
            time.sleep(.02)
        raise TimeoutError("Stage did not complete: "+stage["name"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--preflight", action="store_true")
    modes.add_argument("--execute", action="store_true")
    parser.add_argument("--squat-deg", type=float, default=30)
    parser.add_argument("--turn-deg", type=float, default=45)
    args = parser.parse_args()
    if args.execute:
        raise RuntimeError("Motion disabled after BODY buffer incident; manual teaching and interface validation required")
    # Reject configuration before linking to the robot.
    build_plan(pose(*HOME, [0]*6, [0]*3), args.squat_deg, args.turn_deg)
    import fcntl
    sys.path.insert(0, SDK_ROOT+"/PYTHON_SDK")
    from GentoRobot import GentoRobot, FXLogMask
    def interrupted(signum, frame):
        raise KeyboardInterrupt("Signal %s" % signum)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    geometry = None
    runner = None
    with open("/tmp/gento_sdk_6_6_7_190.lock", "a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        robot = GentoRobot()
        try:
            result = robot.link(6, 6, 7, 190, log_level=FXLogMask.FX_LOG_INFO_FLAG)
            if not robot._connected or robot.check_link_state() != 1:
                raise RuntimeError("SDK link failed: %s" % result)
            runner = Runner(robot)
            start = runner.initial()
            stages = build_plan(start, args.squat_deg, args.turn_deg)
            emit("plan", empty_handed=True, squat_deg=args.squat_deg, turn_deg=args.turn_deg,
                 nominal_seconds=sum(s["seconds"] for s in stages),
                 stages=[{k: v for k, v in s.items() if k != "rows"} for s in stages])
            geometry = Geometry()
            geometry.verify_sdk(robot, stages)
            report = geometry.check_plan(stages)
            runner.stationary(start, ratios=False)
            if args.preflight:
                emit("preflight_complete", motion_commands_sent=0, geometry=report)
                return
            runner.enable()
            for stage in stages:
                runner.execute_stage(stage)
            final = runner.stationary(stages[-1]["target"])
            emit("complete", returned_arms_home=True, returned_body_start=True, final=final)
            runner.active = False
        finally:
            try:
                if runner and runner.active:
                    runner.stop()
            finally:
                try:
                    if geometry:
                        geometry.close()
                finally:
                    if getattr(robot, "_connected", False):
                        robot.unlink()
                        emit("sdk_disconnected")


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:
        emit("failed", error=type(exc).__name__+": "+str(exc))
        raise SystemExit(1)
