"""Six full taught mimes in one SDK session; three turns in each direction."""

import copy
import math
import re
import sys
import time

from box_motion_plan import FIELDS, delta, validate_pose
from box_motion_robot import Geometry, SDK_ROOT, emit
from box_mime_taught_plan import build_stages, controller_settings
from box_mime_taught_robot import MimeRunner, check_geometry
from box_mime_cache import fast_check, model_fingerprint, main_stages
from native_arm_motion import read_native_limits
from position_acceptance import POSITION_TOLERANCE_DEG
from recover_box_start_robot import OBJECTS

TURN_SEQUENCE = (-45, 45, -45, 45, -45, 45)
CACHE_SCHEMA = "luna_six_cycle_geometry_v1"


def build_cycle(start, home, squat, waist, turn):
    if isinstance(turn, bool) or turn not in (-45, 45):
        raise ValueError("Demo yaw must be HOME-relative -45 or +45 degrees")
    stages = build_stages(start, home, squat, waist)
    if turn == 45:
        # Mirror only the standing yaw section. Squat, arms, head and HOME
        # targets remain exactly the values of the proven negative-yaw flow.
        yaw_section = False
        for stage in stages:
            if stage["name"] == "turn_waist_negative_45":
                yaw_section = True
                stage["name"] = "turn_waist_positive_45"
            if yaw_section:
                for endpoint in ("start", "target"):
                    stage[endpoint] = copy.deepcopy(stage[endpoint])
                    stage[endpoint]["body"][5] = (
                        2*home["body"][5]-stage[endpoint]["body"][5])
                    validate_pose(stage[endpoint])
    return stages


def build_cycles(start, home, squat, waist):
    cycles, current = [], copy.deepcopy(start)
    for index, turn in enumerate(TURN_SEQUENCE, 1):
        stages = build_cycle(current, home, squat, waist, turn)
        current = copy.deepcopy(stages[-1]["target"])
        if delta(current, home, ("arm0", "arm1", "body")) > 1e-9:
            raise ValueError("Every demo cycle must return to the same HOME")
        cycles.append({"index": index, "turn_relative_deg": turn, "stages": stages,
                       "target": current})
    return cycles


def validate_bundle(certificate, code_id):
    if (not certificate or certificate.get("schema") != CACHE_SCHEMA
            or certificate.get("code_id") != code_id
            or set(certificate.get("directions", {})) != {"-45", "45"}):
        raise RuntimeError("Both demo direction caches are required; run --prepare-cache")


def prepare(home, squat, waist, frame_parameters, code_id):
    """Compute both nominal model paths without opening a controller connection."""
    geometry = Geometry()
    try:
        before = model_fingerprint(geometry)
        for arm in (0, 1):
            values = frame_parameters[str(arm)]
            alpha, theta = (math.radians(values["FLANGE."+k]) for k in ("Alpha", "Theta"))
            ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
            a, d = values["FLANGE.A"], values["FLANGE.D"]
            geometry.flange_transforms[arm] = [
                [ct, -st, 0, a], [ca*st, ca*ct, -sa, -d*sa],
                [sa*st, sa*ct, ca, d*ca], [0, 0, 0, 1]]
        geometry.verify_sdk = lambda *_: emit(
            "offline_geometry_only", live_vendor_fk_required_on_start=True)
        # Both flows share the same pre-turn poses. Reuse only exactly equal
        # full poses checked against this same, fingerprint-verified model.
        checked_poses, check = {}, geometry.check
        def check_once(state):
            key = tuple(q for group in FIELDS for q in state[group])
            if key not in checked_poses:
                checked_poses[key] = check(state)
            return checked_poses[key]
        geometry.check = check_once
        directions = {}
        for turn in (-45, 45):
            emit("demo_geometry_direction", turn_relative_deg=turn)
            stages = build_cycle(home, home, squat, waist, turn)
            report = check_geometry(geometry, None, stages)
            directions[str(turn)] = {
                "schema": "luna_mime_geometry_cache_v1", "code_id": code_id,
                "model_files": before, "stages": main_stages(stages), "report": report,
                "flange_transforms": {str(k): v for k, v in geometry.flange_transforms.items()},
                "motion_commands_sent": 0}
        if model_fingerprint(geometry) != before:
            raise RuntimeError("Model changed while building demo cache")
        certificate = {
            "schema": CACHE_SCHEMA, "code_id": code_id, "directions": directions,
            "turn_sequence_deg": list(TURN_SEQUENCE), "motion_commands_sent": 0,
            "unique_checked_poses": len(checked_poses)}
        emit("demo_geometry_cache_prepared", certificate=certificate)
        return certificate
    finally:
        geometry.close()


def execute_cycles(runner, geometry, cycles):
    results = []
    for cycle in cycles:
        runner.cycle_index = cycle["index"]
        runner.cycle_turn_deg = cycle["turn_relative_deg"]
        began = time.monotonic()
        emit("demo_cycle_start", cycle=cycle["index"], total_cycles=len(cycles),
             turn_relative_deg=cycle["turn_relative_deg"])
        runner.stream_stages_with_wait(cycle["stages"])
        final = runner.wait_target(cycle["target"])
        geometry.check(final)
        error = delta(final, cycle["target"])
        if error > POSITION_TOLERANCE_DEG+1e-9:
            raise RuntimeError("Demo cycle has not returned HOME")
        result = {"cycle": cycle["index"], "turn_relative_deg": cycle["turn_relative_deg"],
                  "maximum_home_error_deg": error, "pose": final,
                  "elapsed_seconds": time.monotonic()-began}
        results.append(result)
        emit("demo_cycle_complete", **result, returned_home=True)
    return results


def run(home, squat, waist, execute, heartbeat=lambda: None, certificate=None, code_id=None):
    import fcntl
    sys.path.insert(0, SDK_ROOT+"/PYTHON_SDK")
    from GentoRobot import GentoRobot
    runner, geometry, robot = None, None, GentoRobot()
    with open("/tmp/gento_sdk_6_6_7_190.lock", "a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            validate_bundle(certificate, code_id)
            robot.link(*map(int, __import__("os").environ["LUNA_CONTROLLER_IP"].split(".")))
            if not robot._connected:
                raise RuntimeError("SDK unavailable; another client may hold control")
            runner = MimeRunner(robot, heartbeat)
            start = runner.stationary()
            cycles = build_cycles(start, home, squat, waist)
            runner.native_limits = read_native_limits(robot)
            for key, obj in OBJECTS.items():
                code, text = robot.get_servo_error_codes(obj)
                codes = re.findall(r"0x([0-9a-fA-F]{4})", text)
                if code or len(codes) != len(start[key]) or any(int(c, 16) for c in codes):
                    raise RuntimeError("Servo diagnostic rejection: "+key)
            emit("demo_plan", start=start, cycles=cycles, execute=execute,
                 controller_ratios=controller_settings(), cycle_count=len(cycles),
                 turn_sequence_deg=list(TURN_SEQUENCE), arrival_tolerance_deg=POSITION_TOLERANCE_DEG)
            geometry = Geometry()
            runner.geometry = geometry
            # Both directions are validated before any mode change or movement.
            for cycle in cycles[:2]:
                fast_check(geometry, robot, cycle["stages"],
                           certificate["directions"][str(cycle["turn_relative_deg"])], code_id)
            if delta(start, runner.stationary()) > .02:
                raise RuntimeError("Robot moved during demo preflight")
            if not execute:
                emit("preflight_complete", cycles_checked=len(cycles),
                     directions_checked=[-45, 45], motion_commands_sent=0, mode_changes_sent=0)
                return
            runner.enable_all()
            current = runner.stationary()
            runner.segment(current, current, tuple(OBJECTS), .5)
            results = execute_cycles(runner, geometry, cycles)
            emit("demo_complete", completed_cycles=len(results), results=results,
                 returned_home=True, retained_position_mode=True,
                 maximum_home_error_deg=results[-1]["maximum_home_error_deg"])
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
            if runner:
                runner.save_response_trace()
