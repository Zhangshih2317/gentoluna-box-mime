"""Native arm targets and coordinated BODY streaming with arrival monitoring."""

import copy
import json
import math
import re
import sys
import tempfile
import time
from collections import deque

from box_motion_plan import FIELDS, blend, collision_samples, delta, interpolate
from box_motion_robot import Geometry, SDK_ROOT, emit, require_zero
from box_mime_taught_plan import (SPEED_PERCENT, ACCELERATION_PERCENT,
                                  STREAM_RATE, STREAM_SPEED, STREAM_ACCELERATION,
                                  BODY_STREAM_SPEED, BODY_STREAM_ACCELERATION,
                                  SQUAT_DROP_MM, COORDINATED_TIMING, TIMING_PROFILE, NATIVE_ARM_POSITION,
                                  coordinated_timing, controller_ratios, controller_settings,
                                  build_stages, continuous_seconds, waypoints)
from recover_box_start_robot import OBJECTS, Recovery, guard_pose
from stand_up_robot import displacement_mm, model_transform
from feedback_observation import FeedbackObservation
from position_acceptance import PositionAcceptance, POSITION_TOLERANCE_DEG
from native_arm_motion import (is_native_arm_stage, execute_native_stage, read_native_limits,
                               envelope_samples, envelope_sample_count, ENVELOPE_GRID_DEG,
                               ENVELOPE_MODEL)

TRACKING_HISTORY_SECONDS = 2.5
ECHO_HISTORY_SECONDS = .25
ARRIVAL_TIMEOUT_SECONDS = 5.
# Bound each submitted increment by the configured velocity over the longest
# permitted tick (50 ms), retaining the original 20% rounding margins.
ARM_COMMAND_STEP_DEG = STREAM_SPEED*.05+.02
BODY_COMMAND_STEP_DEG = BODY_STREAM_SPEED*.05+.04


class MimeRunner(Recovery):
    max_feedback_speed = max(STREAM_SPEED, BODY_STREAM_SPEED)+.6

    def __init__(self, robot, heartbeat):
        super().__init__(robot)
        self.heartbeat = heartbeat
        self.command = None
        self.stage = None
        self.feedback_observation = FeedbackObservation()
        self.last_read_started = self.last_read_finished = None
        self.native_limits = None
        self.native_stage = None
        self.geometry = None
        self.cycle_index = 0
        self.cycle_turn_deg = 0
        self.stream_history = None
        self.ordinary_wait_active = False
        # Memory only in the control loop; write after SDK cleanup so logging
        # disk/SSH latency cannot stall a position-command tick.
        self.response_samples = deque(maxlen=30000) if COORDINATED_TIMING else None
        self.response_sample_count = 0

    def save_response_trace(self):
        if not self.response_samples:
            return
        try:
            with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8", prefix="luna-mime-response-",
                    suffix=".jsonl", delete=False) as trace:
                trace.write(json.dumps({
                    "event": "response_trace_metadata", "schema": "luna_mime_response_v1",
                    "timing_profile": TIMING_PROFILE, "sample_rate_requested_hz": STREAM_RATE,
                    "controller_ratios": controller_settings(),
                    "native_arm_position": NATIVE_ARM_POSITION,
                    "native_arm_limits": self.native_limits,
                    "cycle_count": 6,
                    "samples_recorded": len(self.response_samples),
                    "samples_dropped": self.response_sample_count-len(self.response_samples),
                    "feedback_precedes_command_after_tick": True,
                    "dynamic_delay_compensation": False})+"\n")
                for sample in self.response_samples:
                    trace.write(json.dumps(sample)+"\n")
            emit("response_trace_saved", path=trace.name, samples=len(self.response_samples),
                 samples_dropped=self.response_sample_count-len(self.response_samples))
        except Exception as error:
            emit("response_trace_save_failed", error=str(error))

    def tracking_reference(self, actual):
        if self.stream_history is None:
            return self.command
        now = time.monotonic()
        while (len(self.stream_history) > 1
               and self.stream_history[0][0] < now-TRACKING_HISTORY_SECONDS):
            self.stream_history.popleft()
        # Each stage follows a straight joint-space line. Project onto the
        # recent sent interval, using one fraction for all joints. This avoids
        # scanning 125 full poses twice per tick on the robot controller.
        first, last = self.stream_history[0][1], self.stream_history[-1][1]
        key, joint = max(((k, i) for k in FIELDS for i in range(len(first[k]))),
                         key=lambda item: abs(last[item[0]][item[1]]-first[item[0]][item[1]]))
        span = last[key][joint]-first[key][joint]
        fraction = max(0., min(1., (actual[key][joint]-first[key][joint])/span)) if span else 0.
        reference = interpolate(first, last, fraction)
        error = delta(actual, reference)
        if error > .12:
            emit("path_tracking_rejected", stage=self.stage["name"],
                 actual=actual, last_command=self.command, reference=reference,
                 error_deg=error, history_seconds=TRACKING_HISTORY_SECONDS)
            raise RuntimeError("Path tracking deviation or stalled motion")
        return reference

    def read(self, allow_disabled=False):
        self.heartbeat()
        if NATIVE_ARM_POSITION and self.native_limits is not None:
            self.max_feedback_speed = max(
                BODY_STREAM_SPEED+.6,
                *(row["VelMax"]*controller_ratios(key)[0]/100.+.6
                  for key in ("arm0", "arm1") for row in self.native_limits[key]))
        started = time.monotonic()
        state, rt, sg = super().read(allow_disabled=False)
        finished = time.monotonic()
        self.last_read_started, self.last_read_finished = started, finished
        limits = {}
        for key in FIELDS:
            speed = BODY_STREAM_SPEED if key == "body" else STREAM_SPEED
            limits[key] = [speed+.6]*len(state[key])
            if NATIVE_ARM_POSITION and self.native_limits is not None and key in ("arm0", "arm1"):
                limits[key] = [row["VelMax"]*controller_ratios(key)[0]/100.+.6
                               for row in self.native_limits[key]]
        for key in ("arm0", "arm1", "body"):
            fb = rt["body"] if key == "body" else rt["arms"][OBJECTS[key]]["fb"]
            if any(abs(v) > limit for v, limit in zip(fb["fb_vel"], limits[key])):
                raise RuntimeError("Unexpected joint velocity: "+key)
        if self.command and not self.ordinary_wait_active:
            reference = self.tracking_reference(state)
            for key in FIELDS:
                allowed = (POSITION_TOLERANCE_DEG if NATIVE_ARM_POSITION else
                           (.25 if self.stage and key in self.stage["moving"] else .05))
                if delta(state, reference, (key,)) > allowed:
                    raise RuntimeError("Command tracking/uncommanded movement: "+key)
            pitch_limit = POSITION_TOLERANCE_DEG if NATIVE_ARM_POSITION else .2
            if abs(sum(state["body"][1:4])-sum(self.command["body"][1:4])) > pitch_limit+1e-9:
                raise RuntimeError("Torso pitch compensation mismatch")
        if self.stage:
            for i in range(6):
                if abs(self.stage["start"]["body"][i]-self.stage["target"]["body"][i]) < 1e-9:
                    held_limit = POSITION_TOLERANCE_DEG if NATIVE_ARM_POSITION else .05
                    if abs(state["body"][i]-self.stage["start"]["body"][i]) > held_limit+1e-9:
                        raise RuntimeError("Uncommanded BODY axis movement")
        for key in self.enabled:
            config = sg["body"] if key == "body" else sg["arms"][OBJECTS[key]]
            values = [config["set"][k] for k in ("vel_ratio", "acc_ratio")]
            if any(not math.isfinite(v) or abs(v-expected) > .1
                   for v, expected in zip(values, controller_ratios(key))):
                raise RuntimeError("Speed/acceleration ratio mismatch: "+key)
        violation = self.feedback_observation.check(
            state, rt["frame_serial"], started, finished, limits)
        if violation:
            emit("joint_jump_rejected", stage=self.stage["name"] if self.stage else None,
                 **violation)
            raise RuntimeError("Measured joint jump")
        return state, rt, sg

    def wait_target(self, target):
        """Confirm final whole-body arrival without switching out of Position."""
        acceptance = PositionAcceptance()
        prior_wait = self.ordinary_wait_active
        self.ordinary_wait_active = True
        try:
            while True:
                actual, rt, _ = self.read()
                self.check_echo(rt)
                if acceptance.update(actual, target, rt["frame_serial"], time.monotonic()):
                    return actual
                time.sleep(.1)
        finally:
            self.ordinary_wait_active = prior_wait

    def send(self, command):
        self.heartbeat()
        super().send(command)
        self.command = copy.deepcopy(command)
        if self.stream_history is not None:
            now = time.monotonic()
            self.stream_history.append((now, self.command))
            if self.ordinary_wait_active:
                # Waiting can last indefinitely; only recent input echoes need history.
                while (len(self.stream_history) > 1
                       and self.stream_history[0][0] < now-ECHO_HISTORY_SECONDS):
                    self.stream_history.popleft()

    def enable_all(self):
        self.command = self.stationary()
        observed_ratios = {}
        for key, obj in OBJECTS.items():
            speed_percent, acceleration_percent = controller_ratios(key)
            before, rt, _ = self.read()
            raw = rt["body"] if key == "body" else rt["arms"][obj]
            self.changed = True
            if raw["state"]["cur"] == 0:
                require_zero(self.robot.switch_to_position_mode(obj, 2000, speed_percent, acceleration_percent),
                             "Position "+key)
            elif raw["state"]["cur"] == 1:
                require_zero(self.robot.runtime_set_speed_ratio(obj, speed_percent, acceleration_percent),
                             "Ratios "+key)
            else:
                raise RuntimeError("Automatic error reset is prohibited")
            deadline = time.monotonic()+1.
            while True:
                current, rt, sg = self.read()
                if delta(current, before) > .02:
                    raise RuntimeError("Pose drift during mode/ratio setup")
                config = sg["body"] if key == "body" else sg["arms"][obj]
                raw = rt["body"] if key == "body" else rt["arms"][obj]
                values = [config["set"][k] for k in ("vel_ratio", "acc_ratio")]
                if (raw["state"]["cur"] == 1
                        and all(math.isfinite(v) and abs(v-expected) <= .1
                                for v, expected in zip(values, (speed_percent, acceleration_percent)))):
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("Mode/ratio update not confirmed: "+key)
                time.sleep(.02)
            self.enabled.add(key)
            observed_ratios[key] = dict(zip(("vel_ratio", "acc_ratio"), values))
        emit("position_ready", controller_ratios=controller_settings(),
             controller_ratios_observed=observed_ratios)

    def check_echo(self, rt):
        now = time.monotonic()
        for key in self.enabled:
            if key == "body":
                raw = rt["body"]
                values, fb = raw.get("cmd_pos", []), raw
                output_command = None
            else:
                raw = rt["arms"][OBJECTS[key]]
                # ARM_IN is the submitted target; ARM_OUT's Joint_Cmd is a
                # controller feedback value, not the input command echo.
                values = raw.get("cmd", {}).get("joint_pos", [])
                fb = raw["fb"]
                output_command = fb.get("cmd_pos")
            candidates = [self.command[key]]
            if self.stream_history is not None:
                candidates.extend(pose[key] for sent_at, pose in self.stream_history
                                  if now-sent_at <= ECHO_HISTORY_SECONDS)
            valid = (len(values) == len(self.command[key])
                     and all(math.isfinite(v) for v in values))
            # Input slots can arrive a frame after the command was submitted.
            # Accept only a complete recent vector actually sent to this group.
            matched = valid and any(
                max(abs(a-b) for a, b in zip(values, candidate)) <= .05
                for candidate in candidates)
            if not matched:
                emit("command_echo_rejected", group=key, frame_serial=rt["frame_serial"],
                     stage=self.stage["name"] if self.stage else None,
                     expected_input=self.command[key], observed_input=values,
                     accepted_history_seconds=ECHO_HISTORY_SECONDS,
                     controller_output_command=output_command,
                     measured_position=fb["fb_pos"])
                raise RuntimeError("Position command echo mismatch: "+key)

    def stream_stages_with_wait(self, stages):
        """Run native arm targets or coordinated streams, waiting at each stage."""
        from motion_wait_policy import CandidateStageDriver
        if not stages or delta(self.command, stages[0]["start"]) > .03:
            raise RuntimeError("Continuous stream start mismatch")
        runner = self

        class Adapter:
            actual = None
            observation = None

            def read_checked(self):
                actual, rt, _ = runner.read()
                runner.check_echo(rt)
                self.actual = actual
                velocities = {
                    "arm0": rt["arms"][0]["fb"]["fb_vel"],
                    "arm1": rt["arms"][1]["fb"]["fb_vel"],
                    "body": rt["body"]["fb_vel"],
                    # HEAD is not commanded; read() and the wait policy still
                    # check its position, finite values and stationary axes.
                    "head": [0.]*len(actual["head"]),
                }
                if runner.response_samples is not None:
                    self.observation = {
                        "cycle_index": runner.cycle_index,
                        "turn_relative_deg": runner.cycle_turn_deg,
                        "observed_monotonic": time.monotonic(),
                        "read_started": runner.last_read_started,
                        "read_finished": runner.last_read_finished,
                        "frame_serial": rt["frame_serial"],
                        "command_before_read": copy.deepcopy(runner.command),
                        "actual": copy.deepcopy(actual),
                        "velocity_deg_s": copy.deepcopy(velocities),
                        "input_echo": {
                            "arm0": list(rt["arms"][0]["cmd"]["joint_pos"]),
                            "arm1": list(rt["arms"][1]["cmd"]["joint_pos"]),
                            "body": list(rt["body"]["cmd_pos"])},
                        "arm_controller_output": [
                            list(arm["fb"].get("cmd_pos", [])) for arm in rt["arms"]],
                        "body_sensor_raw": list(rt["body"].get("fb_sensor", [])),
                        "body_gyro_raw": list(rt["body"].get("gyro", []))}
                return actual, velocities, rt["frame_serial"]

            def send_checked(self, command):
                if (delta(command, runner.command, ("body",)) > BODY_COMMAND_STEP_DEG
                        or delta(command, runner.command, ("arm0", "arm1", "head")) > ARM_COMMAND_STEP_DEG):
                    raise RuntimeError("Excessive continuous command increment")
                runner.send(command)

        adapter = Adapter()
        period = 1./STREAM_RATE
        self.ordinary_wait_active = True
        try:
            for index, stage in enumerate(stages):
                if NATIVE_ARM_POSITION and is_native_arm_stage(stage):
                    adapter.actual = execute_native_stage(self, stage)
                    continue
                self.stage = stage
                now = time.monotonic()
                self.stream_history = deque([(now, copy.deepcopy(self.command))])
                driver = CandidateStageDriver(
                    stage, now, final_home=index == len(stages)-1,
                    timing_planner=continuous_seconds if COORDINATED_TIMING else None,
                    clock=time.monotonic if COORDINATED_TIMING else None)
                # Legacy recovery keeps its tighter arrival contract. The
                # native mime uses the same whole-body tolerance across stages.
                driver.policy.arrival_tolerance_deg = .03
                driver.policy.home_tolerance_deg = .03
                if NATIVE_ARM_POSITION:
                    driver.policy.use_whole_body_position_acceptance()
                seconds = continuous_seconds(stage)
                next_tick = now
                last_log, previous_action = now, None
                emit("stage_start", name=stage["name"], target=stage["target"],
                     motion_style="arrival_wait",
                     timing_profile=TIMING_PROFILE,
                     timing=coordinated_timing(stage) if COORDINATED_TIMING else None,
                     path_tolerance_deg=driver.policy.path_tolerance_deg,
                     torso_sum_tolerance_deg=driver.policy.torso_sum_tolerance_deg)
                while True:
                    time.sleep(max(0., next_tick-time.monotonic()))
                    now = time.monotonic()
                    try:
                        decision = driver.tick(adapter, now, seconds)
                    except RuntimeError as error:
                        if str(error).startswith("Position stream scheduling gap"):
                            emit("stream_timing_fault", name=stage["name"],
                                 tick_interval_seconds=driver.last_tick_interval,
                                 feedback_read_seconds=driver.last_read_seconds)
                        raise
                    if self.response_samples is not None:
                        self.response_sample_count += 1
                        self.response_samples.append(dict(
                            adapter.observation, stage=stage["name"],
                            tick_finished_monotonic=time.monotonic(),
                            command_after_tick=copy.deepcopy(self.command),
                            action=decision.action, reason=decision.reason,
                            error_deg=decision.maximum_error_deg,
                            elapsed_motion_seconds=driver.elapsed,
                            smooth_restart_count=driver.restart_count,
                            segment_seconds=driver.segment_seconds or seconds,
                            segment_elapsed_seconds=driver.segment_elapsed,
                            tick_interval_seconds=driver.last_tick_interval,
                            feedback_read_seconds=driver.last_read_seconds))
                    if decision.action != previous_action or now-last_log >= 5.:
                        emit("motion_wait_status", name=stage["name"],
                             action=decision.action, reason=decision.reason,
                             maximum_error_deg=decision.maximum_error_deg,
                             elapsed_motion_seconds=driver.elapsed,
                             smooth_restart_count=driver.restart_count,
                             pose=adapter.actual)
                        previous_action, last_log = decision.action, now
                    if decision.action == "complete":
                        if self.geometry is not None:
                            self.geometry.check(adapter.actual)
                        emit("stage_complete", name=stage["name"], pose=adapter.actual)
                        break
                    # A normal hold stays in this loop: no raise, IDLE,
                    # disconnect or emergency command. Hard faults still raise.
                    next_tick += period
                    after_send = time.monotonic()
                    if next_tick <= after_send:
                        next_tick += (math.floor((after_send-next_tick)/period)+1)*period
            return adapter.actual
        finally:
            self.ordinary_wait_active = False
            self.stream_history = None

    def stream_stages(self, stages):
        """Keep one 50 Hz stream; stop-at-waypoint waits are absent inside the flow."""
        if not stages or delta(self.command, stages[0]["start"]) > .03:
            raise RuntimeError("Continuous stream start mismatch")
        period = 1./STREAM_RATE
        index, endpoint_sent = 0, False
        self.stage = stages[0]
        self.stream_history = deque([(time.monotonic(), copy.deepcopy(self.command))])
        began = last_tick = next_tick = time.monotonic()
        last_log = began
        endpoint_at, stable, stable_serial = None, 0, None
        emit("stage_start", name=self.stage["name"], target=self.stage["target"],
             motion_style="continuous")
        while True:
            time.sleep(max(0., next_tick-time.monotonic()))
            now = time.monotonic()
            if now-last_tick > .05:
                raise RuntimeError("Continuous position stream scheduling gap")
            last_tick = now
            actual, rt, _ = self.read()
            self.check_echo(rt)
            guard_pose(actual, self.tracking_reference(actual), self.stage["start"],
                       self.stage["target"], self.stage["moving"])
            if endpoint_sent:
                velocities = rt["body"]["fb_vel"] + [
                    v for arm in rt["arms"] for v in arm["fb"]["fb_vel"]]
                arrived = (delta(actual, self.stage["target"]) <= .03
                           and max(abs(v) for v in velocities) <= .15)
                stable = stable+1 if arrived and rt["frame_serial"] != stable_serial else 0
                stable_serial = rt["frame_serial"]
                if stable < 3:
                    if now-endpoint_at > ARRIVAL_TIMEOUT_SECONDS:
                        raise RuntimeError("Continuous stage endpoint not reached before timeout")
                    self.send(self.stage["target"])
                    next_tick = time.monotonic()+period
                    continue
                emit("stage_complete", name=self.stage["name"], pose=actual)
                index += 1
                if index == len(stages):
                    self.stream_history = None
                    return actual
                self.stage = stages[index]
                began, endpoint_sent = now, False
                endpoint_at, stable, stable_serial = None, 0, None
                self.stream_history.clear()
                self.stream_history.append((now, copy.deepcopy(self.command)))
                emit("stage_start", name=self.stage["name"], target=self.stage["target"],
                     motion_style="continuous")
            fraction = min(1., (now-began)/continuous_seconds(self.stage))
            command = interpolate(self.stage["start"], self.stage["target"], blend(fraction))
            if (delta(command, self.command, ("body",)) > BODY_COMMAND_STEP_DEG
                    or delta(command, self.command, ("arm0", "arm1", "head")) > ARM_COMMAND_STEP_DEG):
                raise RuntimeError("Excessive continuous command increment")
            self.send(command)
            endpoint_sent = fraction == 1.
            if endpoint_sent:
                endpoint_at = now
                emit("waiting_for_arrival", name=self.stage["name"],
                     timeout_seconds=ARRIVAL_TIMEOUT_SECONDS)
            if now-last_log >= 5.:
                emit("progress", name=self.stage["name"], fraction=fraction, pose=actual,
                     motion_style="continuous")
                last_log = now
            # Late ticks advance to the next future slot, never issue a burst.
            next_tick += period
            after_send = time.monotonic()
            if next_tick <= after_send:
                next_tick += (math.floor((after_send-next_tick)/period)+1)*period


def check_geometry(geometry, robot, stages):
    def samples(stage):
        return (envelope_samples(stage) if NATIVE_ARM_POSITION and is_native_arm_stage(stage)
                else collision_samples(stage, .1))
    total = sum(envelope_sample_count(stage) if NATIVE_ARM_POSITION and is_native_arm_stage(stage)
                else len(collision_samples(stage, .1)) for stage in stages)
    emit("preflight_progress", phase="vendor_fk", samples_checked=0, samples_total=total)
    geometry.verify_sdk(robot, stages)
    # HOME is reached before the arms open. The head is held at its measured angle.
    home = next(s["start"] for s in stages if s["name"] == "open_before_crouch")
    origin = model_transform(geometry, home)
    count, minimum = 0, math.inf
    heights = {}
    last_progress = time.monotonic()
    for stage in stages:
        previous_z = model_transform(geometry, stage["start"])[2][3]
        stage_min = math.inf
        for sample in samples(stage):
            spacing, _ = geometry.check(sample)
            transform = model_transform(geometry, sample)
            change = displacement_mm(origin, transform)
            if stage["name"] in ("crouch", "stand"):
                direction = -1 if stage["name"] == "crouch" else 1
                if direction*(transform[2][3]-previous_z) < -1e-8:
                    raise RuntimeError("Non-monotonic crouch/rise")
                if max(abs(transform[i][j]-origin[i][j]) for i in range(3) for j in range(3)) > 1e-6:
                    raise RuntimeError("Squat would tilt the torso")
            if abs(change[0]) > 30 or abs(change[1]) > 5:
                raise RuntimeError("BODY leaves validated horizontal corridor")
            if stage["name"] in ("turn_waist_negative_45", "turn_waist_positive_45",
                                 "open_arms_standing", "arms_home", "body_home"):
                if abs(change[2]) > .1:
                    raise RuntimeError("Standing release/return lowers the body")
            previous_z = transform[2][3]
            stage_min, minimum, count = min(stage_min, spacing), min(minimum, spacing), count+1
            if time.monotonic()-last_progress >= 5. or count == total:
                emit("preflight_progress", phase="collision", stage=stage["name"],
                     samples_checked=count, samples_total=total)
                last_progress = time.monotonic()
        heights[stage["name"]] = transform[2][3]*1000
        emit("geometry_stage_checked", name=stage["name"], minimum_flange_spacing_mm=stage_min)
    drop = heights["open_before_crouch"]-heights["crouch"]
    if SQUAT_DROP_MM is None:
        if not 1 < drop < 100:
            raise RuntimeError("Unexpected shallow squat height")
    elif abs(drop-SQUAT_DROP_MM) > .1:
        raise RuntimeError("Model squat depth differs from the requested height")
    report = {
        "samples": count, "maximum_sample_step_deg": ENVELOPE_GRID_DEG if NATIVE_ARM_POSITION else .1,
        "streamed_path_step_deg": .1,
        "native_arm_envelope_model": ENVELOPE_MODEL if NATIVE_ARM_POSITION else None,
        "native_arm_envelope_step_deg": ENVELOPE_GRID_DEG if NATIVE_ARM_POSITION else None,
        "native_arm_envelopes_checked": [s["name"] for s in stages
                                       if NATIVE_ARM_POSITION and is_native_arm_stage(s)],
        "minimum_flange_spacing_mm": minimum, "predicted_body_drop_mm": drop,
        "requested_body_drop_mm": SQUAT_DROP_MM,
        "opposite_arm_padding_per_arm_mm": 30,
        "continuous_collision_detection": False, "external_obstacles_checked": False,
        "hardware_execution_verified": False,
        "continuous_streaming_seconds": (None if NATIVE_ARM_POSITION else
                                         sum(continuous_seconds(s) for s in stages)),
        "stepped_minimum_streaming_seconds": sum(p["seconds"] for s in stages for p in waypoints(s)),
        "continuous_peak_joint_speed_deg_s": None if NATIVE_ARM_POSITION else STREAM_SPEED,
        "continuous_peak_joint_acceleration_deg_s2": None if NATIVE_ARM_POSITION else STREAM_ACCELERATION,
        "alignment_stream_speed_limit_deg_s": STREAM_SPEED,
        "alignment_stream_acceleration_limit_deg_s2": STREAM_ACCELERATION,
        "body_peak_joint_speed_deg_s": BODY_STREAM_SPEED,
        "body_peak_joint_acceleration_deg_s2": BODY_STREAM_ACCELERATION,
    }
    emit("geometry_verified", **report)
    return report


def run(home, squat, waist, execute, heartbeat=lambda: None, continuous=True,
        certificate=None, code_id=None, full_preflight=False):
    import fcntl
    sys.path.insert(0, SDK_ROOT+"/PYTHON_SDK")
    from GentoRobot import GentoRobot
    runner, geometry, robot = None, None, GentoRobot()
    with open("/tmp/gento_sdk_6_6_7_190.lock", "a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            robot.link(*map(int, __import__("os").environ["LUNA_CONTROLLER_IP"].split(".")))
            if not robot._connected:
                raise RuntimeError("SDK unavailable; another client may hold control")
            runner = MimeRunner(robot, heartbeat)
            start = runner.stationary()
            stages = build_stages(start, home, squat, waist)
            if NATIVE_ARM_POSITION:
                runner.native_limits = read_native_limits(robot)
            for key, obj in OBJECTS.items():
                code, text = robot.get_servo_error_codes(obj)
                codes = re.findall(r"0x([0-9a-fA-F]{4})", text)
                if code or len(codes) != len(start[key]) or any(int(c, 16) for c in codes):
                    raise RuntimeError("Servo diagnostic rejection: "+key)
            emit("plan", start=start, stages=stages, execute=execute, empty_handed=True,
                 controller_ratios=controller_settings(), turn_relative_deg=-45,
                 requested_body_drop_mm=SQUAT_DROP_MM,
                 timing_profile=TIMING_PROFILE,
                 dynamic_delay_compensation=False,
                 native_arm_position=NATIVE_ARM_POSITION,
                 native_arm_limits=runner.native_limits,
                 planned_streaming_seconds=(None if NATIVE_ARM_POSITION else
                                            sum(continuous_seconds(s) for s in stages)),
                 planned_body_and_recovery_streaming_seconds=sum(
                     continuous_seconds(s) for s in stages
                     if not (NATIVE_ARM_POSITION and is_native_arm_stage(s))),
                 arm_peak_joint_speed_deg_s=None if NATIVE_ARM_POSITION else STREAM_SPEED,
                 body_peak_joint_speed_deg_s=BODY_STREAM_SPEED,
                 motion_style="continuous" if continuous else "stepped")
            emit("preflight_progress", phase="load_model", samples_checked=0, samples_total=0)
            geometry = Geometry()
            runner.geometry = geometry
            if full_preflight:
                check_geometry(geometry, robot, stages)
            else:
                from box_mime_cache import fast_check
                fast_check(geometry, robot, stages, certificate, code_id)
            if delta(start, runner.stationary()) > .02:
                raise RuntimeError("Robot moved during preflight")
            if not execute:
                emit("preflight_complete", motion_commands_sent=0, mode_changes_sent=0)
                return
            runner.enable_all()
            current = runner.stationary()
            runner.segment(current, current, tuple(OBJECTS), .5)
            if continuous:
                current = runner.stream_stages_with_wait(stages)
                if NATIVE_ARM_POSITION:
                    current = runner.wait_target(stages[-1]["target"])
                else:
                    current = runner.segment(current, stages[-1]["target"], tuple(OBJECTS), .5)
                geometry.check(current)
                if not NATIVE_ARM_POSITION:
                    current = runner.stationary()
            else:
                for stage in stages:
                    runner.stage = stage
                    if delta(current, stage["start"]) > .03:
                        raise RuntimeError("Stage start mismatch")
                    emit("stage_start", name=stage["name"], target=stage["target"])
                    points = waypoints(stage)
                    for i, point in enumerate(points, 1):
                        current = runner.segment(current, point["pose"], stage["moving"], point["seconds"])
                        geometry.check(current)
                        if i == 1 or i % 10 == 0 or i == len(points):
                            emit("progress", name=stage["name"], segment=i, total=len(points), pose=current)
                    current = runner.stationary()
                    emit("stage_complete", name=stage["name"], pose=current)
            arrival_limit = POSITION_TOLERANCE_DEG if NATIVE_ARM_POSITION else .05
            completion_target = stages[-1]["target"] if NATIVE_ARM_POSITION else home
            if delta(current, completion_target) > arrival_limit+1e-9:
                raise RuntimeError("Final HOME arrival mismatch")
            if NATIVE_ARM_POSITION:
                final = current
            else:
                require_zero(robot.switch_to_idle(3, 2000), "BODY IDLE after HOME")
                runner.enabled.remove("body")
                final = runner.stationary()
            if delta(final, completion_target) > arrival_limit+1e-9:
                raise RuntimeError("HOME drift after BODY IDLE")
            emit("mime_complete", pose=final, returned_home=True,
                 retained_position_mode=NATIVE_ARM_POSITION,
                 arrival_tolerance_deg=arrival_limit,
                 maximum_home_error_deg=delta(final, completion_target))
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
