"""Ordinary tracking waits; not a safety-stop controller.

This policy never overrides hard limits, controller faults, stale feedback,
collision checks or link loss.
"""

import copy
import math
from dataclasses import dataclass

from box_motion_plan import FIELDS, delta
from recover_box_start_robot import validate_limits
from position_acceptance import PositionAcceptance, POSITION_TOLERANCE_DEG


@dataclass(frozen=True)
class WaitDecision:
    action: str
    reason: str
    maximum_error_deg: float


class MotionWaitPolicy:
    path_tolerance_deg = 2.
    torso_sum_tolerance_deg = 2.
    arrival_tolerance_deg = .2
    home_tolerance_deg = .05
    held_axis_tolerance_deg = .05
    arrival_speed_deg_s = .15
    arrival_frames = 3
    operator_wait_after_seconds = 10.
    resume_tolerance_fraction = 1.
    settle_before_resume = False

    def __init__(self, stage, now, final_home=False):
        self.stage = copy.deepcopy(stage)
        self.final_home = final_home
        self.last_time = now
        self.last_serial = None
        self.stable = 0
        self.wait_since = None
        self.latched = False
        self.reason = ""
        self.deviation_waiting = False
        self.resume_stable = 0
        self.position_acceptance = None

    def use_whole_body_position_acceptance(self):
        self.position_acceptance = PositionAcceptance()
        self.held_axis_tolerance_deg = POSITION_TOLERANCE_DEG
        self.arrival_tolerance_deg = self.home_tolerance_deg = POSITION_TOLERANCE_DEG
        self.path_tolerance_deg = self.torso_sum_tolerance_deg = POSITION_TOLERANCE_DEG
        self.resume_tolerance_fraction = 1.

    def _pose(self, value):
        validate_limits(value)
        if set(value) != set(FIELDS):
            raise ValueError("Invalid feedback groups")

    def update(self, actual, command, velocities, serial, now, endpoint_sent):
        self._pose(actual)
        self._pose(command)
        if not math.isfinite(now) or now < self.last_time:
            raise ValueError("Invalid feedback time")
        self.last_time = now
        if set(velocities) != set(FIELDS):
            raise ValueError("Invalid velocity groups")
        for key in FIELDS:
            if (len(velocities[key]) != len(actual[key])
                    or any(not math.isfinite(v) for v in velocities[key])):
                raise ValueError("Invalid velocity feedback")
            for i, (a, b) in enumerate(zip(self.stage["start"][key], self.stage["target"][key])):
                if abs(a-b) < 1e-9 and abs(actual[key][i]-a) > self.held_axis_tolerance_deg+1e-9:
                    raise RuntimeError("Uncommanded stationary-axis movement: "+key)

        # Endpoint arrival is a whole-pose check. Summing three individual
        # joint errors must not override an otherwise qualifying endpoint.
        if self.position_acceptance is not None and endpoint_sent:
            arrived = self.position_acceptance.update(actual, self.stage["target"], serial, now)
            if delta(actual, self.stage["target"]) <= POSITION_TOLERANCE_DEG+1e-9:
                self.latched, self.reason = False, ""
                self.wait_since = None
                self.last_serial = serial
                return WaitDecision("complete" if arrived else "hold",
                                    "arrived" if arrived else "waiting_for_second_feedback",
                                    delta(actual, self.stage["target"]))

        # One shared progress parameter retains the coordinated joint path.
        start = self.stage["start"]
        key, i = max(((k, j) for k in FIELDS for j in range(len(start[k]))),
                     key=lambda item: abs(command[item[0]][item[1]]-start[item[0]][item[1]]))
        span = command[key][i]-start[key][i]
        fraction = max(0., min(1., (actual[key][i]-start[key][i])/span)) if span else 0.
        reference = {k: [a+(b-a)*fraction for a, b in zip(start[k], command[k])]
                     for k in FIELDS}
        path_error = delta(actual, reference)
        pitch_error = abs(sum(actual["body"][1:4])-sum(command["body"][1:4]))
        reason = ""
        factor = self.resume_tolerance_fraction if self.deviation_waiting else 1.
        if path_error > self.path_tolerance_deg*factor:
            self.stable = 0
            reason = "path_deviation"
        elif pitch_error > self.torso_sum_tolerance_deg*factor:
            self.stable = 0
            reason = "torso_sum_deviation"
        if reason:
            self.deviation_waiting = True
            self.resume_stable = 0
        elif self.deviation_waiting and self.settle_before_resume:
            quiet = max(abs(v) for key in FIELDS for v in velocities[key]) <= self.arrival_speed_deg_s
            if not quiet:
                self.resume_stable = 0
            elif serial != self.last_serial:
                self.resume_stable += 1
            if self.resume_stable < self.arrival_frames:
                reason = "settling_before_resume"
        if not reason:
            self.deviation_waiting = False
        if not reason and endpoint_sent and self.position_acceptance is None:
            tolerance = self.home_tolerance_deg if self.final_home else self.arrival_tolerance_deg
            arrived = (delta(actual, self.stage["target"]) <= tolerance
                       and max(abs(v) for key in FIELDS for v in velocities[key]) <= self.arrival_speed_deg_s)
            if serial != self.last_serial:
                self.stable = self.stable+1 if arrived else 0
            elif not arrived:
                self.stable = 0
            if self.stable >= self.arrival_frames and not self.latched:
                self.last_serial = serial
                self.wait_since = None
                return WaitDecision("complete", "arrived", delta(actual, self.stage["target"]))
            reason = "waiting_for_arrival"
        elif not reason and endpoint_sent:
            reason = "waiting_for_arrival"
        self.last_serial = serial
        error = max(path_error, pitch_error)
        if self.latched:
            return WaitDecision("operator_hold", self.reason, error)
        if reason:
            if self.wait_since is None:
                self.wait_since = now
            if now-self.wait_since >= self.operator_wait_after_seconds:
                self.latched, self.reason = True, reason
                return WaitDecision("operator_hold", reason, error)
            return WaitDecision("hold", reason, error)
        self.stable = 0
        self.wait_since = None
        return WaitDecision("advance", "within_tolerance", error)


class CandidateStageDriver:
    """Drive a checked adapter; no SDK connection or mode APIs.

    adapter.read_checked must independently reject unsafe states. send_checked
    must validate commands and their echoes. Hard failures propagate to the
    surrounding safety supervisor; this class cannot certify powered holding.
    """

    def __init__(self, stage, now, final_home=False, timing_planner=None, clock=None):
        self.policy = MotionWaitPolicy(stage, now, final_home)
        self.command = copy.deepcopy(stage["start"])
        self.last_time = now
        self.elapsed = 0.
        self.endpoint_sent = False
        self.held = False
        self.timing_planner = timing_planner
        self.segment_start = copy.deepcopy(stage["start"])
        self.segment_elapsed = 0.
        self.segment_seconds = None
        self.restart_count = 0
        self.clock = clock
        self.last_tick_interval = 0.
        self.last_read_seconds = 0.
        self.scheduling_waiting = False
        self.scheduling_stable = 0
        self.scheduling_serial = None
        self.scheduling_since = None
        self.scheduling_latched = False
        if timing_planner:
            self.policy.resume_tolerance_fraction = .75
            self.policy.settle_before_resume = True

    def tick(self, adapter, now, seconds):
        from box_motion_plan import blend, interpolate
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError("Invalid stage duration")
        dt = now-self.last_time
        self.last_tick_interval = dt
        self.last_read_seconds = 0.
        # Only the coordinated runner can restart from a held command. Keep
        # the legacy HOME driver's original contract.
        hard_gap = .2 if self.timing_planner else .05
        if not math.isfinite(dt) or not 0 <= dt <= hard_gap:
            raise RuntimeError("Position stream scheduling gap")
        self.last_time = now
        actual, velocities, serial = adapter.read_checked()
        checked_at = self.clock() if self.clock else now
        self.last_read_seconds = checked_at-now
        if not math.isfinite(checked_at) or not 0 <= self.last_read_seconds <= hard_gap:
            raise RuntimeError("Position stream scheduling gap during feedback read")
        late = dt > .05 or self.last_read_seconds > .05
        decision = self.policy.update(actual, self.command, velocities, serial,
                                      checked_at, self.endpoint_sent)
        if late:
            self.scheduling_waiting = True
            self.scheduling_stable = 0
            if self.scheduling_since is None:
                self.scheduling_since = checked_at
        if self.scheduling_waiting or self.scheduling_latched:
            if checked_at-self.scheduling_since >= self.policy.operator_wait_after_seconds:
                self.scheduling_latched = True
            quiet = max(abs(v) for values in velocities.values() for v in values) <= self.policy.arrival_speed_deg_s
            if not late and quiet and decision.action in ("advance", "complete"):
                if serial != self.scheduling_serial:
                    self.scheduling_stable += 1
            else:
                self.scheduling_stable = 0
            self.scheduling_serial = serial
            if self.scheduling_latched or decision.action == "operator_hold":
                decision = WaitDecision("operator_hold", "scheduling_gap", decision.maximum_error_deg)
            elif self.scheduling_stable < self.policy.arrival_frames:
                decision = WaitDecision("hold", "scheduling_gap", decision.maximum_error_deg)
            else:
                self.scheduling_waiting = False
                self.scheduling_since = None
        if decision.action == "complete":
            return decision
        if decision.action == "advance":
            if self.held and self.timing_planner:
                # Restart the remaining straight path from rest. Reusing the
                # old phase here would jump directly back to its old velocity.
                remaining = dict(self.policy.stage, start=copy.deepcopy(self.command))
                self.segment_seconds = self.timing_planner(remaining)
                if not math.isfinite(self.segment_seconds) or self.segment_seconds <= 0:
                    raise ValueError("Invalid remaining stage duration")
                self.segment_start = copy.deepcopy(self.command)
                self.segment_elapsed = 0.
                self.restart_count += 1
            advance = 0. if self.held else dt
            self.elapsed += advance
            self.segment_elapsed += advance
            fraction = min(1., self.segment_elapsed/(self.segment_seconds or seconds))
            self.command = interpolate(self.segment_start,
                                       self.policy.stage["target"], blend(fraction))
            self.endpoint_sent = fraction == 1.
            self.held = False
        else:
            self.held = True
        adapter.send_checked(copy.deepcopy(self.command))
        return decision
