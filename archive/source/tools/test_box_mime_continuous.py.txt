"""Exercise the actual streaming loop with deterministic time and joint feedback."""

import copy
import math
import unittest
from unittest.mock import Mock, patch

import numpy as np

from box_motion_plan import blend, delta, interpolate
from box_mime_taught_plan import (continuous_seconds, continuous_limits,
                                 STREAM_SPEED, STREAM_ACCELERATION,
                                 BODY_STREAM_SPEED, BODY_STREAM_ACCELERATION)
from box_mime_taught_robot import MimeRunner
from recover_box_start_robot import Recovery
import test_box_mime_taught as fixtures


class Clock:
    def __init__(self):
        self.now = 0.
        self.stall = False

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0., seconds)
        if self.stall:
            self.now += .11
            self.stall = False


class Tests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.Tests()
        self.fixture.setUp()
        self.stages = self.fixture.stages

    def setup_stream(self, heartbeat=lambda: None):
        self.robot = self.fixture.fake()
        self.runner = MimeRunner(self.robot, heartbeat)
        self.runner.enabled = {"arm0", "arm1", "body"}
        self.runner.command = copy.deepcopy(self.fixture.start)
        self.clock = Clock()
        self.sent = []

        def send(obj, values):
            key = {0: "arm0", 1: "arm1", 3: "body"}[obj]
            self.robot.pose[key] = list(values)
            if obj == 3:
                self.sent.append((self.clock.now, copy.deepcopy(self.robot.pose)))
            return 0

        self.robot.runtime_set_joint_pos_cmd.side_effect = send

    def stream(self, stages=None):
        with patch("box_mime_taught_robot.time.monotonic", self.clock.monotonic), \
                patch("box_mime_taught_robot.time.sleep", self.clock.sleep), \
                patch("box_mime_taught_robot.emit"), \
                patch.object(self.runner, "segment", side_effect=AssertionError("No segment dwell")), \
                patch.object(self.runner, "stationary", side_effect=AssertionError("No stationary dwell")):
            return self.runner.stream_stages(stages or self.stages)

    def test_whole_flow_has_regular_commands_and_returns_home(self):
        self.setup_stream()
        final = self.stream()
        self.assertLess(delta(final, self.fixture.home), 1e-8)
        gaps = np.diff([t for t, _ in self.sent])
        self.assertLessEqual(max(gaps), .0400001)
        self.assertGreater(min(gaps), .019999)
        expected = sum(continuous_seconds(s) for s in self.stages)
        self.assertLess(self.clock.now-expected, 1.)
        self.assertGreaterEqual(self.clock.now, expected)
        # Finite differences across every stage boundary bound the actual
        # emitted velocity and acceleration, including direction changes.
        values = np.asarray([p["arm0"]+p["arm1"]+p["body"] for _, p in self.sent])
        velocities = np.diff(values, axis=0)/gaps[:, None]
        accelerations = np.diff(velocities, axis=0)/((gaps[1:]+gaps[:-1])/2)[:, None]
        self.assertLessEqual(np.max(abs(velocities[:, :14])), STREAM_SPEED+1e-5)
        self.assertLessEqual(np.max(abs(accelerations[:, :14])), STREAM_ACCELERATION+1e-4)
        self.assertLessEqual(np.max(abs(velocities[:, 14:])), BODY_STREAM_SPEED+1e-5)
        self.assertLessEqual(np.max(abs(accelerations[:, 14:])), BODY_STREAM_ACCELERATION+1e-4)
        self.robot.switch_to_idle.assert_not_called()

    def test_timing_keeps_same_geometric_path_and_zero_endpoint_velocity(self):
        for stage in self.stages:
            seconds = continuous_seconds(stage)
            speed, acceleration = continuous_limits(stage)
            distance = delta(stage["start"], stage["target"])
            self.assertLessEqual(1.875*distance/seconds, speed+1e-9)
            self.assertLessEqual(10/math.sqrt(3)*distance/seconds**2, acceleration+1e-9)
            previous = stage["start"]
            for index in range(1, 101):
                sample = interpolate(stage["start"], stage["target"], blend(index/100))
                for key in sample:
                    for q, a, b in zip(sample[key], stage["start"][key], stage["target"][key]):
                        self.assertTrue(min(a, b)-1e-9 <= q <= max(a, b)+1e-9)
                previous = sample
            self.assertLess(delta(previous, stage["target"]), 1e-9)
            self.assertLess(blend(1e-5)/1e-5, 1e-8)
            self.assertLess((1-blend(1-1e-5))/1e-5, 1e-8)

    def test_scheduling_gap_stops_before_any_command(self):
        self.setup_stream()
        self.clock.stall = True
        with self.assertRaisesRegex(RuntimeError, "scheduling gap"):
            self.stream()
        self.robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_frozen_feedback_stops_instead_of_advancing(self):
        self.setup_stream()
        self.robot.runtime_set_joint_pos_cmd.side_effect = None
        with self.assertRaisesRegex(RuntimeError, "echo mismatch"):
            self.stream()
        self.assertLess(self.clock.now, continuous_seconds(self.stages[0]))

    def test_delayed_arm_output_is_not_mistaken_for_input_echo(self):
        self.setup_stream()
        original_rt = self.robot.get_rt_dict
        largest_output_lag = [0.]

        def rt():
            result = original_rt()
            past = next((p for t, p in reversed(self.sent)
                         if t <= self.clock.now-.08), self.fixture.start)
            for index, key in enumerate(("arm0", "arm1")):
                result["arms"][index]["fb"]["cmd_pos"] = list(past[key])
                largest_output_lag[0] = max(
                    largest_output_lag[0],
                    max(abs(a-b) for a, b in zip(past[key], self.runner.command[key])))
            return result

        self.robot.get_rt_dict = rt
        final = self.stream()
        self.assertLess(delta(final, self.fixture.home), 1e-8)
        self.assertGreater(largest_output_lag[0], .05)

    def test_wrong_input_echo_stops_even_with_matching_arm_output(self):
        self.setup_stream()
        original_rt = self.robot.get_rt_dict

        def rt():
            result = original_rt()
            result["arms"][1]["cmd"]["joint_pos"] = list(self.runner.command["arm1"])
            result["arms"][1]["cmd"]["joint_pos"][2] += .06
            return result

        self.robot.get_rt_dict = rt
        with self.assertRaisesRegex(RuntimeError, "echo mismatch: arm1"):
            self.stream()
        self.robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_missing_input_field_is_not_replaced_by_feedback_command(self):
        self.setup_stream()
        original_rt = self.robot.get_rt_dict

        def rt():
            result = original_rt()
            del result["arms"][0]["cmd"]
            return result

        self.robot.get_rt_dict = rt
        with self.assertRaisesRegex(RuntimeError, "echo mismatch: arm0"):
            self.stream()
        self.robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_tracking_error_stops_even_with_matching_command_echo(self):
        self.setup_stream()
        original_rt = self.robot.get_rt_dict

        def rt():
            result = original_rt()
            if self.clock.now > .1:
                result["body"] = copy.deepcopy(result["body"])
                result["body"]["fb_pos"][5] += .3
            return result

        self.robot.get_rt_dict = rt
        with self.assertRaisesRegex(RuntimeError, "tracking"):
            self.stream()
        self.assertLess(self.clock.now, .2)

    def test_heartbeat_failure_interrupts_stream(self):
        self.setup_stream()
        self.runner.heartbeat = Mock(side_effect=RuntimeError("heartbeat lost"))
        with self.assertRaisesRegex(RuntimeError, "heartbeat lost"):
            self.stream()
        self.robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_higher_speed_feedback_is_scoped_to_mime_runner(self):
        self.setup_stream()
        original_rt = self.robot.get_rt_dict
        velocity = [3.6]

        def rt():
            result = original_rt()
            result["body"]["fb_vel"][5] = velocity[0]
            return result

        self.robot.get_rt_dict = rt
        self.runner.read()
        with self.assertRaisesRegex(RuntimeError, "velocity"):
            Recovery(self.robot).read(allow_disabled=False)
        velocity[0] = self.runner.max_feedback_speed+.01
        with self.assertRaisesRegex(RuntimeError, "velocity"):
            self.runner.read()

    def test_lagging_endpoint_is_not_allowed_to_start_next_stage(self):
        self.setup_stream()
        stage = self.stages[0]
        original_rt = self.robot.get_rt_dict

        def rt():
            result = original_rt()
            if abs(self.runner.command["body"][5]-stage["target"]["body"][5]) < 1e-10:
                result["body"] = copy.deepcopy(result["body"])
                result["body"]["fb_pos"][5] += .04
            return result

        self.robot.get_rt_dict = rt
        with self.assertRaisesRegex(RuntimeError, "endpoint not reached"):
            self.stream()
        self.assertEqual(self.runner.stage["name"], stage["name"])

    def delayed_feedback(self, delay, frozen=False):
        original_rt = self.robot.get_rt_dict

        def rt():
            result = original_rt()
            past = self.fixture.start if frozen else next(
                (p for t, p in reversed(self.sent) if t <= self.clock.now-delay),
                self.fixture.start)
            for i, key in enumerate(("arm0", "arm1")):
                result["arms"][i]["fb"]["fb_pos"] = list(past[key])
            result["body"]["fb_pos"] = list(past["body"])
            return result

        self.robot.get_rt_dict = rt

    def test_coordinated_delayed_actual_feedback_completes_after_arrival(self):
        self.setup_stream()
        self.delayed_feedback(.2)
        final = self.stream()
        self.assertLess(delta(final, self.fixture.home), .03)
        self.assertGreater(self.clock.now, sum(continuous_seconds(s) for s in self.stages))

    def test_long_feedback_lag_is_rejected(self):
        self.setup_stream()
        self.delayed_feedback(3.)
        with self.assertRaisesRegex(RuntimeError, "Path tracking"):
            self.stream()

    def test_two_second_coordinated_delay_reaches_every_endpoint(self):
        self.setup_stream()
        self.delayed_feedback(2.)
        final = self.stream()
        self.assertLess(delta(final, self.fixture.home), .03)

    def test_delayed_input_echo_matches_recent_sent_commands(self):
        self.setup_stream()
        original_rt = self.robot.get_rt_dict

        def rt():
            result = original_rt()
            past = next((p for t, p in reversed(self.sent)
                         if t <= self.clock.now-.08), self.fixture.start)
            result["body"]["cmd_pos"] = list(past["body"])
            for i, key in enumerate(("arm0", "arm1")):
                result["arms"][i]["cmd"]["joint_pos"] = list(past[key])
            return result

        self.robot.get_rt_dict = rt
        self.assertLess(delta(self.stream(), self.fixture.home), .03)

    def test_stale_input_echo_is_rejected_despite_actual_arrival(self):
        self.setup_stream()
        original_rt = self.robot.get_rt_dict

        def rt():
            result = original_rt()
            result["body"]["cmd_pos"] = list(self.fixture.start["body"])
            return result

        self.robot.get_rt_dict = rt
        with self.assertRaisesRegex(RuntimeError, "echo mismatch"):
            self.stream()

    def test_body_duration_is_halved_without_speeding_up_arms(self):
        for stage in self.stages:
            if stage["name"] in ("crouch", "stand", "turn_waist_negative_45", "body_home"):
                distance = delta(stage["start"], stage["target"])
                old = max(2., 1.875*distance/STREAM_SPEED,
                          math.sqrt((10/math.sqrt(3))*distance/STREAM_ACCELERATION))
                self.assertAlmostEqual(continuous_seconds(stage), old/2)

    def test_frozen_actual_position_with_valid_echo_is_rejected(self):
        self.setup_stream()
        self.delayed_feedback(0, frozen=True)
        with self.assertRaisesRegex(RuntimeError, "Path tracking"):
            self.stream()

    def test_endpoint_wait_keeps_sending_until_fresh_arrival_frames(self):
        self.setup_stream()
        stage = self.stages[0]
        original_rt = self.robot.get_rt_dict
        endpoint_time = [None]

        def rt():
            result = original_rt()
            if delta(self.runner.command, stage["target"]) < 1e-9:
                if endpoint_time[0] is None:
                    endpoint_time[0] = self.clock.now
                if self.clock.now-endpoint_time[0] < .2:
                    result["body"]["fb_pos"][5] += .04
            return result

        self.robot.get_rt_dict = rt
        self.stream([stage])
        self.assertGreaterEqual(self.clock.now-endpoint_time[0], .24-1e-8)
        self.assertGreater(len([t for t, _ in self.sent if t >= endpoint_time[0]]), 8)


if __name__ == "__main__":
    unittest.main()
