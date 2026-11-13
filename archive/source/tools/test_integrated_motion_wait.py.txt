"""Exercise the robot runner integration without SDK connections or motion."""

import copy
import math
import unittest
from unittest.mock import patch

from box_motion_plan import delta
from box_mime_taught_plan import STREAM_RATE, continuous_seconds
from box_mime_taught_robot import ECHO_HISTORY_SECONDS
import test_box_mime_continuous as stream_fixtures


class EndObservation(Exception):
    pass


class Tests(unittest.TestCase):
    def setup_runner(self, body_only=False):
        fixture = stream_fixtures.Tests()
        fixture.setUp()
        if body_only:
            stage = next(s for s in fixture.stages if s["name"] == "crouch")
            fixture.fixture.start = copy.deepcopy(stage["start"])
            fixture.stages = [stage]
        fixture.setup_stream()
        return fixture

    def run_stream(self, fixture):
        with patch("box_mime_taught_robot.time.monotonic", fixture.clock.monotonic), \
                patch("box_mime_taught_robot.time.sleep", fixture.clock.sleep), \
                patch("box_mime_taught_robot.emit") as log:
            result = fixture.runner.stream_stages_with_wait(fixture.stages)
        return result, log

    def test_complete_flow_with_two_second_actual_feedback_delay(self):
        fixture = self.setup_runner()
        fixture.delayed_feedback(2.)
        result, log = self.run_stream(fixture)
        self.assertLess(delta(result, fixture.fixture.home), .03)
        self.assertEqual(sum(call.args[0] == "stage_complete" for call in log.call_args_list),
                         len(fixture.stages))
        fixture.robot.emergency_stop.assert_not_called()
        fixture.robot.switch_to_idle.assert_not_called()
        self.assertFalse(fixture.runner.ordinary_wait_active)

    def test_old_point_two_degree_pitch_threshold_no_longer_aborts(self):
        fixture = self.setup_runner(body_only=True)
        original = fixture.robot.get_rt_dict
        seconds = continuous_seconds(fixture.stages[0])

        def rt():
            result = original()
            result["body"]["fb_pos"] = list(result["body"]["fb_pos"])
            result["body"]["fb_pos"][1] += .4*math.sin(math.pi*min(1., fixture.clock.now/seconds))
            return result

        fixture.robot.get_rt_dict = rt
        result, _ = self.run_stream(fixture)
        self.assertLess(delta(result, fixture.stages[0]["target"]), .03)
        fixture.robot.emergency_stop.assert_not_called()

    def test_endpoint_timeout_keeps_streaming_instead_of_exiting(self):
        fixture = self.setup_runner(body_only=True)
        original = fixture.robot.get_rt_dict
        seconds = continuous_seconds(fixture.stages[0])
        actions = []

        def rt():
            if fixture.clock.now > seconds+12:
                self.assertLessEqual(len(fixture.runner.stream_history),
                                     math.ceil(ECHO_HISTORY_SECONDS*STREAM_RATE)+2)
                raise EndObservation()
            result = original()
            result["body"]["fb_pos"] = list(result["body"]["fb_pos"])
            result["body"]["fb_pos"][1] += .1*min(1., fixture.clock.now)
            return result

        def emit(event, **data):
            if event == "motion_wait_status":
                actions.append(data["action"])

        fixture.robot.get_rt_dict = rt
        with patch("box_mime_taught_robot.time.monotonic", fixture.clock.monotonic), \
                patch("box_mime_taught_robot.time.sleep", fixture.clock.sleep), \
                patch("box_mime_taught_robot.emit", emit):
            with self.assertRaises(EndObservation):
                fixture.runner.stream_stages_with_wait(fixture.stages)
        self.assertIn("operator_hold", actions)
        self.assertGreater(fixture.sent[-1][0], seconds+11)
        self.assertLess(delta(fixture.sent[-1][1], fixture.stages[0]["target"]), 1e-9)
        fixture.robot.emergency_stop.assert_not_called()
        fixture.robot.switch_to_idle.assert_not_called()

    def test_pitch_above_two_degrees_waits_then_resumes_without_emergency(self):
        fixture = self.setup_runner(body_only=True)
        original = fixture.robot.get_rt_dict

        def rt():
            result = original()
            t = fixture.clock.now
            fraction = min(1., t/3.) if t <= 5 else max(0., 1-(t-5)/3.)
            result["body"]["fb_pos"] = list(result["body"]["fb_pos"])
            result["body"]["fb_pos"][1] += 1.1*fraction
            result["body"]["fb_pos"][3] += 1.1*fraction
            return result

        fixture.robot.get_rt_dict = rt
        result, log = self.run_stream(fixture)
        statuses = [call.kwargs for call in log.call_args_list
                    if call.args[0] == "motion_wait_status"]
        self.assertTrue(any(s["action"] == "hold" and s["reason"] == "torso_sum_deviation"
                            for s in statuses))
        self.assertEqual(statuses[-1]["action"], "complete")
        self.assertLess(delta(result, fixture.stages[0]["target"]), .03)
        fixture.robot.emergency_stop.assert_not_called()
        fixture.robot.switch_to_idle.assert_not_called()

    def test_controller_fault_still_rejects_before_first_command(self):
        fixture = self.setup_runner()
        fixture.robot.error, fixture.robot.body_mode = 110, 100
        with self.assertRaisesRegex(RuntimeError, "Unexpected state"):
            self.run_stream(fixture)
        fixture.robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_heartbeat_loss_still_rejects_before_first_command(self):
        fixture = self.setup_runner()
        def failed():
            raise RuntimeError("heartbeat lost")
        fixture.runner.heartbeat = failed
        with self.assertRaisesRegex(RuntimeError, "heartbeat"):
            self.run_stream(fixture)
        fixture.robot.runtime_set_joint_pos_cmd.assert_not_called()


if __name__ == "__main__":
    unittest.main()
