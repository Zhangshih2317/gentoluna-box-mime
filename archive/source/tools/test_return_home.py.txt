"""Offline checks for the separate wrist/arm/standing-BODY recovery."""

import copy
from contextlib import ExitStack
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, mock_open, patch

from box_motion_plan import delta
from box_mime_taught_plan import continuous_seconds
import return_home_robot as control
import test_box_mime_continuous as stream_fixtures
import test_box_mime_taught as fixtures


class Tests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.Tests()
        self.fixture.setUp()
        self.home = self.fixture.home
        self.start = copy.deepcopy(self.home)
        self.start["arm0"] = [90.006, -90., -78.004, -79.999, -5.029, -44.286, -9.316]
        self.start["arm1"] = [-90.003, -90., 78.003, -79.999, -7.548, -48.378, -8.349]
        self.start["body"] = [-.744, 9.281, -17.37, 5.455, -.296, -.173]
        # Live recovery points change after teaching; keep this squat fixture stable.
        recorded = copy.deepcopy(self.start)
        recorded["body"] = [-.744, 46.083, -83.792, 35.015, -.296, -1.653]
        self.reference = {
            "schema": "luna_home_recovery_reference_v1", "scope": "preflight_only",
            "joint_units": "deg", "start": recorded, "home": copy.deepcopy(self.home)}

    def test_current_wrists_recover_one_arm_at_a_time_then_body(self):
        stages, target = control.build_home_stages(self.start, self.home)
        self.assertEqual([s["name"] for s in stages],
                         ["arm0_wrist_home", "arm0_home", "arm1_wrist_home",
                          "arm1_home", "body_home"])
        for stage in stages:
            for group in self.start:
                if group not in stage["moving"]:
                    self.assertEqual(stage["start"][group], stage["target"][group])
        self.assertEqual(stages[0]["target"]["arm0"][:4], self.start["arm0"][:4])
        self.assertEqual(stages[0]["target"]["arm0"][4:], [0., 0., 0.])
        self.assertLess(delta(stages[-1]["target"], self.home), 1e-9)
        self.assertEqual(target, self.home)

    def test_nonstanding_body_and_bad_head_are_rejected(self):
        for key, index, change in (("body", 2, -5), ("body", 5, 50), ("head", 0, 1)):
            start = copy.deepcopy(self.start)
            start[key][index] += change
            with self.assertRaises(ValueError):
                control.build_home_stages(start, self.home)

    def invoke(self, robot, execute=False, collision_error=None, stream_error=None,
               clock=None, recovery_reference=None):
        sdk = types.ModuleType("GentoRobot")
        sdk.GentoRobot = lambda: robot
        flock = types.ModuleType("fcntl")
        flock.LOCK_EX, flock.LOCK_NB, flock.flock = 1, 2, Mock()
        geometry = Mock()
        geometry.check.return_value = (300., 900.)
        if collision_error:
            geometry.check.side_effect = collision_error
        events = []

        def record(event, **data):
            events.append((clock.now if clock else None, event, data))

        def stream(runner, stages):
            if stream_error:
                raise stream_error
            robot.pose = copy.deepcopy(stages[-1]["target"])
            runner.command = copy.deepcopy(robot.pose)
            runner.feedback_observation.reset()
            return robot.pose

        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, {"GentoRobot": sdk, "fcntl": flock}))
            stack.enter_context(patch("builtins.open", mock_open()))
            stack.enter_context(patch.object(control, "Geometry", return_value=geometry))
            if clock:
                stack.enter_context(patch("time.monotonic", clock.monotonic))
                stack.enter_context(patch("time.sleep", clock.sleep))
            else:
                stack.enter_context(patch("time.sleep"))
                stack.enter_context(patch.object(control.HomeRunner,
                                                 "stream_stages_with_wait", stream))
            for module in ("return_home_robot", "box_mime_taught_robot", "recover_box_start_robot"):
                stack.enter_context(patch(module+".emit", record))
            stack.enter_context(patch.object(
                control.HomeRunner, "stream_stages",
                side_effect=AssertionError("HOME must use arrival-wait streaming")))
            control.run(self.home, execute, recovery_reference=recovery_reference)
        self.assertFalse(robot._connected)
        return events

    def test_recorded_nonstanding_pose_plans_body_before_wrists(self):
        start = copy.deepcopy(self.reference["start"])
        before = copy.deepcopy(self.reference)
        stages, target = control.build_home_stages(start, self.home, self.reference)
        self.assertEqual([stage["name"] for stage in stages],
                         ["body_to_home", "arm0_wrist_home", "arm0_home",
                          "arm1_wrist_home", "arm1_home"])
        self.assertEqual(stages[0]["target"]["body"], self.home["body"])
        for stage in stages:
            for group in start:
                if group not in stage["moving"]:
                    self.assertEqual(stage["start"][group], stage["target"][group])
        self.assertEqual(target["head"], start["head"])
        self.assertLess(delta(target, self.home, ("arm0", "arm1", "body")), 1e-9)
        self.assertEqual(self.reference, before)
        self.assertEqual(start, before["start"])

    def test_reference_matching_accepts_rounding_but_rejects_changed_pose(self):
        start = copy.deepcopy(self.reference["start"])
        start["body"][2] += .01
        control.build_home_stages(start, self.home, self.reference)
        for group in self.reference["start"]:
            start = copy.deepcopy(self.reference["start"])
            start[group][0] += .06
            with self.subTest(group=group), self.assertRaises(ValueError):
                control.build_home_stages(start, self.home, self.reference)
        home = copy.deepcopy(self.home)
        home["arm0"][4] += .01
        with self.assertRaisesRegex(ValueError, "HOME differs"):
            control.build_home_stages(self.reference["start"], home, self.reference)

    def test_reference_cannot_claim_execution_scope_or_invalid_units(self):
        for key, value in (("scope", "execute"), ("schema", "other"), ("joint_units", "rad")):
            reference = copy.deepcopy(self.reference)
            reference[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "Invalid"):
                control.build_home_stages(reference["start"], self.home, reference)

    def test_body_first_preflight_checks_path_without_motion(self):
        robot = self.fixture.fake(self.reference["start"])
        events = self.invoke(robot, recovery_reference=self.reference)
        plan = next(data for _, name, data in events if name == "home_plan")
        self.assertTrue(plan["body_first"])
        self.assertEqual(plan["stages"][0]["name"], "body_to_home")
        self.assertTrue(any(name == "home_geometry_verified" for _, name, _ in events))
        for method in (robot.runtime_set_joint_pos_cmd, robot.runtime_set_speed_ratio,
                       robot.switch_to_position_mode, robot.switch_to_idle,
                       robot.reset_error, robot.emergency_stop):
            method.assert_not_called()

    def test_body_first_execution_reaches_home_without_reset(self):
        robot = self.fixture.fake(self.reference["start"])
        events = self.invoke(robot, True, recovery_reference=self.reference)
        self.assertLess(delta(robot.pose, self.home, ("arm0", "arm1", "body")), 1e-9)
        self.assertEqual(robot.pose["head"], self.reference["start"]["head"])
        robot.switch_to_idle.assert_called_once_with(3, 2000)
        robot.reset_error.assert_not_called()
        robot.emergency_stop.assert_not_called()
        self.assertTrue(any(name == "home_complete" for _, name, _ in events))

    def test_preflight_never_changes_modes_or_sends_motion(self):
        robot = self.fixture.fake(self.start)
        self.invoke(robot)
        for call in (robot.runtime_set_joint_pos_cmd, robot.runtime_set_speed_ratio,
                     robot.switch_to_position_mode, robot.switch_to_idle,
                     robot.reset_error, robot.emergency_stop):
            call.assert_not_called()

    def test_collision_failure_prevents_enable_even_in_execution_mode(self):
        robot = self.fixture.fake(self.start)
        with self.assertRaisesRegex(RuntimeError, "collision"):
            self.invoke(robot, True, collision_error=RuntimeError("collision"))
        robot.runtime_set_joint_pos_cmd.assert_not_called()
        robot.runtime_set_speed_ratio.assert_not_called()
        robot.emergency_stop.assert_not_called()
        self.assertFalse(robot._connected)

    def test_fault_is_not_reset(self):
        robot = self.fixture.fake(self.start)
        robot.error, robot.body_mode = 110, 100
        with self.assertRaisesRegex(RuntimeError, "Unexpected state"):
            self.invoke(robot, True)
        robot.reset_error.assert_not_called()
        robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_execution_ends_at_home_and_only_body_enters_idle(self):
        robot = self.fixture.fake(self.start)
        self.invoke(robot, True)
        self.assertLess(delta(robot.pose, self.home), 1e-9)
        robot.switch_to_idle.assert_called_once_with(3, 2000)
        robot.reset_error.assert_not_called()

    def test_execution_failure_stops_without_auto_return(self):
        robot = self.fixture.fake(self.start)
        with self.assertRaisesRegex(RuntimeError, "stream failure"):
            self.invoke(robot, True, stream_error=RuntimeError("stream failure"))
        robot.emergency_stop.assert_called_once_with(11)
        self.assertEqual(robot.pose, self.start)
        robot.switch_to_idle.assert_not_called()

    def test_home_launcher_embeds_recovery_and_heartbeat_not_mime_execution(self):
        import importlib.util
        root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location("home_entry", root/"run_box_mime_taught.py")
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        source = entry.source_for(False, home_only=True)
        compile(source, "<return-home>", "exec")
        self.assertIn("receive_pings", source)
        self.assertIn("module.run(", source)
        import ast
        call = ast.parse(source).body[-1].value
        reference = next(kw.value for kw in call.keywords if kw.arg == "recovery_reference")
        self.assertEqual(ast.literal_eval(reference), json.loads(
            (root/"points/HOME_RECOVERY_PREFLIGHT_REFERENCE.json").read_text("utf-8")))
        self.assertNotIn("sys.modules['box_mime_taught_robot'].run(", source)

    def test_idle_slots_seed_measured_pose_before_enabling(self):
        robot = self.fixture.fake(self.start)
        robot.body_mode = 0
        calls = []
        robot.runtime_set_joint_pos_cmd.side_effect = lambda obj, q: calls.append(("seed", obj, q)) or 0

        def position(obj, *args):
            calls.append(("enable", obj))
            robot.body_mode = 1
            return 0

        robot.switch_to_position_mode.side_effect = position
        with patch("time.sleep"), patch("box_mime_taught_robot.emit"):
            control.HomeRunner(robot, lambda: None).enable_all()
        self.assertEqual(calls[0], ("seed", 3, self.start["body"]))
        self.assertEqual(calls[1], ("enable", 3))

    def simulation(self):
        simulation = stream_fixtures.Tests()
        simulation.setUp()
        simulation.fixture.start = copy.deepcopy(self.start)
        simulation.setup_stream()
        return simulation

    def test_recovery_path_runs_through_actual_stream_loop_with_delay(self):
        simulation = self.simulation()
        simulation.delayed_feedback(2.)
        stages, target = control.build_home_stages(self.start, self.home)
        events = self.invoke(simulation.robot, True, clock=simulation.clock)
        self.assertLess(delta(simulation.robot.pose, target), .03)
        completed = [data["name"] for _, event, data in events if event == "stage_complete"]
        self.assertEqual(completed, [stage["name"] for stage in stages])
        simulation.robot.emergency_stop.assert_not_called()

    def test_moving_wrist_deviation_below_two_degrees_does_not_abort_home(self):
        simulation = self.simulation()
        original = simulation.robot.get_rt_dict
        stages, _ = control.build_home_stages(self.start, self.home)
        seconds = continuous_seconds(stages[0])

        def rt():
            result = original()
            result["arms"][0]["fb"]["fb_pos"] = list(result["arms"][0]["fb"]["fb_pos"])
            if simulation.sent:
                elapsed = simulation.clock.now-simulation.sent[0][0]
                offset = .4*math.sin(math.pi*min(1., elapsed/seconds))
                result["arms"][0]["fb"]["fb_pos"][4] += offset
            return result

        simulation.robot.get_rt_dict = rt
        self.invoke(simulation.robot, True, clock=simulation.clock)
        self.assertLess(delta(simulation.robot.pose, self.home), .03)
        simulation.robot.emergency_stop.assert_not_called()

    def test_home_waits_past_old_five_second_timeout_then_finishes(self):
        simulation = self.simulation()
        original = simulation.robot.get_rt_dict
        stages, _ = control.build_home_stages(self.start, self.home)
        endpoint_at = [None]
        observed_wait = []

        def rt():
            result = original()
            result["arms"][0]["fb"]["fb_pos"] = list(result["arms"][0]["fb"]["fb_pos"])
            if simulation.sent:
                now = simulation.clock.now
                if endpoint_at[0] is None and delta(simulation.robot.pose, stages[0]["target"]) < 1e-9:
                    endpoint_at[0] = now
                if endpoint_at[0] is None:
                    offset = .1*min(1., now-simulation.sent[0][0])
                else:
                    elapsed = now-endpoint_at[0]
                    offset = .1*max(0., min(1., (6.5-elapsed)/.5))
                    if 5.5 < elapsed < 6.:
                        simulation.robot.emergency_stop.assert_not_called()
                        simulation.robot.switch_to_idle.assert_not_called()
                        self.assertTrue(simulation.robot._connected)
                        self.assertLess(delta(simulation.robot.pose, stages[0]["target"]), 1e-9)
                        observed_wait.append(elapsed)
                result["arms"][0]["fb"]["fb_pos"][4] += offset
            return result

        simulation.robot.get_rt_dict = rt
        events = self.invoke(simulation.robot, True, clock=simulation.clock)
        next_stage_at = next(t for t, event, data in events
                             if event == "stage_start" and data["name"] == stages[1]["name"])
        self.assertTrue(observed_wait)
        self.assertGreater(next_stage_at-endpoint_at[0], 6.)
        self.assertLess(delta(simulation.robot.pose, self.home), .03)
        simulation.robot.emergency_stop.assert_not_called()

    def test_long_home_wait_does_not_enter_cleanup_or_start_next_stage(self):
        simulation = self.simulation()
        original = simulation.robot.get_rt_dict
        stages, _ = control.build_home_stages(self.start, self.home)
        endpoint_at = [None]
        observed = []
        interrupted = [False]

        class EndObservation(Exception):
            pass

        def rt():
            result = original()
            result["arms"][0]["fb"]["fb_pos"] = list(result["arms"][0]["fb"]["fb_pos"])
            if simulation.sent and not interrupted[0]:
                now = simulation.clock.now
                if endpoint_at[0] is None and delta(simulation.robot.pose, stages[0]["target"]) < 1e-9:
                    endpoint_at[0] = now
                result["arms"][0]["fb"]["fb_pos"][4] += .1*min(1., now-simulation.sent[0][0])
                if endpoint_at[0] is not None and now-endpoint_at[0] > 12.:
                    simulation.robot.emergency_stop.assert_not_called()
                    simulation.robot.switch_to_idle.assert_not_called()
                    self.assertTrue(simulation.robot._connected)
                    self.assertLess(delta(simulation.sent[-1][1], stages[0]["target"]), 1e-9)
                    self.assertLess(now-simulation.sent[-1][0], .041)
                    observed.append(now)
                    interrupted[0] = True
                    raise EndObservation()
            return result

        simulation.robot.get_rt_dict = rt
        with self.assertRaises(EndObservation):
            self.invoke(simulation.robot, True, clock=simulation.clock)
        self.assertTrue(observed)
        # Only the explicit test interruption reaches the existing fault handler.
        simulation.robot.emergency_stop.assert_called_once_with(11)
        simulation.robot.switch_to_idle.assert_not_called()
        self.assertFalse(simulation.robot._connected)

    def test_already_home_does_not_enable_stream_or_stop(self):
        robot = self.fixture.fake(self.home)
        events = self.invoke(robot, True)
        self.assertTrue(any(event == "home_complete" for _, event, _ in events))
        for call in (robot.runtime_set_joint_pos_cmd, robot.runtime_set_speed_ratio,
                     robot.switch_to_position_mode, robot.switch_to_idle,
                     robot.emergency_stop):
            call.assert_not_called()


if __name__ == "__main__":
    unittest.main()
