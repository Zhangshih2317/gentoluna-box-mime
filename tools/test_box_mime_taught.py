"""Regression checks for teaching-based mime planning and control boundaries."""

import copy
from contextlib import ExitStack
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, mock_open, patch

import numpy as np

from box_motion_plan import collision_samples, delta
from box_mime_taught_plan import build_stages, waypoints
import box_mime_taught_robot as control
from stand_up_plan import BodyHeadKinematics, body_transform
from test_stand_up import FakeRobot

ROOT = Path(__file__).resolve().parents[1]


class Tests(unittest.TestCase):
    def setUp(self):
        self.home = json.loads((ROOT/"points/BOX_START_BEFORE_INCIDENT.json").read_text())["pose"]
        self.squat = json.loads((ROOT/"points/taught/SQUAT_TAUGHT-20261111-150201.json").read_text())["pose"]
        self.waist = json.loads((ROOT/"points/taught/WAIST_TAUGHT_01_UI-20261111-153805.json").read_text())["pose"]
        self.start = copy.deepcopy(self.home)
        self.start["body"][5] = -.651
        self.stages = build_stages(self.start, self.home, self.squat, self.waist)

    def fake(self, state=None):
        robot = FakeRobot(state or self.start)
        original_sg = robot.get_sg_dict

        def sg():
            result = original_sg()
            for group in result["arms"]+[result["body"]]:
                group["set"] = {"vel_ratio": 20., "acc_ratio": 5.}
            return result

        robot.get_sg_dict = sg
        robot.switch_to_idle.side_effect = lambda *args: setattr(robot, "body_mode", 0) or 0
        return robot

    def test_order_direction_and_final_home(self):
        self.assertEqual([s["name"] for s in self.stages], [
            "align_standing_home", "open_before_crouch", "crouch",
            "embrace_empty_space", "stand", "turn_waist_negative_45",
            "open_arms_standing", "arms_home", "body_home"])
        turn = self.stages[5]
        self.assertAlmostEqual(turn["target"]["body"][5]-turn["start"]["body"][5], -45)
        for stage in self.stages[6:]:
            self.assertEqual(stage["target"]["body"][1:4], self.home["body"][1:4])
        self.assertLess(delta(self.stages[-1]["target"], self.home), 1e-10)
        for stage in self.stages:
            for key in ("arm0", "arm1", "body", "head"):
                if key not in stage["moving"]:
                    self.assertEqual(stage["start"][key], stage["target"][key])

    def test_shallow_squat_preserves_orientation_and_reverses(self):
        kin = BodyHeadKinematics()
        stage = self.stages[2]
        origin = body_transform(kin, self.home["body"])
        prior_z = origin[2, 3]
        for sample in collision_samples(stage, .1):
            transform = body_transform(kin, sample["body"])
            np.testing.assert_allclose(transform[:3, :3], origin[:3, :3], atol=1e-7)
            self.assertLessEqual(transform[2, 3], prior_z+1e-8)
            self.assertLess(abs(transform[0, 3]-origin[0, 3]), 30)
            self.assertLess(abs(transform[1, 3]-origin[1, 3]), 5)
            self.assertLessEqual(delta(sample, stage["start"], ("body",)), 30.000000001)
            prior_z = transform[2, 3]
        self.assertAlmostEqual(origin[2, 3]-prior_z, 44.0334708, places=5)
        self.assertEqual(self.stages[4]["target"]["body"], self.home["body"])

    def test_small_steps_and_command_speed_bound(self):
        for stage in self.stages:
            prior = stage["start"]
            for i, point in enumerate(waypoints(stage)):
                distance = delta(prior, point["pose"])
                self.assertLessEqual(distance, .200000001 if i == 0 else .490000001)
                self.assertLess(1.875*distance/point["seconds"], 1.)
                prior = point["pose"]
            self.assertLess(delta(prior, stage["target"]), 1e-9)

    def test_only_known_start_region_is_allowed(self):
        build_stages(self.home, self.home, self.squat, self.waist)
        build_stages(self.waist, self.home, self.squat, self.waist)
        for key, index, value in (("body", 5, -20), ("body", 1, 40),
                                  ("arm0", 2, -80), ("head", 0, 5),
                                  ("body", 5, math.nan)):
            state = copy.deepcopy(self.start)
            state[key][index] = value
            with self.assertRaises((RuntimeError, ValueError)):
                build_stages(state, self.home, self.squat, self.waist)

    def test_wrong_teaching_axis_or_direction_is_rejected(self):
        for axis, amount in ((5, 90), (1, 1)):
            waist = copy.deepcopy(self.waist)
            waist["body"][axis] += amount
            with self.assertRaises(ValueError):
                build_stages(self.start, self.home, self.squat, waist)

    def test_enable_sets_all_ratios_without_reset(self):
        robot = self.fake()
        runner = control.MimeRunner(robot, lambda: None)
        with patch("box_mime_taught_robot.time.sleep"), patch("box_mime_taught_robot.emit"):
            runner.enable_all()
        self.assertEqual(runner.enabled, {"arm0", "arm1", "body"})
        self.assertEqual(robot.runtime_set_speed_ratio.call_count, 3)
        for obj in (0, 1, 3):
            robot.runtime_set_speed_ratio.assert_any_call(obj, 20., 5.)
        robot.reset_error.assert_not_called()
        robot.switch_to_position_mode.assert_not_called()
        robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_fault_stale_feedback_and_heartbeat_reject_before_motion(self):
        robot = self.fake()
        robot.error, robot.body_mode = 110, 100
        runner = control.MimeRunner(robot, lambda: None)
        with self.assertRaises(RuntimeError):
            runner.read()
        robot.reset_error.assert_not_called()
        robot = self.fake()
        runner = control.MimeRunner(robot, Mock(side_effect=RuntimeError("heartbeat lost")))
        runner.enabled.add("body")
        with self.assertRaisesRegex(RuntimeError, "heartbeat"):
            runner.send(self.start)
        robot.runtime_set_joint_pos_cmd.assert_not_called()
        runner = control.MimeRunner(robot, lambda: None)
        fixed = robot.get_rt_dict()
        robot.get_rt_dict = lambda: fixed
        runner.read()
        runner.fresh_at -= 1
        with self.assertRaisesRegex(RuntimeError, "Stale"):
            runner.read()

    def test_idle_body_enters_position_at_twenty_speed_five_acceleration(self):
        robot = self.fake()
        robot.body_mode = 0
        robot.switch_to_position_mode.side_effect = lambda *args: setattr(robot, "body_mode", 1) or 0
        runner = control.MimeRunner(robot, lambda: None)
        with patch("box_mime_taught_robot.time.sleep"), patch("box_mime_taught_robot.emit"):
            runner.enable_all()
        robot.switch_to_position_mode.assert_called_once_with(3, 2000, 20., 5.)
        robot.reset_error.assert_not_called()

    def test_wrong_speed_or_acceleration_feedback_is_rejected(self):
        for key, value in (("vel_ratio", 5.), ("acc_ratio", 20.)):
            robot = self.fake()
            sg = robot.get_sg_dict()
            sg["body"]["set"][key] = value
            robot.get_sg_dict = lambda: sg
            runner = control.MimeRunner(robot, lambda: None)
            runner.enabled.add("body")
            with self.assertRaisesRegex(RuntimeError, "ratio"):
                runner.read()

    def test_launcher_watchdog_detects_timeout_and_input_close(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location("mime_entry", ROOT/"run_box_mime_taught.py")
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        source = entry.source_for(False)
        compile(source, "<remote-mime>", "exec")
        prefix = source.split("module=types.ModuleType", 1)[0]
        namespace = {}
        with patch("threading.Thread"), patch("signal.signal"), patch("builtins.print"), \
                patch("signal.SIGHUP", 1, create=True):
            exec(prefix, namespace)
        namespace["heartbeat"]()
        namespace["last_ping"] -= 3
        with self.assertRaisesRegex(RuntimeError, "heartbeat lost"):
            namespace["heartbeat"]()
        namespace["last_ping"] = namespace["time"].monotonic()
        namespace["input_alive"] = False
        with self.assertRaisesRegex(RuntimeError, "heartbeat lost"):
            namespace["heartbeat"]()

    def test_uncommanded_axis_pitch_and_invalid_ratio_rejected(self):
        stage = self.stages[2]
        for axis, offset in ((5, .06), (0, .06), (1, .21)):
            robot = self.fake(stage["start"])
            runner = control.MimeRunner(robot, lambda: None)
            runner.stage, runner.command = stage, copy.deepcopy(stage["start"])
            robot.pose["body"][axis] += offset
            with self.assertRaises(RuntimeError):
                runner.read()
        robot = self.fake()
        sg = robot.get_sg_dict()
        sg["arms"][0]["set"]["vel_ratio"] = math.nan
        robot.get_sg_dict = lambda: sg
        runner = control.MimeRunner(robot, lambda: None)
        runner.enabled.add("arm0")
        with self.assertRaisesRegex(RuntimeError, "ratio"):
            runner.read()

    def invoke(self, robot, execute=False, geometry_error=None, segment_error=None):
        sdk = types.ModuleType("GentoRobot")
        sdk.GentoRobot = lambda: robot
        flock = types.ModuleType("fcntl")
        flock.LOCK_EX, flock.LOCK_NB, flock.flock = 1, 2, Mock()
        targets = []

        def segment(runner, start, target, moving, seconds):
            if segment_error:
                raise segment_error
            robot.pose = copy.deepcopy(target)
            runner.command = copy.deepcopy(target)
            runner.read()
            targets.append(copy.deepcopy(target))
            return copy.deepcopy(target)

        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, {"GentoRobot": sdk, "fcntl": flock}))
            stack.enter_context(patch("builtins.open", mock_open()))
            stack.enter_context(patch.object(control, "Geometry"))
            stack.enter_context(patch.object(control, "check_geometry", side_effect=geometry_error))
            stack.enter_context(patch.object(control.time, "sleep"))
            stack.enter_context(patch.object(control, "emit"))
            stack.enter_context(patch("recover_box_start_robot.emit"))
            if execute:
                stack.enter_context(patch.object(control.MimeRunner, "segment", segment))
                stack.enter_context(patch.object(control.time, "monotonic",
                                                  side_effect=iter(range(100000))))
            control.run(self.home, self.squat, self.waist, execute,
                        continuous=False, full_preflight=True)
        return targets

    def test_preflight_has_no_motion_mode_ratio_or_stop_commands(self):
        robot = self.fake()
        self.invoke(robot)
        self.assertFalse(robot._connected)
        for method in (robot.runtime_set_joint_pos_cmd, robot.runtime_set_speed_ratio,
                       robot.switch_to_position_mode, robot.switch_to_idle,
                       robot.reset_error, robot.emergency_stop):
            method.assert_not_called()

    def test_geometry_failure_does_not_enable_or_move(self):
        robot = self.fake()
        with self.assertRaisesRegex(RuntimeError, "collision"):
            self.invoke(robot, geometry_error=RuntimeError("collision"))
        robot.runtime_set_speed_ratio.assert_not_called()
        robot.runtime_set_joint_pos_cmd.assert_not_called()
        robot.emergency_stop.assert_not_called()
        self.assertFalse(robot._connected)

    def test_mock_complete_flow_returns_home_and_disconnects(self):
        robot = self.fake()
        targets = self.invoke(robot, execute=True)
        self.assertGreater(len(targets), 100)
        self.assertLess(delta(robot.pose, self.home), 1e-9)
        robot.switch_to_idle.assert_called_once_with(3, 2000)
        robot.emergency_stop.assert_not_called()
        self.assertFalse(robot._connected)

    def test_motion_failure_requests_stop_and_does_not_attempt_home(self):
        robot = self.fake()
        with self.assertRaisesRegex(RuntimeError, "tracking failure"):
            self.invoke(robot, execute=True, segment_error=RuntimeError("tracking failure"))
        robot.emergency_stop.assert_called_once_with(11)
        robot.switch_to_idle.assert_not_called()
        self.assertFalse(robot._connected)


if __name__ == "__main__":
    unittest.main()
