"""Offline kinematics and control-boundary tests for the 100 mm rise."""

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

from box_motion_plan import delta, interpolate
from stand_up_plan import BodyHeadKinematics, ROOT, SOURCE, body_transform, build_plan, solve_height
import stand_up_robot
from stand_up_robot import RiseRunner, verify_plan_structure


class FakeRobot:
    def __init__(self, state):
        self.pose = copy.deepcopy(state)
        self._connected = False
        self.serial = 0
        self.error = 0
        self.body_mode = 1
        self.runtime_set_joint_pos_cmd = Mock(return_value=0)
        self.runtime_set_speed_ratio = Mock(return_value=0)
        self.switch_to_position_mode = Mock(return_value=0)
        self.switch_to_idle = Mock(return_value=0)
        self.reset_error = Mock(side_effect=AssertionError("No automatic reset permitted"))
        self.emergency_stop = Mock(return_value=11)

    def link(self, *args):
        self._connected = True

    def unlink(self):
        self._connected = False

    def check_link_state(self):
        return 1

    def get_rt_dict(self):
        self.serial += 1
        return {
            "frame_serial": self.serial,
            "arms": [{"state": {"cur": 1, "cmd": 1, "err": 0},
                      "cmd": {"joint_pos": self.pose[key]},
                      "fb": {"fb_pos": self.pose[key], "fb_vel": [0]*7,
                             "cmd_pos": self.pose[key]}}
                     for key in ("arm0", "arm1")],
            "body": {"state": {"cur": self.body_mode, "cmd": self.body_mode, "err": self.error},
                     "fb_pos": self.pose["body"], "fb_vel": [0]*6,
                     "cmd_pos": self.pose["body"]},
            "head": {"state": {"cur": 0, "cmd": 0, "err": 0}, "fb_pos": self.pose["head"]},
        }

    def get_sg_dict(self):
        group = {"get": {"traj_state": 0}, "set": {"vel_ratio": 3., "acc_ratio": 3.}}
        return {"arms": [copy.deepcopy(group), copy.deepcopy(group)],
                "body": copy.deepcopy(group)}

    def get_servo_error_codes(self, obj):
        return 0, "\n".join("J%d: 0x0000 - No error" % i for i in range(1, 7 if obj == 3 else 8))


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = json.loads((ROOT/SOURCE).read_text("utf-8"))
        cls.plan = build_plan(cls.source)
        cls.kin = BodyHeadKinematics()

    def test_all_waypoints_preserve_orientation_and_rise_100mm(self):
        plan = self.plan
        verify_plan_structure(plan)
        initial = body_transform(self.kin, plan["start"]["body"])
        prior = plan["start"]
        last_height = 0.
        for point in plan["waypoints"]:
            for fraction in (0., .25, .5, .75, 1.):
                sample = interpolate(prior, point["pose"], fraction)
                actual = body_transform(self.kin, sample["body"])
                height = actual[2, 3]-initial[2, 3]
                self.assertGreaterEqual(height+1e-6, last_height)
                self.assertAlmostEqual(actual[0, 3], initial[0, 3], delta=.02)
                np.testing.assert_allclose(actual[:3, :3], initial[:3, :3], atol=1e-7)
                self.assertLessEqual(abs(actual[1, 3]-initial[1, 3]), 2.)
                last_height = height
            self.assertLessEqual(delta(prior, point["pose"], ("body",)), .5)
            self.assertLess(point["pose"]["body"][3], 55.)
            prior = point["pose"]
        self.assertAlmostEqual(last_height, 100., places=5)
        self.assertEqual(prior["arm0"], plan["start"]["arm0"])
        self.assertEqual(prior["arm1"], plan["start"]["arm1"])

    def test_out_of_scope_height_rejected(self):
        for height in (-1, 101, math.nan, math.inf):
            with self.assertRaises(ValueError):
                solve_height(self.kin, self.plan["start"]["body"], height)

    def test_modified_or_invalid_plans_rejected(self):
        cases = []
        for name in ("arm0", "head"):
            modified = copy.deepcopy(self.plan)
            modified["waypoints"][0]["pose"][name][0] += 1
            cases.append(modified)
        for value in (math.nan, 56., 70.456):
            modified = copy.deepcopy(self.plan)
            modified["waypoints"][0]["pose"]["body"][3] = value
            cases.append(modified)
        modified = copy.deepcopy(self.plan)
        modified["waypoints"][0]["seconds"] = .1
        cases.append(modified)
        modified = copy.deepcopy(self.plan)
        modified["rise_mm"] = 200.
        cases.append(modified)
        for modified in cases:
            with self.assertRaises((ValueError, RuntimeError)):
                verify_plan_structure(modified)

    def test_any_system_fault_rejected_without_reset(self):
        robot = FakeRobot(self.plan["start"])
        robot.error, robot.body_mode = 110, 100
        runner = RiseRunner(robot)
        with self.assertRaises(RuntimeError):
            runner.read()
        robot.reset_error.assert_not_called()
        robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_body_preparation_keeps_existing_position_mode(self):
        robot = FakeRobot(self.plan["start"])
        robot._connected = True
        runner = RiseRunner(robot)
        with patch("stand_up_robot.time.sleep"), patch("stand_up_robot.emit"):
            runner.enable_body()
        robot.runtime_set_speed_ratio.assert_called_once_with(3, 3., 3.)
        robot.switch_to_idle.assert_not_called()
        robot.switch_to_position_mode.assert_not_called()
        robot.reset_error.assert_not_called()
        runner.send(self.plan["start"])
        robot.runtime_set_joint_pos_cmd.assert_called_once_with(3, self.plan["start"]["body"])

    def invoke_preflight(self, robot):
        sdk = types.ModuleType("GentoRobot")
        sdk.GentoRobot = lambda: robot
        flock = types.ModuleType("fcntl")
        flock.LOCK_EX, flock.LOCK_NB, flock.flock = 1, 2, Mock()
        geometry = Mock()
        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, {"GentoRobot": sdk, "fcntl": flock}))
            stack.enter_context(patch("builtins.open", mock_open()))
            stack.enter_context(patch("stand_up_robot.Geometry", return_value=geometry))
            check = stack.enter_context(patch("stand_up_robot.validate_geometry"))
            stack.enter_context(patch("stand_up_robot.time.sleep"))
            stack.enter_context(patch("stand_up_robot.emit"))
            stand_up_robot.run(self.plan, False)
        return check

    def test_read_only_preflight_never_sends_motion_or_changes_mode(self):
        robot = FakeRobot(self.plan["start"])
        check = self.invoke_preflight(robot)
        check.assert_called_once()
        self.assertFalse(robot._connected)
        for method in (robot.runtime_set_joint_pos_cmd, robot.runtime_set_speed_ratio,
                       robot.switch_to_position_mode, robot.switch_to_idle,
                       robot.reset_error, robot.emergency_stop):
            method.assert_not_called()

    def test_changed_start_or_completed_run_cannot_execute_again(self):
        for state in (self.plan["target"], self.plan["start"]):
            robot = FakeRobot(state)
            robot.pose["body"][1] += .1
            with self.assertRaisesRegex(RuntimeError, "differs from captured squat"):
                self.invoke_preflight(robot)
            self.assertFalse(robot._connected)
            robot.runtime_set_joint_pos_cmd.assert_not_called()
            robot.runtime_set_speed_ratio.assert_not_called()
            robot.emergency_stop.assert_not_called()

    def test_runtime_torso_and_uncommanded_axis_guards(self):
        for axis, amount in ((1, .21), (0, .06), (4, .06), (5, .06)):
            robot = FakeRobot(self.plan["start"])
            runner = RiseRunner(robot)
            runner.task_start = copy.deepcopy(robot.pose)
            robot.pose["body"][axis] += amount
            with self.assertRaises(RuntimeError):
                runner.read()
        robot = FakeRobot(self.plan["start"])
        runner = RiseRunner(robot)
        runner.task_start = copy.deepcopy(robot.pose)
        robot.pose["arm0"][0] += .06
        with self.assertRaises(RuntimeError):
            runner.read()


if __name__ == "__main__":
    unittest.main()
