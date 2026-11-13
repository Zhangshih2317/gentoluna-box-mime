"""Offline regressions for the no-load gesture; never import the SDK."""

from contextlib import nullcontext
import math
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import box_motion_plan as p
import box_motion_robot as r


def initial():
    return p.pose(*p.HOME, [-.744, 9.281, -17.364, 5.456, -.296, -1.653], [2.107, -15.998, 0])


class PlanTests(unittest.TestCase):
    def test_order_squat_bound_standing_release_and_return(self):
        start = initial()
        stages = p.build_plan(start)
        self.assertEqual([s["name"] for s in stages],
                         ["open_before_crouch", "crouch", "embrace_empty_space", "stand",
                          "turn_waist", "open_arms_standing", "arms_home", "body_home"])
        for a, b, expected in zip(start["body"], stages[1]["target"]["body"], [0, 15, -30, 15, 0, 0]):
            self.assertAlmostEqual(b-a, expected)
        for stage in stages[3:]:
            self.assertEqual(stage["target"]["body"][:5], start["body"][:5])
        self.assertEqual(stages[4]["target"]["body"][5], start["body"][5]+45)
        self.assertEqual(stages[5]["target"]["body"], stages[4]["target"]["body"])
        self.assertEqual(stages[-1]["target"], start)

    def test_reverse_turn_and_smaller_squat(self):
        stages = p.build_plan(initial(), 20, -45)
        self.assertAlmostEqual(stages[1]["target"]["body"][2]-initial()["body"][2], -20)
        self.assertAlmostEqual(stages[4]["target"]["body"][5]-initial()["body"][5], -45)

    def test_invalid_amplitude_or_start_rejected(self):
        for value in (0, -1, 30.01, math.nan, math.inf):
            with self.assertRaises(ValueError):
                p.build_plan(initial(), value)
        for value in (0, 46, math.nan, math.inf):
            with self.assertRaises(ValueError):
                p.build_plan(initial(), turn_deg=value)
        bad = initial()
        bad["arm0"][0] += 2
        with self.assertRaises(ValueError):
            p.build_plan(bad)
        bad = initial()
        bad["body"][5] = 150
        with self.assertRaises(ValueError):
            p.build_plan(bad)

    def test_samples_continuity_speed_acceleration_and_inactive_parts(self):
        stages = p.build_plan(initial())
        previous = initial()
        for stage in stages:
            self.assertEqual(stage["rows"][0], previous)
            self.assertLess(p.delta(stage["rows"][-1], stage["target"]), 1e-10)
            self.assertLessEqual(len(stage["rows"]), 5000)
            inactive = ("body", "head") if stage["active"] == "arms" else ("arm0", "arm1", "head")
            for key in p.FIELDS:
                for j in range(len(previous[key])):
                    q = [row[key][j] for row in stage["rows"]]
                    velocities = [(b-a)*p.RATE for a, b in zip(q, q[1:])]
                    self.assertLessEqual(max(map(abs, velocities)), p.MAX_SPEED+1e-8)
                    acceleration = [(b-a)*p.RATE for a, b in zip(velocities, velocities[1:])]
                    self.assertLessEqual(max(map(abs, acceleration)), p.MAX_ACCELERATION+1e-8)
            for row in stage["rows"]:
                p.validate_pose(row)
                self.assertEqual(p.delta(row, stage["start"], inactive), 0)
            samples = p.collision_samples(stage)
            self.assertTrue(all(p.delta(a, b) <= .500001 for a, b in zip(samples, samples[1:])))
            previous = stage["target"]

    def test_nonfinite_pose_rejected(self):
        for key in p.FIELDS:
            state = initial()
            state[key][0] = math.nan
            with self.assertRaises(ValueError):
                p.build_plan(state)


class RuntimeTests(unittest.TestCase):
    def runner(self):
        robot = Mock()
        robot._lock = nullcontext()
        return r.Runner(robot)

    def test_upload_failure_never_starts(self):
        runner = self.runner()
        runner.stationary = Mock()
        runner.upload = Mock(side_effect=RuntimeError("upload failed"))
        with self.assertRaises(RuntimeError):
            runner.execute_stage(p.build_plan(initial())[0])
        runner.robot.runtime_run_traj.assert_not_called()
        runner.robot.dll.FX_L0_Body_Runtime_RunTraj.assert_not_called()

    def test_missing_second_ready_buffer_never_starts(self):
        runner = self.runner()
        runner.stationary = Mock(return_value={"components": [{"trajectory": 2}, {"trajectory": 1}]})
        runner.upload = Mock()
        with self.assertRaises(RuntimeError):
            runner.execute_stage(p.build_plan(initial())[0])
        runner.robot.runtime_run_traj.assert_not_called()

    def test_body_uses_dedicated_start_not_unsupported_mask(self):
        runner = self.runner()
        runner.stationary = Mock(return_value={"components": [{"trajectory": 0},
                                                             {"trajectory": 0}, {"trajectory": 2}]})
        runner.upload = Mock()
        runner.call = Mock(return_value=0)
        runner.snapshot = Mock(side_effect=RuntimeError("test ends after start"))
        with patch.object(r, "emit"), self.assertRaises(RuntimeError):
            runner.execute_stage(p.build_plan(initial())[1])
        runner.call.assert_called_once_with(2, "RunTraj")
        runner.robot.runtime_run_traj.assert_not_called()

    def test_stop_requests_body_and_both_arms(self):
        runner = self.runner()
        runner.call = Mock(return_value=0)
        runner.snapshot = Mock(return_value={"components": [{"trajectory": 0, "velocity": [0]}]*3})
        with patch.object(r, "emit"):
            runner.stop()
        self.assertEqual([call.args for call in runner.call.call_args_list],
                         [(0, "StopTraj"), (1, "StopTraj"), (2, "StopTraj")])

    def test_stop_escalates_when_feedback_cannot_confirm(self):
        runner = self.runner()
        runner.call = Mock(return_value=0)
        runner.snapshot = Mock(side_effect=RuntimeError("lost feedback"))
        with patch.object(r, "emit"):
            runner.stop()
        runner.robot.emergency_stop.assert_called_once_with(11)

    def test_body_upload_packs_six_axes_and_requires_new_ack(self):
        runner = self.runner()
        states = [0, 1, 1, 1, 1, 2]
        runner.stationary = Mock(side_effect=[
            {"components": [{}, {}, {"trajectory": value}]} for value in states])
        runner.call = Mock(return_value=0)
        rows = p.build_plan(initial())[1]["rows"][:125]
        with patch.object(r.time, "sleep"):
            runner.upload(2, rows, initial())
        calls = runner.call.call_args_list
        self.assertEqual(calls[0].args, (2, "InitTraj", 125))
        self.assertEqual([call.args[2:4] for call in calls[1:]], [(0, 50), (1, 50), (2, 25)])
        actual = [value for call in calls[1:] for value in call.args[4]]
        self.assertEqual(actual, [value for row in rows for value in row["body"]])
        runner.robot.runtime_run_traj.assert_not_called()

    def test_old_ready_buffer_is_not_accepted_as_new_upload(self):
        runner = self.runner()
        runner.stationary = Mock(return_value={"components": [{}, {}, {"trajectory": 2}]})
        runner.call = Mock(return_value=0)
        with patch.object(r.time, "sleep"), patch.object(r.time, "monotonic", side_effect=[0, 3]):
            with self.assertRaisesRegex(RuntimeError, "initialization"):
                runner.upload(2, p.build_plan(initial())[1]["rows"], initial())
        self.assertEqual([call.args[1] for call in runner.call.call_args_list], ["InitTraj"])

    def test_body_completion_requires_running_then_settled_target(self):
        runner = self.runner()
        stage = p.build_plan(initial())[1]
        def feedback(trajectory, state):
            return {"pose": state, "components": [
                {"trajectory": 0, "velocity": [0]*7},
                {"trajectory": 0, "velocity": [0]*7},
                {"trajectory": trajectory, "velocity": [0]*6}]}
        runner.stationary = Mock(return_value=feedback(2, stage["start"]))
        runner.upload = Mock()
        runner.call = Mock(return_value=0)
        runner.snapshot = Mock(side_effect=[
            feedback(3, stage["start"]),
            feedback(0, stage["target"]), feedback(0, stage["target"]), feedback(0, stage["target"])])
        with patch.object(r.time, "sleep"), patch.object(r, "emit") as output:
            runner.execute_stage(stage)
        self.assertEqual(output.call_args.args[0], "stage_complete")
        self.assertEqual(runner.snapshot.call_count, 4)


class GeometryTests(unittest.TestCase):
    def test_sdk_frame_conversion_and_mismatch_rejection(self):
        identity = ((1., 0., 0., 0.), (0., 1., 0., 0.),
                    (0., 0., 1., 0.), (0., 0., 0., 1.))
        matrices = {"base": identity, "Arm_L7_Link": identity, "Arm_R7_Link": identity}
        model = SimpleNamespace(
            joints={name: SimpleNamespace(parent="base", origin=identity)
                    for name in ("Arm_L1_Joint", "Arm_R1_Joint")},
            link_transforms=lambda _: matrices)
        geometry = r.Geometry.__new__(r.Geometry)
        geometry.checker = SimpleNamespace(model=model)
        geometry.state_type = lambda *args: SimpleNamespace(radians_by_name=lambda: {})
        geometry.flange_transforms = {}
        robot = Mock()
        robot.init_single_arm_config.return_value = 0
        parameters = {"L0.DH.D": 158.6, "FLANGE.D": 87.,
                      "FLANGE.Alpha": 90., "FLANGE.Theta": 90.}
        robot.param_get_float.side_effect = lambda name: (0, parameters.get(".".join(name.split(".")[2:]), 0.))
        expected = [[0., -1., 0., 0.], [0., 0., -1., -87.],
                    [1., 0., 0., 158.6], [0., 0., 0., 1.]]
        def matmul(a, b):
            return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4))
                         for i in range(4))
        modules = {"collision_guard": Mock(),
                   "collision_guard.math3d": SimpleNamespace(matmul=matmul)}
        with patch.dict(sys.modules, modules), patch.object(r, "emit"):
            robot.forward_kinematics.return_value = expected
            geometry.verify_sdk(robot, p.build_plan(initial()))
            bad = [row.copy() for row in expected]
            bad[0][3] = 20
            robot.forward_kinematics.return_value = bad
            with self.assertRaisesRegex(RuntimeError, "disagree"):
                geometry.verify_sdk(robot, p.build_plan(initial()))
            bad[0][3] = math.nan
            with self.assertRaisesRegex(RuntimeError, "FK failed"):
                geometry.verify_sdk(robot, p.build_plan(initial()))


if __name__ == "__main__":
    unittest.main()
