"""Exercise the generated remote modules with no SDK or SSH connections."""

import ast
import copy
from contextlib import contextmanager, nullcontext
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

from box_motion_plan import delta
from test_box_mime_continuous import Clock
import test_box_mime_taught as fixtures


class Tests(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location("speed_entry", root/"run_box_mime_taught.py")
        self.entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.entry)
        self.fixture = fixtures.Tests()
        self.fixture.setUp()

    @contextmanager
    def remote_modules(self, speed, home_only=False, acceleration=5., body_speed=None,
                       body_acceleration=5., native=False):
        settings = {} if speed is None else {
            "ARM_SPEED_PERCENT": speed, "ARM_ACCELERATION_PERCENT": acceleration,
            "BODY_SPEED_PERCENT": speed if body_speed is None else body_speed,
            "BODY_ACCELERATION_PERCENT": body_acceleration}
        with patch.multiple(self.entry, **settings) if settings else nullcontext():
            tree = ast.parse(self.entry.source_for(False, home_only=home_only, continuous=native))
        first = next(i for i, node in enumerate(tree.body)
                     if isinstance(node, ast.Assign)
                     and isinstance(node.targets[0], ast.Name)
                     and node.targets[0].id == "module")
        # Load exactly the generated modules/configuration, omitting the
        # heartbeat thread and final hardware run() call.
        code = compile(ast.Module(body=tree.body[first:-1], type_ignores=[]),
                       "<offline-speed-modules>", "exec")
        with patch.dict(sys.modules):
            exec(code, {"sys": sys, "types": types})
            yield (sys.modules["box_mime_taught_plan"],
                   sys.modules["box_mime_taught_robot"], tree.body[-1].value)

    def robot(self, speed, state=None, acceleration=5., body_speed=None, body_acceleration=5.):
        robot = self.fixture.fake(state)
        original = robot.get_sg_dict

        def sg():
            values = original()
            for group in values["arms"]:
                group["set"]["vel_ratio"] = speed
                group["set"]["acc_ratio"] = acceleration
            values["body"]["set"]["vel_ratio"] = speed if body_speed is None else body_speed
            values["body"]["set"]["acc_ratio"] = body_acceleration
            return values

        robot.get_sg_dict = sg
        return robot

    def test_group_settings_reach_idle_or_enabled_body(self):
        for speed in (20., 30., 80., 100.):
            for body_idle in (False, True):
                with self.subTest(speed=speed, idle=body_idle), self.remote_modules(speed) as (_, control, _):
                    robot = self.robot(speed)
                    if body_idle:
                        robot.body_mode = 0
                        robot.switch_to_position_mode.side_effect = (
                            lambda *args: setattr(robot, "body_mode", 1) or 0)
                    runner = control.MimeRunner(robot, lambda: None)
                    with patch.object(control.time, "sleep"), patch.object(control, "emit"):
                        runner.enable_all()
                    for obj in (0, 1):
                        robot.runtime_set_speed_ratio.assert_any_call(obj, speed, 5.)
                    if body_idle:
                        robot.switch_to_position_mode.assert_called_once_with(3, 2000, speed, 5.)
                    else:
                        robot.runtime_set_speed_ratio.assert_any_call(3, speed, 5.)
                    robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_speed_scales_until_acceleration_cap_without_target_or_cache_changes(self):
        baseline, stages, code_id = None, None, None
        for speed in (10., 20., 30., 80., 100.):
            with self.remote_modules(speed) as (plan, _, call):
                current = plan.build_stages(self.fixture.start, self.fixture.home,
                                            self.fixture.squat, self.fixture.waist)
                durations = [plan.continuous_seconds(s) for s in current]
                current_id = next(ast.literal_eval(k.value) for k in call.keywords
                                  if k.arg == "code_id")
                if baseline is None:
                    baseline, stages, code_id = durations, current, current_id
                else:
                    self.assertEqual(current, stages)
                    self.assertEqual(current_id, code_id)
                    for stage, old, new in zip(current, baseline, durations):
                        if stage["name"] != "align_standing_home":
                            self.assertLessEqual(new, old, stage["name"])
                self.assertEqual(plan.ACCELERATION_PERCENT, 5.)
                self.assertLessEqual(plan.STREAM_ACCELERATION, .45)
                self.assertLessEqual(plan.BODY_STREAM_ACCELERATION, 1.8)
                self.assertTrue(plan.COORDINATED_TIMING)
                baseline = durations

    def test_accelerated_full_flow_and_delayed_arrival_with_mock_feedback(self):
        for speed, delay, acceleration, body_speed in (
                (10., 0., 5., 10.), (30., 0., 5., 30.), (80., 0., 5., 80.),
                (100., 0., 5., 100.), (80., 2., 5., 80.),
                (100., 0., 100., 10.), (100., 2., 100., 10.)):
            with self.subTest(speed=speed, delay=delay, acceleration=acceleration), \
                    self.remote_modules(speed, acceleration=acceleration,
                                        body_speed=body_speed) as (plan, control, _):
                stages = plan.build_stages(self.fixture.start, self.fixture.home,
                                          self.fixture.squat, self.fixture.waist)
                robot = self.robot(speed, acceleration=acceleration, body_speed=body_speed)
                runner = control.MimeRunner(robot, lambda: None)
                runner.enabled = {"arm0", "arm1", "body"}
                runner.command = copy.deepcopy(self.fixture.start)
                clock, sent = Clock(), []

                def send(obj, values):
                    robot.pose[{0: "arm0", 1: "arm1", 3: "body"}[obj]] = list(values)
                    if obj == 3:
                        sent.append((clock.now, copy.deepcopy(robot.pose)))
                    return 0

                robot.runtime_set_joint_pos_cmd.side_effect = send
                original_rt = robot.get_rt_dict

                def rt():
                    result = original_rt()
                    past = next((p for t, p in reversed(sent) if t <= clock.now-delay),
                                self.fixture.start)
                    for i, key in enumerate(("arm0", "arm1")):
                        result["arms"][i]["fb"]["fb_pos"] = list(past[key])
                    result["body"]["fb_pos"] = list(past["body"])
                    return result

                if delay:
                    robot.get_rt_dict = rt
                def paced_sleep(seconds):
                    # Exercise allowed late ticks as well as nominal 50 Hz.
                    clock.sleep(max(seconds, .04) if speed == 100. else seconds)

                with patch.object(control.time, "monotonic", clock.monotonic), \
                        patch.object(control.time, "sleep", paced_sleep), \
                        patch.object(control, "emit") as events:
                    final = runner.stream_stages_with_wait(stages)
                self.assertLess(delta(final, self.fixture.home), .03)
                self.assertEqual(sum(c.args[0] == "stage_complete" for c in events.call_args_list),
                                 len(stages))
                nominal = sum(plan.continuous_seconds(s) for s in stages)
                # Arrival uses a position tolerance, so it can precede the
                # exact endpoint's delayed sample near a zero-velocity finish.
                self.assertGreaterEqual(clock.now, nominal+delay)
                robot.emergency_stop.assert_not_called()
                robot.switch_to_idle.assert_not_called()
                self.assertGreater(len(runner.response_samples), 0)
                self.assertEqual(runner.response_samples[-1]["action"], "complete")
                self.assertLess(delta(runner.response_samples[-1]["command_after_tick"],
                                      stages[-1]["target"]), 1e-9)

    def test_actual_entry_defaults_match_latest_successful_100_50_run(self):
        self.assertFalse(hasattr(self.entry, "MOTION_SPEED_PERCENT"))
        with self.remote_modules(None, native=True) as (plan, control, _):
            self.assertTrue(plan.NATIVE_ARM_POSITION)
            self.assertEqual(plan.controller_settings(), {
                "arm0": {"speed_percent": 100., "acceleration_percent": 50.},
                "arm1": {"speed_percent": 100., "acceleration_percent": 50.},
                "body": {"speed_percent": 100., "acceleration_percent": 50.}})
            for body_idle in (False, True):
                robot = self.robot(100., acceleration=50., body_speed=100., body_acceleration=50.)
                if body_idle:
                    robot.body_mode = 0
                    robot.switch_to_position_mode.side_effect = (
                        lambda *args: setattr(robot, "body_mode", 1) or 0)
                runner = control.MimeRunner(robot, lambda: None)
                with patch.object(control.time, "sleep"), patch.object(control, "emit"):
                    runner.enable_all()
                for obj in (0, 1):
                    robot.runtime_set_speed_ratio.assert_any_call(obj, 100., 50.)
                if body_idle:
                    robot.switch_to_position_mode.assert_called_once_with(3, 2000, 100., 50.)
                else:
                    robot.runtime_set_speed_ratio.assert_any_call(3, 100., 50.)
                robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_arm_speed_and_acceleration_are_independent_of_body_timing(self):
        snapshots = []
        for speed, acceleration in ((10., 5.), (100., 5.), (100., 100.)):
            with self.remote_modules(speed, acceleration=acceleration, body_speed=10.) as (plan, _, call):
                stages = plan.build_stages(self.fixture.start, self.fixture.home,
                                          self.fixture.squat, self.fixture.waist)
                snapshots.append({
                    "stages": stages,
                    "code_id": next(ast.literal_eval(k.value) for k in call.keywords if k.arg == "code_id"),
                    "body_timings": [plan.coordinated_timing(s) for s in stages
                                     if s["moving"] == ["body"]],
                    "arm_seconds": [plan.continuous_seconds(s) for s in stages
                                    if s["moving"] == ["arm0", "arm1"]],
                    "arm_limits": (plan.STREAM_SPEED, plan.STREAM_ACCELERATION),
                    "body_limits": (plan.BODY_STREAM_SPEED, plan.BODY_STREAM_ACCELERATION),
                })
        for current in snapshots[1:]:
            for name in ("stages", "code_id", "body_timings", "body_limits"):
                self.assertEqual(current[name], snapshots[0][name], name)
        self.assertEqual(snapshots[-1]["arm_limits"], (18., 9.))
        self.assertEqual(snapshots[-1]["body_limits"], (3.6, .9))
        self.assertEqual(snapshots[1]["arm_limits"][1], snapshots[0]["arm_limits"][1])
        self.assertEqual(snapshots[2]["arm_limits"][0], snapshots[1]["arm_limits"][0])
        for old, new in zip(snapshots[1]["arm_seconds"], snapshots[2]["arm_seconds"]):
            self.assertLess(new, old)

    def test_high_arm_speed_does_not_relax_body_velocity_protection(self):
        with self.remote_modules(100., acceleration=100., body_speed=10.) as (plan, control, _):
            robot = self.robot(100., acceleration=100., body_speed=10.)
            original = robot.get_rt_dict
            def rt():
                result = original()
                result["body"]["fb_vel"][2] = 4.21
                return result
            robot.get_rt_dict = rt
            with self.assertRaisesRegex(RuntimeError, "velocity: body"):
                control.MimeRunner(robot, lambda: None).read()

    def test_invalid_speed_rejects_before_connector_import(self):
        for name in ("ARM_SPEED_PERCENT", "ARM_ACCELERATION_PERCENT",
                     "BODY_SPEED_PERCENT", "BODY_ACCELERATION_PERCENT"):
            for value in (0, 101, float("nan"), float("inf"), True, "invalid"):
                with self.subTest(name=name, value=value), patch.object(self.entry, name, value), \
                        patch("sys.argv", ["entry", "--execute"]), patch("sys.stderr"), \
                        patch.object(self.entry.importlib.util, "spec_from_file_location") as connect:
                    with self.assertRaises(SystemExit) as error:
                        self.entry.main()
                    self.assertEqual(error.exception.code, 2)
                    connect.assert_not_called()

    def test_independent_home_entry_uses_thirty_speed_ten_acceleration(self):
        with self.remote_modules(80., home_only=True) as (plan, control, _):
            self.assertEqual(plan.SPEED_PERCENT, 30.)
            self.assertEqual(control.SPEED_PERCENT, 30.)
            self.assertEqual(control.ACCELERATION_PERCENT, 10.)
            self.assertAlmostEqual(plan.STREAM_SPEED, 5.4)
            self.assertAlmostEqual(plan.BODY_STREAM_SPEED, 10.8)
            self.assertAlmostEqual(plan.STREAM_ACCELERATION, .9)
            self.assertAlmostEqual(plan.BODY_STREAM_ACCELERATION, 3.6)
            self.assertFalse(plan.COORDINATED_TIMING)
            self.assertFalse(plan.NATIVE_ARM_POSITION)
            for body_idle in (False, True):
                robot = self.robot(30., acceleration=10., body_acceleration=10.)
                if body_idle:
                    robot.body_mode = 0
                    robot.switch_to_position_mode.side_effect = (
                        lambda *args: setattr(robot, "body_mode", 1) or 0)
                runner = control.MimeRunner(robot, lambda: None)
                with patch.object(control.time, "sleep"), patch.object(control, "emit"):
                    runner.enable_all()
                for obj in (0, 1):
                    robot.runtime_set_speed_ratio.assert_any_call(obj, 30., 10.)
                if body_idle:
                    robot.switch_to_position_mode.assert_called_once_with(3, 2000, 30., 10.)
                else:
                    robot.runtime_set_speed_ratio.assert_any_call(3, 30., 10.)

    def test_invalid_home_ratios_reject_before_connector_import(self):
        for name in ("HOME_SPEED_PERCENT", "HOME_ACCELERATION_PERCENT"):
            for value in (0, 101, float("nan"), float("inf"), True, "invalid"):
                with self.subTest(name=name, value=value), patch.object(self.entry, name, value), \
                        patch("sys.argv", ["entry", "--execute"]), patch("sys.stderr"), \
                        patch.object(self.entry.importlib.util, "spec_from_file_location") as connect:
                    with self.assertRaises(SystemExit) as error:
                        self.entry.main(home_only=True)
                    self.assertEqual(error.exception.code, 2)
                    connect.assert_not_called()

    def test_wrong_body_ratio_and_excess_velocity_are_still_rejected(self):
        with self.remote_modules(80.) as (plan, control, _):
            robot = self.robot(20.)
            runner = control.MimeRunner(robot, lambda: None)
            runner.enabled = {"body"}
            with self.assertRaisesRegex(RuntimeError, "ratio mismatch: body"):
                runner.read()
            robot = self.robot(80.)
            runner = control.MimeRunner(robot, lambda: None)
            original = robot.get_rt_dict

            def rt():
                values = original()
                values["body"]["fb_vel"][5] = plan.BODY_STREAM_SPEED+.61
                return values

            robot.get_rt_dict = rt
            with self.assertRaisesRegex(RuntimeError, "velocity: body"):
                runner.read()


if __name__ == "__main__":
    unittest.main()
