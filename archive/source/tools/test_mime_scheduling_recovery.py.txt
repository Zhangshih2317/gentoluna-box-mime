"""Bounded scheduling jitter holds the target; hard faults still propagate."""

import copy
import unittest
from unittest.mock import Mock, patch

from box_motion_plan import delta
from motion_wait_policy import CandidateStageDriver
from test_box_mime_continuous import Clock
import test_box_mime_taught as fixtures
import test_mime_speed_setting as entry_fixtures


class Tests(unittest.TestCase):
    def setUp(self):
        f = fixtures.Tests()
        f.setUp()
        self.stage = next(s for s in f.stages if s["name"] == "crouch")
        self.adapter = Mock()
        self.velocity = {k: [0.]*len(v) for k, v in self.stage["start"].items()}
        self.now = 0.
        self.delay = 0.
        self.serial = 0
        self.driver = CandidateStageDriver(
            self.stage, 0., timing_planner=lambda stage: 10.,
            clock=lambda: self.now+self.delay)

    def tick(self, dt=.02, fresh=True):
        self.now += dt
        self.serial += int(fresh)
        self.adapter.read_checked.return_value = (
            copy.deepcopy(self.driver.command), self.velocity, self.serial)
        return self.driver.tick(self.adapter, self.now, 10.)

    def test_eighty_ms_between_ticks_holds_and_restarts_without_catchup(self):
        for _ in range(100):
            self.tick()
        before = copy.deepcopy(self.driver.command)
        elapsed = self.driver.elapsed
        self.assertEqual(self.tick(.08).reason, "scheduling_gap")
        self.assertEqual(self.driver.command, before)
        self.assertEqual(self.driver.elapsed, elapsed)
        self.assertEqual(self.tick(fresh=False).action, "hold")
        self.assertEqual(self.tick().action, "hold")
        self.assertEqual(self.tick().action, "hold")
        self.assertEqual(self.tick().action, "advance")
        self.assertEqual(self.driver.command, before)
        self.assertEqual(self.driver.restart_count, 1)
        self.assertEqual(self.driver.elapsed, elapsed)
        for _ in range(600):
            result = self.tick()
            if result.action == "complete":
                break
        self.assertEqual(result.action, "complete")

    def test_delayed_feedback_read_does_not_send_a_new_target(self):
        self.tick()
        before = copy.deepcopy(self.driver.command)
        self.delay = .08
        result = self.tick()
        self.assertEqual(result.reason, "scheduling_gap")
        self.adapter.send_checked.assert_called_with(before)
        self.assertAlmostEqual(self.driver.last_read_seconds, .08)

    def test_persistent_short_delays_latch_without_emergency_or_auto_resume(self):
        for _ in range(130):
            result = self.tick(.08)
        self.assertEqual(result.action, "operator_hold")
        for _ in range(5):
            self.assertEqual(self.tick().action, "operator_hold")
        self.assertEqual(self.driver.elapsed, 0.)
        self.adapter.emergency_stop.assert_not_called()

    def test_long_gap_and_long_read_still_reject_before_send(self):
        with self.assertRaisesRegex(RuntimeError, "scheduling gap"):
            self.tick(.201)
        self.adapter.send_checked.assert_not_called()
        self.setUp()
        self.delay = .201
        with self.assertRaisesRegex(RuntimeError, "during feedback read"):
            self.tick()
        self.adapter.send_checked.assert_not_called()

    def test_controller_fault_during_short_delay_is_not_suppressed(self):
        self.adapter.read_checked.side_effect = RuntimeError("controller fault")
        with self.assertRaisesRegex(RuntimeError, "controller fault"):
            self.tick(.08)
        self.adapter.send_checked.assert_not_called()

    def test_legacy_driver_retains_its_scheduling_contract(self):
        driver = CandidateStageDriver(self.stage, 0.)
        with self.assertRaisesRegex(RuntimeError, "scheduling gap"):
            driver.tick(self.adapter, .08, 10.)
        self.adapter.read_checked.assert_not_called()

    def test_generated_runner_recovers_from_slow_sdk_read_and_completes(self):
        fixture = entry_fixtures.Tests()
        fixture.setUp()
        with fixture.remote_modules(10.) as (plan, control, _):
            stage = next(s for s in plan.build_stages(
                fixture.fixture.home, fixture.fixture.home, fixture.fixture.squat,
                fixture.fixture.waist) if s["name"] == "crouch")
            robot = fixture.robot(10., stage["start"])
            runner = control.MimeRunner(robot, lambda: None)
            runner.enabled = {"arm0", "arm1", "body"}
            runner.command = copy.deepcopy(stage["start"])
            clock = Clock()
            original_read = robot.get_rt_dict
            delayed = [False]

            def slow_read():
                result = original_read()
                if clock.now >= 2. and not delayed[0]:
                    delayed[0] = True
                    clock.sleep(.08)
                return result

            def send(obj, command):
                robot.pose[{0: "arm0", 1: "arm1", 3: "body"}[obj]] = list(command)
                return 0

            robot.get_rt_dict = slow_read
            robot.runtime_set_joint_pos_cmd.side_effect = send
            with patch.object(control.time, "monotonic", clock.monotonic), \
                    patch.object(control.time, "sleep", clock.sleep), \
                    patch.object(control, "emit"):
                result = runner.stream_stages_with_wait([stage])
            self.assertLess(delta(result, stage["target"]), .03)
            rows = list(runner.response_samples)
            late = next(row for row in rows if row["feedback_read_seconds"] > .05)
            self.assertEqual(late["reason"], "scheduling_gap")
            self.assertEqual(late["command_before_read"], late["command_after_tick"])
            self.assertEqual(rows[-1]["smooth_restart_count"], 1)
            robot.emergency_stop.assert_not_called()
            robot.switch_to_idle.assert_not_called()


if __name__ == "__main__":
    unittest.main()
