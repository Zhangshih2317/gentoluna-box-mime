"""Offline tests only; no loaded-hardware execution authorization."""

import copy
import unittest
from unittest.mock import Mock

from box_motion_plan import interpolate
from motion_wait_policy import MotionWaitPolicy, CandidateStageDriver
import test_box_mime_taught as fixtures


class Tests(unittest.TestCase):
    def setUp(self):
        f = fixtures.Tests()
        f.setUp()
        self.stage = next(s for s in f.stages if s["name"] == "crouch")
        self.start, self.target = self.stage["start"], self.stage["target"]
        self.velocity = {k: [0.]*len(v) for k, v in self.start.items()}
        self.policy = MotionWaitPolicy(self.stage, 0.)

    def decide(self, actual, now=0., serial=1, endpoint=False, command=None):
        return self.policy.update(actual, command or self.target, self.velocity,
                                  serial, now, endpoint)

    def test_two_degree_torso_error_is_accepted_but_not_blanket_axis_drift(self):
        actual = interpolate(self.start, self.target, .5)
        actual["body"][1] += 1.99
        self.assertEqual(self.decide(actual).action, "advance")
        actual["body"][0] += .06
        with self.assertRaisesRegex(RuntimeError, "stationary-axis"):
            self.decide(actual)

    def test_larger_pitch_deviation_holds_not_raises_or_completes(self):
        actual = interpolate(self.start, self.target, .5)
        actual["body"][1] += 1.1
        actual["body"][3] += 1.1
        self.assertEqual(self.decide(actual).reason, "torso_sum_deviation")
        self.assertEqual(self.decide(actual, now=2).action, "hold")

    def test_two_second_endpoint_delay_does_not_stop(self):
        actual = interpolate(self.start, self.target, .8)
        for i in range(101):
            self.assertEqual(self.decide(actual, now=i*.02, serial=i, endpoint=True).action, "hold")
        for i in range(3):
            result = self.decide(self.target, now=2.02+i*.02, serial=200+i, endpoint=True)
        self.assertEqual(result.action, "complete")

    def test_persistent_deviation_latches_operator_hold_without_auto_resume(self):
        actual = interpolate(self.start, self.target, .5)
        self.decide(actual, endpoint=True)
        self.assertEqual(self.decide(actual, now=10, endpoint=True).action, "operator_hold")
        for i in range(4):
            result = self.decide(self.target, now=11+i, serial=100+i, endpoint=True)
        self.assertEqual(result.action, "operator_hold")

    def test_same_feedback_frame_cannot_satisfy_arrival(self):
        for i in range(4):
            result = self.decide(self.target, now=i*.02, endpoint=True)
        self.assertEqual(result.action, "hold")

    def test_home_tolerance_remains_tighter_than_stage_tolerance(self):
        actual = copy.deepcopy(self.target)
        actual["body"][2] += .1
        for i in range(3):
            result = self.decide(actual, now=i*.02, serial=i, endpoint=True)
        self.assertEqual(result.action, "complete")
        self.policy = MotionWaitPolicy(self.stage, 0., final_home=True)
        for i in range(3):
            result = self.decide(actual, now=i*.02, serial=i, endpoint=True)
        self.assertEqual(result.action, "hold")

    def test_invalid_feedback_and_hard_joint_limits_still_reject(self):
        for value in (float("nan"), -999):
            actual = copy.deepcopy(self.target)
            actual["body"][2] = value
            with self.assertRaises((ValueError, RuntimeError)):
                self.decide(actual)

    def test_driver_holds_last_command_and_freezes_trajectory_clock(self):
        adapter = Mock()
        driver = CandidateStageDriver(self.stage, 0.)
        actual = copy.deepcopy(self.start)
        actual["body"][1] += 1.1
        actual["body"][3] += 1.1
        adapter.read_checked.return_value = (actual, self.velocity, 1)
        result = driver.tick(adapter, .02, 10.)
        self.assertEqual(result.action, "hold")
        self.assertEqual(driver.elapsed, 0.)
        adapter.send_checked.assert_called_with(self.start)
        adapter.read_checked.return_value = (self.start, self.velocity, 2)
        driver.tick(adapter, .04, 10.)
        self.assertEqual(driver.elapsed, 0.)
        adapter.read_checked.return_value = (self.start, self.velocity, 3)
        driver.tick(adapter, .06, 10.)
        self.assertAlmostEqual(driver.elapsed, .02)
        adapter.emergency_stop.assert_not_called()
        adapter.switch_to_idle.assert_not_called()
        adapter.unlink.assert_not_called()

    def test_hard_adapter_fault_does_not_issue_another_command(self):
        adapter = Mock()
        adapter.read_checked.side_effect = RuntimeError("controller fault")
        with self.assertRaisesRegex(RuntimeError, "controller fault"):
            CandidateStageDriver(self.stage, 0.).tick(adapter, .02, 10.)
        adapter.send_checked.assert_not_called()

    def test_full_candidate_stage_with_two_second_delayed_feedback(self):
        driver = CandidateStageDriver(self.stage, 0.)
        sent = [(0., copy.deepcopy(self.start))]
        adapter = Mock()
        for i in range(1, 800):
            now = i*.02
            actual = next(p for t, p in reversed(sent) if t <= max(0., now-2.))
            adapter.read_checked.return_value = (actual, self.velocity, i)
            adapter.send_checked.side_effect = lambda p: sent.append((now, p))
            decision = driver.tick(adapter, now, 10.)
            if decision.action == "complete":
                break
        self.assertEqual(decision.action, "complete")
        self.assertGreater(now, 11.)
        self.assertFalse(driver.policy.latched)
        adapter.emergency_stop.assert_not_called()


if __name__ == "__main__":
    unittest.main()
