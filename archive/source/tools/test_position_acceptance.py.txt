"""Whole-body two-degree acceptance, independent feedback and time windows."""

import copy
import unittest

from position_acceptance import PositionAcceptance
from motion_wait_policy import MotionWaitPolicy
import test_box_mime_taught as fixtures


class Tests(unittest.TestCase):
    def setUp(self):
        fixture = fixtures.Tests()
        fixture.setUp()
        self.stage = next(s for s in fixture.stages if s["name"] == "crouch")
        self.target = self.stage["target"]
        self.velocity = {k: [0.]*len(v) for k, v in self.target.items()}

    def test_two_frames_require_spacing_and_expire_after_one_second(self):
        a = PositionAcceptance()
        self.assertFalse(a.update(self.target, self.target, 1, 0.))
        self.assertFalse(a.update(self.target, self.target, 2, .02))
        self.assertFalse(a.update(self.target, self.target, 1, .1))
        self.assertFalse(a.update(self.target, self.target, 3, 1.01))
        self.assertTrue(a.update(self.target, self.target, 4, 1.11))

    def test_outside_tolerance_resets_confirmation(self):
        a = PositionAcceptance()
        bad = copy.deepcopy(self.target)
        bad["arm0"][6] += 2.001
        self.assertFalse(a.update(self.target, self.target, 1, 0.))
        self.assertFalse(a.update(bad, self.target, 2, .1))
        self.assertFalse(a.update(self.target, self.target, 3, .2))
        self.assertTrue(a.update(self.target, self.target, 4, .3))

    def test_whole_body_endpoint_is_accepted_including_held_axes_and_torso_sum(self):
        policy = MotionWaitPolicy(self.stage, 0., final_home=True)
        policy.use_whole_body_position_acceptance()
        actual = copy.deepcopy(self.target)
        for key in actual:
            actual[key][0] += 2.
        actual["body"][1] += 1.1
        actual["body"][3] += 1.1
        self.velocity["arm0"][6] = .3
        first = policy.update(actual, self.target, self.velocity, 1, .1, True)
        second = policy.update(actual, self.target, self.velocity, 2, .2, True)
        self.assertEqual(first.action, "hold")
        self.assertEqual(second.action, "complete")

    def test_controller_held_axis_outside_allowance_still_rejects(self):
        policy = MotionWaitPolicy(self.stage, 0.)
        policy.use_whole_body_position_acceptance()
        actual = copy.deepcopy(self.target)
        actual["body"][0] += 2.001
        with self.assertRaisesRegex(RuntimeError, "stationary-axis"):
            policy.update(actual, self.target, self.velocity, 1, .1, True)

    def test_no_confirmation_before_endpoint_command(self):
        policy = MotionWaitPolicy(self.stage, 0.)
        policy.use_whole_body_position_acceptance()
        for serial in range(1, 4):
            self.assertNotEqual(policy.update(
                self.target, self.target, self.velocity, serial, serial*.1, False).action,
                "complete")


if __name__ == "__main__":
    unittest.main()
