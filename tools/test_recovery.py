"""Recovery guard regressions; no controller connection."""

import copy
import unittest
from unittest.mock import patch

from box_motion_plan import interpolate
from recover_box_start_robot import Recovery, guard_pose


START = {"arm0": [90, -90, -108, -90, 0, 0, 0],
         "arm1": [-90, -90, 108, -90, 0, 0, 0],
         "body": [-9.259, -2.928, 2.691, -1.258, -2.888, 9.411],
         "head": [2.108, -15.998, 0]}
TARGET = copy.deepcopy(START)
TARGET["body"] = [-.744, 9.281, -17.364, 5.456, -.296, -1.653]


class Tests(unittest.TestCase):
    def test_inward_recovery_near_limit_is_allowed(self):
        sample = interpolate(START, TARGET, .01)
        guard_pose(sample, sample, START, TARGET, ("body",))

    def test_wrong_direction_tracking_and_uncommanded_motion_rejected(self):
        end = interpolate(START, TARGET, .01)
        for key, axis, amount in (("body", 0, -.2), ("body", 1, 1.),
                                  ("arm0", 0, .06), ("head", 0, .06)):
            actual = copy.deepcopy(START)
            actual[key][axis] += amount
            with self.assertRaises(RuntimeError):
                guard_pose(actual, START, START, end, ("body",))
        actual = copy.deepcopy(START)
        actual["body"][0] = float("nan")
        with self.assertRaises(ValueError):
            guard_pose(actual, START, START, end, ("body",))

    def test_detected_deviation_prevents_next_command(self):
        runner = Recovery(None)
        actual = copy.deepcopy(START)
        actual["arm1"][0] += .1
        with patch.object(runner, "read", return_value=(actual, {}, {})), \
                patch.object(runner, "send") as send:
            with self.assertRaises(RuntimeError):
                runner.segment(START, TARGET, ("body",))
            send.assert_not_called()

    def test_send_targets_only_enabled_groups(self):
        class SDK:
            def __init__(self):
                self.calls = []

            def runtime_set_joint_pos_cmd(self, obj, positions):
                self.calls.append((obj, positions))
                return 0
        robot = SDK()
        runner = Recovery(robot)
        runner.enabled.add("body")
        runner.send(START)
        self.assertEqual(robot.calls, [(3, START["body"])])


if __name__ == "__main__":
    unittest.main()
