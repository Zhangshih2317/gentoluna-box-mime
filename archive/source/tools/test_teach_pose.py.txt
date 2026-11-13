"""Read-only teaching capture and incident lockout regressions."""

import copy
import unittest
from unittest.mock import patch

import box_motion_robot
import teach_pose


def samples():
    output = []
    for serial in range(5):
        output.append({
            "rt": {"frame_serial": serial,
                   "arms": [{"state": {"err": 0}, "fb": {"fb_pos": [0]*7, "fb_vel": [0]*7}}
                            for _ in range(2)],
                   "body": {"state": {"err": 0}, "fb_pos": [0]*6, "fb_vel": [0]*6},
                   "head": {"state": {"err": 0}, "fb_pos": [0]*3}},
            "sg": {"arms": [{"get": {"traj_state": 0}} for _ in range(2)],
                   "body": {"get": {"traj_state": 0}}}})
    return output


class Tests(unittest.TestCase):
    def test_stationary_point_is_preserved(self):
        data = samples()
        self.assertEqual(teach_pose.validate_recording(data), teach_pose.joint_pose(data[-1]))

    def test_stale_moving_faulted_or_nonfinite_capture_rejected(self):
        variants = []
        stale = samples()
        stale[-1]["rt"]["frame_serial"] = 0
        variants.append(stale)
        moving = samples()
        moving[-1]["rt"]["body"]["fb_pos"][1] = .2
        variants.append(moving)
        fault = samples()
        fault[-1]["rt"]["body"]["state"]["err"] = 110
        variants.append(fault)
        invalid = samples()
        invalid[-1]["rt"]["arms"][0]["fb"]["fb_vel"][0] = float("nan")
        variants.append(invalid)
        for data in variants:
            with self.assertRaises(ValueError):
                teach_pose.validate_recording(data)

    def test_old_motion_entry_rejected_before_sdk_access(self):
        with patch("sys.argv", ["box_motion_robot.py", "--execute"]):
            with self.assertRaisesRegex(RuntimeError, "disabled after BODY"):
                box_motion_robot.main()


if __name__ == "__main__":
    unittest.main()
