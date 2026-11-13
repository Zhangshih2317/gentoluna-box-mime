"""Run tests with network connections blocked, never operating hardware."""

import argparse
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
PURE = (
    "test_box_motion", "test_position_acceptance", "test_feedback_observation",
    "test_motion_wait_policy", "test_integrated_motion_wait",
    "test_coordinated_joint_timing", "test_recovery", "test_teach_pose",
    "test_box_mime_cache", "test_public_config",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--with-calibration", action="store_true",
                        help="Also test height solving and generated entries using locally provided calibration")
    args = parser.parse_args()
    if args.with_calibration and not (ROOT/"reference/robot_frames/calibration_transforms").is_dir():
        parser.error("First provide your licensed calibration_transforms under reference/robot_frames/")
    loader = unittest.TestLoader()
    with patch.dict(os.environ, {"LUNA_CONTROLLER_IP": "192.0.2.20"}), \
            patch("socket.socket.connect", side_effect=AssertionError("Offline tests forbid network")), \
            patch("socket.create_connection", side_effect=AssertionError("Offline tests forbid network")):
        suite = (loader.discover(str(ROOT/"tools"), "test_*.py")
                 if args.with_calibration else loader.loadTestsFromNames(PURE))
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
