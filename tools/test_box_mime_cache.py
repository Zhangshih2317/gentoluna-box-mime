"""Cache binding and bounded interrupted-opening HOME recovery."""

import copy
import unittest
from unittest.mock import Mock, patch

from box_mime_cache import fast_check, main_stages, verify_binding
from box_mime_taught_plan import build_stages
from box_motion_plan import delta
import test_box_mime_taught as fixtures


class Tests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.Tests()
        self.fixture.setUp()
        self.stages = self.fixture.stages
        self.certificate = {"schema": "luna_mime_geometry_cache_v1",
                            "code_id": "same-code", "model_files": {"model": "same-hash"},
                            "flange_transforms": {"0": [[1]], "1": [[1]]},
                            "stages": main_stages(self.stages), "report": {
                                "samples": 2470, "minimum_flange_spacing_mm": 279.27}}

    def test_verified_main_path_is_reused(self):
        verify_binding(self.certificate, "same-code", {"model": "same-hash"}, self.stages)
        geometry, robot = Mock(), Mock()
        geometry.flange_transforms = {0: [[1]], 1: [[1]]}
        with patch("box_mime_cache.model_fingerprint", return_value={"model": "same-hash"}), \
                patch("box_mime_cache.emit"):
            fast_check(geometry, robot, self.stages, self.certificate, "same-code")
        geometry.verify_sdk.assert_called_once_with(robot, self.stages)
        self.assertGreater(geometry.check.call_count, 0)
        self.assertLess(geometry.check.call_count, 25)

    def test_changed_flange_configuration_rejects_cache(self):
        geometry, robot = Mock(), Mock()
        geometry.flange_transforms = {0: [[2]], 1: [[1]]}
        with patch("box_mime_cache.model_fingerprint", return_value={"model": "same-hash"}):
            with self.assertRaisesRegex(RuntimeError, "flange"):
                fast_check(geometry, robot, self.stages, self.certificate, "same-code")
        geometry.check.assert_not_called()
    def test_missing_changed_model_or_changed_code_rejects_cache(self):
        for certificate, code, files in (
                (None, "same-code", {"model": "same-hash"}),
                (self.certificate, "new-code", {"model": "same-hash"}),
                (self.certificate, "same-code", {"model": "new-hash"})):
            with self.assertRaises(RuntimeError):
                verify_binding(certificate, code, files, self.stages)

    def test_target_order_or_head_change_is_not_silently_accepted(self):
        for kind in ("arm", "order", "head"):
            changed = copy.deepcopy(self.stages)
            if kind == "arm":
                changed[3]["target"]["arm0"][2] += .001
            elif kind == "head":
                changed[3]["target"]["head"][0] += .01
            else:
                changed[2], changed[3] = changed[3], changed[2]
            with self.assertRaises(RuntimeError):
                verify_binding(self.certificate, "same-code", {"model": "same-hash"}, changed)

    def test_this_interruption_recovers_arms_before_body_then_runs_normal_flow(self):
        start = copy.deepcopy(self.fixture.start)
        start["arm0"][2] = -91.218
        start["arm1"][2] = 91.217
        start["arm1"][5] = .041
        stages = build_stages(start, self.fixture.home, self.fixture.squat, self.fixture.waist)
        self.assertEqual(stages[0]["name"], "align_arms_home")
        self.assertEqual(stages[0]["moving"], ["arm0", "arm1"])
        self.assertEqual(stages[0]["target"]["body"], start["body"])
        self.assertLess(delta(stages[0]["target"], self.fixture.home, ("arm0", "arm1")), 1e-9)
        self.assertEqual(stages[1]["name"], "align_standing_home")
        self.assertEqual(stages[2]["name"], "open_before_crouch")
        verify_binding(self.certificate, "same-code", {"model": "same-hash"}, stages)

    def test_recovery_does_not_accept_arbitrary_arm_or_body_positions(self):
        for key, joint, value in (("arm0", 2, -93), ("arm0", 2, -89),
                                  ("arm1", 5, .1), ("body", 1, 20)):
            start = copy.deepcopy(self.fixture.start)
            start[key][joint] = value
            with self.assertRaises(ValueError):
                build_stages(start, self.fixture.home, self.fixture.squat, self.fixture.waist)

    def test_observed_interrupted_crouch_stands_before_closing_arms(self):
        start = copy.deepcopy(self.fixture.home)
        start["body"] = [-.744, 10.122, -18.868, 6.132, -.296, -1.653]
        start["arm0"][2] = -107.999
        start["arm1"][2] = 107.998
        start["arm1"][5] = .041
        stages = build_stages(start, self.fixture.home, self.fixture.squat, self.fixture.waist)
        self.assertEqual(stages[0]["name"], "align_body_from_crouch")
        self.assertEqual(stages[0]["target"]["arm0"], start["arm0"])
        self.assertEqual(stages[0]["target"]["body"], self.fixture.home["body"])
        self.assertEqual(stages[1]["name"], "align_arms_home")
        verify_binding(self.certificate, "same-code", {"model": "same-hash"}, stages)
        start["body"][3] += .1
        with self.assertRaisesRegex(ValueError, "coordinated"):
            build_stages(start, self.fixture.home, self.fixture.squat, self.fixture.waist)

    def test_interrupted_turn_unwinds_waist_before_arms(self):
        start = copy.deepcopy(self.fixture.home)
        start["body"][5] = -32.173
        start["arm0"][2:4] = [-78.003, -80.]
        start["arm1"][2:4] = [78., -80.]
        stages = build_stages(start, self.fixture.home, self.fixture.squat, self.fixture.waist)
        self.assertEqual(stages[0]["name"], "align_waist_from_turn")
        self.assertEqual(stages[0]["target"]["arm0"], start["arm0"])
        self.assertEqual(stages[1]["name"], "align_arms_home")
        verify_binding(self.certificate, "same-code", {"model": "same-hash"}, stages)
        start["body"][1] += .1
        with self.assertRaises(ValueError):
            build_stages(start, self.fixture.home, self.fixture.squat, self.fixture.waist)


if __name__ == "__main__":
    unittest.main()
