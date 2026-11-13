"""Physical-depth planning and remote binding, without robot connections."""

import copy
import math
import unittest
from unittest.mock import patch

import numpy as np

from box_mime_cache import verify_binding, main_stages
from box_motion_plan import collision_samples, validate_pose
from mime_squat_height import solve_squat_depth
from stand_up_plan import BodyHeadKinematics, body_transform
import test_mime_speed_setting as entry_fixtures


class Tests(unittest.TestCase):
    def setUp(self):
        self.fixture = entry_fixtures.Tests()
        self.fixture.setUp()
        self.home = self.fixture.fixture.home
        self.taught = self.fixture.fixture.squat
        self.kin = BodyHeadKinematics()

    def test_requested_centimetres_are_physical_drop_not_joint_degrees(self):
        origin = body_transform(self.kin, self.home["body"])
        for cm in (3., 15., 30.):
            with self.subTest(cm=cm):
                solved = solve_squat_depth(self.home, self.taught, cm)
                actual = body_transform(self.kin, solved["target_body"])
                self.assertAlmostEqual(origin[2, 3]-actual[2, 3], cm*10., places=6)
                np.testing.assert_allclose(actual[:3, :3], origin[:3, :3], atol=1e-10)
                self.assertEqual([solved["target_body"][i] for i in (0, 4, 5)],
                                 [self.home["body"][i] for i in (0, 4, 5)])

    def test_generated_thirty_cm_path_preserves_orientation_limits_and_reverses(self):
        with self.fixture.remote_modules(30.) as (plan, control, _):
            self.assertEqual(plan.SQUAT_DROP_MM, 300.)
            self.assertEqual(control.SQUAT_DROP_MM, 300.)
            stages = plan.build_stages(self.home, self.home, self.taught, self.fixture.fixture.waist)
            crouch = next(s for s in stages if s["name"] == "crouch")
            stand = next(s for s in stages if s["name"] == "stand")
            origin = body_transform(self.kin, crouch["start"]["body"])
            last_z = origin[2, 3]
            for sample in collision_samples(crouch, .1):
                validate_pose(sample)
                actual = body_transform(self.kin, sample["body"])
                self.assertLessEqual(actual[2, 3], last_z+1e-8)
                np.testing.assert_allclose(actual[:3, :3], origin[:3, :3], atol=1e-10)
                self.assertLessEqual(abs(actual[0, 3]-origin[0, 3]), 30)
                self.assertLessEqual(abs(actual[1, 3]-origin[1, 3]), 5)
                last_z = actual[2, 3]
            self.assertAlmostEqual(origin[2, 3]-last_z, 300., places=6)
            self.assertEqual(stand["start"]["body"], crouch["target"]["body"])
            self.assertEqual(stand["target"]["body"], self.home["body"])
            self.assertEqual(stages[-1]["target"], self.home)

    def test_changed_depth_requires_new_cache_even_with_same_code_hash(self):
        paths = []
        for cm in (3., 30.):
            with patch.object(self.fixture.entry, "SQUAT_DEPTH_CM", cm):
                with self.fixture.remote_modules(30.) as (plan, _, _):
                    paths.append(plan.build_stages(self.home, self.home, self.taught,
                                                  self.fixture.fixture.waist))
        certificate = {"schema": "luna_mime_geometry_cache_v1", "code_id": "same-code",
                       "model_files": {}, "stages": main_stages(paths[0])}
        with self.assertRaisesRegex(RuntimeError, "arm/body targets"):
            verify_binding(certificate, "same-code", {}, paths[1])

    def test_invalid_depth_and_invalid_taught_branch_rejected(self):
        for cm in (0., -1., 30.1, math.nan, math.inf, True):
            with self.subTest(cm=cm), self.assertRaises(ValueError):
                solve_squat_depth(self.home, self.taught, cm)
        changed = copy.deepcopy(self.taught)
        changed["body"][2] = self.home["body"][2]+1
        with self.assertRaisesRegex(ValueError, "coordination"):
            solve_squat_depth(self.home, changed, 30.)
        changed = copy.deepcopy(self.home)
        changed["body"][1] += 5.
        changed["body"][2] -= 10.
        with self.assertRaisesRegex(ValueError, "exceeds"):
            solve_squat_depth(self.home, changed, 30.)


if __name__ == "__main__":
    unittest.main()
