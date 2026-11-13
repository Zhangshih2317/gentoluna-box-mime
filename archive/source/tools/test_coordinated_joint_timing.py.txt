"""Offline mathematical checks; test limits are synthetic, not robot settings."""

import math
import unittest

from coordinated_joint_timing import AxisLimits, plan_timing, sample
from analyze_body_speed_response import inverse_blend
from box_motion_plan import blend


class Tests(unittest.TestCase):
    def test_distinct_axis_limits_produce_one_phase_and_preserve_pitch(self):
        start, target = [9., -17., 5.], [64., -117., 50.]
        limits = [AxisLimits(5., 12., 1., 2.), AxisLimits(9., 20., 2., 4.),
                  AxisLimits(4., 10., .8, 1.6)]
        plan = plan_timing(start, target, limits, 80.)
        for axis, bound in enumerate(limits):
            self.assertLessEqual(abs(plan["signed_peak_velocity_deg_s"][axis]),
                                 min(bound.maximum_velocity_deg_s,
                                     bound.reference_velocity_deg_s*80/30)+1e-9)
            self.assertLessEqual(plan["peak_acceleration_deg_s2"][axis], bound.acceleration_deg_s2+1e-9)
            self.assertLessEqual(plan["peak_jerk_deg_s3"][axis], bound.jerk_deg_s3+1e-9)
        for tick in range(plan["ticks"]+1):
            point = sample(plan, tick/plan["rate_hz"])
            self.assertAlmostEqual(sum(point), sum(start), places=10)
            phases = [(q-a)/(b-a) for q, a, b in zip(point, start, target)]
            self.assertLess(max(phases)-min(phases), 1e-10)
        self.assertEqual(sample(plan, 0.), start)
        self.assertEqual(sample(plan, plan["seconds"]), target)

    def test_acceleration_and_jerk_do_not_scale_up_with_user_speed(self):
        limit = AxisLimits(1., 100., .5, .4)
        low = plan_timing([0.], [10.], [limit], 10.)
        high = plan_timing([0.], [10.], [limit], 100.)
        self.assertLess(high["seconds"], low["seconds"])
        self.assertLessEqual(high["peak_acceleration_deg_s2"][0], .5)
        self.assertLessEqual(high["peak_jerk_deg_s3"][0], .4)

    def test_velocity_saturates_at_calibrated_limit(self):
        bound = AxisLimits(2., 3., 100., 1000.)
        at_limit = plan_timing([0.], [100.], [bound], 45.)
        above = plan_timing([0.], [100.], [bound], 100.)
        self.assertEqual(at_limit["seconds"], above["seconds"])

    def test_missing_calibration_is_not_replaced_by_guessed_limits(self):
        with self.assertRaisesRegex(ValueError, "Missing configured"):
            plan_timing([0.], [1.], [None], 30.)
        unchanged = plan_timing([2.], [2.], [None], 30.)
        self.assertEqual(sample(unchanged, 1.), [2.])

    def test_invalid_values_are_rejected(self):
        for value in (0, -1, math.nan, math.inf, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                AxisLimits(value, 3., 1., 1.)
        bound = AxisLimits(1., 2., 1., 1.)
        for speed in (0, 101, math.nan, True):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                plan_timing([0.], [1.], [bound], speed)
        with self.assertRaises(ValueError):
            plan_timing([0.], [math.nan], [bound], 30.)

    def test_inverse_quintic_recovers_progress(self):
        for fraction in (.01, .1, .25, .5, .75, .9, .99):
            self.assertAlmostEqual(inverse_blend(blend(fraction)), fraction, places=10)


if __name__ == "__main__":
    unittest.main()
