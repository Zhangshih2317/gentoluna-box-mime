"""Delayed SDK read regression: feedback is not timestamped at SG return."""

import unittest

from feedback_observation import FeedbackObservation


class Tests(unittest.TestCase):
    def test_slow_previous_read_does_not_create_a_false_jump(self):
        guard = FeedbackObservation()
        limits = {"body": [4.2]}
        self.assertIsNone(guard.check({"body": [.068]}, 10, .02, .11, limits))
        self.assertIsNone(guard.check({"body": [.408]}, 20, .12, .122, limits))

    def test_real_excessive_jump_is_reported_with_both_read_windows(self):
        guard = FeedbackObservation()
        guard.check({"body": [.068]}, 10, .02, .11, {"body": [4.2]})
        fault = guard.check({"body": [3.]}, 20, .12, .122, {"body": [4.2]})
        self.assertEqual(fault["previous_frame"], 10)
        self.assertEqual(fault["frame_serial"], 20)
        self.assertAlmostEqual(fault["observation_window_seconds"], .102)
        self.assertGreater(fault["delta_deg"], fault["allowed_delta_deg"])
        self.assertEqual(guard.serial, 10)

    def test_duplicate_frame_does_not_retimestamp_old_position(self):
        guard = FeedbackObservation()
        limits = {"arm0": [10.]}
        guard.check({"arm0": [0.]}, 1, 0., .001, limits)
        guard.check({"arm0": [0.]}, 1, .09, .091, limits)
        self.assertEqual(guard.started, 0.)
        self.assertIsNone(guard.check({"arm0": [1.]}, 2, .1, .101, limits))

    def test_same_frame_cannot_contain_a_position_jump(self):
        guard = FeedbackObservation()
        guard.check({"arm0": [0.]}, 1, 0., .001, {"arm0": [180.]})
        fault = guard.check({"arm0": [1.]}, 1, .09, .091, {"arm0": [180.]})
        self.assertTrue(fault["same_frame"])

    def test_invalid_observation_times_reject(self):
        guard = FeedbackObservation()
        for start, end in ((1., .9), (float("nan"), 1.)):
            with self.assertRaises(RuntimeError):
                guard.check({"body": [0.]}, 1, start, end, {"body": [4.2]})


if __name__ == "__main__":
    unittest.main()
