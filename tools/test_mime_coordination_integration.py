"""Generated entry, common-phase limits, and smooth restarts without hardware."""

import copy
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

from box_motion_plan import FIELDS, delta, interpolate
import test_mime_speed_setting as fixtures


class Tests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.Tests()
        self.fixture.setUp()

    def stages(self, plan):
        f = self.fixture.fixture
        return plan.build_stages(f.home, f.home, f.squat, f.waist)

    def test_generated_entry_limits_every_axis_and_preserves_original_path(self):
        for speed in (10., 20., 80., 100.):
            with self.subTest(speed=speed), self.fixture.remote_modules(speed) as (plan, control, _):
                self.assertTrue(control.COORDINATED_TIMING)
                for stage in self.stages(plan):
                    timing = plan.coordinated_timing(stage)
                    self.assertEqual(timing["seconds"], plan.continuous_seconds(stage))
                    for i, name in enumerate(timing["axes"]):
                        group = name.split(":")[0]
                        acceleration = (plan.BODY_STREAM_ACCELERATION if group == "body"
                                        else plan.STREAM_ACCELERATION)
                        speed_cap = plan.BODY_STREAM_SPEED if group == "body" else plan.STREAM_SPEED
                        self.assertLessEqual(abs(timing["signed_peak_velocity_deg_s"][i]), speed_cap+1e-9)
                        self.assertLessEqual(timing["peak_acceleration_deg_s2"][i], acceleration+1e-9)
                        self.assertLessEqual(timing["peak_jerk_deg_s3"][i], acceleration+1e-9)
                    if stage["name"] == "crouch":
                        velocities = timing["signed_peak_velocity_deg_s"][14:20]
                        self.assertAlmostEqual(velocities[1]/-velocities[2], .554545454545, places=8)
                        self.assertAlmostEqual(velocities[3]/-velocities[2], .445454545455, places=8)
                        self.assertAlmostEqual(sum(velocities[1:4]), 0., places=10)

    def test_nonzero_phase_hold_restarts_from_rest_with_limits_and_arrival(self):
        with self.fixture.remote_modules(10.) as (plan, control, _):
            from motion_wait_policy import CandidateStageDriver
            stage = next(s for s in self.stages(plan) if s["name"] == "crouch")
            seconds = plan.continuous_seconds(stage)
            driver = CandidateStageDriver(stage, 0., timing_planner=plan.continuous_seconds)
            adapter, sent = Mock(), []
            velocity = {k: [0.]*len(stage["start"][k]) for k in FIELDS}
            old, now, restarted_at, paused_command = stage["start"], 0., None, None
            actions = []
            for tick in range(1, math.ceil(seconds*3/.02)):
                now = tick*.02
                actual = copy.deepcopy(old)
                if .35*seconds < now < .35*seconds+1.:
                    actual["body"][1] += 1.1
                    actual["body"][3] += 1.1
                adapter.read_checked.return_value = (actual, velocity, tick)
                adapter.send_checked.side_effect = lambda p: sent.append((now, p))
                previous_count = driver.restart_count
                decision = driver.tick(adapter, now, seconds)
                actions.append(decision.action)
                if decision.action == "hold" and paused_command is None:
                    paused_command = copy.deepcopy(driver.command)
                if driver.restart_count > previous_count:
                    restarted_at = len(sent)-1
                    self.assertEqual(driver.command, paused_command)
                old = copy.deepcopy(driver.command)
                self.assertAlmostEqual(sum(old["body"][1:4]), sum(stage["start"]["body"][1:4]), places=9)
                if decision.action == "complete":
                    break
            self.assertEqual(decision.action, "complete")
            self.assertIn("hold", actions)
            self.assertEqual(driver.restart_count, 1)
            self.assertGreater(delta(paused_command, stage["start"]), 1.)
            # Include a held sample before restart. The protective freeze itself
            # remains immediate; smooth bounds apply to the resumed commands.
            positions = np.asarray([p["body"] for _, p in sent[restarted_at-1:]])
            v = np.diff(positions, axis=0)/.02
            a = np.diff(v, axis=0)/.02
            j = np.diff(a, axis=0)/.02
            self.assertLess(np.max(abs(v)), plan.BODY_STREAM_SPEED+1e-7)
            self.assertLess(np.max(abs(a)), plan.BODY_STREAM_ACCELERATION+1e-6)
            self.assertLess(np.max(abs(j)), plan.BODY_STREAM_ACCELERATION+1e-5)
            self.assertLess(np.max(abs(v[0])), 1e-9)
            self.assertLess(delta(driver.command, stage["target"]), 1e-9)

    def test_resume_hysteresis_and_fresh_stationary_frames(self):
        with self.fixture.remote_modules(10.) as (plan, _, _):
            from motion_wait_policy import CandidateStageDriver
            stage = next(s for s in self.stages(plan) if s["name"] == "crouch")
            driver = CandidateStageDriver(stage, 0., timing_planner=plan.continuous_seconds)
            policy = driver.policy
            velocity = {k: [0.]*len(stage["start"][k]) for k in FIELDS}
            actual = interpolate(stage["start"], stage["target"], .5)
            for serial, error in ((1, 2.1), (2, 1.9)):
                disturbed = copy.deepcopy(actual)
                disturbed["body"][1] += error/2
                disturbed["body"][3] += error/2
                result = policy.update(disturbed, stage["target"], velocity, serial, serial*.02, False)
                self.assertEqual(result.action, "hold")
            # Three fresh quiet frames, not three reads of one frame.
            for serial, now in ((3, .06), (3, .08), (4, .10)):
                result = policy.update(actual, stage["target"], velocity, serial, now, False)
                self.assertEqual(result.action, "hold")
            result = policy.update(actual, stage["target"], velocity, 5, .12, False)
            self.assertEqual(result.action, "advance")

    def test_trace_write_occurs_only_when_explicitly_saved(self):
        with self.fixture.remote_modules(10.) as (_, control, _):
            runner = control.MimeRunner(self.fixture.robot(10.), lambda: None)
            self.assertEqual(len(runner.response_samples), 0)
            runner.response_samples.append({"stage": "crouch", "frame_serial": 1})
            runner.response_sample_count = 2
            with tempfile.TemporaryDirectory() as folder:
                real_create = tempfile.NamedTemporaryFile
                with patch.object(control.tempfile, "NamedTemporaryFile",
                                  side_effect=lambda **kw: real_create(dir=folder, **kw)), \
                        patch.object(control, "emit") as events:
                    runner.save_response_trace()
                path = Path(events.call_args.kwargs["path"])
                rows = [json.loads(line) for line in path.read_text("utf-8").splitlines()]
                self.assertEqual(rows[0]["samples_dropped"], 1)
                self.assertEqual(rows[1]["frame_serial"], 1)
                runner.robot.runtime_set_joint_pos_cmd.assert_not_called()
                runner.robot.emergency_stop.assert_not_called()


if __name__ == "__main__":
    unittest.main()
