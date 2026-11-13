"""Offline cache preparation uses measured HEAD without changing saved HOME."""

import ast
import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from box_mime_cache import verify_binding
from box_mime_taught_plan import build_stages


class Tests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location("cache_head_entry",
                                                     self.root/"run_box_mime_taught.py")
        self.entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.entry)

    def test_observed_head_is_bound_without_changing_arm_body_or_home_file(self):
        path = self.root/"points/BOX_START_BEFORE_INCIDENT.json"
        original = path.read_bytes()
        saved = json.loads(original)["pose"]
        observed = [2.107, -16.002, 0.]
        source = self.entry.source_for(False, prepare_cache=True, cache_head=observed)
        call = ast.parse(source).body[-1].value
        home, squat, waist, _, code_id = [ast.literal_eval(arg) for arg in call.args]
        self.assertEqual(call.func.attr, "prepare")
        self.assertEqual(home["head"], observed)
        for key in ("arm0", "arm1", "body"):
            self.assertEqual(home[key], saved[key])
        self.assertEqual(path.read_bytes(), original)
        stages = build_stages(home, home, squat, waist)
        certificate = {"schema": "luna_mime_geometry_cache_v1", "code_id": code_id,
                       "model_files": {}, "stages": stages}
        verify_binding(certificate, code_id, {}, stages)
        changed = build_stages(saved, saved, squat, waist)
        with self.assertRaisesRegex(RuntimeError, "Held head"):
            verify_binding(certificate, code_id, {}, changed)

    def test_override_rejects_execution_and_other_modes(self):
        for options in ({}, {"execute": True}, {"home_only": True, "prepare_cache": True},
                        {"execute": True, "prepare_cache": True}):
            values = {"execute": False, **options, "cache_head": [2.107, -16.002, 0.]}
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.entry.source_for(**values)

    def test_invalid_angles_are_rejected(self):
        for values in ([2., -16., 0.], [2.107, -16.002], [float("nan"), 0., 0.],
                       [2.107, -16.002, 0., 0.]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.entry.source_for(False, prepare_cache=True, cache_head=values)

    def test_cli_invalid_mode_never_imports_connector(self):
        with patch("sys.argv", ["entry", "--execute", "--cache-head", "2.107", "-16.002", "0"]), \
                patch.object(self.entry.importlib.util, "spec_from_file_location") as load, \
                patch("sys.stderr"):
            with self.assertRaises(SystemExit) as error:
                self.entry.main()
        self.assertEqual(error.exception.code, 2)
        load.assert_not_called()


if __name__ == "__main__":
    unittest.main()
