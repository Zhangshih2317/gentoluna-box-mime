"""Portable gripper configuration and command construction, without hardware."""
import json
from pathlib import Path
import shlex
import tempfile
import unittest

from gripper_config import ROOT, load_grippers
from luna_config import load_config
from run_gripper_remote import command_for


class Tests(unittest.TestCase):
    def configuration(self):
        return {
            "remote_directory": "/home/operator/grippers",
            "sdk_file": "vendor/sdk.py", "open_close_mapping_confirmed": True,
            "devices": [
                {"port": "/dev/serial/by-id/device-one", "identity": {
                    "ID_VENDOR_ID": "0403", "ID_MODEL_ID": "6001", "ID_SERIAL_SHORT": "one"}},
                {"port": "/dev/serial/by-path/device-two", "identity": {
                    "ID_VENDOR_ID": "1a86", "ID_MODEL_ID": "7523", "ID_PATH": "two"}},
            ]}

    def read(self, config, **kwargs):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"config.json"
            path.write_text(json.dumps(config), "utf-8")
            return load_grippers(path, require_sdk=False, **kwargs)

    def test_template_requires_real_identities(self):
        with self.assertRaises(ValueError):
            load_grippers(ROOT/"config/grippers.example.json", require_sdk=False)

    def test_identity_and_mapping_checked(self):
        self.read(self.configuration(), require_mapping=True)
        for kind in ("duplicate", "missing_identity", "mapping", "unsafe_path"):
            config = self.configuration()
            if kind == "duplicate":
                config["devices"][1] = config["devices"][0]
            elif kind == "missing_identity":
                del config["devices"][0]["identity"]["ID_SERIAL_SHORT"]
            elif kind == "mapping":
                config["open_close_mapping_confirmed"] = False
            else:
                config["remote_directory"] = "/home/operator/../"
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.read(config, require_mapping=True)

    def test_grippers_do_not_require_arm_sdk(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"ssh.json"
            config = {"ssh_host": "robot.example.test", "ssh_user": "operator"}
            path.write_text(json.dumps(config), "utf-8")
            self.assertEqual(load_config(path, motion=False), config)
            with self.assertRaises(ValueError):
                load_config(path)

    def test_remote_paths_quoted_and_no_reversal(self):
        remote = "/home/operator/gripper test$(literal)"
        command = command_for("open", remote, remote+"/runs/abc", "abc", 10, 30, False)
        words = shlex.split(command)
        self.assertIn(remote+"/.venv/bin/python", words)
        self.assertIn(remote+"/runs/abc/check_gripper_sdk.py", words)
        self.assertIn("--open", words)
        self.assertNotIn("--execute", words)
        self.assertNotIn("--ignore-drop-alarm", words)
        self.assertIn("--ignore-drop-alarm",
                      shlex.split(command_for("close", remote, remote+"/runs/abc",
                                              "abc", 10, 30, True)))


if __name__ == "__main__":
    unittest.main()
