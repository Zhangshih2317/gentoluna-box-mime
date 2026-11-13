"""Public deployment tests, with no credentials, robot or model required."""

import importlib.util
import json
from pathlib import Path
import shlex
import tempfile
import unittest

from luna_config import ROOT, load_config, remote_environment, source_environment


class Tests(unittest.TestCase):
    def configuration(self):
        config = json.loads((ROOT/"config/robot.example.json").read_text("utf-8"))
        config.update(ssh_host="robot.example.test", controller_ip="127.0.0.1")
        return config

    def read(self, config):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"config.json"
            path.write_text(json.dumps(config), "utf-8")
            return load_config(path)

    def test_example_addresses_cannot_be_used_for_hardware(self):
        with self.assertRaisesRegex(ValueError, "example addresses"):
            load_config(ROOT/"config/robot.example.json")

    def test_invalid_config_is_rejected(self):
        for key, value in (("controller_ip", "bad"), ("ssh_port", True),
                           ("ssh_port", 0), ("model_path", "relative"),
                           ("ssh_host", "a\nb"), ("ssh_user", "")):
            config = self.configuration()
            config[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.read(config)

    def test_shell_environment_path_is_quoted(self):
        config = self.configuration()
        config["remote_env_script"] = "/robot/my env$(do-not-run).sh"
        self.assertEqual(shlex.split(source_environment(self.read(config))),
                         ["source", config["remote_env_script"]])
        self.assertEqual(set(remote_environment(config)),
                         {"LUNA_CONTROLLER_IP", "LUNA_SDK_ROOT", "LUNA_MODEL_PATH"})

    def test_remote_modules_receive_paths_and_address_before_import(self):
        config = self.configuration()
        for filename in ("run_box_mime_taught.py", "run_box_mime_six_cycles.py"):
            text = (ROOT/filename).read_text("utf-8")
            self.assertLess(text.index('source += f"os.environ.update('),
                            text.index("for name, text in texts.items():"))
            self.assertIn("runtime_config=config", text)
            compile(text, filename, "exec")


if __name__ == "__main__":
    unittest.main()
