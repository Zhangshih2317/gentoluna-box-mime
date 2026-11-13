"""Portable host configuration, validated before opening any connection."""

import ipaddress
import json
import os
from pathlib import Path, PurePosixPath
import shlex

ROOT = Path(__file__).resolve().parents[1]


def load_config(path=None):
    path = Path(path or os.environ.get("LUNA_CONFIG", ROOT/"config/robot.local.json"))
    try:
        config = json.loads(path.read_text("utf-8-sig"))
    except FileNotFoundError as error:
        raise ValueError("Create config/robot.local.json from config/robot.example.json first") from error
    for key in ("ssh_host", "ssh_user", "controller_ip", "remote_env_script", "sdk_root", "model_path"):
        if not isinstance(config.get(key), str) or not config[key].strip():
            raise ValueError("Missing configuration: "+key)
        if any(c in config[key] for c in ("\x00", "\n", "\r")):
            raise ValueError("Invalid configuration: "+key)
    address = ipaddress.IPv4Address(config["controller_ip"])
    if address in ipaddress.IPv4Network("192.0.2.0/24") or config["ssh_host"].startswith("192.0.2."):
        raise ValueError("Replace documentation-only example addresses with your robot addresses")
    port = config.get("ssh_port", 22)
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("ssh_port must be an integer in 1..65535")
    for key in ("remote_env_script", "sdk_root", "model_path"):
        if not PurePosixPath(config[key]).is_absolute():
            raise ValueError(key+" must be an absolute Linux path")
    return config


def remote_environment(config):
    return {"LUNA_CONTROLLER_IP": config["controller_ip"],
            "LUNA_SDK_ROOT": config["sdk_root"], "LUNA_MODEL_PATH": config["model_path"]}


def source_environment(config):
    return "source "+shlex.quote(config["remote_env_script"])
