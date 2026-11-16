"""Local gripper settings; never infer USB identity or motion direction."""
import json
import os
from pathlib import Path, PurePosixPath
import re

ROOT = Path(__file__).resolve().parents[1]


def load_grippers(path=None, require_sdk=True, require_mapping=False):
    path = Path(path or os.environ.get("LUNA_GRIPPERS_CONFIG",
                                      ROOT/"config/grippers.local.json"))
    try:
        config = json.loads(path.read_text("utf-8-sig"))
    except FileNotFoundError as error:
        raise ValueError("Create config/grippers.local.json from grippers.example.json") from error
    remote = config.get("remote_directory", "")
    if (not isinstance(remote, str) or not remote.startswith("/home/")
            or len(PurePosixPath(remote).parts) < 4 or ".." in PurePosixPath(remote).parts
            or any(c in remote for c in "\r\n\x00")):
        raise ValueError("remote_directory must be a dedicated absolute /home/user/subdirectory")
    devices = config.get("devices", [])
    if len(devices) != 2:
        raise ValueError("Exactly two gripper devices are required")
    ports, identities = [], []
    for device in devices:
        port = device.get("port", "")
        identity = device.get("identity", {})
        if (not isinstance(port, str) or not port.startswith("/dev/")
                or ".." in PurePosixPath(port).parts or any(c in port for c in "\r\n\x00")):
            raise ValueError("Device port must be an absolute /dev/ path")
        if not all(re.fullmatch("[0-9a-f]{4}", identity.get(key, ""))
                   for key in ("ID_VENDOR_ID", "ID_MODEL_ID")):
            raise ValueError("Each USB identity requires lowercase four-digit vendor/product IDs")
        if not (identity.get("ID_SERIAL_SHORT") or identity.get("ID_PATH")):
            raise ValueError("Each device requires its serial number or physical ID_PATH")
        if set(identity) - {"ID_VENDOR_ID", "ID_MODEL_ID", "ID_SERIAL_SHORT", "ID_PATH"}:
            raise ValueError("Unsupported USB identity property")
        if any(not isinstance(value, str) or not value or
               any(c in value for c in "\r\n\x00") or "REPLACE" in value or "EXAMPLE" in value
               for value in identity.values()) or "REPLACE" in port:
            raise ValueError("Replace all example USB identities with your inventory")
        ports.append(port)
        identities.append(identity)
    if len(set(ports)) != 2 or identities[0] == identities[1]:
        raise ValueError("Two distinct ports and USB identities are required")
    if require_mapping and config.get("open_close_mapping_confirmed") is not True:
        raise ValueError("Confirm both devices use 0=open, 9000=closed in grippers.local.json")
    sdk_value = config.get("sdk_file", "")
    if not isinstance(sdk_value, str) or not sdk_value.strip():
        raise ValueError("sdk_file must name your licensed vendor Python SDK")
    sdk = Path(sdk_value).expanduser()
    config["sdk_file"] = str(sdk if sdk.is_absolute() else ROOT/sdk)
    if require_sdk and not Path(config["sdk_file"]).is_file():
        raise ValueError("Vendor SDK not found: set sdk_file to your licensed changingtek_p_rtu_Servo.py")
    return config
