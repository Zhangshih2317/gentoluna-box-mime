"""Copy the robot's existing, read-only frame library for offline evaluation."""

import hashlib
import importlib.util
import json

from robot_status import DEFAULT_CONNECTOR, ROOT
from luna_config import load_config


FILES = (
    "__init__.py", "README.md", "body_head_fk.py", "geometry.py", "store.py",
    "transforms.py", "hook_approach.py", "calibration_data/calibration_matrices.json",
    "calibration_data/gento_luna_body_head.urdf",
)
# Source paths come from the user-owned robot installation.


def main():
    remote_root = load_config()["sdk_root"].rstrip("/") + "/robot/calibration_transforms"
    spec = importlib.util.spec_from_file_location("luna_connector", DEFAULT_CONNECTOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    local_root = ROOT / "reference/robot_frames/calibration_transforms"
    manifest = []
    with module.connect() as client, client.open_sftp() as sftp:
        for name in FILES:
            target = local_root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            sftp.get(remote_root + "/" + name, str(target))
            manifest.append({
                "source": remote_root + "/" + name,
                "local": str(target),
                "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
            })
    (ROOT / "evidence").mkdir(exist_ok=True)
    (ROOT / "evidence/ground-frame-source-manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    print("Copied %d existing reference files; no robot files changed." % len(manifest))


if __name__ == "__main__":
    main()
