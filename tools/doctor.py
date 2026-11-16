"""Check downloaded files and local configuration without contacting hardware."""
import argparse
import importlib.util
from pathlib import Path
import sys

from gripper_config import ROOT, load_grippers
from luna_config import load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--component", choices=("grippers", "motion"), default="grippers")
    args = parser.parse_args()
    failures = []
    if sys.version_info < (3, 10):
        failures.append("Python 3.10+ required")
    for package in ["paramiko"] + (["numpy"] if args.component == "motion" else []):
        if importlib.util.find_spec(package) is None:
            failures.append("Missing "+package+": python -m pip install -r requirements.txt")
    try:
        config = load_config(motion=args.component == "motion")
        if not (Path.home()/".ssh/known_hosts").is_file():
            failures.append("Missing ~/.ssh/known_hosts: verify your robot host via ssh first")
        key = config.get("ssh_key_file")
        if key and not Path(key).expanduser().is_file():
            failures.append("Configured SSH key file does not exist (or use null for agent)")
    except (ValueError, OSError) as error:
        failures.append(str(error))
    if args.component == "grippers":
        try:
            load_grippers(require_mapping=True)
        except (ValueError, OSError) as error:
            failures.append(str(error))
    elif not (ROOT/"reference/robot_frames/calibration_transforms").is_dir():
        failures.append("Missing your calibration package: see docs/SETUP.md")
    for failure in failures:
        print("MISSING: "+failure)
    print("No network connection or motion attempted." if failures else
          "Local checks passed. Remote dependencies, access and device readiness are not checked.")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
