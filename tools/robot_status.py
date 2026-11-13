"""Shared project paths; direct invocation uses the portable read-only sampler."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONNECTOR = ROOT / "config/ssh_connector.py"


if __name__ == "__main__":
    from teach_pose import main
    raise SystemExit(main())
