"""HOME preflight or --execute for standing or recorded BODY-first recovery."""

import sys

from run_box_mime_taught import main


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    raise SystemExit(main(home_only=True))
