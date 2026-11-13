"""SSH connection using local configuration; no credentials are stored here."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"tools"))
from luna_config import load_config


def connect():
    import paramiko

    config = load_config()
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    key = config.get("ssh_key_file")
    try:
        client.connect(
            config["ssh_host"], port=config.get("ssh_port", 22),
            username=config["ssh_user"],
            key_filename=str(Path(key).expanduser()) if key else None,
            look_for_keys=True, allow_agent=True, timeout=15,
            auth_timeout=15, banner_timeout=15,
        )
    except Exception:
        client.close()
        raise
    return client
