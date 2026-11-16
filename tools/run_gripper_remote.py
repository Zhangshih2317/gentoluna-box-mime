"""Deploy/run the configured gripper controller through verified SSH."""
import argparse
from datetime import datetime
import importlib.util
import json
import shlex
import sys
import uuid

from gripper_config import ROOT, load_grippers
from luna_config import load_config

INVENTORY_SOURCE = """
import glob,json,subprocess
from pathlib import Path
for path in sorted(glob.glob('/dev/ttyUSB*')):
    r=subprocess.run(['udevadm','info','-q','property','-n',path],check=True,capture_output=True,text=True)
    props=dict(line.split('=',1) for line in r.stdout.splitlines() if '=' in line)
    aliases=[p for pattern in ('/dev/serial/by-id/*','/dev/serial/by-path/*') for p in glob.glob(pattern) if Path(p).resolve()==Path(path).resolve()]
    print(json.dumps({'port':path,'aliases':aliases,'identity':{k:props[k] for k in ('ID_VENDOR_ID','ID_MODEL_ID','ID_SERIAL_SHORT','ID_PATH') if k in props}}),flush=True)
"""


def connect(config):
    spec = importlib.util.spec_from_file_location("public_ssh", ROOT/"config/ssh_connector.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.connect(config)


def mkdirs(sftp, path):
    from pathlib import PurePosixPath
    current = ""
    for part in PurePosixPath(path).parts[1:]:
        current += "/"+part
        try:
            sftp.stat(current)
        except FileNotFoundError:
            sftp.mkdir(current)


def command_for(mode, remote, deployment, run_id, speed, force, ignore):
    q = shlex.quote
    python = remote+"/.venv/bin/python"
    if mode == "install":
        return ("python3 -m venv "+q(remote+"/.venv")+" && "+q(python)+
                " -m pip install --disable-pip-version-check -r "+q(deployment+"/requirements-gripper.txt"))
    command = ("LUNA_GRIPPER_CONFIG="+q(deployment+"/grippers.json")+
               " LUNA_GRIPPER_RUN_ID="+q(run_id)+
               " timeout --signal=TERM --kill-after=5s 150s "+q(python)+" -u "+
               q(deployment+"/check_gripper_sdk.py")+" --"+mode+
               " --speed "+str(speed)+" --force "+str(force))
    return command+(" --ignore-drop-alarm" if ignore else "")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("inventory", "install", "inspect", "open", "close"))
    parser.add_argument("--speed", type=int, choices=range(1, 51), default=10)
    parser.add_argument("--force", type=int, choices=range(1, 31), default=30)
    parser.add_argument("--ignore-drop-alarm", action="store_true")
    args = parser.parse_args(argv)
    ssh = load_config(motion=False)
    motion = args.mode in ("open", "close")
    config = (None if args.mode == "inventory" else
              load_grippers(require_mapping=motion))
    run_id, pid = uuid.uuid4().hex, None
    evidence = ROOT/"evidence"
    evidence.mkdir(exist_ok=True)
    path = evidence/("gripper-"+args.mode+"-"+datetime.now().strftime("%Y%m%d-%H%M%S")+
                    "-"+run_id[:8]+".jsonl")
    print("Evidence: "+str(path), flush=True)
    with connect(ssh) as client, path.open("w", encoding="utf-8") as log:
        if config:
            remote = config["remote_directory"]
            deployment = remote+"/runs/"+run_id
            with client.open_sftp() as sftp:
                mkdirs(sftp, remote)
                if args.mode != "install":
                    try:
                        sftp.stat(remote+"/.venv/bin/python")
                    except FileNotFoundError as error:
                        raise ValueError("First run: python tools/run_gripper_remote.py install") from error
                # Separate deployments prevent overwriting another process's loaded sources.
                mkdirs(sftp, deployment)
                sftp.put(config["sdk_file"], deployment+"/vendor_gripper_sdk.py")
                sftp.put(str(ROOT/"tools/check_gripper_sdk.py"), deployment+"/check_gripper_sdk.py")
                sftp.put(str(ROOT/"requirements-gripper.txt"), deployment+"/requirements-gripper.txt")
                with sftp.open(deployment+"/grippers.json", "w") as stream:
                    stream.write(json.dumps({"devices": config["devices"]}))
            command = command_for(args.mode, remote, deployment, run_id, args.speed,
                                  args.force, args.ignore_drop_alarm)
        else:
            command = "python3 -u -c "+shlex.quote(INVENTORY_SOURCE)
        stdin, stdout, stderr = client.exec_command(command+" 2>&1", timeout=180)
        stdin.channel.shutdown_write()
        try:
            for line in stdout:
                log.write(line)
                log.flush()
                print(line, end="", flush=True)
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if event.get("event") == "remote_process" and event.get("run_id") == run_id:
                    pid = int(event["pid"])
            code = stdout.channel.recv_exit_status()
        except BaseException:
            if motion and pid is not None:
                source = (
                    "import os,signal\n"
                    f"pid={pid}\n"
                    f"token={('LUNA_GRIPPER_RUN_ID='+run_id).encode()!r}\n"
                    "with open('/proc/%d/environ'%pid,'rb') as f: values=f.read().split(b'\\0')\n"
                    "if token not in values: raise RuntimeError('Run identity mismatch')\n"
                    "os.kill(pid,signal.SIGTERM)\n")
                try:
                    with connect(ssh) as stop_client:
                        si, so, se = stop_client.exec_command("python3 -", timeout=15)
                        si.write(source)
                        si.channel.shutdown_write()
                        if so.channel.recv_exit_status():
                            raise RuntimeError("Remote signal rejected")
                    stdout.channel.settimeout(10)
                    for line in stdout:
                        log.write(line)
                        log.flush()
                        print(line, end="", flush=True)
                except Exception:
                    print("Stop not confirmed. Use the on-site emergency stop.", flush=True)
            raise
        log.write(json.dumps({"remote_exit_code": code})+"\n")
    return code


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    raise SystemExit(main())
