"""Read numeric SDK flange parameters, without enabling or moving the robot."""

import importlib.util
import json
from pathlib import Path
import shlex

from luna_config import ROOT, load_config, remote_environment, source_environment
from robot_status import DEFAULT_CONNECTOR

REMOTE = r'''
import fcntl,json,math,os,sys
sys.path.insert(0,os.environ["LUNA_SDK_ROOT"]+"/PYTHON_SDK")
from GentoRobot import GentoRobot
with open("/tmp/gento_sdk_6_6_7_190.lock","a+") as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    robot=GentoRobot()
    try:
        robot.link(*map(int,os.environ["LUNA_CONTROLLER_IP"].split(".")))
        if not robot._connected or robot.check_link_state()!=1:
            raise RuntimeError("SDK connection unavailable")
        parameters={}
        for arm in (0,1):
            values={}
            for part in ("L0.DH","FLANGE"):
                for item in ("Alpha","A","D","Theta"):
                    code,value=robot.param_get_float("R.A%d.%s.%s"%(arm,part,item))
                    if code or not math.isfinite(value):
                        raise RuntimeError("Invalid SDK parameter read")
                    values[part+"."+item]=value
            parameters[str(arm)]=values
        print(json.dumps({"event":"frame_parameters_read","frame_parameters":parameters,
                          "motion_commands_sent":0,"mode_changes_sent":0}),flush=True)
    finally:
        if robot._connected:
            robot.unlink()
'''


def main():
    config = load_config()
    spec = importlib.util.spec_from_file_location("frame_connector", DEFAULT_CONNECTOR)
    connector = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(connector)
    source = "import os\nos.environ.update("+repr(remote_environment(config))+")\n"+REMOTE
    command = "bash -lc "+shlex.quote(source_environment(config)+" && python3 -u -")
    with connector.connect() as client:
        stdin, stdout, stderr = client.exec_command(command, timeout=30)
        stdin.write(source)
        stdin.flush()
        stdin.channel.shutdown_write()
        output, errors = stdout.read().decode("utf-8"), stderr.read().decode("utf-8")
        if stdout.channel.recv_exit_status():
            raise RuntimeError("Read failed: "+errors+"\n"+output)
    events = [json.loads(line) for line in output.splitlines() if line.startswith("{")]
    result = next(e for e in events if e.get("event") == "frame_parameters_read")
    path = ROOT/"points/SDK_FRAME_PARAMETERS.json"
    path.write_text(json.dumps(result["frame_parameters"], indent=2)+"\n", "utf-8")
    print("Read-only parameters saved:", path)


if __name__ == "__main__":
    main()
