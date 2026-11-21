"""从本机手动操作 Luna 的 8 针腕部夹爪。"""

import argparse
import codecs
import shlex
import sys
import time
import getpass
import os
from pathlib import Path

import paramiko


def connect():
    host = os.getenv("LUNA_HOST", "6.6.7.100")
    user = os.getenv("LUNA_USER", "nvidia")
    client = paramiko.SSHClient()
    client.load_host_keys(str(Path.home() / ".ssh" / "known_hosts"))
    try:
        client.connect(host, username=user, timeout=6)
    except paramiko.AuthenticationException:
        client.connect(host, username=user, password=getpass.getpass(f"{user}@{host}: "),
                       allow_agent=False, look_for_keys=False, timeout=6)
    except paramiko.SSHException as exc:
        if str(exc) != "No authentication methods available":
            raise
        client.connect(host, username=user, password=getpass.getpass(f"{user}@{host}: "),
                       allow_agent=False, look_for_keys=False, timeout=6)
    return client


REMOTE_PROGRAM = "/home/nvidia/luna-wrist-manual/wrist_manual"
SIDE_NAMES = {"left": "左爪", "right": "右爪", "both": "双爪"}


def percent(text):
    value = int(text)
    if not 60 <= value <= 100:
        raise argparse.ArgumentTypeError("速度和扭矩都必须在 60～100% 之间")
    return value


def collect(channel, deadline=None):
    decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
    while not channel.exit_status_ready() or channel.recv_ready():
        if channel.recv_ready():
            print(decoder.decode(channel.recv(16384)), end="", flush=True)
        elif deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError("未收到远端结束确认，请现场核实夹爪状态")
        else:
            time.sleep(.05)
    print(decoder.decode(b"", final=True), end="", flush=True)
    return channel.recv_exit_status()


def run(action, side, speed, torque):
    command = shlex.join([REMOTE_PROGRAM, action, side, str(speed), str(torque)])
    # Load the same ROS libraries used to build the native executable.
    command = "source /etc/apex/apex_ros_env.sh && exec " + command
    with connect() as client:
        transport = client.get_transport()
        transport.set_keepalive(3)
        with transport.open_session(timeout=6) as channel:
            channel.set_combine_stderr(True)
            channel.exec_command(command)
            try:
                return collect(channel)
            except KeyboardInterrupt:
                print("\n已请求中断，等待读回停止状态。", flush=True)
                channel.shutdown_write()
                collect(channel, deadline=time.monotonic()+12)
                return 130


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("status", "open", "close"), nargs="?",
                        help="status 读取状态，open 张开，close 闭合；省略时进入菜单")
    parser.add_argument("--side", choices=SIDE_NAMES, default="both")
    parser.add_argument("--speed", type=percent, default=60, help="速度百分比，默认 60")
    parser.add_argument("--torque", type=percent, default=60, help="扭矩百分比，默认 60")
    args = parser.parse_args()
    print(f"{SIDE_NAMES[args.side]}  速度 {args.speed}%  扭矩 {args.torque}%", flush=True)
    while True:
        action = args.action
        if action is None:
            if not sys.stdin.isatty():
                return run("status", args.side, args.speed, args.torque)
            choice = input("1 张开  2 闭合  3 读状态  0 退出\n请选择 ").strip()
            if choice == "0":
                return 0
            action = {"1": "open", "2": "close", "3": "status"}.get(choice)
            if action is None:
                continue
        result = run(action, args.side, args.speed, args.torque)
        if result or args.action:
            return result


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (EOFError, KeyboardInterrupt):
        raise SystemExit(130)
    except Exception as exc:
        print(f"夹爪指令未确认完成：{exc}", file=sys.stderr)
        raise SystemExit(1)
