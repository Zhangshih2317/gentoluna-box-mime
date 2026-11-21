"""Keep the CTAG tool selected for the lifetime of apex-tool.service."""
import fcntl
import os
import signal
import threading


def main():
    stopped = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stopped.set())
    path = os.path.join(os.environ["RUNTIME_DIRECTORY"], "enabled")
    with open(path, "a+b") as lease:
        fcntl.lockf(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        print("CTAG2F90D wrist tool enabled for ARM0/485A and ARM1/485A", flush=True)
        stopped.wait()


if __name__ == "__main__":
    main()
