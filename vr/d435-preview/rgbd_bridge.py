"""Single-owner RealSense RGB-D acquisition for Apex's two RTP inputs."""
import argparse
import json
import os
import select
import signal
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

stopping = False


def stop(signum, frame):
    global stopping
    stopping = True


class RTPEncoder:
    def __init__(self, port, width=1920, height=1080):
        self.process = subprocess.Popen([
            "/usr/bin/ffmpeg", "-hide_banner", "-loglevel", "warning",
            "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{width}x{height}",
            "-r", "30", "-i", "pipe:0", "-an", "-c:v", "libx264",
            "-preset", "ultrafast", "-tune", "zerolatency", "-threads", "4",
            "-pix_fmt", "yuv420p", "-b:v", "10M", "-g", "15",
            "-x264-params", "repeat-headers=1:scenecut=0",
            "-f", "rtp", "-payload_type", "96",
            f"rtp://127.0.0.1:{port}?pkt_size=1200",
        ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, bufsize=0)
        os.set_blocking(self.process.stdin.fileno(), False)

    def write(self, image):
        data = memoryview(image.tobytes())
        fd = self.process.stdin.fileno()
        deadline = time.monotonic() + 3
        while data and not stopping:
            if self.process.poll() is not None:
                raise RuntimeError(f"Encoder exited: {self.process.returncode}")
            if time.monotonic() > deadline:
                raise TimeoutError("Encoder did not accept a frame within 3 seconds")
            if select.select([], [fd], [], .1)[1]:
                try:
                    data = data[os.write(fd, data):]
                except BlockingIOError:
                    pass

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", required=True)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    encoders = []
    started = False
    status_path = Path(__file__).with_name("rgbd-status.json")

    def report(data):
        temporary = status_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(data), encoding="utf-8")
        temporary.replace(status_path)

    try:
        report({"state": "starting", "updated_unix": time.time()})
        profile = pipeline.start(config)
        started = True
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        active_profiles = {}
        for stream in (rs.stream.color, rs.stream.depth):
            actual = profile.get_stream(stream).as_video_stream_profile()
            active_profiles[str(stream)] = {
                "width": actual.width(), "height": actual.height(), "fps": actual.fps(),
            }
        align = rs.align(rs.stream.color)
        for port in (5600, 5602):
            encoders.append(RTPEncoder(port))
        count, begin = 0, time.monotonic()
        window_start = begin
        while not stopping:
            frames = align.process(pipeline.wait_for_frames(3000))
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if not color or not depth:
                continue
            rgb = np.asanyarray(color.get_data()).copy()
            raw = np.asanyarray(depth.get_data())
            display = cv2.applyColorMap(
                np.uint8(np.clip(raw * scale / 4, 0, 1) * 255), cv2.COLORMAP_TURBO)
            display[raw == 0] = 0
            count += 1
            for image, title, encoder in zip(
                    (rgb, display), ("RGB", "DEPTH 0-4m"), encoders):
                cv2.rectangle(image, (0, 0), (image.shape[1], 40), (20, 20, 20), -1)
                cv2.putText(image, f"D435 {title} | {count}", (12, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2,
                            cv2.LINE_AA)
                encoder.write(image)
            if count % 30 == 0:
                now = time.monotonic()
                report({
                    "state": "running", "updated_unix": time.time(), "frames": count,
                    "fps": count / max(now - begin, .001),
                    "recent_fps": 30 / max(now - window_start, .001),
                    "active_profiles": active_profiles,
                    "aligned_depth_width": raw.shape[1],
                    "aligned_depth_height": raw.shape[0],
                    "serial": args.serial, "depth_scale_m": scale,
                    "depth_valid_fraction": float(np.count_nonzero(raw) / raw.size),
                    "aligned": True, "ports": [5600, 5602],
                })
                window_start = now
    except Exception as exc:
        report({"state": "error", "updated_unix": time.time(), "error": str(exc)})
        raise
    finally:
        for encoder in encoders:
            encoder.close()
        if started:
            pipeline.stop()
        if stopping:
            report({"state": "stopped", "updated_unix": time.time()})


if __name__ == "__main__":
    main()
