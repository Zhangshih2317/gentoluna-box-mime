import argparse
import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import cv2
import numpy as np
import pyrealsense2 as rs


class Camera:
    def __init__(self, serial):
        self.serial = serial
        self.condition = threading.Condition()
        self.enabled = threading.Event()
        self.quit = threading.Event()
        self.state = "stopped"
        self.error = ""
        self.sequence = 0
        self.updated = 0
        self.fps = 0
        self.images = {}
        self.scale = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def status(self):
        with self.condition:
            age = time.monotonic() - self.updated if self.updated else None
            return dict(state=self.state, error=self.error, serial=self.serial,
                        sequence=self.sequence, frame_age_seconds=age,
                        fps=round(self.fps, 1), width=640, height=480,
                        depth_scale_m=self.scale, depth_aligned_to_color=True)

    def run(self):
        while not self.quit.is_set():
            if not self.enabled.wait(.2):
                continue
            pipeline = rs.pipeline()
            started = False
            try:
                with self.condition:
                    self.state, self.error = "starting", ""
                    self.images = {}
                config = rs.config()
                config.enable_device(self.serial)
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                profile = pipeline.start(config)
                started = True
                self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
                align = rs.align(rs.stream.color)
                begin, count = time.monotonic(), 0
                while self.enabled.is_set() and not self.quit.is_set():
                    frames = align.process(pipeline.wait_for_frames(3000))
                    color, depth = frames.get_color_frame(), frames.get_depth_frame()
                    if not color or not depth:
                        continue
                    rgb = np.asanyarray(color.get_data())
                    raw = np.asanyarray(depth.get_data())
                    # Fixed 0-4 metre display range; raw PNG retains depth units.
                    display = cv2.applyColorMap(
                        np.uint8(np.clip(raw * self.scale / 4, 0, 1) * 255),
                        cv2.COLORMAP_TURBO)
                    display[raw == 0] = 0
                    images = {}
                    for key, image, ext in [
                        ("color", rgb, ".jpg"), ("depth", display, ".jpg"),
                        ("depth-raw", raw, ".png")
                    ]:
                        ok, encoded = cv2.imencode(
                            ext, image, [cv2.IMWRITE_JPEG_QUALITY, 80] if ext == ".jpg" else [])
                        if not ok:
                            raise RuntimeError("Image encoding failed")
                        images[key] = encoded.tobytes()
                    count += 1
                    now = time.monotonic()
                    with self.condition:
                        self.images = images
                        self.sequence += 1
                        self.updated = now
                        self.fps = count / max(now - begin, .001)
                        self.state = "running"
                        self.condition.notify_all()
            except Exception as exc:
                with self.condition:
                    self.error = str(exc)
                    self.state = "error"
                self.enabled.clear()
            finally:
                if started:
                    try:
                        pipeline.stop()
                    except Exception as exc:
                        with self.condition:
                            self.error = str(exc)
                with self.condition:
                    if not self.error:
                        self.state = "stopped"
                    self.images = {}
                    self.condition.notify_all()


class Handler(BaseHTTPRequestHandler):
    def send(self, body, content_type="application/json", status=200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/":
            return self.send(Path(__file__).with_name("index.html").read_bytes(),
                             "text/html; charset=utf-8")
        if path == "/api/status":
            return self.send(json.dumps(camera.status()).encode())
        images = {"/color.jpg": "color", "/depth.jpg": "depth", "/depth.png": "depth-raw"}
        if path in images:
            with camera.condition:
                data = camera.images.get(images[path])
                fresh = time.monotonic() - camera.updated < 3
            if not data or not fresh:
                return self.send(b'{"error":"No fresh frame"}', status=503)
            return self.send(data, "image/png" if path.endswith(".png") else "image/jpeg")
        if path in ("/color.mjpg", "/depth.mjpg"):
            key = path.split(".")[0][1:]
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            self.connection.settimeout(5)
            try:
                while not camera.quit.is_set():
                    with camera.condition:
                        camera.condition.wait_for(
                            lambda: camera.sequence != last or camera.quit.is_set(), timeout=1)
                        data = camera.images.get(key)
                        fresh = time.monotonic() - camera.updated < 3
                        sequence = camera.sequence
                    if not data or not fresh or sequence == last:
                        time.sleep(.05)
                        continue
                    last = sequence
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
            except (OSError, TimeoutError):
                pass
            return
        self.send(b'{"error":"Not found"}', status=404)

    def do_POST(self):
        origin = self.headers.get("Origin")
        if origin and urlsplit(origin).netloc != self.headers.get("Host"):
            return self.send(b'{"error":"Origin rejected"}', status=403)
        if self.headers.get("X-Camera-Control") != "1":
            return self.send(b'{"error":"Control header required"}', status=403)
        if self.path == "/api/start":
            camera.enabled.set()
        elif self.path == "/api/stop":
            camera.enabled.clear()
        else:
            return self.send(b'{"error":"Not found"}', status=404)
        self.send(json.dumps(camera.status()).encode())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", required=True)
    parser.add_argument("--port", type=int, default=8761)
    args = parser.parse_args()
    camera = Camera(args.serial)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    def stop(signum, frame):
        camera.quit.set()
        camera.enabled.clear()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    camera.enabled.set()
    try:
        server.serve_forever()
    finally:
        camera.quit.set()
        camera.enabled.clear()
        camera.thread.join(timeout=8)
        server.server_close()
