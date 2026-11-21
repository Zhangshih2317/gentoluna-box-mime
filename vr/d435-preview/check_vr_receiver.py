"""Check the PICO video room from a PC using Apex's installed fullscreen viewer."""

import argparse
import base64
import json
from math import isfinite
from statistics import median
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import IPv4Address
from pathlib import Path
from threading import Thread
from time import monotonic

from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("viewer", type=Path, help="Apex dist/fullscreen.html")
    parser.add_argument("host", type=IPv4Address)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--room", choices=["10", "11"], default="10")
    parser.add_argument("--seconds", type=int, default=0,
                        help="Measure decoded frames for this many seconds after warmup")
    args = parser.parse_args()
    if args.seconds < 0:
        parser.error("--seconds must be nonnegative")
    viewer = args.viewer.resolve(strict=True)
    handler = partial(SimpleHTTPRequestHandler, directory=str(viewer.parent))
    with ThreadingHTTPServer(("127.0.0.1", 0), handler) as server, sync_playwright() as pw:
        Thread(target=server.serve_forever, daemon=True).start()
        origin = f"http://127.0.0.1:{server.server_port}"
        try:
            with pw.chromium.launch(executable_path=pw.chromium.executable_path) as browser:
                ctx = browser.new_context()
                ctx.grant_permissions(["local-network-access"], origin=origin)
                page = ctx.new_page()
                page.goto(f"{origin}/{viewer.name}?host={args.host}&stream={args.room}")
                page.evaluate("""() => {
                    const observe = (_, frame) => {
                        window.videoTiming = {
                            captureToPresentation: frame.expectedDisplayTime - frame.captureTime,
                            receiveToPresentation: frame.expectedDisplayTime - frame.receiveTime
                        };
                        videoEl.requestVideoFrameCallback(observe);
                    };
                    videoEl.requestVideoFrameCallback(observe);
                }""")
                deadline = monotonic() + 22
                first = None
                presentation = []
                while monotonic() < deadline:
                    page.wait_for_timeout(1000)
                    report = page.evaluate("""async () => {
                        const stats = pc ? [...(await pc.getStats()).values()] : [];
                        const rx = stats.find(s => s.type === 'inbound-rtp' && s.kind === 'video');
                        return {status: statusText.textContent, ice: pc?.iceConnectionState,
                            width: videoEl.videoWidth, height: videoEl.videoHeight,
                            decoded: rx?.framesDecoded || 0, keyFrames: rx?.keyFramesDecoded,
                            received: rx?.framesReceived, lost: rx?.packetsLost,
                            bytes: rx?.bytesReceived, fps: rx?.framesPerSecond,
                            timestamp: rx?.timestamp,
                            jitterBufferDelay: rx?.jitterBufferDelay,
                            emitted: rx?.jitterBufferEmittedCount,
                            decodeTime: rx?.totalDecodeTime,
                            ...window.videoTiming,
                            freezes: rx?.freezeCount, dropped: rx?.framesDropped};
                    }""")
                    print(report, flush=True)
                    if report["decoded"] >= 20 and report["width"]:
                        if first is None:
                            first = report
                            deadline = monotonic() + args.seconds + 5
                        presentation.append(report)
                        elapsed = (report["timestamp"] - first["timestamp"]) / 1000
                        if elapsed < args.seconds:
                            continue
                        jpeg = page.evaluate("""() => {
                            const c = document.createElement('canvas');
                            c.width = Math.min(960, videoEl.videoWidth);
                            c.height = c.width * videoEl.videoHeight / videoEl.videoWidth;
                            c.getContext('2d').drawImage(videoEl, 0, 0, c.width, c.height);
                            return c.toDataURL('image/jpeg', .9).split(',')[1];
                        }""")
                        args.snapshot.write_bytes(base64.b64decode(jpeg))
                        if elapsed:
                            count = report["decoded"] - first["decoded"]
                            result = {
                                "seconds": round(elapsed, 2),
                                "decoded_frames": count,
                                "decoded_fps": round(count / elapsed, 2),
                                "dropped": report["dropped"] - first["dropped"],
                                "freezes": report["freezes"] - first["freezes"],
                            }
                            for key, numerator, denominator in (
                                ("jitter_buffer_ms", "jitterBufferDelay", "emitted"),
                                ("decode_ms", "decodeTime", "decoded"),
                            ):
                                if all(s.get(k) is not None for s in (report, first)
                                       for k in (numerator, denominator)):
                                    n = report[denominator] - first[denominator]
                                    if n:
                                        result[key] = round(1000 * (report[numerator] -
                                                            first[numerator]) / n, 1)
                            for key in ("captureToPresentation", "receiveToPresentation"):
                                values = [s[key] for s in presentation
                                          if s.get(key) is not None and isfinite(s[key])]
                                if values:
                                    result[key + "_median_ms"] = round(median(values), 1)
                            print(json.dumps(result))
                        return
                    if report["status"] == "订阅失败":
                        break
                raise SystemExit("No decoded video sample. Room may be occupied.")
        finally:
            server.shutdown()


if __name__ == "__main__":
    main()
