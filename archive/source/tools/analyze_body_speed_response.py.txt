"""Analyze the three recorded 30 cm runs; never connect to the robot.

Delay estimates are based only on the unpaused prefix of each stage. These
sparse status samples are not a calibrated plant model or compensation table.
"""

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path

from box_motion_plan import blend

ROOT = Path(__file__).resolve().parents[1]
RUNS = tuple("box-mime-taught-20261113-"+stamp+".jsonl"
             for stamp in ("132948", "133244", "133502"))


def inverse_blend(fraction):
    if not math.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError("Joint progress outside the stage")
    low, high = 0., 1.
    for _ in range(60):
        middle = (low+high)/2
        if blend(middle) < fraction:
            low = middle
        else:
            high = middle
    return (low+high)/2


def seconds_between(a, b):
    return (datetime.fromisoformat(b["time"])-datetime.fromisoformat(a["time"])).total_seconds()


def analyze(path):
    raw = path.read_bytes()
    numbered = [(i, json.loads(line)) for i, line in enumerate(raw.decode("utf-8").splitlines(), 1)
                if line.startswith("{")]
    events = [event for _, event in numbered]
    plan = next(e for e in events if e.get("event") == "plan")
    request = next(e for e in events if e.get("event") == "request")
    if not request["execute"] or plan.get("requested_body_drop_mm") != 300:
        raise ValueError("Expected a recorded 300 mm execution")
    speed = float(plan["controller_velocity_percent"])
    scale = speed/20.
    results = []
    for stage in plan["stages"]:
        if stage["name"] not in ("crouch", "stand"):
            continue
        movement = [b-a for a, b in zip(stage["start"]["body"], stage["target"]["body"])]
        distance = max(map(abs, movement))
        nominal = max(2., 1.875*distance/(7.2*scale),
                      math.sqrt((10/math.sqrt(3))*distance/(1.8*scale)))
        stage_events = [(i, e) for i, e in numbered if e.get("name") == stage["name"]]
        statuses = [(i, e) for i, e in stage_events if e["event"] == "motion_wait_status"]
        holds, delays = [], []
        prefix = True
        for index, (line, event) in enumerate(statuses):
            elapsed = event["elapsed_motion_seconds"]
            if prefix and .15*nominal < elapsed < .85*nominal:
                apparent = []
                for joint in (1, 2, 3):
                    progress = ((event["pose"]["body"][joint]-stage["start"]["body"][joint])
                                /movement[joint])
                    apparent.append(elapsed-nominal*inverse_blend(progress))
                delays.append({
                    "source_line": line, "elapsed_motion_seconds": elapsed,
                    "apparent_delay_B2_B3_B4_seconds": apparent,
                    "B3_minus_B2_seconds": apparent[1]-apparent[0],
                    "B3_minus_B4_seconds": apparent[1]-apparent[2],
                    "pitch_sum_error_deg": (sum(event["pose"]["body"][1:4])
                                            -sum(stage["start"]["body"][1:4])),
                })
            if event["action"] == "hold":
                prefix = False
                if event["reason"] != "waiting_for_arrival":
                    resumed = next((e for _, e in statuses[index+1:] if e["action"] == "advance"), None)
                    holds.append({
                        "source_line": line, "reason": event["reason"],
                        "recorded_error_deg": event["maximum_error_deg"],
                        "wait_seconds": seconds_between(event, resumed) if resumed else None,
                    })
        began = next(e for _, e in stage_events if e["event"] == "stage_start")
        ended = next(e for _, e in stage_events if e["event"] == "stage_complete")
        results.append({
            "stage": stage["name"], "joint_delta_B2_B3_B4_deg": movement[1:4],
            "nominal_seconds": nominal, "actual_seconds": seconds_between(began, ended),
            "nominal_peak_B3_speed_deg_s": 1.875*distance/nominal,
            "midmotion_holds": holds, "unpaused_delay_estimates": delays,
        })
    completion = next(e for e in events if e.get("event") == "mime_complete")
    return {
        "source": str(path.relative_to(ROOT)), "sha256": hashlib.sha256(raw).hexdigest(),
        "speed_percent": speed, "controller_acceleration_percent": plan["controller_acceleration_percent"],
        "software_BODY_acceleration_limit_deg_s2": 1.8*scale,
        "returned_home": completion["returned_home"],
        "home_error_deg": completion["maximum_home_error_deg"], "stages": results,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT/"evidence/body-speed-response-20261113.json")
    args = parser.parse_args()
    snapshot = ROOT/"reference/sdk_motion_review"
    result = {
        "schema": "luna_offline_body_response_review_v1",
        "motion_commands_sent": 0, "calibration_valid": False,
        "limitations": [
            "Snapshots occur at transitions or about every five seconds; no per-tick history.",
            "Joint command times are reconstructed from the recorded quintic v1 timing policy.",
            "Apparent delays combine controller planning, servo response and observation timing.",
            "No fixed feedforward delay or motor gain is certified by this analysis.",
            "Controller firmware internals and per-axis BODY physical limits were not supplied.",
        ],
        "runs": [analyze(ROOT/"evidence"/name) for name in RUNS],
        "sdk_source": "/home/nvidia/tianji-robot-SDK (installed source snapshot, not a firmware dump)",
        "sdk_snapshot_sha256": {
            str(path.relative_to(snapshot)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(snapshot.rglob("*")) if path.is_file()
        },
    }
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=True), encoding="utf-8")
    print(json.dumps({"report": str(args.output), "runs": len(result["runs"]),
                      "calibration_valid": False, "motion_commands_sent": 0}))


if __name__ == "__main__":
    main()
