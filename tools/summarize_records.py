"""Build an event index from the public archive, without contacting a robot."""

import gzip
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVENTS = ROOT/"records/evidence"
FAILURES = {"recovery_failed", "emergency_stop_requested", "stop_unconfirmed",
            "native_guard_rejected", "failed"}


def load_events(path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as source:
        for line in source:
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                yield value


def summarize(path):
    events = list(load_events(path))
    complete = [e for e in events if e.get("event") in ("mime_complete", "demo_complete")]
    cycles = [e for e in events if e.get("event") == "demo_cycle_complete"]
    request = next((e for e in events if e.get("event") == "request"), {})
    exit_event = next((e for e in reversed(events) if "remote_exit_code" in e), {})
    result = {
        "file": path.name,
        "execute_requested": request.get("execute"),
        "controller_ratios": request.get("controller_ratios"),
        "stage_completions": sum(e.get("event") == "stage_complete" for e in events),
        "cycle_completions": len(cycles),
        "failure_events": [e for e in events if e.get("event") in FAILURES],
        "remote_exit_code": exit_event.get("remote_exit_code"),
        "completion_confirmed": exit_event.get("completion_confirmed"),
    }
    if complete:
        result["completion"] = complete[-1]
    if cycles:
        result["cycle_seconds"] = [e["elapsed_seconds"] for e in cycles]
        result["sum_cycle_seconds"] = sum(result["cycle_seconds"])
    trace = next((e for e in reversed(events) if e.get("event") == "response_trace_saved"), None)
    if trace:
        result["response_samples"] = trace["samples"]
        result["response_samples_dropped"] = trace.get("samples_dropped")
    return result


def main():
    index = [summarize(path) for path in sorted(EVENTS.glob("*.jsonl"))
             if not path.name.endswith("-response.jsonl")]
    selected = {
        "box-mime-taught-20261113-162035.jsonl",
        "box-mime-taught-20261113-162152.jsonl",
        "box-mime-taught-20261113-162337.jsonl",
        "box-mime-six-cycles-20261113-164309.jsonl",
    }
    summary = [entry for entry in index if entry["file"] in selected]
    for entry in summary:
        if (entry["failure_events"] or entry["remote_exit_code"] != 0
                or not entry["completion_confirmed"] or "completion" not in entry):
            raise RuntimeError("Expected successful baseline is not confirmed: "+entry["file"])
    for name, value in (("run-index.json", index), ("summary.json", summary)):
        (ROOT/"records"/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+"\n", "utf-8")
    print(json.dumps({"indexed_logs": len(index), "verified_successful_runs": len(summary),
                      "six_cycle_elapsed_sum": summary[0].get("sum_cycle_seconds")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
