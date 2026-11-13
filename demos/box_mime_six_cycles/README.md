# Six-Cycle Box Mime Demo

Entry: `run_box_mime_six_cycles.py` at the repository root.

Public packaging update: see the root README and `docs/SETUP.md` for portable
configuration. The original six-cycle program subsequently completed all
48 stages in `box-mime-six-cycles-20261113-164309.jsonl`, with maximum HOME
error 0.002 degrees. The deployment-adapted public copy has not been
hardware-executed. Historical preparation evidence below belongs to the
original local snapshot.

This is an independent motion-source snapshot. The original
`run_box_mime_taught.py`, `tools/` control sources, and original geometry cache
are not modified by the demo. User settings are at the top of the new entry.
Shared teaching points and the existing SSH connection helper remain in use.

## Baseline Reviewed

The three recent execution logs ending `162035`, `162152`, and `162337` on
2026-11-13 each completed all eight stages, returned HOME, and exited zero.
None recorded `recovery_failed`, `emergency_stop_requested`, or
`native_guard_rejected`. Final maximum HOME errors were 0.001, 0.002, and
0.001 degrees respectively.

The latest run, `evidence/box-mime-taught-20261113-162337.jsonl`, confirmed
speed 100% and acceleration 50% for both arms and BODY. The demo copies those
settings, the 300 mm squat, and the two-degree whole-body arrival policy.
These are observations from those runs, not proof of future loaded motion.

## Sequence

Six complete cycles use HOME-relative waist targets:

| Cycle | B6 Offset From HOME |
|---|---:|
| 1 | -45 degrees |
| 2 | +45 degrees |
| 3 | -45 degrees |
| 4 | +45 degrees |
| 5 | -45 degrees |
| 6 | +45 degrees |

Each cycle opens the arms, crouches, embraces, stands, turns the waist,
opens the arms to release, returns the arms HOME, and returns the waist HOME.
The opposite turn changes only B6 in the standing turn/release/return section.
It does not move directly 90 degrees from one side to the other.

The SDK connection and Position modes remain active between cycles.
Two distinct qualifying position frames, 0.1 to 1 second apart and within
two degrees on all axes, confirm each endpoint. Normal completion retains
Position mode. Existing fault handling remains active; a fault aborts the
remaining cycles rather than restarting or skipping the failed cycle.

## Run Manually

```powershell
python "C:\Users\operator\Documents\ChatGPT\luna\run_box_mime_six_cycles.py" --execute
```

Without `--execute`, the entry only performs a read-only preflight.
`--stepped`, `--full-preflight`, and HOME-only execution are not demo modes.
Use the existing separate HOME entry for recovery.

Both direction caches are checked before any mode change or movement.
The new cache is `points/BOX_MIME_SIX_CYCLES_GEOMETRY_CACHE.json`.
To rebuild it without connecting to the robot controller:

```powershell
python "C:\Users\operator\Documents\ChatGPT\luna\run_box_mime_six_cycles.py" --prepare-cache --cache-head 2.107 -16.002 0
```

This is sampled bare-robot geometry, not continuous collision certification
or a model of external obstacles and installed grippers.

## Evidence

New runs write `evidence/box-mime-six-cycles-<timestamp>.jsonl` and a matching
`-response.jsonl`. `demo_cycle_start` and `demo_cycle_complete` identify each
cycle and yaw sign. Feedback samples also contain `cycle_index` and
`turn_relative_deg`. Only `demo_complete` confirms all six cycles finished.

Offline tests:

```powershell
python -m unittest discover -s tools -p "test_box_mime_six_cycles.py"
```

Coverage includes all 48 stages, one full target per arm per native stage,
delayed arm arrival, same HOME after every cycle, yaw-only reversal,
both-direction validation before one enable/connection, and interruption
preventing later cycles. No real motion is performed by these tests.

Verification completed on 2026-11-13:

- All nine offline tests passed.
- `evidence/box-mime-six-cycles-20261113-163518.jsonl` records successful
  model checks for 9,128 samples per direction (18,256 total), with minimum
  bare-flange spacing about 275.754 mm. Cache preparation sent no motion.
- `evidence/box-mime-six-cycles-20261113-164014.jsonl` records successful
  live read-only preflight for all six cycles and both directions, with
  zero motion commands and zero mode changes. The process exited zero.
- The assistant did not start physical execution. The later operator-run
  six-cycle completion is documented in `docs/VALIDATION.md`.
