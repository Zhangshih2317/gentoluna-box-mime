# Waist teaching

Recorded from FXPlatform realtime fields on 2026-11-11, local UTC+08:00.
Point: `WAIST_TAUGHT_01_UI-20261111-153805.json`.
Baseline: `../WAIST_TEACHING_BASELINE-20261111-153141.json`.

The demonstrated waist rotation changes B6. B1 through B5 retain the
standing baseline at displayed precision.

| Axis | Baseline (deg) | Observed (deg) | Change (deg) |
| --- | ---: | ---: | ---: |
| B1 | -0.744 | -0.744 | 0 |
| B2 | 9.281 | 9.281 | 0 |
| B3 | -17.364 | -17.364 | 0 |
| B4 | 5.456 | 5.456 | 0 |
| B5 | -0.296 | -0.296 | 0 |
| B6 | -1.653 | -45.651 | -43.998 |

The user describes this as a 45-degree turn. Preserve the measured endpoint
as taught; do not replace it with an inferred exact 45-degree target.
Physical left/right direction has not been established.

BODY readings match in two UI captures. Displayed BODY speed and acceleration
are both 5 percent; displayed system error is zero. These UI observations
do not provide SDK frame freshness, servo error checks, or path validation.
The teaching application holds the connection, so the separate read-only
SDK attempt did not succeed. No robot motion commands were sent for this
recording.

This example distinguishes waist rotation (B6) from the previously taught
squat coordination (B2, B3, B4). It records an endpoint only; it does not
validate replay or combining waist rotation with squatting.
