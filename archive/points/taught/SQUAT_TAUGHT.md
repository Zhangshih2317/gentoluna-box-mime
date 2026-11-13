# User-Taught Squat, 2026-11-11

Recorded point: `SQUAT_TAUGHT_UI-20261111-143323.json`.

The user manually positioned the robot and reported that the torso appeared
approximately vertical. The recorded body feedback is:

| Axis | Recovered Start (deg) | Taught Squat (deg) | Change (deg) |
| --- | ---: | ---: | ---: |
| B1 | -0.744 | -0.744 | 0 |
| B2 | 9.281 | 40.281 | +31 |
| B3 | -17.364 | -42.364 | -25 |
| B4 | 5.456 | 5.456 | 0 |
| B5 | -0.296 | -0.296 | 0 |
| B6 | -1.653 | -1.651 | +0.002 |

The principal manually changed axes are B2 and B3. B4 stays at its starting
value. This differs from the disabled B2/B3/B4 crouch recipe. The user's
observation of torso orientation is recorded as an observation, not as a
measured pitch or roll.

Both arms display HOME. BODY displays Position, error 0, speed 5%, acceleration
5%. Two readings of the BODY Realtime Data field agree to the displayed 0.001
degree precision. The source screenshots are referenced in the point file.

FXPlatform retained the connection; the independent SDK and gateway could not
supply feedback. Values were transcribed from Realtime Data, not the editable
Position Cmd fields. This is a UI observation record, not the five-frame SDK
capture produced by `teach_pose.py`; velocities and frame freshness were not
verified. Head and arm values were observed in separate views.

The measured B2 change is 31 degrees, one degree above the earlier requested
30-degree bound. The record preserves the demonstrated pose without changing
the measurement or authorizing automatic playback. No motion or mode commands
were sent. The original automatic crouch remains disabled; a demonstrated
endpoint alone does not validate an interpolated motion.

## Second Taught Point

Recorded point: `SQUAT_TAUGHT_02_UI-20261111-143743.json`.
Source: the user-supplied FXPlatform screenshot displaying local time
2026-11-11 14:37:43. The first taught point remains unchanged.

| Axis | Second Squat (deg) | Change from First Squat (deg) | Change from Recovered Start (deg) |
| --- | ---: | ---: | ---: |
| B1 | -0.744 | 0 | 0 |
| B2 | 40.281 | 0 | +31 |
| B3 | -80.364 | -38 | -63 |
| B4 | 40.456 | +35 | +35 |
| B5 | -0.296 | 0 | 0 |
| B6 | -1.651 | 0 | +0.002 |

The user reports that the torso is also vertical at this point and identifies
B3 and B4 as the axes adjusted since the first squat. B2 remains at 40.281
degrees. The user also states that B2 through B4 should coordinate for a very
low squat. Preserve this as teaching guidance; these two endpoint observations
do not establish a fixed coordination ratio or a validated motion path.

BODY displays Position, error 0, speed 5%, acceleration 5%. ARM1 displays HOME,
Position, Stopped, error 0. HEAD displays [2.107, -15.998, 0] degrees, IDLE,
error 0. ARM0 is outside the supplied screenshot and is explicitly unrecorded
rather than copied from the previous point.

This record transcribes the screenshot's Realtime Data fields. There was no
SDK connection, new live sampling, motion command, or mode change for this
capture. The source screenshot is preserved in the evidence directory.

## Third Taught Point

Recorded point: `SQUAT_TAUGHT_03_UI-20261111-144906.json`.
Two current FXPlatform observations at 14:48:47 and 14:49:06 agree on BODY:
[-0.744, 70.281, -127.364, 55.000, -0.296, -1.651] degrees.
The changes from point 2 are B2 +30, B3 -47, B4 +14.544 degrees.
Relative to recovered start, they are B2 +61, B3 -110, B4 +49.544 degrees.

The user describes the pose as approximately at the squat limit. This does not
establish a mechanical endstop. The actual B4 Realtime Data is 55.000 degrees;
the editable Position Cmd field contains 70.456 degrees. Preserve both with
their provenance and do not substitute the editable value for measured feedback.
Whether that input was accepted, clipped, rejected, or never submitted is not
established by the screenshot.

Detailed offline inference and model limitations are in
`SQUAT_REASONING-20261111.md`. All three observations remain non-executable
records. No robot connection or motion commands were made for this capture.
