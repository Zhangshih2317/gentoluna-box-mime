"""Empty-handed Luna gesture planning. No SDK imports or hardware operations."""

import math

HOME = [[90., -90., -90., -90., 0., 0., 0.],
        [-90., -90., 90., -90., 0., 0., 0.]]
ARM_LIMITS = [(-173, 173), (-118, 118), (-173, 173), (-145, 45),
              (-173, 173), (-60, 60), (-90, 90)]
# Intersection of the archived controller ranges and the manufacturer model.
BODY_LIMITS = [(-11, 11), (-90, 90), (-140, 60), (-55, 55), (-30, 30), (-170, 170)]
RATE = 50
MAX_SPEED = 3.0
MAX_ACCELERATION = 3.0
FIELDS = ("arm0", "arm1", "body", "head")


def vector(values, size):
    result = [float(x) for x in values]
    if len(result) != size or not all(math.isfinite(x) for x in result):
        raise ValueError("Expected %d finite joint values" % size)
    return result


def pose(arm0, arm1, body, head):
    return dict(zip(FIELDS, [vector(arm0, 7), vector(arm1, 7),
                            vector(body, 6), vector(head, 3)]))


def delta(a, b, fields=FIELDS):
    return max(abs(x-y) for key in fields for x, y in zip(a[key], b[key]))


def validate_pose(state):
    pose(**state)
    for key, limits in (("arm0", ARM_LIMITS), ("arm1", ARM_LIMITS), ("body", BODY_LIMITS)):
        for index, (value, (low, high)) in enumerate(zip(state[key], limits)):
            if not low+2 <= value <= high-2:
                raise ValueError("%s joint %d violates conservative limits" % (key, index+1))


def interpolate(a, b, fraction):
    return {key: [x+(y-x)*fraction for x, y in zip(a[key], b[key])] for key in FIELDS}


def blend(t):
    return t*t*t*(10+t*(-15+6*t))


def build_plan(start, squat_deg=30., turn_deg=45.):
    start = pose(**start)
    if not math.isfinite(squat_deg) or not 0 < squat_deg <= 30:
        raise ValueError("Squat amplitude must be in (0, 30] degrees")
    if not math.isfinite(turn_deg) or abs(turn_deg) != 45:
        raise ValueError("Waist turn must be +45 or -45 degrees")
    if any(max(abs(x-y) for x, y in zip(start["arm%d" % i], HOME[i])) > 1.1
           for i in (0, 1)):
        raise ValueError("Both arms must start within 1.1 degrees of taught HOME")
    body = start["body"]
    crouch = [q+d for q, d in zip(body, (0, squat_deg/2, -squat_deg, squat_deg/2, 0, 0))]
    turned = body.copy()
    turned[5] += turn_deg
    opened = [q.copy() for q in HOME]
    embrace = [q.copy() for q in HOME]
    for arm, sign in ((0, 1), (1, -1)):
        opened[arm][2] -= sign*18
        embrace[arm][2] += sign*12
        embrace[arm][3] += 10
    targets = [
        ("open_before_crouch", opened, body, "arms"),
        ("crouch", opened, crouch, "body"),
        ("embrace_empty_space", embrace, crouch, "arms"),
        ("stand", embrace, body, "body"),
        ("turn_waist", embrace, turned, "body"),
        ("open_arms_standing", opened, turned, "arms"),
        ("arms_home", HOME, turned, "arms"),
        ("body_home", HOME, body, "body"),
    ]
    validate_pose(start)
    result = []
    previous = start
    for name, arms, body_target, active in targets:
        target = pose(*arms, body_target, start["head"])
        validate_pose(target)
        change = delta(previous, target)
        # Quintic peak derivatives: 1.875 and 10/sqrt(3).
        seconds = max(2., 1.875*change/MAX_SPEED,
                      math.sqrt((10/math.sqrt(3))*change/MAX_ACCELERATION))
        steps = math.ceil(seconds*RATE)
        rows = [interpolate(previous, target, blend(i/steps)) for i in range(steps+1)]
        rows.extend([target]*(RATE//2))
        if len(rows) > 5000:
            raise ValueError("Stage exceeds controller capacity")
        result.append({"name": name, "active": active, "start": previous,
                       "target": target, "rows": rows, "seconds": len(rows)/RATE})
        previous = target
    return result


def collision_samples(stage, max_step=.5):
    count = max(1, math.ceil(delta(stage["start"], stage["target"])/max_step))
    return [interpolate(stage["start"], stage["target"], i/count) for i in range(count+1)]
