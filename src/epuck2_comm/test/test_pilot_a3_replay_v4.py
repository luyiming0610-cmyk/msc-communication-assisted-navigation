"""controller_v4_full_sensor_bypass_20260717: pilot_a3 replay check.

*** This is NOT a safety-acceptance test. *** Only a real, Webots-instrumented
v4 pilot may certify a safe geometric margin. This test replays pilot_a3's
own recorded encounter #1 shape (front-warn -> turn -> straighten, the exact
sequence that leaked through the legacy v1/v2 LOCAL_BYPASS fallback and left
the robot only ~0.046m off the box's centerline -- well inside the ~0.065m
danger band -- before the second, fatal encounter) through
EncounterAvoidanceV4, using only what a v3-era EpuckState message could ever
have measured (no left_rear_m/right_rear_m ever existed on that message
schema). It asserts the new PASS_CONFIRM gate is never satisfied under that
information -- i.e. v4 would have refused to recover control at the point
where v1/v2/v3 all silently did, which is the specific gap that produced the
real collision.

Ground-truth values are taken directly from
experiments/controller_v3_unified_encounter_20260717/'s pilot_a3 forensic
trace: encounter #1 opened at t=5.16s (raw front-warn), turn settled to
yaw~-0.4566rad by t=7.0s, and the old code declared "bypass complete" at
local_bypass_distance_m=0.08m of straight-line travel (~t=6.73-13.71s),
handing off to a pure-heading LOCAL_RECOVER that left the robot's lateral
offset from the box centerline at only ~0.046m by the time of the second,
fatal encounter (see the pilot_a3 diagnostic report).
"""

import math

from epuck2_comm.local_obstacle_logic import (
    EncounterAvoidanceV4,
    LocalObstacleDecision,
    ZoneSnapshot,
    normalize_angle,
)


def _zones(**overrides):
    base = dict(
        left_front_m=math.inf, left_mid_m=math.inf, left_rear_m=math.inf,
        right_front_m=math.inf, right_mid_m=math.inf, right_rear_m=math.inf,
    )
    base.update(overrides)
    return ZoneSnapshot(**base)


def test_pilot_a3_encounter1_shape_never_satisfies_pass_confirm_without_rear_data():
    latch = EncounterAvoidanceV4()  # default (uncalibrated) v4 candidates
    robot = {"x": -0.55, "y": 0.0, "yaw": 0.0}
    t = 0.0
    dt = 0.05

    def step(linear, angular):
        robot["x"] += linear * math.cos(robot["yaw"]) * dt
        robot["y"] += linear * math.sin(robot["yaw"]) * dt
        robot["yaw"] = normalize_angle(robot["yaw"] + angular * dt)

    # Front-warn turning phase (pilot_a3's actual recorded turn, ~1.5s to
    # settle near yaw=-0.457rad).
    for _ in range(30):
        t += dt
        d = latch.apply(
            LocalObstacleDecision(True, False, "LOCAL_FRONT_WARN", 0.010, -0.45),
            _zones(), t, robot["x"], robot["y"], robot["yaw"],
        )
        step(d.linear_mps, d.angular_rps)
        if robot["yaw"] <= -0.45:
            break
    assert latch.phase in ("DETECT_TURN", "SIDE_TRACK")

    # ~7s of straight travel at the settled heading (matching pilot_a3's
    # real BYPASS leg, t=6.73-13.71s) -- raw front/side never retrigger in
    # this replay, exactly like the real recorded encounter #1 (no raw
    # LOCAL_LEFT_SIDE ever fired during it).
    for _ in range(140):
        t += dt
        d = latch.apply(
            LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0),
            _zones(), t, robot["x"], robot["y"], robot["yaw"],
        )
        step(d.linear_mps, d.angular_rps)
        assert d.mode != "LOCAL_RECOVERY_READY", (
            "PASS_CONFIRM must never be satisfied here: no v3-era message "
            "ever carried rear-zone data, so rear_seen can never become "
            "True from this replay -- exactly the gap that let v1/v2/v3 "
            "hand back control with insufficient real lateral clearance"
        )

    assert not latch.rear_seen
    assert latch.phase in ("DETECT_TURN", "SIDE_TRACK")
