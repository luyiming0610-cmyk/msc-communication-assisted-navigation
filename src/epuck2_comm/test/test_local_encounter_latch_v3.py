"""controller_v3_unified_encounter_20260717 regression tests.

New tests, itemised to match
controller_v3_unified_encounter_design_20260717.md section 10 (items
1-9; item 10 is a process/tracking note about controller_v2's retired
test files, not a runnable test; item 11 is the aggregate colcon count).
Reported separately from the pre-existing controller_v1 suite (41 tests,
unmodified except two LocalAvoidanceLatch.apply() call-site updates for
the new required own_yaw_rad argument) and from controller_v2's 23 tests,
which are retired from the working tree in this revision (recoverable via
``git show 922a580:src/epuck2_comm/test/test_local_avoidance_latch_v2.py``
and the equivalent integration test path) because the two-lane structure
they exercised no longer exists.
"""

import math

import pytest

from epuck2_comm.local_obstacle_logic import (
    LocalAvoidanceLatch,
    LocalObstacleDecision,
    normalize_angle,
)


def _left_side(angular_rps=-0.30, linear_mps=0.012):
    return LocalObstacleDecision(True, False, "LOCAL_LEFT_SIDE", linear_mps, angular_rps)


def _right_side(angular_rps=0.30, linear_mps=0.012):
    return LocalObstacleDecision(True, False, "LOCAL_RIGHT_SIDE", linear_mps, angular_rps)


def _narrow():
    return LocalObstacleDecision(True, False, "LOCAL_NARROW", 0.006, 0.0)


def _clear():
    return LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0)


def _front_danger(angular_rps=-0.65):
    return LocalObstacleDecision(True, False, "LOCAL_FRONT_DANGER", 0.0, angular_rps)


def _front_warn(angular_rps=-0.45):
    return LocalObstacleDecision(True, False, "LOCAL_FRONT_WARN", 0.010, angular_rps)


def _sensor_invalid():
    return LocalObstacleDecision(True, True, "LOCAL_SENSOR_INVALID", 0.0, 0.0)


def _reach_constrained(**overrides):
    """Drive a fresh latch through ACTIVE into CONSTRAINED via a tiny
    max_turn_ledger_rad, returning (latch, t, yaw, x, y)."""
    params = dict(max_turn_ledger_rad=0.02, local_bypass_distance_m=0.01)
    params.update(overrides)
    latch = LocalAvoidanceLatch(**params)
    t, yaw, x, y = 0.0, 0.0, 0.0, 0.0
    latch.apply(_left_side(), t, x, y, yaw)
    t += 0.5
    yaw += -0.30 * 0.5
    latch.apply(_left_side(), t, x, y, yaw)
    assert latch.phase == "CONSTRAINED"
    return latch, t, yaw, x, y


def _reach_recovery_allowed(latch, t, yaw, x, y):
    far_x = x + latch.local_bypass_distance_m + 0.02
    t += 0.1
    latch.apply(_clear(), t, far_x, y, yaw)
    t += latch.side_clear_confirm_s + 0.05
    d = latch.apply(_clear(), t, far_x, y, yaw)
    assert latch.phase == "RECOVERY_ALLOWED"
    return t, far_x, d


# 1. ±π wraparound: a small true rotation across the seam must not produce
#    a spurious near-2π ledger jump.
def test_yaw_ledger_handles_pi_wraparound():
    latch = LocalAvoidanceLatch(max_turn_ledger_rad=10.0)
    latch.apply(_front_warn(angular_rps=0.45), 0.0, 0.0, 0.0, 3.10)
    wrapped_yaw = normalize_angle(3.10 + 0.08)  # true +0.08 rad, wraps past +pi
    latch.apply(_front_warn(angular_rps=0.45), 0.05, 0.0, 0.0, wrapped_yaw)
    assert latch.turn_ledger_used_rad == pytest.approx(0.08, abs=1e-6)


# 2. FRONT_DANGER -> its own CLEARANCE tail -> LEFT_SIDE share one ledger;
#    LEFT_SIDE gets no second, independent allowance.
def test_front_then_side_share_one_ledger_no_second_allowance():
    latch = LocalAvoidanceLatch(max_turn_ledger_rad=0.55, local_bypass_distance_m=0.01)
    t, yaw, x, y = 0.0, 0.0, 0.0, 0.0
    phases_seen = []
    for _ in range(10):
        t += 0.05
        yaw += -0.65 * 0.05
        d = latch.apply(_front_danger(), t, x, y, yaw)
        phases_seen.append(latch.phase)
        assert d.angular_rps == -0.65  # front never clipped while raw-active
    ledger_after_front = latch.turn_ledger_used_rad
    # 10 ticks at dt=0.05s, -0.65rad/s: the first tick contributes 0 by
    # design (encounter-open resets previous_yaw to the current sample), so
    # 9 ticks of real motion -> 9*0.65*0.05=0.2925rad expected.
    assert ledger_after_front == pytest.approx(0.2925, abs=1e-6)
    assert ledger_after_front > 0.25  # front alone used most of the 0.55 budget

    t += 0.05
    latch.apply(_clear(), t, x, y, yaw)  # front's own clearance tail begins
    t += 0.05
    latch.apply(_left_side(), t, x, y, yaw)  # same obstacle, now via the side sensor

    assert "CLOSED" not in phases_seen
    assert latch.phase != "CLOSED"
    assert latch.turn_ledger_used_rad >= ledger_after_front


# 3. Oscillation stress test: cumulative |Δyaw| still reaches the cap even
#    though peak-deflection-from-start would not.
def test_oscillation_reaches_cap_via_cumulative_not_peak():
    latch = LocalAvoidanceLatch(max_turn_ledger_rad=0.6, local_bypass_distance_m=0.01)
    t, yaw, x, y = 0.0, 0.0, 0.0, 0.0
    for _ in range(8):
        t += 0.1
        yaw += -0.2
        latch.apply(_left_side(angular_rps=-0.30), t, x, y, yaw)
        if latch.phase != "ACTIVE":
            break
        t += 0.1
        yaw += 0.2  # turn back by the same amount -- net drift is ~0
        latch.apply(_right_side(angular_rps=0.30), t, x, y, yaw)
        if latch.phase != "ACTIVE":
            break
    assert latch.phase == "CONSTRAINED"


# 4. CONSTRAINED holds (zero velocity) while a raw local decision is still
#    active -- never creeps into a sensor that is still reporting proximity.
def test_constrained_holds_when_raw_still_active():
    latch, t, yaw, x, y = _reach_constrained()
    t += 0.05
    d = latch.apply(_left_side(), t, x, y, yaw)
    assert d.mode == "LOCAL_ENCOUNTER_HOLD"
    assert d.linear_mps == 0.0
    assert d.angular_rps == 0.0


# 5. CONSTRAINED creeps only once genuinely clear; a re-trigger reverts to
#    HOLD on the same tick, not the tick after.
def test_constrained_creeps_when_clear_then_holds_on_retrigger_same_tick():
    latch, t, yaw, x, y = _reach_constrained()
    t += 0.1
    d_creep = latch.apply(_clear(), t, x, y, yaw)
    assert d_creep.mode == "LOCAL_ENCOUNTER_CREEP"
    assert d_creep.linear_mps == latch.constrained_speed_mps
    assert d_creep.angular_rps == 0.0

    t += 0.1
    d_retrigger = latch.apply(_left_side(), t, x, y, yaw)
    assert d_retrigger.mode == "LOCAL_ENCOUNTER_HOLD"
    assert d_retrigger.linear_mps == 0.0
    assert d_retrigger.angular_rps == 0.0


# 6. FAILSAFE is a hard latch: no automatic exit, even after a long
#    genuinely-clear stream well past rearm_quiet_s.
def test_failsafe_never_auto_exits():
    latch, t, yaw, x, y = _reach_constrained(max_bypass_extension_m=0.05, rearm_quiet_s=0.2)
    t += 0.1
    x = 0.10  # distance now exceeds max_bypass_extension_m=0.05
    d = latch.apply(_left_side(), t, x, y, yaw)
    assert latch.phase == "FAILSAFE"
    assert d.mode == "LOCAL_ENCOUNTER_FAILSAFE"
    assert d.safety_stop is True
    assert d.linear_mps == 0.0 and d.angular_rps == 0.0

    for _ in range(50):
        t += 0.1
        d = latch.apply(_clear(), t, x, y, yaw)
        assert latch.phase == "FAILSAFE"
        assert d.mode == "LOCAL_ENCOUNTER_FAILSAFE"
        assert d.safety_stop is True


# 7a. SENSOR_INVALID preempts every pre-latch phase, phase left untouched.
def test_sensor_invalid_preempts_active_phase():
    latch = LocalAvoidanceLatch()
    latch.apply(_left_side(), 0.0, 0.0, 0.0, 0.0)
    assert latch.phase == "ACTIVE"
    d = latch.apply(_sensor_invalid(), 0.05, 0.0, 0.0, 0.0)
    assert d.safety_stop and d.mode == "LOCAL_SENSOR_INVALID"
    assert latch.phase == "ACTIVE"


def test_sensor_invalid_preempts_constrained_phase():
    latch, t, yaw, x, y = _reach_constrained()
    d = latch.apply(_sensor_invalid(), t + 0.05, x, y, yaw)
    assert d.safety_stop and d.mode == "LOCAL_SENSOR_INVALID"
    assert latch.phase == "CONSTRAINED"


# 7b. raw FRONT_DANGER still preempts (unmodified output) while CONSTRAINED;
#     phase/origin are untouched by the preemption itself.
def test_front_danger_preempts_constrained_without_disturbing_origin():
    latch, t, yaw, x, y = _reach_constrained()
    origin_before = latch.origin
    d = latch.apply(_front_danger(), t + 0.05, x, y, yaw + 0.1)
    assert d.mode == "LOCAL_FRONT_DANGER"
    assert d.angular_rps == -0.65
    assert latch.phase == "CONSTRAINED"
    assert latch.origin == origin_before


# 7c. Once latched in FAILSAFE, a fresh raw FRONT_DANGER does NOT get a
#     peek-through -- the output stays the frozen FAILSAFE decision and no
#     field changes.
def test_failsafe_latched_ignores_fresh_front_danger():
    latch, t, yaw, x, y = _reach_constrained(max_bypass_extension_m=0.05)
    x = 0.10
    t += 0.1
    latch.apply(_left_side(), t, x, y, yaw)
    assert latch.phase == "FAILSAFE"
    ledger_before = latch.turn_ledger_used_rad
    t += 0.1
    d = latch.apply(_front_danger(), t, x, y, yaw + 1.0)
    assert d.mode == "LOCAL_ENCOUNTER_FAILSAFE"
    assert d.safety_stop is True
    assert latch.turn_ledger_used_rad == ledger_before


# 8. RECOVERY_ALLOWED interrupted by front, side, or narrow all fall back
#    to CONSTRAINED (never ACTIVE), origin retained.
@pytest.mark.parametrize(
    "trigger,expect_mode",
    [
        (_front_danger(), "LOCAL_FRONT_DANGER"),
        (_left_side(), "LOCAL_ENCOUNTER_HOLD"),
        (_narrow(), "LOCAL_ENCOUNTER_HOLD"),
    ],
)
def test_recovery_allowed_interrupted_reverts_to_constrained(trigger, expect_mode):
    latch, t, yaw, x, y = _reach_constrained(side_clear_confirm_s=0.1)
    origin = latch.origin
    t, far_x, _ = _reach_recovery_allowed(latch, t, yaw, x, y)
    d = latch.apply(trigger, t + 0.1, far_x, y, yaw)
    assert latch.phase == "CONSTRAINED"
    assert latch.origin == origin
    assert d.mode == expect_mode


# 9. LOCAL_RECOVERY_READY: exact signal shape, fires exactly once per
#    RECOVERY_ALLOWED entry.
def test_recovery_ready_signal_shape_and_single_pulse():
    latch, t, yaw, x, y = _reach_constrained(side_clear_confirm_s=0.1, rearm_quiet_s=0.2)
    t, far_x, d = _reach_recovery_allowed(latch, t, yaw, x, y)
    assert d == LocalObstacleDecision(True, False, "LOCAL_RECOVERY_READY", 0.0, 0.0)
    for _ in range(5):
        t += 0.05
        d2 = latch.apply(_clear(), t, far_x, y, yaw)
        assert d2.mode != "LOCAL_RECOVERY_READY"
        assert d2.active is False


# hysteresis_hint(): locked raw mode for the whole open encounter, "" only
# once CLOSED.
def test_hysteresis_hint_covers_whole_encounter():
    latch, t, yaw, x, y = _reach_constrained(side_clear_confirm_s=0.1, rearm_quiet_s=0.1)
    assert latch.hysteresis_hint() == "LOCAL_LEFT_SIDE"
    t, far_x, _ = _reach_recovery_allowed(latch, t, yaw, x, y)
    assert latch.hysteresis_hint() == "LOCAL_LEFT_SIDE"
    t += 0.2
    latch.apply(_clear(), t, far_x, y, yaw)
    assert latch.phase == "CLOSED"
    assert latch.hysteresis_hint() == ""


# LOCAL_NARROW mid-encounter: passes through unmodified, never adds to the
# ledger, never itself replaced.
def test_narrow_passes_through_and_does_not_grow_ledger():
    latch, t, yaw, x, y = _reach_constrained()
    ledger_before = latch.turn_ledger_used_rad
    d = latch.apply(_narrow(), t + 0.1, x, y, yaw)
    assert d.mode == "LOCAL_ENCOUNTER_HOLD"  # CONSTRAINED still gates by raw-active
    assert latch.turn_ledger_used_rad == pytest.approx(ledger_before, abs=1e-9)
