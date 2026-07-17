"""controller_v2_local_latch_20260717 regression tests.

These are new tests, itemised 1-15 to match
controller_v2_local_latch_design_20260717.md section 7. They are reported
separately from the pre-existing controller_v1 suite
(test_local_obstacle_logic.py etc.), which must stay green unmodified aside
from the two LocalAvoidanceLatch.apply() call sites that needed the new
own_x/own_y arguments.
"""

import math

import pytest

from epuck2_comm.local_obstacle_logic import (
    LocalAvoidanceLatch,
    LocalObstacleDecision,
    decide_local_obstacle,
)


def _left_side(angular_rps=-0.30, linear_mps=0.012):
    return LocalObstacleDecision(True, False, "LOCAL_LEFT_SIDE", linear_mps, angular_rps)


def _narrow():
    return LocalObstacleDecision(True, False, "LOCAL_NARROW", 0.006, 0.0)


def _clear():
    return LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0)


def _front_danger():
    return LocalObstacleDecision(True, False, "LOCAL_FRONT_DANGER", 0.0, -0.65)


def _front_warn():
    return LocalObstacleDecision(True, False, "LOCAL_FRONT_WARN", 0.010, -0.45)


def _sensor_invalid():
    return LocalObstacleDecision(True, True, "LOCAL_SENSOR_INVALID", 0.0, 0.0)


def _reach_capped_bypass(**overrides):
    params = dict(
        max_side_encounter_turn_rad=0.01,
        local_bypass_distance_m=0.01,
        side_clear_confirm_s=0.1,
        max_bypass_extension_m=0.15,
    )
    params.update(overrides)
    latch = LocalAvoidanceLatch(**params)
    latch.apply(_left_side(), 0.0, 0.0, 0.0)
    latch.apply(_left_side(), 0.1, 0.0, 0.0)
    assert latch.side_phase == "CAPPED_BYPASS"
    return latch


def _reach_recovery_allowed(**overrides):
    latch = _reach_capped_bypass(**overrides)
    origin = latch.side_origin
    far_x = origin[0] + 0.05
    latch.apply(_clear(), 0.2, far_x, origin[1])
    latch.apply(_clear(), 0.35, far_x, origin[1])
    assert latch.side_phase == "RECOVERY_ALLOWED"
    return latch, far_x, origin[1]


def _reach_failsafe(**overrides):
    latch = _reach_capped_bypass(**overrides)
    origin = latch.side_origin
    latch.apply(_left_side(), 0.2, origin[0] + 0.20, origin[1])
    assert latch.side_phase == "FAILSAFE"
    return latch


# 1. TURNING budget clipping: exact, no overshoot.
def test_turning_clips_final_tick_to_exact_budget():
    latch = LocalAvoidanceLatch(max_side_encounter_turn_rad=0.10)
    t = 0.0
    angular = -0.30
    latch.apply(_left_side(angular_rps=angular), t, 0.0, 0.0)
    last_decision = None
    while latch.side_phase == "TURNING":
        t += 0.05
        last_decision = latch.apply(_left_side(angular_rps=angular), t, 0.0, 0.0)
    assert latch.side_phase == "CAPPED_BYPASS"
    assert latch.side_budget_used_rad == pytest.approx(0.10, abs=1e-9)
    assert abs(last_decision.angular_rps) < abs(angular)
    assert last_decision.mode == "LOCAL_LEFT_SIDE"


# 2. LOCAL_CLEARANCE ticks during TURNING count toward the budget.
def test_clearance_ticks_count_toward_budget():
    latch = LocalAvoidanceLatch(
        max_side_encounter_turn_rad=0.05, clear_hold_s=1.0, clearance_turn_rps=0.30
    )
    t = 0.0
    latch.apply(_left_side(), t, 0.0, 0.0)
    t += 0.02
    latch.apply(_left_side(), t, 0.0, 0.0)
    assert latch.side_phase == "TURNING"
    reached_capped = False
    for _ in range(200):
        t += 0.02
        decision = latch.apply(_clear(), t, 0.0, 0.0)
        if latch.side_phase == "CAPPED_BYPASS":
            reached_capped = True
            break
        assert decision.mode == "LOCAL_CLEARANCE"
    assert reached_capped, "budget must be exhausted by LOCAL_CLEARANCE ticks alone"


# 3. Single inactive tick in CAPPED_BYPASS followed by active: no premature RECOVERY_ALLOWED.
def test_single_clear_tick_does_not_confirm_recovery():
    latch = _reach_capped_bypass(local_bypass_distance_m=0.01, side_clear_confirm_s=1.0)
    origin = latch.side_origin
    far_x = origin[0] + 0.05
    d1 = latch.apply(_clear(), 0.2, far_x, origin[1])
    assert latch.side_phase == "CAPPED_BYPASS"
    assert d1.mode == "LOCAL_SIDE_BYPASS"
    d2 = latch.apply(_left_side(), 0.25, far_x, origin[1])
    assert latch.side_phase == "CAPPED_BYPASS"
    assert d2.mode == "LOCAL_SIDE_BYPASS"
    assert d2.angular_rps == 0.0
    assert latch.side_quiet_since_s is None


# 4. Flicker that never reaches an unbroken side_clear_confirm_s run: must reach
#    FAILSAFE at max_bypass_extension_m, not stall.
def test_flicker_without_confirmation_reaches_failsafe():
    latch = _reach_capped_bypass(
        local_bypass_distance_m=0.01, side_clear_confirm_s=1.0, max_bypass_extension_m=0.20
    )
    origin = latch.side_origin
    t, x = 0.2, origin[0]
    toggle = True
    reached_failsafe = False
    for _ in range(500):
        t += 0.1
        x += 0.01
        decision = latch.apply(_clear() if toggle else _left_side(), t, x, origin[1])
        toggle = not toggle
        if latch.side_phase == "FAILSAFE":
            reached_failsafe = True
            assert decision.safety_stop is True
            assert decision.mode == "LOCAL_SIDE_ENCOUNTER_FAILSAFE"
            break
        assert latch.side_phase == "CAPPED_BYPASS"
    assert reached_failsafe


# 5. Genuinely unbroken quiet run of exactly side_clear_confirm_s: transitions on
#    the threshold tick, not before.
def test_confirmed_quiet_triggers_recovery_on_threshold_tick():
    latch = _reach_capped_bypass(local_bypass_distance_m=0.01, side_clear_confirm_s=0.3)
    origin = latch.side_origin
    far_x = origin[0] + 0.05
    t = 0.2
    latch.apply(_clear(), t, far_x, origin[1])
    quiet_start = latch.side_quiet_since_s
    assert quiet_start == t
    latch.apply(_clear(), quiet_start + 0.29, far_x, origin[1])
    assert latch.side_phase == "CAPPED_BYPASS"
    d = latch.apply(_clear(), quiet_start + 0.30, far_x, origin[1])
    assert latch.side_phase == "RECOVERY_ALLOWED"
    assert d.mode == "LOCAL_RECOVERY_READY"


# 6. The CAPPED_BYPASS -> RECOVERY_ALLOWED transition tick returns exactly the
#    LOCAL_RECOVERY_READY signal shape.
def test_recovery_ready_signal_shape():
    latch = _reach_capped_bypass(local_bypass_distance_m=0.01, side_clear_confirm_s=0.1)
    origin = latch.side_origin
    far_x = origin[0] + 0.05
    t = 0.2
    latch.apply(_clear(), t, far_x, origin[1])
    d = latch.apply(_clear(), t + 0.15, far_x, origin[1])
    assert latch.side_phase == "RECOVERY_ALLOWED"
    assert d == LocalObstacleDecision(True, False, "LOCAL_RECOVERY_READY", 0.0, 0.0)


# 7. RECOVERY_ALLOWED interrupted by a genuine side re-trigger: reverts to
#    CAPPED_BYPASS the same tick; origin/budget retained; output is straight,
#    zero angular (not the side-avoidance turn rate).
def test_recovery_allowed_interrupted_by_side_reverts_to_capped_bypass():
    latch = _reach_capped_bypass(
        max_side_encounter_turn_rad=0.02, local_bypass_distance_m=0.01, side_clear_confirm_s=0.1
    )
    origin = latch.side_origin
    budget_before = latch.side_budget_used_rad
    far_x = origin[0] + 0.05
    t = 0.2
    latch.apply(_clear(), t, far_x, origin[1])
    latch.apply(_clear(), t + 0.15, far_x, origin[1])
    assert latch.side_phase == "RECOVERY_ALLOWED"
    d = latch.apply(_left_side(), t + 0.20, far_x, origin[1])
    assert latch.side_phase == "CAPPED_BYPASS"
    assert latch.side_origin == origin
    assert latch.side_budget_used_rad == budget_before
    assert d.mode == "LOCAL_SIDE_BYPASS"
    assert d.angular_rps == 0.0
    assert latch.side_quiet_since_s is None


# 8. Same as 7 but the re-trigger is LOCAL_NARROW.
def test_recovery_allowed_interrupted_by_narrow_reverts_to_capped_bypass():
    latch = _reach_capped_bypass(
        max_side_encounter_turn_rad=0.02, local_bypass_distance_m=0.01, side_clear_confirm_s=0.1
    )
    origin = latch.side_origin
    budget_before = latch.side_budget_used_rad
    far_x = origin[0] + 0.05
    t = 0.2
    latch.apply(_clear(), t, far_x, origin[1])
    latch.apply(_clear(), t + 0.15, far_x, origin[1])
    assert latch.side_phase == "RECOVERY_ALLOWED"
    d = latch.apply(_narrow(), t + 0.20, far_x, origin[1])
    assert latch.side_phase == "CAPPED_BYPASS"
    assert latch.side_origin == origin
    assert latch.side_budget_used_rad == budget_before
    # LOCAL_NARROW keeps its own cautious decision (not the faster
    # LOCAL_SIDE_BYPASS speed) — its own raw form already has zero angular,
    # so no turn is resumed either way.
    assert d.mode == "LOCAL_NARROW"
    assert d.angular_rps == 0.0


# 9. After an interruption sends the phase back to CAPPED_BYPASS, a flicker that
#    never re-confirms must still reach FAILSAFE at max_bypass_extension_m.
def test_reentered_capped_bypass_flicker_reaches_failsafe():
    latch = _reach_capped_bypass(
        local_bypass_distance_m=0.01, side_clear_confirm_s=0.1, max_bypass_extension_m=0.15
    )
    origin = latch.side_origin
    far_x = origin[0] + 0.05
    t = 0.2
    latch.apply(_clear(), t, far_x, origin[1])
    latch.apply(_clear(), t + 0.15, far_x, origin[1])
    assert latch.side_phase == "RECOVERY_ALLOWED"
    latch.apply(_left_side(), t + 0.20, far_x, origin[1])
    assert latch.side_phase == "CAPPED_BYPASS"

    tt, x = t + 0.20, far_x
    toggle = True
    reached_failsafe = False
    for _ in range(500):
        tt += 0.05
        x += 0.01
        latch.apply(_clear() if toggle else _left_side(), tt, x, origin[1])
        toggle = not toggle
        if latch.side_phase == "FAILSAFE":
            reached_failsafe = True
            break
    assert reached_failsafe


# 10. Uninterrupted RECOVERY_ALLOWED emits exactly one LOCAL_RECOVERY_READY
#     pulse (on entry); every later tick is a plain inactive decision.
def test_recovery_allowed_pulses_signal_exactly_once():
    latch = _reach_capped_bypass(
        local_bypass_distance_m=0.01, side_clear_confirm_s=0.1, rearm_quiet_s=0.3
    )
    origin = latch.side_origin
    far_x = origin[0] + 0.05
    t = 0.2
    latch.apply(_clear(), t, far_x, origin[1])
    d_pulse = latch.apply(_clear(), t + 0.15, far_x, origin[1])
    assert latch.side_phase == "RECOVERY_ALLOWED"
    assert d_pulse.mode == "LOCAL_RECOVERY_READY"

    tt = t + 0.15
    closed = False
    for _ in range(50):
        tt += 0.05
        d = latch.apply(_clear(), tt, far_x, origin[1])
        assert d.mode != "LOCAL_RECOVERY_READY"
        assert d.active is False
        if latch.side_phase == "CLOSED":
            closed = True
            break
    assert closed


# 12. Front preemption mid-CAPPED_BYPASS and mid-RECOVERY_ALLOWED leaves origin,
#     budget, quiet timer and phase byte-identical.
def test_front_preemption_leaves_side_state_untouched_in_capped_bypass():
    latch = _reach_capped_bypass(local_bypass_distance_m=0.01)
    origin_snapshot = latch.side_origin
    budget_snapshot = latch.side_budget_used_rad
    phase_snapshot = latch.side_phase
    quiet_snapshot = latch.side_quiet_since_s
    d = latch.apply(_front_danger(), 999.0, 5.0, 5.0)
    assert d.mode == "LOCAL_FRONT_DANGER"
    assert latch.side_origin == origin_snapshot
    assert latch.side_budget_used_rad == budget_snapshot
    assert latch.side_phase == phase_snapshot
    assert latch.side_quiet_since_s == quiet_snapshot


def test_front_preemption_leaves_side_state_untouched_in_recovery_allowed():
    latch, x, y = _reach_recovery_allowed(local_bypass_distance_m=0.01, side_clear_confirm_s=0.1)
    quiet_snapshot = latch.side_quiet_since_s
    d = latch.apply(_front_warn(), 999.0, 5.0, 5.0)
    assert d.mode == "LOCAL_FRONT_WARN"
    assert latch.side_phase == "RECOVERY_ALLOWED"
    assert latch.side_quiet_since_s == quiet_snapshot


# 13. LOCAL_FRONT_DANGER/LOCAL_FRONT_WARN/LOCAL_SENSOR_INVALID immediately
#     pre-empt every side phase.
def test_sensor_invalid_preempts_turning():
    latch = LocalAvoidanceLatch()
    latch.apply(_left_side(), 0.0, 0.0, 0.0)
    assert latch.side_phase == "TURNING"
    d = latch.apply(_sensor_invalid(), 0.05, 0.0, 0.0)
    assert d.safety_stop is True
    assert d.mode == "LOCAL_SENSOR_INVALID"
    assert latch.side_phase == "TURNING"


def test_sensor_invalid_preempts_capped_bypass():
    latch = _reach_capped_bypass()
    d = latch.apply(_sensor_invalid(), 999.0, 0.0, 0.0)
    assert d.safety_stop is True
    assert latch.side_phase == "CAPPED_BYPASS"


def test_sensor_invalid_preempts_recovery_allowed():
    latch, x, y = _reach_recovery_allowed()
    d = latch.apply(_sensor_invalid(), 999.0, x, y)
    assert d.safety_stop is True
    assert latch.side_phase == "RECOVERY_ALLOWED"


def test_sensor_invalid_preempts_failsafe():
    latch = _reach_failsafe()
    d = latch.apply(_sensor_invalid(), 999.0, 0.0, 0.0)
    assert d.safety_stop is True
    assert d.mode == "LOCAL_SENSOR_INVALID"
    assert latch.side_phase == "FAILSAFE"


def test_front_danger_preempts_failsafe_without_clearing_it():
    latch = _reach_failsafe()
    d = latch.apply(_front_danger(), 999.0, 0.0, 0.0)
    assert d.mode == "LOCAL_FRONT_DANGER"
    assert latch.side_phase == "FAILSAFE"


# 14. Cumulative side turn across multiple CAPPED_BYPASS <-> RECOVERY_ALLOWED
#     interruption cycles never exceeds the budget.
def test_cumulative_turn_never_exceeds_budget_across_interruptions():
    max_turn = 0.05
    latch = LocalAvoidanceLatch(
        max_side_encounter_turn_rad=max_turn,
        local_bypass_distance_m=0.01,
        side_clear_confirm_s=0.1,
    )
    t = 0.0
    latch.apply(_left_side(), t, 0.0, 0.0)
    t += 0.5
    latch.apply(_left_side(), t, 0.0, 0.0)
    assert latch.side_phase == "CAPPED_BYPASS"
    assert latch.side_budget_used_rad <= max_turn + 1e-9

    origin = latch.side_origin
    far_x = origin[0] + 0.05
    for _ in range(3):
        t += 0.2
        latch.apply(_clear(), t, far_x, origin[1])
        t += 0.15
        latch.apply(_clear(), t, far_x, origin[1])
        if latch.side_phase != "RECOVERY_ALLOWED":
            break
        t += 0.05
        latch.apply(_left_side(), t, far_x, origin[1])
        assert latch.side_budget_used_rad <= max_turn + 1e-9
    assert latch.side_budget_used_rad <= max_turn + 1e-9


# 15. Rearm requires an uninterrupted quiet run of rearm_quiet_s; any
#     interruption anywhere restarts the requirement from zero.
def test_rearm_requires_uninterrupted_quiet():
    latch, x, y = _reach_recovery_allowed(
        local_bypass_distance_m=0.01, side_clear_confirm_s=0.1, rearm_quiet_s=0.3
    )
    quiet_start = latch.side_quiet_since_s

    latch.apply(_left_side(), quiet_start + 0.25, x, y)
    assert latch.side_phase == "CAPPED_BYPASS"

    latch.apply(_clear(), quiet_start + 0.30, x, y)
    latch.apply(_clear(), quiet_start + 0.45, x, y)
    assert latch.side_phase == "RECOVERY_ALLOWED"
    new_quiet_start = latch.side_quiet_since_s

    latch.apply(_clear(), new_quiet_start + 0.35, x, y)
    assert latch.side_phase == "CLOSED"


# hysteresis_hint(): returns the locked side mode through every side phase,
# "" only once CLOSED (regression test for the previous_mode structural bug).
def test_hysteresis_hint_covers_every_side_phase():
    latch = LocalAvoidanceLatch(
        max_side_encounter_turn_rad=0.01,
        local_bypass_distance_m=0.01,
        side_clear_confirm_s=0.1,
        rearm_quiet_s=0.2,
    )
    assert latch.hysteresis_hint() == ""
    latch.apply(_left_side(), 0.0, 0.0, 0.0)
    assert latch.hysteresis_hint() == "LOCAL_LEFT_SIDE"
    latch.apply(_left_side(), 0.5, 0.0, 0.0)
    assert latch.side_phase == "CAPPED_BYPASS"
    assert latch.hysteresis_hint() == "LOCAL_LEFT_SIDE"
    origin = latch.side_origin
    far_x = origin[0] + 0.05
    latch.apply(_clear(), 0.6, far_x, origin[1])
    latch.apply(_clear(), 0.75, far_x, origin[1])
    assert latch.side_phase == "RECOVERY_ALLOWED"
    assert latch.hysteresis_hint() == "LOCAL_LEFT_SIDE"
    latch.apply(_clear(), 0.75 + 0.25, far_x, origin[1])
    assert latch.side_phase == "CLOSED"
    assert latch.hysteresis_hint() == ""


# dt clamp: a large gap between consecutive now_s values does not produce an
# outsized budget increment.
def test_dt_clamp_bounds_budget_increment_after_a_missed_tick():
    latch = LocalAvoidanceLatch(max_side_encounter_turn_rad=100.0)
    latch.apply(_left_side(angular_rps=-0.30), 0.0, 0.0, 0.0)
    latch.apply(_left_side(angular_rps=-0.30), 500.0, 0.0, 0.0)
    # dt is clamped to 0.20s regardless of the 500s gap between calls.
    assert latch.side_budget_used_rad == pytest.approx(0.30 * 0.20, abs=1e-9)


# LOCAL_NARROW mid-encounter (TURNING/CAPPED_BYPASS): passes through
# unmodified, resets the quiet clock like an active tick, never advances the
# turn budget, never changes phase.
def test_narrow_mid_capped_bypass_resets_quiet_without_changing_phase_or_budget():
    latch = _reach_capped_bypass(local_bypass_distance_m=0.01)
    budget_before = latch.side_budget_used_rad
    origin = latch.side_origin
    d = latch.apply(_narrow(), 0.5, origin[0], origin[1])
    assert d.mode == "LOCAL_NARROW"
    assert latch.side_phase == "CAPPED_BYPASS"
    assert latch.side_budget_used_rad == budget_before
    assert latch.side_quiet_since_s is None
