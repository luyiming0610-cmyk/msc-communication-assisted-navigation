"""controller_v4_full_sensor_bypass_20260717 EncounterAvoidanceV4 tests.

Covers the pilot_a3-motivated redesign: the DETECT_TURN -> SIDE_TRACK ->
PASS_CONFIRM -> RECOVERY_ALLOWED -> CLOSED state machine, its FAILSAFE hard
latch, and -- most importantly -- that recovery is gated on the FULL joint
condition set (front->mid->rear sensor sequence AND encounter-local lateral
offset AND longitudinal progress AND a continuous clear-confirm hold), never
any single one of those alone. This is what pilot_a2/pilot_a3 (v2/v3) never
enforced.
"""

import math

import pytest

from epuck2_comm.local_obstacle_logic import (
    EncounterAvoidanceV4,
    LocalObstacleDecision,
    ZoneSnapshot,
    decide_local_obstacle,
    normalize_angle,
)


INF = math.inf


def _zones(**overrides):
    base = dict(
        left_front_m=INF, left_mid_m=INF, left_rear_m=INF,
        right_front_m=INF, right_mid_m=INF, right_rear_m=INF,
    )
    base.update(overrides)
    return ZoneSnapshot(**base)


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


class _Robot:
    """Tiny unicycle model driven by whatever the latch commands, so tests
    can drive a real trajectory instead of hand-picking x/y/yaw."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw

    def step(self, linear, angular, dt):
        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt
        self.yaw = normalize_angle(self.yaw + angular * dt)


def _drive_full_pass(latch, robot, t, dt=0.05, side="LEFT", danger_m=0.042):
    """Script a full, honest DETECT_TURN -> SIDE_TRACK -> PASS_CONFIRM pass
    on the given side: raw side trigger, tracking zone sweeping
    front -> mid -> rear then clearing, sufficient odometry displacement.
    Returns (final_decision, t)."""
    raw_side = _left_side() if side == "LEFT" else _right_side()
    zone_names = ("left_front_m", "left_mid_m", "left_rear_m") if side == "LEFT" else (
        "right_front_m", "right_mid_m", "right_rear_m"
    )

    # DETECT_TURN: raw front, then raw side triggers.
    t += dt
    d = latch.apply(_front_warn(angular_rps=-0.45 if side == "LEFT" else 0.45), _zones(), t, robot.x, robot.y, robot.yaw)
    robot.step(d.linear_mps, d.angular_rps, dt)

    t += dt
    d = latch.apply(raw_side, _zones(), t, robot.x, robot.y, robot.yaw)
    robot.step(d.linear_mps, d.angular_rps, dt)
    assert latch.phase == "SIDE_TRACK"

    # SIDE_TRACK: sweep front -> mid -> rear on the tracking zone while
    # still raw-side-active, then raw side clears.
    for zone_value in (0.045, INF):
        for name in zone_names:
            t += dt
            z = _zones(**{name: zone_value})
            d = latch.apply(raw_side, z, t, robot.x, robot.y, robot.yaw)
            robot.step(d.linear_mps, d.angular_rps, dt)

    # Raw side clears: keep ticking with the robot physically clear of all
    # tracking zones and enough forward+lateral travel accumulated.
    for _ in range(40):
        t += dt
        # Nudge the robot along its own heading to build genuine
        # longitudinal + lateral displacement independent of the latch's
        # own (small) commanded speed, keeping the test's geometry honest
        # without needing hundreds of ticks.
        robot.x += 0.006 * math.cos(robot.yaw + (1.2 if side == "LEFT" else -1.2))
        robot.y += 0.006 * math.sin(robot.yaw + (1.2 if side == "LEFT" else -1.2))
        d = latch.apply(_clear(), _zones(), t, robot.x, robot.y, robot.yaw)
        if d.mode == "LOCAL_RECOVERY_READY":
            return d, t
    return d, t


def test_yaw_ledger_handles_pi_wraparound():
    latch = EncounterAvoidanceV4(max_turn_ledger_rad=100.0)
    robot = _Robot(yaw=3.10)
    t = 0.0
    d = latch.apply(_left_side(), _zones(), t, robot.x, robot.y, robot.yaw)
    robot.yaw = normalize_angle(robot.yaw + 0.15)  # crosses +pi to negative
    t += 0.05
    latch.apply(_left_side(), _zones(), t, robot.x, robot.y, robot.yaw)
    assert latch.turn_ledger_used_rad == pytest.approx(0.15, abs=1e-6)


def test_front_danger_forces_zero_linear_even_though_v1_warn_allowed_creep():
    latch = EncounterAvoidanceV4()
    d = latch.apply(_front_warn(), _zones(), 0.0, 0.0, 0.0, 0.0)
    assert latch.phase == "DETECT_TURN"
    assert d.linear_mps == 0.0
    assert d.angular_rps < 0.0


def test_detect_turn_continues_in_place_after_front_clears_without_side_trigger():
    latch = EncounterAvoidanceV4(max_inplace_turn_rad=10.0)
    latch.apply(_front_warn(), _zones(), 0.0, 0.0, 0.0, 0.0)
    d = latch.apply(_clear(), _zones(), 0.05, 0.0, 0.0, -0.1)
    assert latch.phase == "DETECT_TURN"
    assert d.mode == "LOCAL_DETECT_TURN"
    assert d.linear_mps == 0.0


def test_full_front_mid_rear_sequence_reaches_recovery_ready():
    latch = EncounterAvoidanceV4(
        required_lateral_offset_m=0.05, required_longitudinal_progress_m=0.03,
        pass_confirm_hold_s=0.2,
    )
    robot = _Robot()
    d, _ = _drive_full_pass(latch, robot, 0.0, side="LEFT")
    assert d.mode == "LOCAL_RECOVERY_READY"
    assert latch.rear_seen and latch.mid_seen and latch.front_seen


def test_never_seeing_rear_zone_blocks_recovery():
    latch = EncounterAvoidanceV4(
        required_lateral_offset_m=0.02, required_longitudinal_progress_m=0.01,
        pass_confirm_hold_s=0.1,
    )
    robot = _Robot()
    t = 0.0
    t += 0.05
    d = latch.apply(_left_side(), _zones(), t, robot.x, robot.y, robot.yaw)
    robot.step(d.linear_mps, d.angular_rps, 0.05)
    assert latch.phase == "SIDE_TRACK"
    # Only ever show front+mid, never rear -- plenty of lateral/longitudinal
    # travel and continuous raw-clear ticks, but rear_seen stays False.
    for name in ("left_front_m", "left_mid_m"):
        t += 0.05
        latch.apply(_left_side(), _zones(**{name: 0.045}), t, robot.x, robot.y, robot.yaw)
    for _ in range(60):
        t += 0.05
        robot.x += 0.01
        robot.y -= 0.01
        d = latch.apply(_clear(), _zones(), t, robot.x, robot.y, robot.yaw)
        assert d.mode != "LOCAL_RECOVERY_READY"
    assert not latch.rear_seen


def test_front_clearing_alone_does_not_grant_recovery():
    latch = EncounterAvoidanceV4(max_inplace_turn_rad=10.0, max_turn_ledger_rad=100.0)
    latch.apply(_front_warn(), _zones(), 0.0, 0.0, 0.0, 0.0)
    for i in range(1, 30):
        d = latch.apply(_clear(), _zones(), i * 0.05, 0.0, 0.0, -0.01 * i)
        assert d.mode != "LOCAL_RECOVERY_READY"
        assert latch.phase in ("DETECT_TURN", "SIDE_TRACK")


def test_fixed_distance_alone_does_not_grant_recovery_without_rear_or_offset():
    latch = EncounterAvoidanceV4(
        required_lateral_offset_m=0.5,  # deliberately unreachable in this test
        required_longitudinal_progress_m=0.05,
        pass_confirm_hold_s=0.1,
    )
    robot = _Robot()
    t = 0.05
    latch.apply(_left_side(), _zones(), t, robot.x, robot.y, robot.yaw)
    for _ in range(60):
        t += 0.05
        robot.x += 0.01  # plenty of forward travel, ~0.6m
        d = latch.apply(_clear(), _zones(), t, robot.x, robot.y, robot.yaw)
        assert d.mode != "LOCAL_RECOVERY_READY"


def test_lateral_offset_alone_does_not_grant_recovery_without_rear_seen():
    latch = EncounterAvoidanceV4(
        required_lateral_offset_m=0.05, required_longitudinal_progress_m=0.5,
        pass_confirm_hold_s=0.1,
    )
    robot = _Robot()
    t = 0.05
    latch.apply(_left_side(), _zones(), t, robot.x, robot.y, robot.yaw)
    for _ in range(60):
        t += 0.05
        robot.y += 0.01  # plenty of lateral travel, but no longitudinal and no rear
        d = latch.apply(_clear(), _zones(), t, robot.x, robot.y, robot.yaw)
        assert d.mode != "LOCAL_RECOVERY_READY"
    assert not latch.rear_seen


def test_reopening_after_recovery_interrupt_reverts_to_side_track_not_fresh_ledger():
    latch = EncounterAvoidanceV4(
        required_lateral_offset_m=0.05, required_longitudinal_progress_m=0.03,
        pass_confirm_hold_s=0.1,
    )
    robot = _Robot()
    d, t = _drive_full_pass(latch, robot, 0.0, side="LEFT")
    assert d.mode == "LOCAL_RECOVERY_READY"
    assert latch.phase == "RECOVERY_ALLOWED"
    ledger_before = latch.turn_ledger_used_rad

    t += 0.05
    d = latch.apply(_left_side(), _zones(left_front_m=0.045), t, robot.x, robot.y, robot.yaw)
    assert latch.phase == "SIDE_TRACK"
    assert d.mode != "LOCAL_RECOVERY_READY"
    # No fresh ledger: continues accumulating from where it was, never reset
    # to 0.
    assert latch.turn_ledger_used_rad >= ledger_before
    assert not latch.rear_seen  # re-require a full PASS_CONFIRM before retrying


def test_right_side_mirrors_left_side():
    latch = EncounterAvoidanceV4(
        required_lateral_offset_m=0.05, required_longitudinal_progress_m=0.03,
        pass_confirm_hold_s=0.2,
    )
    robot = _Robot()
    d, _ = _drive_full_pass(latch, robot, 0.0, side="RIGHT")
    assert d.mode == "LOCAL_RECOVERY_READY"
    assert latch.tracking_side == "RIGHT"


@pytest.mark.parametrize("start_yaw", [0.0, 1.2, -1.2, 3.05, -3.05, math.pi - 0.05, -math.pi + 0.05])
def test_lateral_offset_projection_correct_for_arbitrary_initial_yaw(start_yaw):
    latch = EncounterAvoidanceV4()
    latch.apply(_left_side(), _zones(), 0.0, 0.0, 0.0, start_yaw)
    assert latch.encounter_start_yaw == pytest.approx(start_yaw, abs=1e-9)
    # Move exactly 0.10m purely "sideways" relative to the encounter start
    # heading (perpendicular, +90deg) -- lateral must read ~0.10, longitudinal ~0.
    perp = start_yaw + math.pi / 2.0
    own_x = 0.10 * math.cos(perp)
    own_y = 0.10 * math.sin(perp)
    longitudinal, lateral = latch._encounter_local_offset(own_x, own_y)
    assert longitudinal == pytest.approx(0.0, abs=1e-6)
    assert abs(lateral) == pytest.approx(0.10, abs=1e-6)


def test_failsafe_latches_on_turn_ledger_ceiling_and_never_auto_exits():
    latch = EncounterAvoidanceV4(max_turn_ledger_rad=0.2, max_inplace_turn_rad=10.0)
    t = 0.0
    latch.apply(_left_side(), _zones(), t, 0.0, 0.0, 0.0)
    for i in range(1, 20):
        t = i * 0.05
        latch.apply(_left_side(), _zones(), t, 0.0, 0.0, -0.05 * i)
        if latch.phase == "FAILSAFE":
            break
    assert latch.phase == "FAILSAFE"
    # Even a fully clear, quiet tick much later must stay latched.
    d = latch.apply(_clear(), _zones(), t + 100.0, 0.0, 0.0, 0.0)
    assert d.mode == "LOCAL_ENCOUNTER_FAILSAFE"
    assert d.safety_stop
    assert d.linear_mps == 0.0 and d.angular_rps == 0.0


def test_failsafe_latches_on_bypass_extension_distance_ceiling():
    latch = EncounterAvoidanceV4(max_bypass_extension_m=0.05, max_turn_ledger_rad=100.0)
    t = 0.0
    latch.apply(_left_side(), _zones(), t, 0.0, 0.0, 0.0)
    t += 0.05
    d = latch.apply(_left_side(), _zones(), t, 0.10, 0.0, 0.0)
    assert d.mode == "LOCAL_ENCOUNTER_FAILSAFE"


def test_failsafe_latches_on_duration_ceiling():
    latch = EncounterAvoidanceV4(max_encounter_duration_s=1.0, max_turn_ledger_rad=100.0)
    latch.apply(_left_side(), _zones(), 0.0, 0.0, 0.0, 0.0)
    d = latch.apply(_left_side(), _zones(), 1.5, 0.0, 0.0, 0.0)
    assert d.mode == "LOCAL_ENCOUNTER_FAILSAFE"


def test_sensor_invalid_preempts_every_phase():
    latch = EncounterAvoidanceV4()
    latch.apply(_left_side(), _zones(), 0.0, 0.0, 0.0, 0.0)
    d = latch.apply(_sensor_invalid(), _zones(), 0.05, 0.0, 0.0, 0.0)
    assert d.mode == "LOCAL_SENSOR_INVALID"
    assert d.safety_stop


def test_narrow_triggers_detect_turn_and_side_track_same_as_side():
    latch = EncounterAvoidanceV4(max_inplace_turn_rad=10.0)
    latch.apply(_front_warn(), _zones(), 0.0, 0.0, 0.0, 0.0)
    d = latch.apply(_narrow(), _zones(), 0.05, 0.0, 0.0, -0.1)
    assert latch.phase == "SIDE_TRACK"
    assert d.mode == "LOCAL_SIDE_TRACK"


def test_tracking_zone_danger_band_forces_zero_linear_same_tick():
    latch = EncounterAvoidanceV4()
    latch.apply(_left_side(), _zones(), 0.0, 0.0, 0.0, 0.0)
    d = latch.apply(_left_side(), _zones(left_mid_m=0.03), 0.05, 0.0, 0.0, 0.0)
    assert d.mode == "LOCAL_SIDE_TRACK"
    assert d.linear_mps == 0.0


def test_hysteresis_hint_covers_whole_encounter():
    latch = EncounterAvoidanceV4(max_inplace_turn_rad=10.0)
    assert latch.hysteresis_hint() == ""
    latch.apply(_left_side(), _zones(), 0.0, 0.0, 0.0, 0.0)
    assert latch.hysteresis_hint() == "LOCAL_LEFT_SIDE"
    latch.apply(_clear(), _zones(), 0.05, 0.0, 0.0, -0.02)
    assert latch.hysteresis_hint() == "LOCAL_LEFT_SIDE"


def test_nan_and_negative_zone_values_treated_as_no_detection():
    latch = EncounterAvoidanceV4()
    latch.apply(_left_side(), _zones(), 0.0, 0.0, 0.0, 0.0)
    z = _zones(left_front_m=float("nan"), left_mid_m=-0.01)
    d = latch.apply(_left_side(), z, 0.05, 0.0, 0.0, 0.0)
    # Must not crash, and must not be treated as a valid close detection.
    assert d.mode == "LOCAL_SIDE_TRACK"
    assert not latch.front_seen
    assert not latch.mid_seen
