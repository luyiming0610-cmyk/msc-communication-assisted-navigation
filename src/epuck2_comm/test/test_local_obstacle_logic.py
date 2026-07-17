import math

from epuck2_comm.local_obstacle_logic import (
    LocalAvoidanceLatch,
    decide_local_obstacle,
)


VALID_ALL = 1 | 2 | 4


def test_invalid_local_sensors_request_safe_stop():
    decision = decide_local_obstacle(math.inf, math.inf, math.inf, 1)
    assert decision.safety_stop
    assert decision.mode == "LOCAL_SENSOR_INVALID"


def test_clear_space_does_not_override_cooperative_policy():
    decision = decide_local_obstacle(math.inf, math.inf, math.inf, VALID_ALL)
    assert not decision.active
    assert decision.mode == "LOCAL_CLEAR"


def test_centered_front_danger_uses_deterministic_pass_right():
    decision = decide_local_obstacle(0.07, 0.04, 0.04, VALID_ALL)
    assert decision.mode == "LOCAL_FRONT_DANGER"
    assert decision.linear_mps == 0.0
    assert decision.angular_rps < 0.0


def test_left_front_obstacle_turns_right():
    decision = decide_local_obstacle(0.14, 0.03, 0.055, VALID_ALL)
    assert decision.mode == "LOCAL_FRONT_WARN"
    assert decision.angular_rps < 0.0


def test_right_front_obstacle_turns_left():
    decision = decide_local_obstacle(0.14, 0.055, 0.03, VALID_ALL)
    assert decision.mode == "LOCAL_FRONT_WARN"
    assert decision.angular_rps > 0.0


def test_front_hysteresis_holds_warning_until_release_distance():
    decision = decide_local_obstacle(
        0.20, math.inf, math.inf, VALID_ALL, previous_mode="LOCAL_FRONT_WARN"
    )
    assert decision.mode == "LOCAL_FRONT_WARN"


def test_left_side_obstacle_turns_right():
    decision = decide_local_obstacle(math.inf, 0.05, math.inf, VALID_ALL)
    assert decision.mode == "LOCAL_LEFT_SIDE"
    assert decision.angular_rps < 0.0


def test_tof_only_front_detection_remains_available():
    decision = decide_local_obstacle(0.09, 0.01, 0.01, 1 | 4)
    assert decision.mode == "LOCAL_FRONT_DANGER"
    assert decision.angular_rps < 0.0


def test_latch_holds_turn_direction_during_short_range_dropout():
    latch = LocalAvoidanceLatch(clear_hold_s=1.0)
    detected = decide_local_obstacle(0.12, math.inf, math.inf, VALID_ALL)
    assert latch.apply(detected, 10.0, 0.0, 0.0, 0.0).mode == "LOCAL_FRONT_WARN"
    clear = decide_local_obstacle(math.inf, math.inf, math.inf, VALID_ALL)
    held = latch.apply(clear, 10.5, 0.0, 0.0, 0.0)
    assert held.mode == "LOCAL_CLEARANCE"
    assert held.angular_rps < 0.0


def test_latch_releases_after_clear_hold_interval():
    latch = LocalAvoidanceLatch(clear_hold_s=1.0)
    detected = decide_local_obstacle(0.12, math.inf, math.inf, VALID_ALL)
    latch.apply(detected, 10.0, 0.0, 0.0, 0.0)
    clear = decide_local_obstacle(math.inf, math.inf, math.inf, VALID_ALL)
    released = latch.apply(clear, 11.1, 0.0, 0.0, 0.0)
    assert not released.active
