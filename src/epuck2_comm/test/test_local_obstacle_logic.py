"""controller_v1-era decide_local_obstacle() regression tests.

controller_v4_full_sensor_bypass_20260717: the two LocalAvoidanceLatch smoke
tests formerly here (test_latch_holds_turn_direction_during_short_range_dropout,
test_latch_releases_after_clear_hold_interval) are retired -- LocalAvoidanceLatch
(controller_v3's class) no longer exists; EncounterAvoidanceV4 replaces it with
a materially different phase set and apply() signature. Equivalent-purpose
coverage (turn-direction persistence through a raw dropout, and a clean close
with no encounter ever opening) now lives in
test_encounter_avoidance_v4.py::test_side_track_holds_turn_direction_through_raw_dropout
and ::test_no_raw_trigger_never_opens_an_encounter. The old v3 tests remain
recoverable via `git show d2ef811:src/epuck2_comm/test/test_local_obstacle_logic.py`.

decide_local_obstacle() itself is unchanged since controller_v1 -- every test
below is unmodified in intent across v1/v2/v3/v4.
"""

import math

from epuck2_comm.local_obstacle_logic import decide_local_obstacle


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
