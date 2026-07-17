import math

from epuck2_comm.collision_math import (
    CpaResult,
    closest_point_of_approach,
    collision_risk,
    local_to_global,
    right_turn_target_reached,
)


def test_robot_one_origin_transform():
    x_m, y_m, yaw_rad = local_to_global(0.10, 0.0, 0.0, -0.35, 0.0, 0.0)
    assert math.isclose(x_m, -0.25, abs_tol=1e-9)
    assert math.isclose(y_m, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw_rad, 0.0, abs_tol=1e-9)


def test_robot_two_origin_transform_rotates_local_forward():
    x_m, y_m, yaw_rad = local_to_global(0.10, 0.0, 0.0, 0.35, 0.0, math.pi)
    assert math.isclose(x_m, 0.25, abs_tol=1e-9)
    assert math.isclose(y_m, 0.0, abs_tol=1e-9)
    assert math.isclose(abs(yaw_rad), math.pi, abs_tol=1e-9)


def test_head_on_cpa_predicts_collision():
    result = closest_point_of_approach(
        -0.35, 0.0, 0.025, 0.0,
        0.35, 0.0, -0.025, 0.0,
        20.0,
    )
    assert math.isclose(result.current_distance_m, 0.70, abs_tol=1e-9)
    assert math.isclose(result.time_to_cpa_s, 14.0, abs_tol=1e-9)
    assert math.isclose(result.distance_at_cpa_m, 0.0, abs_tol=1e-9)
    assert math.isclose(result.closing_speed_mps, 0.05, abs_tol=1e-9)


def test_parallel_motion_has_no_closing_speed():
    result = closest_point_of_approach(
        0.0, 0.0, 0.02, 0.0,
        0.0, 0.20, 0.02, 0.0,
        4.0,
    )
    assert result.closing_speed_mps == 0.0
    assert result.time_to_cpa_s == 0.0
    assert math.isclose(result.distance_at_cpa_m, 0.20, abs_tol=1e-9)


def test_approaching_pair_triggers_risk():
    result = CpaResult(0.30, 0.04, 2.0, 0.08)
    assert collision_risk(
        result,
        horizon_s=4.0,
        safety_radius_m=0.14,
        trigger_distance_m=0.34,
    )


def test_separating_pair_does_not_retrigger_inside_distance_threshold():
    result = CpaResult(0.24, 0.0, 0.0, 0.24)
    assert not collision_risk(
        result,
        horizon_s=4.0,
        safety_radius_m=0.14,
        trigger_distance_m=0.34,
    )


def test_right_turn_target_accepts_sample_inside_tolerance():
    assert right_turn_target_reached(-0.12, -0.079, tolerance_rad=0.08)


def test_right_turn_target_detects_skipped_tolerance_band():
    assert right_turn_target_reached(-0.115, 0.285, tolerance_rad=0.08)


def test_right_turn_target_rejects_motion_before_target():
    assert not right_turn_target_reached(-0.45, -0.20, tolerance_rad=0.08)
