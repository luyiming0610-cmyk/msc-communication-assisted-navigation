import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from task_completion_analyzer import (
    GoalRegion,
    all_robots_goal_results,
    all_robots_reached_goal,
    build_task_verdict,
    cumulative_absolute_heading_change_rad,
    makespan_s,
    overall_minimum_pairwise_distance_m,
    pairwise_min_distance,
    path_length_m,
    robot_goal_completion_time,
    stop_duration_s,
)

GOAL = GoalRegion(center_x_m=2.0, center_y_m=0.0, radius_m=0.3)


# --- required test 1: goal判定不能被单帧进入误触发 ---
def test_single_frame_dip_into_goal_does_not_trigger_success():
    hold_s = 1.0
    # GOAL is centered at x=2.0 with radius=0.3, so x=1.0 is well outside
    # (distance 1.0) and x=2.0 is the center (well inside).
    samples = [
        (0.0, 1.0, 0.0, 0.0, 0.02),
        (1.0, 1.0, 0.0, 0.0, 0.02),
        (1.1, 2.0, 0.0, 0.0, 0.02),   # single frame inside
        (1.2, 1.0, 0.0, 0.0, 0.02),   # exits again immediately
        (2.0, 1.0, 0.0, 0.0, 0.02),
    ]
    completion, reason = robot_goal_completion_time(samples, GOAL, hold_s)
    assert completion is None
    assert reason == "NEVER_HELD_GOAL_FOR_REQUIRED_DURATION"


def test_sustained_entry_for_full_hold_time_does_trigger_success():
    hold_s = 1.0
    samples = [
        (0.0, 0.0, 0.0, 0.0, 0.02),
        (5.0, 2.0, 0.0, 0.0, 0.02),   # enters and stays
        (5.5, 2.05, 0.0, 0.0, 0.0),
        (6.0, 2.0, 0.0, 0.0, 0.0),
        (6.1, 1.95, 0.0, 0.0, 0.0),
    ]
    completion, reason = robot_goal_completion_time(samples, GOAL, hold_s)
    assert reason is None
    assert completion == 6.0  # entered at 5.0, held hold_s=1.0 -> 6.0


def test_gap_resets_the_hold_timer_not_accumulates_across_it():
    hold_s = 1.0
    samples = [
        (0.0, 2.0, 0.0, 0.0, 0.0),   # inside 0.0s
        (0.6, 5.0, 0.0, 0.0, 0.0),   # leaves before hold_s satisfied
        (0.7, 2.0, 0.0, 0.0, 0.0),   # re-enters -- clock must restart here
        (1.5, 2.0, 0.0, 0.0, 0.0),   # only 0.8s since re-entry -- not yet
        (1.7, 2.0, 0.0, 0.0, 0.0),   # 1.0s since re-entry (0.7->1.7) -- now satisfied
    ]
    completion, reason = robot_goal_completion_time(samples, GOAL, hold_s)
    assert reason is None
    assert abs(completion - 1.7) < 1e-9


def test_no_samples_is_not_measurable_not_a_silent_failure_or_zero():
    completion, reason = robot_goal_completion_time([], GOAL, 1.0)
    assert completion is None
    assert "NOT_MEASURABLE" in reason


# --- required test 2: 所有机器人而非任意一台到达才成功 ---
def test_all_robots_required_not_any_single_robot():
    good = [(0.0, 2.0, 0.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0, 0.0)]
    never_arrives = [(0.0, 0.0, 0.0, 0.0, 0.0), (2.0, 0.1, 0.0, 0.0, 0.0)]
    per_robot = {"epuck1": good, "epuck2": never_arrives, "epuck3": good}
    results = all_robots_goal_results(per_robot, GOAL, hold_time_s=1.0)
    assert all_robots_reached_goal(results) is False
    # 2 of 3 individually reached, but overall success requires ALL 3.
    assert sum(r.reached for r in results) == 2


def test_all_robots_reached_is_true_only_when_every_one_holds():
    good_a = [(0.0, 2.0, 0.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0, 0.0)]
    good_b = [(0.0, 2.0, 0.1, 0.0, 0.0), (2.0, 2.0, 0.1, 0.0, 0.0)]
    per_robot = {"epuck1": good_a, "epuck2": good_b}
    results = all_robots_goal_results(per_robot, GOAL, hold_time_s=1.0)
    assert all_robots_reached_goal(results) is True
    assert makespan_s(results) is not None


def test_makespan_is_none_when_not_all_robots_succeeded():
    good = [(0.0, 2.0, 0.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0, 0.0)]
    bad = [(0.0, 0.0, 0.0, 0.0, 0.0), (2.0, 0.1, 0.0, 0.0, 0.0)]
    results = all_robots_goal_results({"epuck1": good, "epuck2": bad}, GOAL, 1.0)
    assert makespan_s(results) is None


def test_makespan_equals_the_slowest_robots_completion_time():
    fast = [(0.0, 2.0, 0.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0, 0.0)]
    slow = [(0.0, 0.0, 0.0, 0.0, 0.02), (9.0, 2.0, 0.0, 0.0, 0.0), (11.0, 2.0, 0.0, 0.0, 0.0)]
    results = all_robots_goal_results({"fast": fast, "slow": slow}, GOAL, hold_time_s=1.0)
    assert all_robots_reached_goal(results) is True
    fast_time = next(r.completion_time_s for r in results if r.robot_id == "fast")
    slow_time = next(r.completion_time_s for r in results if r.robot_id == "slow")
    assert fast_time == 1.0  # entered at t=0, held 1.0s -> completes at 1.0
    assert slow_time == 10.0  # entered at t=9, held 1.0s -> completes at 10.0
    assert makespan_s(results) == 10.0


# --- required test 3: pairwise距离覆盖所有机器人组合 ---
def test_pairwise_distance_covers_all_combinations_for_n3():
    per_robot = {
        "e1": [(0.0, 0.0, 0.0, 0.0, 0.0)],
        "e2": [(0.0, 1.0, 0.0, 0.0, 0.0)],
        "e3": [(0.0, 0.0, 1.0, 0.0, 0.0)],
    }
    pairwise = pairwise_min_distance(per_robot)
    assert set(pairwise.keys()) == {("e1", "e2"), ("e1", "e3"), ("e2", "e3")}
    assert abs(pairwise[("e1", "e2")] - 1.0) < 1e-9
    assert abs(pairwise[("e1", "e3")] - 1.0) < 1e-9
    assert abs(pairwise[("e2", "e3")] - math.sqrt(2)) < 1e-6


def test_pairwise_distance_covers_all_combinations_for_n4():
    per_robot = {
        f"e{i}": [(0.0, float(i), 0.0, 0.0, 0.0)] for i in range(1, 5)
    }
    pairwise = pairwise_min_distance(per_robot)
    expected_pairs = {
        ("e1", "e2"), ("e1", "e3"), ("e1", "e4"),
        ("e2", "e3"), ("e2", "e4"), ("e3", "e4"),
    }
    assert set(pairwise.keys()) == expected_pairs
    assert len(pairwise) == 6  # C(4,2)


def test_overall_minimum_is_the_true_min_across_all_pairs():
    per_robot = {
        "e1": [(0.0, 0.0, 0.0, 0.0, 0.0)],
        "e2": [(0.0, 5.0, 0.0, 0.0, 0.0)],
        "e3": [(0.0, 0.05, 0.0, 0.0, 0.0)],  # closest pair: e1-e3, 0.05m
    }
    pairwise = pairwise_min_distance(per_robot)
    overall = overall_minimum_pairwise_distance_m(pairwise)
    assert abs(overall - 0.05) < 1e-9


# --- required test 4 (partial): DATA_VALIDITY与TASK_OUTCOME分离 ---
def test_data_validity_invalid_forces_task_failure_but_fields_stay_separate():
    per_robot = {
        "e1": [(0.0, 2.0, 0.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0, 0.0)],
        "e2": [(0.0, 2.0, 0.1, 0.0, 0.0), (2.0, 2.0, 0.1, 0.0, 0.0)],
    }
    verdict = build_task_verdict(
        per_robot_samples=per_robot, goal=GOAL, hold_time_s=1.0,
        safety_radius_m=0.14, collision_contact_distance_m=0.07,
        data_validity_reasons=["bag missing a topic"],
        latched_failsafe=False, ended_by_max_runtime=False,
    )
    assert verdict.data_validity == "INVALID"
    assert verdict.task_outcome == "TASK_FAILURE"
    # even though the goal-region math itself would have said SUCCESS,
    # data_validity is a SEPARATE field, not silently overwritten to hide
    # the actual per-robot result:
    assert verdict.all_robots_reached_goal is True


def test_valid_data_and_successful_task_are_independently_true():
    per_robot = {
        "e1": [(0.0, 2.0, 0.15, 0.0, 0.0), (2.0, 2.0, 0.15, 0.0, 0.0)],
        "e2": [(0.0, 2.0, -0.15, 0.0, 0.0), (2.0, 2.0, -0.15, 0.0, 0.0)],
    }
    verdict = build_task_verdict(
        per_robot_samples=per_robot, goal=GOAL, hold_time_s=1.0,
        safety_radius_m=0.14, collision_contact_distance_m=0.07,
        data_validity_reasons=[],
        latched_failsafe=False, ended_by_max_runtime=False,
    )
    assert verdict.data_validity == "VALID"
    assert verdict.task_outcome == "SUCCESS"


def test_max_runtime_never_read_as_success_even_if_goal_reached():
    # Explicit instruction: max runtime / process exit must NEVER be
    # interpreted as task success.
    per_robot = {
        "e1": [(0.0, 2.0, 0.15, 0.0, 0.0), (2.0, 2.0, 0.15, 0.0, 0.0)],
        "e2": [(0.0, 2.0, -0.15, 0.0, 0.0), (2.0, 2.0, -0.15, 0.0, 0.0)],
    }
    verdict = build_task_verdict(
        per_robot_samples=per_robot, goal=GOAL, hold_time_s=1.0,
        safety_radius_m=0.14, collision_contact_distance_m=0.07,
        data_validity_reasons=[],
        latched_failsafe=False, ended_by_max_runtime=True,
    )
    assert verdict.task_outcome == "TASK_FAILURE"
    assert "max_runtime" in verdict.task_outcome_reason


def test_collision_below_contact_threshold_is_unsafe_failure_not_success():
    per_robot = {
        "e1": [(0.0, 2.0, 0.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0, 0.0)],
        "e2": [(0.0, 2.0, 0.02, 0.0, 0.0), (2.0, 2.0, 0.02, 0.0, 0.0)],  # 0.02m apart -- contact
    }
    verdict = build_task_verdict(
        per_robot_samples=per_robot, goal=GOAL, hold_time_s=1.0,
        safety_radius_m=0.14, collision_contact_distance_m=0.07,
        data_validity_reasons=[],
        latched_failsafe=False, ended_by_max_runtime=False,
    )
    assert verdict.task_outcome == "UNSAFE_FAILURE"
    assert verdict.collision_count >= 1


def test_below_safety_radius_but_above_contact_is_unsafe_failure():
    per_robot = {
        "e1": [(0.0, 2.0, 0.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0, 0.0)],
        "e2": [(0.0, 2.0, 0.10, 0.0, 0.0), (2.0, 2.0, 0.10, 0.0, 0.0)],  # 0.10m: < 0.14 safety radius, > 0.07 contact
    }
    verdict = build_task_verdict(
        per_robot_samples=per_robot, goal=GOAL, hold_time_s=1.0,
        safety_radius_m=0.14, collision_contact_distance_m=0.07,
        data_validity_reasons=[],
        latched_failsafe=False, ended_by_max_runtime=False,
    )
    assert verdict.task_outcome == "UNSAFE_FAILURE"
    assert verdict.collision_count == 0
    assert verdict.safety_margin_m < 0.0


def test_latched_failsafe_forces_task_failure():
    per_robot = {
        "e1": [(0.0, 2.0, 0.15, 0.0, 0.0), (2.0, 2.0, 0.15, 0.0, 0.0)],
        "e2": [(0.0, 2.0, -0.15, 0.0, 0.0), (2.0, 2.0, -0.15, 0.0, 0.0)],
    }
    verdict = build_task_verdict(
        per_robot_samples=per_robot, goal=GOAL, hold_time_s=1.0,
        safety_radius_m=0.14, collision_contact_distance_m=0.07,
        data_validity_reasons=[],
        latched_failsafe=True, ended_by_max_runtime=False,
    )
    assert verdict.task_outcome == "TASK_FAILURE"
    assert "FAILSAFE" in verdict.task_outcome_reason


# --- path/efficiency metrics ---
def test_path_length_sums_segment_distances():
    samples = [(0.0, 0.0, 0.0, 0.0, 0.0), (1.0, 3.0, 0.0, 0.0, 0.0), (2.0, 3.0, 4.0, 0.0, 0.0)]
    assert abs(path_length_m(samples) - 7.0) < 1e-9  # 3 + 4


def test_cumulative_heading_change_unwraps_across_pi_boundary():
    # yaw goes 3.0 -> 3.1 -> -3.1 (wrapped) -> should be a small continued
    # turn, not a near-2pi jump.
    samples = [
        (0.0, 0.0, 0.0, 3.0, 0.0),
        (1.0, 0.0, 0.0, 3.1, 0.0),
        (2.0, 0.0, 0.0, -3.1, 0.0),
    ]
    total = cumulative_absolute_heading_change_rad(samples)
    assert total < 0.5  # true angular travel is small, not ~6.2rad


def test_stop_duration_integrates_only_the_below_threshold_intervals():
    samples = [
        (0.0, 0.0, 0.0, 0.0, 0.0),   # stopped
        (1.0, 0.0, 0.0, 0.0, 0.0),   # stopped -- 1.0s interval counted
        (1.5, 0.0, 0.0, 0.0, 0.02),  # moving
        (2.5, 0.0, 0.0, 0.0, 0.02),  # moving -- 1.0s NOT counted
        (3.0, 0.0, 0.0, 0.0, 0.0),   # stopped
        (3.5, 0.0, 0.0, 0.0, 0.0),   # stopped -- 0.5s counted
    ]
    total = stop_duration_s(samples, stop_threshold_mps=0.002)
    assert abs(total - 1.5) < 1e-9


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([os.path.abspath(__file__), "-v"]))
