import math

import pytest

from navigation_target_state import NavigationTargetState


def test_informed_mode_target_fixed_from_construction():
    state = NavigationTargetState(mode="informed", current_target=(0.5, 0.5))
    assert state.current_target == (0.5, 0.5)
    state.update_position(0.0, 0.0)
    assert state.current_target == (0.5, 0.5)


def test_informed_mode_heading_points_at_target():
    state = NavigationTargetState(mode="informed", current_target=(1.0, 0.0))
    heading = state.desired_heading_rad(0.0, 0.0)
    assert heading == pytest.approx(0.0)

    state2 = NavigationTargetState(mode="informed", current_target=(0.0, 1.0))
    heading2 = state2.desired_heading_rad(0.0, 0.0)
    assert heading2 == pytest.approx(math.pi / 2)


def test_search_mode_requires_at_least_one_waypoint():
    with pytest.raises(ValueError):
        NavigationTargetState(mode="search", waypoints=[])


def test_search_mode_starts_at_first_waypoint():
    state = NavigationTargetState(
        mode="search", waypoints=[(0.0, 0.0), (1.0, 1.0)], waypoint_arrival_radius_m=0.1
    )
    assert state.current_target == (0.0, 0.0)
    assert state.waypoint_index == 0


def test_search_mode_advances_on_arrival():
    state = NavigationTargetState(
        mode="search",
        waypoints=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
        waypoint_arrival_radius_m=0.1,
    )
    # Starting position (0,0) is already within arrival radius of waypoint
    # 0 itself -- this immediately advances to waypoint 1.
    assert state.update_position(0.0, 0.0) is True
    assert state.current_target == (1.0, 0.0)
    assert state.waypoint_index == 1

    assert state.update_position(0.5, 0.0) is False  # not yet arrived at (1.0, 0.0)
    assert state.current_target == (1.0, 0.0)

    assert state.update_position(1.05, 0.0) is True  # arrived within radius
    assert state.current_target == (2.0, 0.0)
    assert state.waypoint_index == 2


def test_search_mode_stays_at_final_waypoint_once_reached():
    state = NavigationTargetState(
        mode="search", waypoints=[(0.0, 0.0), (1.0, 0.0)], waypoint_arrival_radius_m=0.1
    )
    state.update_position(0.0, 0.0)  # advances to index 1 (final)
    assert state.waypoint_index == 1
    assert state.update_position(1.0, 0.0) is False  # no further waypoint to advance to
    assert state.current_target == (1.0, 0.0)


def test_announcement_switches_search_mode_target():
    state = NavigationTargetState(
        mode="search", waypoints=[(0.0, 0.0), (5.0, 5.0)], waypoint_arrival_radius_m=0.1
    )
    switched = state.receive_announcement(0.55, 0.55, valid=True)
    assert switched is True
    assert state.switched_to_goal is True
    assert state.current_target == (0.55, 0.55)


def test_invalid_announcement_never_switches():
    state = NavigationTargetState(
        mode="search", waypoints=[(0.0, 0.0), (5.0, 5.0)], waypoint_arrival_radius_m=0.1
    )
    switched = state.receive_announcement(0.55, 0.55, valid=False)
    assert switched is False
    assert state.switched_to_goal is False
    assert state.current_target == (0.0, 0.0)


def test_only_first_valid_announcement_switches_subsequent_ignored():
    state = NavigationTargetState(
        mode="search", waypoints=[(0.0, 0.0), (5.0, 5.0)], waypoint_arrival_radius_m=0.1
    )
    first = state.receive_announcement(0.55, 0.55, valid=True)
    second = state.receive_announcement(9.0, 9.0, valid=True)
    assert first is True
    assert second is False
    assert state.current_target == (0.55, 0.55)  # unchanged by the second message


def test_switched_search_mode_no_longer_advances_by_position():
    state = NavigationTargetState(
        mode="search", waypoints=[(0.0, 0.0), (5.0, 5.0)], waypoint_arrival_radius_m=0.1
    )
    state.receive_announcement(0.55, 0.55, valid=True)
    advanced = state.update_position(0.0, 0.0)  # would have advanced waypoint pre-switch
    assert advanced is False
    assert state.current_target == (0.55, 0.55)


# -- Part VIII: arrival-lock (ARRIVED_HOLD) tests -----------------------


def test_arrived_hold_latch_is_irreversible():
    state = NavigationTargetState(mode="informed", current_target=(0.5, 0.5))
    first = state.latch_arrived_hold(current_heading_rad=1.0, completion_time_s=12.0)
    assert first is True
    assert state.arrived_hold is True
    assert state.navigation_phase == "ARRIVED_HOLD"
    # A later attempt to re-latch (e.g. the tracker firing again) must be
    # a no-op -- it must never revert to SEARCH/GO_TO_EXIT, and the
    # originally-frozen heading/time must not be overwritten.
    second = state.latch_arrived_hold(current_heading_rad=9.9, completion_time_s=99.0)
    assert second is False
    assert state.frozen_heading_rad == 1.0
    assert state.individual_completion_time_s == 12.0
    assert state.navigation_phase == "ARRIVED_HOLD"


def test_arrived_hold_desired_linear_speed_is_zero():
    state = NavigationTargetState(mode="informed", current_target=(0.5, 0.5))
    assert state.desired_linear_speed_mps(0.04) == 0.04  # not yet arrived
    state.latch_arrived_hold(current_heading_rad=0.2, completion_time_s=5.0)
    assert state.desired_linear_speed_mps(0.04) == 0.0


def test_arrived_hold_freezes_heading_never_recomputes_near_zero_distance_atan2():
    state = NavigationTargetState(mode="informed", current_target=(0.5, 0.5))
    state.latch_arrived_hold(current_heading_rad=0.777, completion_time_s=5.0)
    # Position now essentially AT the target -- a live atan2 recompute here
    # would be a near-zero-distance singularity (the root cause this fix
    # addresses). desired_heading_rad must return the frozen value instead.
    heading_at_target = state.desired_heading_rad(0.5 + 1e-9, 0.5 - 1e-9)
    assert heading_at_target == 0.777
    heading_elsewhere = state.desired_heading_rad(10.0, -10.0)
    assert heading_elsewhere == 0.777  # frozen regardless of position input


def test_arrived_hold_stops_waypoint_and_announcement_updates():
    state = NavigationTargetState(
        mode="search", waypoints=[(0.0, 0.0), (1.0, 0.0)], waypoint_arrival_radius_m=0.1
    )
    state.update_position(0.0, 0.0)  # advances to final waypoint, index 1
    state.latch_arrived_hold(current_heading_rad=0.0, completion_time_s=3.0)
    advanced = state.update_position(1.0, 0.0)
    assert advanced is False
    switched = state.receive_announcement(9.0, 9.0, valid=True)
    assert switched is False
    assert state.current_target == (1.0, 0.0)  # unchanged by either call


def test_exit_to_parking_switch_then_independent_arrival_per_robot():
    """Two independently-constructed NavigationTargetState instances (one
    per robot) must latch ARRIVED_HOLD independently -- Robot A completing
    must never affect Robot B's own state object."""
    robot_a = NavigationTargetState(
        mode="informed", current_target=(0.5, 0.5),
        parking_target_m=(0.64, 0.50), exit_center_m=(0.5, 0.5), exit_radius_m=0.10,
    )
    robot_b = NavigationTargetState(
        mode="search", waypoints=[(-0.2, -0.2), (0.5, 0.5)], waypoint_arrival_radius_m=0.10,
        parking_target_m=(0.50, 0.64), exit_center_m=(0.5, 0.5), exit_radius_m=0.10,
    )
    # Robot A enters the exit region and switches to its own parking point.
    switched_a = robot_a.check_exit_to_parking_switch(0.55, 0.55)
    assert switched_a is True
    assert robot_a.current_target == (0.64, 0.50)
    robot_a.latch_arrived_hold(current_heading_rad=0.0, completion_time_s=20.0)
    assert robot_a.navigation_phase == "ARRIVED_HOLD"
    # Robot B, meanwhile, is still far from the exit and unaffected.
    assert robot_b.navigation_phase == "SEARCH"
    assert robot_b.arrived_hold is False
    assert robot_b.current_target == (-0.2, -0.2)


def test_exit_to_parking_switch_no_op_without_parking_config():
    state = NavigationTargetState(mode="informed", current_target=(0.5, 0.5))
    switched = state.check_exit_to_parking_switch(0.5, 0.5)
    assert switched is False
    assert state.current_target == (0.5, 0.5)
