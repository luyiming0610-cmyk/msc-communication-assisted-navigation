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
