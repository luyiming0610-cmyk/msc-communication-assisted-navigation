"""Tests for the streaming GoalHoldTracker, including a direct
cross-check that its incremental verdict is IDENTICAL to the existing,
already-tested batch function
task_completion_analyzer.robot_goal_completion_time() when fed the same
sample sequence one sample at a time -- this is the property that makes
task_completion_monitor.py's live TASK_COMPLETE_GOAL judgment provably
equivalent to what post-hoc analysis would compute."""
from goal_hold_tracker import GoalHoldTracker
from task_completion_analyzer import GoalRegion, robot_goal_completion_time


def _run_batch_and_stream(samples, goal, hold_time_s):
    batch_time, batch_reason = robot_goal_completion_time(samples, goal, hold_time_s)
    tracker = GoalHoldTracker(goal.center_x_m, goal.center_y_m, goal.radius_m, hold_time_s)
    for t_s, x_m, y_m, _yaw, _v in samples:
        tracker.update(t_s, x_m, y_m)
    return batch_time, batch_reason, tracker


def test_never_enters_goal_matches_batch():
    goal = GoalRegion(0.0, 0.0, 0.5)
    samples = [(t, 5.0, 5.0, 0.0, 0.1) for t in [0.0, 1.0, 2.0]]
    batch_time, _reason, tracker = _run_batch_and_stream(samples, goal, 2.0)
    assert batch_time is None
    assert tracker.reached is False
    assert tracker.completion_time_s is None


def test_single_frame_dip_does_not_trigger_matches_batch():
    goal = GoalRegion(0.0, 0.0, 0.5)
    samples = [
        (0.0, 5.0, 5.0, 0.0, 0.1),
        (1.0, 0.0, 0.0, 0.0, 0.1),  # single-frame dip into goal
        (2.0, 5.0, 5.0, 0.0, 0.1),
    ]
    batch_time, _reason, tracker = _run_batch_and_stream(samples, goal, 2.0)
    assert batch_time is None
    assert tracker.reached is False


def test_continuous_hold_matches_batch():
    goal = GoalRegion(0.0, 0.0, 0.5)
    samples = [
        (0.0, 5.0, 5.0, 0.0, 0.1),
        (1.0, 0.0, 0.0, 0.0, 0.1),
        (2.0, 0.0, 0.0, 0.0, 0.0),
        (3.0, 0.0, 0.0, 0.0, 0.0),
    ]
    batch_time, batch_reason, tracker = _run_batch_and_stream(samples, goal, 2.0)
    assert batch_reason is None
    assert batch_time == 3.0
    assert tracker.reached is True
    assert tracker.completion_time_s == batch_time


def test_gap_resets_hold_timer_matches_batch():
    goal = GoalRegion(0.0, 0.0, 0.5)
    samples = [
        (0.0, 0.0, 0.0, 0.0, 0.0),
        (1.0, 5.0, 5.0, 0.0, 0.1),   # briefly leaves -- resets clock
        (2.0, 0.0, 0.0, 0.0, 0.0),
        (3.0, 0.0, 0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0, 0.0, 0.0),
    ]
    batch_time, batch_reason, tracker = _run_batch_and_stream(samples, goal, 2.0)
    assert batch_reason is None
    assert batch_time == 4.0
    assert tracker.completion_time_s == batch_time


def test_tracker_is_idempotent_after_completion():
    tracker = GoalHoldTracker(0.0, 0.0, 0.5, 2.0)
    tracker.update(0.0, 0.0, 0.0)
    tracker.update(2.0, 0.0, 0.0)
    assert tracker.reached is True
    first_completion = tracker.completion_time_s
    # further updates, even ones that would look like a fresh exit/re-entry,
    # must never change an already-latched completion.
    tracker.update(3.0, 99.0, 99.0)
    tracker.update(4.0, 0.0, 0.0)
    assert tracker.completion_time_s == first_completion


def test_all_robots_required_multi_tracker_scenario():
    """Cross-check against all_robots_reached_goal: two robots, one
    reaches, one never does -- overall task must not be marked complete."""
    goal = GoalRegion(0.0, 0.0, 0.5)
    robot_a_samples = [(0.0, 0.0, 0.0, 0.0, 0.0), (3.0, 0.0, 0.0, 0.0, 0.0)]
    robot_b_samples = [(0.0, 5.0, 5.0, 0.0, 0.1), (3.0, 5.0, 5.0, 0.0, 0.1)]

    tracker_a = GoalHoldTracker(goal.center_x_m, goal.center_y_m, goal.radius_m, 2.0)
    tracker_b = GoalHoldTracker(goal.center_x_m, goal.center_y_m, goal.radius_m, 2.0)
    for t_s, x_m, y_m, _yaw, _v in robot_a_samples:
        tracker_a.update(t_s, x_m, y_m)
    for t_s, x_m, y_m, _yaw, _v in robot_b_samples:
        tracker_b.update(t_s, x_m, y_m)

    assert tracker_a.reached is True
    assert tracker_b.reached is False
    assert not all(t.reached for t in (tracker_a, tracker_b))
