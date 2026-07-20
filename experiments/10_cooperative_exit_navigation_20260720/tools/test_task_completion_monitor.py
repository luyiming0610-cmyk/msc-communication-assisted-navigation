"""Tests for task_completion_monitor.py's per-robot completion
visibility (Part IV): robot_a_completed/robot_b_completed must be
observable independently, a single robot completing must NOT end the
trial (no verdict file, node stays not-done), and only once ALL robots
have completed does the monitor produce a verdict carrying a makespan."""
import json
import os
import tempfile

import rclpy

from epuck2_comm_interfaces.msg import EpuckState
from task_completion_monitor import TaskCompletionMonitor


def _state(x, y, t_s):
    msg = EpuckState()
    msg.stamp.sec = int(t_s)
    msg.stamp.nanosec = int((t_s - int(t_s)) * 1e9)
    msg.x_m = float(x)
    msg.y_m = float(y)
    return msg


def _make_node(verdict_path):
    rclpy.init(args=[])
    return TaskCompletionMonitor(
        robot_ids=["epuck1", "epuck2"],
        state_topics=["/epuck1/state", "/epuck2/state"],
        goal_centers_x_m=[0.64, 0.50],
        goal_centers_y_m=[0.50, 0.64],
        goal_radii_m=[0.04, 0.04],
        goal_hold_time_s=2.0,
        verdict_path=verdict_path,
    )


def test_single_robot_completion_does_not_end_trial():
    with tempfile.TemporaryDirectory() as tmp:
        verdict_path = os.path.join(tmp, "verdict.json")
        node = _make_node(verdict_path)
        try:
            cb_a = node._make_cb("epuck1")
            cb_a(_state(0.64, 0.50, 0.0))
            cb_a(_state(0.64, 0.50, 2.0))
            assert node.per_robot_completed["epuck1"] is True
            assert node.per_robot_completed["epuck2"] is False
            assert node.done is False
            assert not os.path.exists(verdict_path)
        finally:
            node.destroy_node()
            rclpy.shutdown()


def test_both_completed_produces_makespan_and_stop_reason():
    with tempfile.TemporaryDirectory() as tmp:
        verdict_path = os.path.join(tmp, "verdict.json")
        node = _make_node(verdict_path)
        try:
            cb_a = node._make_cb("epuck1")
            cb_b = node._make_cb("epuck2")
            cb_a(_state(0.64, 0.50, 0.0))
            cb_a(_state(0.64, 0.50, 2.0))  # robot A completes at t=2.0
            assert node.done is False
            cb_b(_state(0.50, 0.64, 5.0))
            cb_b(_state(0.50, 0.64, 8.0))  # robot B completes at t=8.0
            assert node.done is True
            assert node.per_robot_completed == {"epuck1": True, "epuck2": True}
            with open(verdict_path, "r", encoding="utf-8") as f:
                verdict = json.load(f)
            assert verdict["stop_reason"] == "TASK_COMPLETE_GOAL"
            assert verdict["per_robot_completed"] == {"epuck1": True, "epuck2": True}
            # GoalHoldTracker's completion_time_s is entry_time_s + hold_time_s
            # (2.0s), not the sample timestamp that first observed `reached` --
            # robot B entered its zone at t=5.0, so its completion_time_s=7.0.
            assert verdict["makespan_s"] == 7.0  # max of the two completion times
        finally:
            node.destroy_node()
            rclpy.shutdown()
