"""Pure-Python streaming goal-hold tracker, no ROS dependency so it can
be unit tested without sourcing ROS.

Mirrors task_completion_analyzer.robot_goal_completion_time()'s exact
per-sample state machine (anti-single-frame trigger: any sample outside
the goal region resets the continuous-entry clock), evaluated
incrementally instead of over a batch, so a live verdict from
task_completion_monitor.py is provably identical to what the existing,
already-tested post-hoc analyzer computes from the same data.
"""
from __future__ import annotations

import math


class GoalHoldTracker:
    def __init__(self, center_x_m: float, center_y_m: float, radius_m: float, hold_time_s: float):
        self.center_x_m = center_x_m
        self.center_y_m = center_y_m
        self.radius_m = radius_m
        self.hold_time_s = hold_time_s
        self.entry_time_s: float | None = None
        self.completion_time_s: float | None = None

    def update(self, t_s: float, x_m: float, y_m: float) -> None:
        if self.completion_time_s is not None:
            return
        inside = math.hypot(x_m - self.center_x_m, y_m - self.center_y_m) <= self.radius_m
        if inside:
            if self.entry_time_s is None:
                self.entry_time_s = t_s
            elif t_s - self.entry_time_s >= self.hold_time_s:
                self.completion_time_s = self.entry_time_s + self.hold_time_s
        else:
            self.entry_time_s = None

    @property
    def reached(self) -> bool:
        return self.completion_time_s is not None
