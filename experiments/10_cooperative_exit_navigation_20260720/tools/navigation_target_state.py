"""Pure-Python navigation-target state machine, no ROS dependency so it
can be unit tested without sourcing ROS. Used by goal_navigator.py.

Two roles:
  - informed: target is fixed from construction (Robot A -- its own
    a-priori knowledge, never changes).
  - search: target advances through a frozen waypoint sequence as the
    robot arrives at each one, ending at the final waypoint (the exit
    itself) -- identical under COMM_OFF and COMM_ON up to the point
    where COMM_ON may switch early on receiving a valid announcement.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class NavigationTargetState:
    mode: str  # "informed" | "search"
    waypoints: list[tuple[float, float]] = field(default_factory=list)
    waypoint_arrival_radius_m: float = 0.10
    waypoint_index: int = 0
    switched_to_goal: bool = False
    current_target: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self):
        if self.mode == "search":
            if not self.waypoints:
                raise ValueError("search mode requires at least one waypoint")
            self.current_target = self.waypoints[0]

    def update_position(self, x_m: float, y_m: float) -> bool:
        """Call on every own-position update. Returns True if the target
        waypoint just advanced (informational, for logging)."""
        if self.mode != "search" or self.switched_to_goal:
            return False
        wx, wy = self.current_target
        if math.hypot(wx - x_m, wy - y_m) <= self.waypoint_arrival_radius_m:
            if self.waypoint_index < len(self.waypoints) - 1:
                self.waypoint_index += 1
                self.current_target = self.waypoints[self.waypoint_index]
                return True
        return False

    def receive_announcement(self, goal_x_m: float, goal_y_m: float, valid: bool) -> bool:
        """Call when a GoalAnnouncement arrives. Returns True if this
        announcement caused a switch (only the FIRST valid one ever
        does; idempotent afterward)."""
        if self.switched_to_goal or not valid:
            return False
        self.current_target = (goal_x_m, goal_y_m)
        self.switched_to_goal = True
        return True

    def desired_heading_rad(self, x_m: float, y_m: float) -> float:
        tx, ty = self.current_target
        return math.atan2(ty - y_m, tx - x_m)
