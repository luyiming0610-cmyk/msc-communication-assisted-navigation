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
    arrived_hold: bool = False
    frozen_heading_rad: float | None = None
    individual_completion_time_s: float | None = None
    # Post-exit parking handoff (Part V): the shared exit stays the single
    # informational/navigation target used for heading and for the
    # discovery/announcement channel -- it is NOT two different task
    # goals. Once a robot geometrically enters the shared exit region,
    # its OWN downstream, non-colliding parking point (distinct per
    # robot) becomes current_target so both robots settle in different
    # physical locations rather than converging on the same point.
    # Optional: absent (None) parking_target_m means no handoff occurs
    # and current_target is used for arrival exactly as before -- keeps
    # this class usable stand-alone in existing tests.
    parking_target_m: tuple[float, float] | None = None
    exit_center_m: tuple[float, float] | None = None
    exit_radius_m: float = 0.0
    switched_to_parking: bool = False

    def __post_init__(self):
        if self.mode == "search":
            if not self.waypoints:
                raise ValueError("search mode requires at least one waypoint")
            self.current_target = self.waypoints[0]

    def update_position(self, x_m: float, y_m: float) -> bool:
        """Call on every own-position update. Returns True if the target
        waypoint just advanced (informational, for logging)."""
        if self.arrived_hold or self.mode != "search" or self.switched_to_goal:
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
        if self.arrived_hold or self.switched_to_goal or not valid:
            return False
        self.current_target = (goal_x_m, goal_y_m)
        self.switched_to_goal = True
        return True

    def check_exit_to_parking_switch(self, x_m: float, y_m: float) -> bool:
        """Call every position update once heading_to_final_target is
        True. Returns True the one time this call causes the handoff
        from the shared exit target to this robot's own parking point.
        Idempotent afterward; no-op if parking wasn't configured."""
        if (
            self.arrived_hold
            or self.switched_to_parking
            or self.parking_target_m is None
            or self.exit_center_m is None
            or not self.heading_to_final_target
        ):
            return False
        cx, cy = self.exit_center_m
        if math.hypot(x_m - cx, y_m - cy) <= self.exit_radius_m:
            self.current_target = self.parking_target_m
            self.switched_to_parking = True
            return True
        return False

    @property
    def heading_to_final_target(self) -> bool:
        """True once this robot's current_target IS the final exit
        target (informed mode always; search mode once switched via an
        announcement, or once it has reached the last frozen waypoint --
        which is itself the exit)."""
        if self.mode == "informed":
            return True
        return self.switched_to_goal or self.waypoint_index == len(self.waypoints) - 1

    @property
    def navigation_phase(self) -> str:
        if self.arrived_hold:
            return "ARRIVED_HOLD"
        if self.heading_to_final_target:
            return "GO_TO_EXIT"
        return "SEARCH"

    def latch_arrived_hold(self, current_heading_rad: float, completion_time_s: float) -> bool:
        """Irreversibly latch ARRIVED_HOLD, freezing the heading given at
        the moment of latching. Idempotent -- returns False (no-op) if
        already latched, so callers may call this every tick once their
        own GoalHoldTracker reports `reached` without re-triggering."""
        if self.arrived_hold:
            return False
        self.arrived_hold = True
        self.frozen_heading_rad = current_heading_rad
        self.individual_completion_time_s = completion_time_s
        return True

    def desired_heading_rad(self, x_m: float, y_m: float) -> float:
        if self.arrived_hold:
            # Frozen at the moment of latching -- never recompute atan2
            # against a near-zero-distance target again.
            return self.frozen_heading_rad
        tx, ty = self.current_target
        return math.atan2(ty - y_m, tx - x_m)

    def desired_linear_speed_mps(self, nominal_speed_mps: float) -> float:
        if self.arrived_hold:
            return 0.0
        return nominal_speed_mps
