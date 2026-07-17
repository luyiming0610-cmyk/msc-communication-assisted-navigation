"""Pure local-obstacle decision logic for simulation and physical e-puck2.

The distance thresholds are derived from the 2026-07-15 calibration of school
robot 5809.  The state publisher normalises clear IR returns to ``+Inf`` and
encodes sensor freshness in ``validity_flags``.  This module intentionally has
no ROS dependency so the safety and priority rules can be unit-tested.
"""

import math
from dataclasses import dataclass


IR_VALID_FLAG = 2
TOF_VALID_FLAG = 4


@dataclass(frozen=True)
class LocalObstacleDecision:
    active: bool
    safety_stop: bool
    mode: str
    linear_mps: float
    angular_rps: float


@dataclass
class LocalAvoidanceLatch:
    """Hold the last avoidance direction through intermittent range dropouts."""

    clear_hold_s: float = 1.0
    clearance_speed_mps: float = 0.006
    clearance_turn_rps: float = 0.30
    last_active_s: float = -math.inf
    turn_sign: float = -1.0

    def apply(
        self, decision: LocalObstacleDecision, now_s: float
    ) -> LocalObstacleDecision:
        if decision.safety_stop:
            return decision
        if decision.active:
            self.last_active_s = float(now_s)
            if abs(decision.angular_rps) > 1e-9:
                self.turn_sign = 1.0 if decision.angular_rps > 0.0 else -1.0
            return decision
        if float(now_s) - self.last_active_s <= self.clear_hold_s:
            return LocalObstacleDecision(
                True,
                False,
                "LOCAL_CLEARANCE",
                self.clearance_speed_mps,
                self.clearance_turn_rps * self.turn_sign,
            )
        return decision


def _distance(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return math.inf
    return value if math.isfinite(value) and value >= 0.0 else math.inf


def _turn_away(left_m: float, right_m: float, tie_margin_m: float = 0.002):
    """Return ROS angular sign: positive left, negative right.

    A centred or ambiguous obstacle uses the same deterministic pass-right
    convention as the communication-aware collision avoider.
    """
    if left_m + tie_margin_m < right_m:
        return -1.0
    if right_m + tie_margin_m < left_m:
        return 1.0
    return -1.0


def decide_local_obstacle(
    front_distance_m,
    left_distance_m,
    right_distance_m,
    validity_flags: int,
    previous_mode: str = "",
    *,
    front_danger_m: float = 0.100,
    front_warn_m: float = 0.180,
    front_release_m: float = 0.220,
    side_danger_m: float = 0.042,
    side_warn_m: float = 0.052,
    side_release_m: float = 0.058,
    warning_speed_mps: float = 0.010,
    side_speed_mps: float = 0.012,
    danger_turn_rps: float = 0.65,
    warning_turn_rps: float = 0.45,
    side_turn_rps: float = 0.30,
) -> LocalObstacleDecision:
    """Select a local safety action from the compact state summary.

    Static-obstacle actions are deliberately returned independently of the
    cooperative policy.  The caller gives them priority over peer-state CPA
    avoidance.  ``previous_mode`` provides release hysteresis.
    """
    flags = int(validity_flags)
    ir_valid = (flags & IR_VALID_FLAG) != 0
    tof_valid = (flags & TOF_VALID_FLAG) != 0
    if not ir_valid and not tof_valid:
        return LocalObstacleDecision(True, True, "LOCAL_SENSOR_INVALID", 0.0, 0.0)

    front = _distance(front_distance_m)
    left = _distance(left_distance_m) if ir_valid else math.inf
    right = _distance(right_distance_m) if ir_valid else math.inf
    turn = _turn_away(left, right)

    front_threshold = front_warn_m
    if previous_mode.startswith("LOCAL_FRONT"):
        front_threshold = front_release_m

    if front <= front_danger_m:
        return LocalObstacleDecision(
            True, False, "LOCAL_FRONT_DANGER", 0.0, danger_turn_rps * turn
        )
    if front <= front_threshold:
        return LocalObstacleDecision(
            True,
            False,
            "LOCAL_FRONT_WARN",
            warning_speed_mps,
            warning_turn_rps * turn,
        )

    if not ir_valid:
        return LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0)

    side_threshold = side_warn_m
    if previous_mode in ("LOCAL_LEFT_SIDE", "LOCAL_RIGHT_SIDE", "LOCAL_NARROW"):
        side_threshold = side_release_m

    if left <= side_danger_m and right <= side_danger_m:
        return LocalObstacleDecision(True, False, "LOCAL_NARROW", 0.0, 0.0)
    if left <= side_threshold and right <= side_threshold:
        return LocalObstacleDecision(
            True, False, "LOCAL_NARROW", min(side_speed_mps, 0.006), 0.0
        )
    if left <= side_threshold:
        return LocalObstacleDecision(
            True, False, "LOCAL_LEFT_SIDE", side_speed_mps, -side_turn_rps
        )
    if right <= side_threshold:
        return LocalObstacleDecision(
            True, False, "LOCAL_RIGHT_SIDE", side_speed_mps, side_turn_rps
        )
    return LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0)
