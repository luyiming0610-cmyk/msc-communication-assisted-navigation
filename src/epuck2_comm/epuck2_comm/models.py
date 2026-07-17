from dataclasses import dataclass


@dataclass(frozen=True)
class RobotStateSnapshot:
    """Transport-independent state used by publication policies."""

    x_m: float
    y_m: float
    yaw_rad: float
    linear_velocity_mps: float
    angular_velocity_rps: float
    front_distance_m: float
    left_distance_m: float
    right_distance_m: float
    obstacle_status: int
    validity_flags: int
