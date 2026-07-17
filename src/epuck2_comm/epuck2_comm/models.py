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
    # controller_v4_full_sensor_bypass_20260717: zone-aggregated raw ps0-ps7
    # coverage, filling the rear gap front/left/right never had.
    left_front_m: float = float("inf")
    left_mid_m: float = float("inf")
    left_rear_m: float = float("inf")
    right_front_m: float = float("inf")
    right_mid_m: float = float("inf")
    right_rear_m: float = float("inf")
