import math
from dataclasses import dataclass


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def right_turn_target_reached(
    previous_error_rad: float | None,
    current_error_rad: float,
    *,
    tolerance_rad: float = 0.08,
) -> bool:
    """Detect a clockwise target even when discrete feedback skips tolerance."""
    if abs(current_error_rad) <= tolerance_rad:
        return True
    return (
        previous_error_rad is not None
        and previous_error_rad < -tolerance_rad
        and current_error_rad > tolerance_rad
    )


def local_to_global(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    origin_x_m: float,
    origin_y_m: float,
    origin_yaw_rad: float,
):
    """Transform robot-local odometry into a shared experiment frame."""
    cosine = math.cos(origin_yaw_rad)
    sine = math.sin(origin_yaw_rad)
    global_x = origin_x_m + cosine * x_m - sine * y_m
    global_y = origin_y_m + sine * x_m + cosine * y_m
    return global_x, global_y, normalize_angle(origin_yaw_rad + yaw_rad)


@dataclass(frozen=True)
class CpaResult:
    current_distance_m: float
    closing_speed_mps: float
    time_to_cpa_s: float
    distance_at_cpa_m: float


def collision_risk(
    result: CpaResult,
    *,
    horizon_s: float,
    safety_radius_m: float,
    trigger_distance_m: float,
    minimum_closing_speed_mps: float = 0.001,
) -> bool:
    """Risk decision with closing-speed gating to prevent post-pass retriggers."""
    if result.closing_speed_mps <= minimum_closing_speed_mps:
        return False
    predicted_conflict = (
        result.time_to_cpa_s <= horizon_s
        and result.distance_at_cpa_m < safety_radius_m
    )
    proximity_conflict = result.current_distance_m < trigger_distance_m
    return predicted_conflict or proximity_conflict


def velocity_vector(linear_velocity_mps: float, yaw_rad: float):
    return (
        linear_velocity_mps * math.cos(yaw_rad),
        linear_velocity_mps * math.sin(yaw_rad),
    )


def closest_point_of_approach(
    own_x_m: float,
    own_y_m: float,
    own_vx_mps: float,
    own_vy_mps: float,
    peer_x_m: float,
    peer_y_m: float,
    peer_vx_mps: float,
    peer_vy_mps: float,
    horizon_s: float,
) -> CpaResult:
    """Predict closest separation under a constant-velocity model."""
    relative_x = peer_x_m - own_x_m
    relative_y = peer_y_m - own_y_m
    relative_vx = peer_vx_mps - own_vx_mps
    relative_vy = peer_vy_mps - own_vy_mps
    current_distance = math.hypot(relative_x, relative_y)
    speed_squared = relative_vx * relative_vx + relative_vy * relative_vy

    if speed_squared <= 1e-12:
        return CpaResult(current_distance, 0.0, 0.0, current_distance)

    dot = relative_x * relative_vx + relative_y * relative_vy
    time_to_cpa = max(0.0, min(float(horizon_s), -dot / speed_squared))
    cpa_x = relative_x + relative_vx * time_to_cpa
    cpa_y = relative_y + relative_vy * time_to_cpa
    closing_speed = max(0.0, -dot / max(current_distance, 1e-9))
    return CpaResult(
        current_distance,
        closing_speed,
        time_to_cpa,
        math.hypot(cpa_x, cpa_y),
    )
