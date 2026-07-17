import math
from abc import ABC, abstractmethod
from typing import Optional

from .models import RobotStateSnapshot


def _angle_difference(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _distance_changed(a: float, b: float, threshold: float) -> bool:
    if math.isfinite(a) != math.isfinite(b):
        return True
    if not math.isfinite(a):
        return False
    return abs(a - b) >= threshold


class TransmissionPolicy(ABC):
    def __init__(self, min_interval_s: float):
        self.min_interval_s = max(0.001, float(min_interval_s))
        self.last_publish_time: Optional[float] = None
        self.last_snapshot: Optional[RobotStateSnapshot] = None

    def can_publish(self, now: float) -> bool:
        return (
            self.last_publish_time is None
            or now - self.last_publish_time >= self.min_interval_s
        )

    @abstractmethod
    def should_publish(self, snapshot: RobotStateSnapshot, now: float) -> bool:
        raise NotImplementedError

    def mark_published(self, snapshot: RobotStateSnapshot, now: float) -> None:
        self.last_snapshot = snapshot
        self.last_publish_time = now


class PeriodicPolicy(TransmissionPolicy):
    def should_publish(self, snapshot: RobotStateSnapshot, now: float) -> bool:
        del snapshot
        return self.can_publish(now)


class EventTriggeredPolicy(TransmissionPolicy):
    def __init__(
        self,
        min_interval_s: float,
        heartbeat_interval_s: float,
        position_threshold_m: float,
        yaw_threshold_rad: float,
        linear_velocity_threshold_mps: float,
        angular_velocity_threshold_rps: float,
        distance_threshold_m: float,
    ):
        super().__init__(min_interval_s=min_interval_s)
        self.heartbeat_interval_s = max(min_interval_s, heartbeat_interval_s)
        self.position_threshold_m = position_threshold_m
        self.yaw_threshold_rad = yaw_threshold_rad
        self.linear_velocity_threshold_mps = linear_velocity_threshold_mps
        self.angular_velocity_threshold_rps = angular_velocity_threshold_rps
        self.distance_threshold_m = distance_threshold_m

    def should_publish(self, snapshot: RobotStateSnapshot, now: float) -> bool:
        if not self.can_publish(now):
            return False
        if self.last_snapshot is None or self.last_publish_time is None:
            return True
        if now - self.last_publish_time >= self.heartbeat_interval_s:
            return True

        previous = self.last_snapshot
        if previous.validity_flags != snapshot.validity_flags:
            return True
        if previous.obstacle_status != snapshot.obstacle_status:
            return True
        if math.hypot(snapshot.x_m - previous.x_m, snapshot.y_m - previous.y_m) >= self.position_threshold_m:
            return True
        if _angle_difference(snapshot.yaw_rad, previous.yaw_rad) >= self.yaw_threshold_rad:
            return True
        if abs(snapshot.linear_velocity_mps - previous.linear_velocity_mps) >= self.linear_velocity_threshold_mps:
            return True
        if abs(snapshot.angular_velocity_rps - previous.angular_velocity_rps) >= self.angular_velocity_threshold_rps:
            return True

        return any(
            (
                _distance_changed(snapshot.front_distance_m, previous.front_distance_m, self.distance_threshold_m),
                _distance_changed(snapshot.left_distance_m, previous.left_distance_m, self.distance_threshold_m),
                _distance_changed(snapshot.right_distance_m, previous.right_distance_m, self.distance_threshold_m),
            )
        )
