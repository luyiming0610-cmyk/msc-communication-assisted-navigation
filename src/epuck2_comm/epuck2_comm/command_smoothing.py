"""Deterministic slew-rate limiting for robot velocity commands."""

from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def slew_towards(current: float, target: float, max_rate: float, dt: float) -> float:
    """Move current toward target without exceeding max_rate per second."""
    if max_rate <= 0.0 or dt <= 0.0:
        return target
    maximum_step = max_rate * dt
    return current + clamp(target - current, -maximum_step, maximum_step)


@dataclass
class CommandSmoother:
    max_linear_accel_mps2: float
    max_linear_decel_mps2: float
    max_angular_accel_rps2: float
    max_angular_decel_rps2: float
    linear_mps: float = 0.0
    angular_rps: float = 0.0

    def reset(self) -> None:
        self.linear_mps = 0.0
        self.angular_rps = 0.0

    @staticmethod
    def _rate(current, target, acceleration, deceleration):
        same_direction = current == 0.0 or current * target >= 0.0
        increasing_magnitude = abs(target) > abs(current)
        return acceleration if same_direction and increasing_magnitude else deceleration

    def step(
        self,
        target_linear_mps: float,
        target_angular_rps: float,
        dt: float,
        force_zero: bool = False,
        force_linear_zero: bool = False,
    ):
        if force_zero:
            self.reset()
            return self.linear_mps, self.angular_rps

        linear_rate = self._rate(
            self.linear_mps,
            target_linear_mps,
            self.max_linear_accel_mps2,
            self.max_linear_decel_mps2,
        )
        angular_rate = self._rate(
            self.angular_rps,
            target_angular_rps,
            self.max_angular_accel_rps2,
            self.max_angular_decel_rps2,
        )
        self.linear_mps = slew_towards(
            self.linear_mps, target_linear_mps, linear_rate, dt
        )
        self.angular_rps = slew_towards(
            self.angular_rps, target_angular_rps, angular_rate, dt
        )
        if force_linear_zero:
            self.linear_mps = 0.0
        return self.linear_mps, self.angular_rps
