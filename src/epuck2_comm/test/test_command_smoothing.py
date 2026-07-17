import math

from epuck2_comm.command_smoothing import CommandSmoother, slew_towards


def _smoother():
    return CommandSmoother(
        max_linear_accel_mps2=0.05,
        max_linear_decel_mps2=0.10,
        max_angular_accel_rps2=3.0,
        max_angular_decel_rps2=4.0,
    )


def test_slew_towards_limits_a_step():
    assert math.isclose(slew_towards(0.0, 1.0, 2.0, 0.1), 0.2)
    assert math.isclose(slew_towards(1.0, 0.0, 2.0, 0.1), 0.8)


def test_smoother_limits_normal_acceleration():
    smoother = _smoother()
    linear, angular = smoother.step(0.025, 0.65, 0.05)
    assert math.isclose(linear, 0.0025, abs_tol=1.0e-9)
    assert math.isclose(angular, 0.15, abs_tol=1.0e-9)


def test_force_zero_bypasses_smoothing_for_safety():
    smoother = _smoother()
    smoother.step(0.025, 0.65, 1.0)
    assert smoother.step(0.0, 0.0, 0.05, force_zero=True) == (0.0, 0.0)


def test_local_emergency_forces_linear_zero_but_smooths_turn():
    smoother = _smoother()
    smoother.step(0.025, 0.0, 1.0)
    linear, angular = smoother.step(
        0.0, -0.65, 0.05, force_linear_zero=True
    )
    assert linear == 0.0
    assert math.isclose(angular, -0.15, abs_tol=1.0e-9)


def test_reversal_uses_deceleration_limit():
    smoother = _smoother()
    smoother.angular_rps = 0.65
    _, angular = smoother.step(0.0, -0.65, 0.05)
    assert math.isclose(angular, 0.45, abs_tol=1.0e-9)
