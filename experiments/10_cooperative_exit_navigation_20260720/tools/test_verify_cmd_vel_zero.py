from verify_cmd_vel_zero import is_zero_twist


def test_exact_zero_is_zero():
    assert is_zero_twist(0.0, 0.0) is True


def test_nonzero_linear_is_not_zero():
    assert is_zero_twist(0.05, 0.0) is False


def test_nonzero_angular_is_not_zero():
    assert is_zero_twist(0.0, 0.05) is False


def test_floating_point_noise_within_tolerance_is_zero():
    assert is_zero_twist(1e-9, -1e-9) is True


def test_negative_linear_is_not_zero():
    assert is_zero_twist(-0.02, 0.0) is False
