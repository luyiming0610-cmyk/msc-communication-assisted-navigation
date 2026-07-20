from verify_state_velocity_settled import is_settled


def test_exact_zero_is_settled():
    assert is_settled(0.0) is True


def test_small_residual_within_threshold_is_settled():
    assert is_settled(0.005) is True


def test_at_threshold_boundary_is_settled():
    assert is_settled(0.01) is True


def test_above_threshold_is_not_settled():
    assert is_settled(0.025) is False


def test_negative_velocity_above_threshold_is_not_settled():
    assert is_settled(-0.03) is False


def test_custom_threshold_is_respected():
    assert is_settled(0.02, threshold_mps=0.03) is True
    assert is_settled(0.02, threshold_mps=0.01) is False
