from epuck2_comm.analyze_cooperative_bag import command_metrics


def test_command_metrics_detects_start_stop_and_smoothness():
    records = [
        (0.0, 0.0, 0.0),
        (0.1, 0.025, 0.0),
        (0.2, 0.012, 0.65),
        (0.3, 0.012, 0.25),
        (0.4, 0.0, 0.0),
    ]
    metrics = command_metrics(records)
    assert metrics["motion_start_s"] == 0.1
    assert metrics["last_motion_command_s"] == 0.3
    assert metrics["peak_abs_linear_mps"] == 0.025
    assert metrics["peak_abs_angular_rps"] == 0.65
    assert metrics["max_angular_step_rps"] == 0.65
    assert metrics["angular_sign_changes"] == 0


def test_command_metrics_counts_real_turn_reversals_only():
    records = [
        (0.0, 0.0, 0.60),
        (0.1, 0.0, 0.01),
        (0.2, 0.0, -0.55),
        (0.3, 0.0, -0.20),
    ]
    assert command_metrics(records)["angular_sign_changes"] == 1


def test_empty_command_metrics_are_well_defined():
    metrics = command_metrics([])
    assert metrics["message_count"] == 0
    assert metrics["motion_start_s"] is None
    assert metrics["angular_sign_changes"] == 0
