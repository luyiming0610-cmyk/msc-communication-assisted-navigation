"""Offline PREDICTED_CPA vs PROXIMITY_FALLBACK classification tests.

Bypasses the rosbag reader (_read_state_bag) with a synthetic generator so
these tests exercise classify_trigger's actual classification logic without
needing a real rosbag2 install or recorded data.
"""

from types import SimpleNamespace

from epuck2_comm import analyze_trigger_reason as atr


def _msg(x, y, yaw, v, validity=1):
    return SimpleNamespace(x_m=x, y_m=y, yaw_rad=yaw, linear_velocity_mps=v, validity_flags=validity)


def _fake_bag(rows):
    """rows: list of (topic, msg, timestamp_ns)."""
    def _gen(bag_path):
        yield from rows
    return _gen


def test_head_on_closing_pair_classifies_as_predicted_cpa(monkeypatch):
    # Two robots 1.0m apart, closing head-on at 0.1m/s each -> tcpa=5s is
    # too far, so use a closer/faster setup: 0.5m apart, closing at
    # 0.5m/s combined, straight at each other (dcpa ~ 0).
    rows = [
        ("/epuck1/state", _msg(-0.25, 0.0, 0.0, 0.25), 0),
        ("/epuck2/state", _msg(0.25, 0.0, 3.14159265, 0.25), int(0.05e9)),
    ]
    monkeypatch.setattr(atr, "_read_state_bag", _fake_bag(rows))
    result, csv_rows = atr.classify_trigger(bag_path="fake", horizon_s=4.0, safety_radius_m=0.14, trigger_distance_m=0.34)
    assert result["trigger_reason"] == "PREDICTED_CPA"
    assert result["tcpa_at_trigger_s"] <= 4.0
    assert result["dcpa_at_trigger_m"] < 0.14
    assert csv_rows[-1][5] == "PREDICTED_CPA"


def test_slow_lateral_pair_within_trigger_distance_classifies_as_proximity_fallback(monkeypatch):
    # Two robots 0.30m apart (< trigger_distance_m=0.34) moving PARALLEL
    # (same heading, same speed) -- relative velocity is ~0, so tcpa/dcpa
    # never predict a real future conflict, but current_distance already
    # violates the proximity threshold.
    rows = [
        ("/epuck1/state", _msg(0.0, 0.0, 0.0, 0.025), 0),
        ("/epuck2/state", _msg(0.0, 0.30, 0.0, 0.025), int(0.05e9)),
    ]
    monkeypatch.setattr(atr, "_read_state_bag", _fake_bag(rows))
    result, csv_rows = atr.classify_trigger(bag_path="fake", horizon_s=4.0, safety_radius_m=0.14, trigger_distance_m=0.34)
    assert result["trigger_reason"] == "PROXIMITY_FALLBACK"
    assert result["trigger_distance_m"] < 0.34


def test_far_apart_and_not_closing_classifies_as_none(monkeypatch):
    rows = [
        ("/epuck1/state", _msg(-1.0, 0.0, 0.0, 0.0), 0),
        ("/epuck2/state", _msg(1.0, 0.0, 0.0, 0.0), int(0.05e9)),
    ]
    monkeypatch.setattr(atr, "_read_state_bag", _fake_bag(rows))
    result, csv_rows = atr.classify_trigger(bag_path="fake", horizon_s=4.0, safety_radius_m=0.14, trigger_distance_m=0.34)
    assert result["trigger_reason"] == "NONE_OBSERVED"
    assert result["trigger_distance_m"] is None


def test_invalid_odometry_samples_are_skipped(monkeypatch):
    rows = [
        ("/epuck1/state", _msg(0.0, 0.0, 0.0, 0.0, validity=0), 0),  # invalid, skipped
        ("/epuck1/state", _msg(-0.25, 0.0, 0.0, 0.25), int(0.01e9)),
        ("/epuck2/state", _msg(0.25, 0.0, 3.14159265, 0.25), int(0.05e9)),
    ]
    monkeypatch.setattr(atr, "_read_state_bag", _fake_bag(rows))
    result, csv_rows = atr.classify_trigger(bag_path="fake", horizon_s=4.0, safety_radius_m=0.14, trigger_distance_m=0.34)
    assert result["trigger_reason"] == "PREDICTED_CPA"
