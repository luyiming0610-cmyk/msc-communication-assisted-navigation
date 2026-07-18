"""Tests for compute_tier_a_delta.py's snapshot-delta computation.

Confirms the delta is genuinely computed from start/end snapshots (not the
end snapshot's cumulative values used directly), including a case where
the cumulative counters look perfect (ratio 1.0) but only because loss
happened before the trial started -- the delta must isolate the trial's
own window, not silently inherit pre-existing cumulative state as if it
were this trial's own result.
"""
import pytest

from compute_tier_a_delta import compute_delta, load_snapshot_json


def _snapshot(seq_last, unique_received, missing=0, out_of_order=0, crc_errors=0, seq_first=0):
    return {
        "state_seq_first": seq_first,
        "state_seq_last": seq_last,
        "state_unique_received": unique_received,
        "state_missing": missing,
        "state_out_of_order": out_of_order,
        "crc_errors": crc_errors,
    }


def test_perfect_delivery_delta():
    start = _snapshot(seq_last=48293, unique_received=48294)
    end = _snapshot(seq_last=51202, unique_received=51203)
    result = compute_delta(start, end)
    assert result["trial_seq_last_delta"] == 2909
    assert result["trial_unique_received_delta"] == 2909
    assert result["trial_expected_count_delta"] == 2909
    assert result["APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO_delta"] == 1.0
    assert result["trial_state_missing_delta"] == 0
    assert result["trial_state_out_of_order_delta"] == 0


def test_delta_isolates_trial_window_from_pre_existing_cumulative_loss():
    """Cumulative state_missing is already 5 BEFORE the trial starts (some
    earlier trial or warmup lost 5 messages). If this trial itself has 0
    NEW loss, the delta must report missing_delta=0 for THIS trial, not
    the misleading absolute cumulative value of 5."""
    start = _snapshot(seq_last=1000, unique_received=996, missing=5, out_of_order=1)
    end = _snapshot(seq_last=1500, unique_received=1496, missing=5, out_of_order=1)
    result = compute_delta(start, end)
    assert result["trial_state_missing_delta"] == 0
    assert result["trial_state_out_of_order_delta"] == 0
    assert result["trial_seq_last_delta"] == 500
    assert result["trial_unique_received_delta"] == 500
    assert result["APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO_delta"] == 1.0


def test_delta_detects_loss_that_happens_within_the_trial_itself():
    start = _snapshot(seq_last=1000, unique_received=1000, missing=0, out_of_order=0)
    end = _snapshot(seq_last=1500, unique_received=1495, missing=5, out_of_order=0)
    result = compute_delta(start, end)
    assert result["trial_seq_last_delta"] == 500
    assert result["trial_unique_received_delta"] == 495
    assert result["trial_state_missing_delta"] == 5
    assert abs(result["APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO_delta"] - (495 / 500)) < 1e-9


def test_duplicate_count_always_none_not_measurable():
    start = _snapshot(seq_last=0, unique_received=0)
    end = _snapshot(seq_last=100, unique_received=100)
    result = compute_delta(start, end)
    assert result["duplicate_count_SEPARATELY_TRACKED"] is None


def test_load_snapshot_json_handles_real_ros2_topic_echo_output_format(tmp_path):
    """Regression test for the bug found running trial02_attempt02: the
    bridge-status snapshot files are written from `ros2 topic echo --field
    data --once`, which ALWAYS appends a YAML document-end marker line
    ("---") after the JSON value regardless of --field -- the file is one
    JSON line followed by that marker, not pure JSON. A naive
    json.loads(file.read()) raises json.decoder.JSONDecodeError: Extra
    data. This is exactly the raw byte layout captured from the real
    trial02_attempt02 run (confirmed via `cat -A`)."""
    path = tmp_path / "bridge_status_trial_start.json"
    path.write_text(
        '{"connected": true, "crc_errors": 0, "state_seq_last": 56571, '
        '"state_unique_received": 56572, "state_missing": 0, "state_out_of_order": 0, '
        '"state_seq_first": 0}\n---\n',
        encoding="utf-8",
    )
    result = load_snapshot_json(str(path))
    assert result["state_seq_last"] == 56571
    assert result["connected"] is True


def test_load_snapshot_json_rejects_empty_file():
    import tempfile
    import os
    fd, path = tempfile.mkstemp()
    os.close(fd)
    try:
        with pytest.raises(ValueError):
            load_snapshot_json(path)
    finally:
        os.unlink(path)
