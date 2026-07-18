"""Tests for compute_tier_a_delta.py's snapshot-delta computation.

Confirms the delta is genuinely computed from start/end snapshots (not the
end snapshot's cumulative values used directly), including a case where
the cumulative counters look perfect (ratio 1.0) but only because loss
happened before the trial started -- the delta must isolate the trial's
own window, not silently inherit pre-existing cumulative state as if it
were this trial's own result.
"""
from compute_tier_a_delta import compute_delta


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
