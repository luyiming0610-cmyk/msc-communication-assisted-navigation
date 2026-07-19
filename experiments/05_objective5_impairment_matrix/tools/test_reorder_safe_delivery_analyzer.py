#!/usr/bin/env python3
"""Unit tests for the reorder-safe delivery analyzer.

Pure-Python (no ROS): run directly with `python3 -m pytest <thisfile>` or via
the standalone `python3 <thisfile>` runner at the bottom. Covers the 10 cases
required for the Condition D sequence-accounting correction.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from reorder_safe_delivery_analyzer import (  # noqa: E402
    RatioOutOfRangeError,
    aligned_window_capture_ratio,
    analyze_received_stream,
    classify_data_validity_for_reordering,
    relay_received_to_forwarded_ratio,
)


# (1) arrival 12,10,11,13: reordering exists, missing=0, capture=1.
def test_reorder_no_loss_capture_one():
    m = analyze_received_stream([12, 10, 11, 13])
    assert m["reordering_present"] is True
    assert m["actual_missing_count"] == 0
    assert m["within_window_capture_ratio"] == 1.0
    assert m["duplicate_count"] == 0


# (2) arrival 0,2,1: reordering exists, missing=0.
def test_small_reorder_no_missing():
    m = analyze_received_stream([0, 2, 1])
    assert m["reordering_present"] is True
    assert m["actual_missing_count"] == 0
    assert m["within_window_capture_ratio"] == 1.0


# (3) arrival 0,2: genuine missing = {1}.
def test_genuine_single_loss():
    m = analyze_received_stream([0, 2])
    assert m["actual_missing_set"] == [1]
    assert m["actual_missing_count"] == 1
    assert m["reordering_present"] is False
    assert m["within_window_capture_ratio"] == pytest.approx(2 / 3)


# (4) first arrival is not the minimum sequence -> expected must use TRUE min,
#     so capture stays <= 1 (this is exactly the D01 capture_ratio>1 bug).
def test_first_arrival_not_minimum():
    m = analyze_received_stream([12, 11, 13, 14])  # first arrival 12, true min 11
    assert m["first_arrival_sequence"] == 12
    assert m["first_arrival_is_min"] is False
    assert m["min_sequence"] == 11
    assert m["expected_count"] == 4  # 14-11+1
    assert m["actual_missing_count"] == 0
    assert m["within_window_capture_ratio"] == 1.0


# (5) multiple reorderings but no duplicates.
def test_multiple_reorders_no_duplicate():
    m = analyze_received_stream([5, 3, 4, 2, 6, 1, 7, 0])
    assert m["duplicate_count"] == 0
    assert m["actual_missing_count"] == 0
    assert m["reordering_present"] is True
    assert m["out_of_order_displaced_count"] > 1
    assert m["within_window_capture_ratio"] == 1.0


# (6) real duplicate message.
def test_real_duplicate():
    m = analyze_received_stream([0, 1, 1, 2])
    assert m["duplicate_count"] == 1
    assert m["actual_missing_count"] == 0
    assert m["unique_count"] == 3
    assert m["total_arrivals"] == 4


# (7) ratio must never exceed 1: the exact impossible D01 value (unique 430
#     over a wrongly-small expected 429 = 1.00233) must RAISE, not pass.
def test_ratio_never_exceeds_one():
    from reorder_safe_delivery_analyzer import _clamp_checked
    with pytest.raises(RatioOutOfRangeError):
        _clamp_checked(430 / 429, "within_window_capture_ratio")
    # and a valid ratio at the boundary is accepted
    assert _clamp_checked(1.0, "boundary") == 1.0
    assert _clamp_checked(0.0, "boundary") == 0.0


# (8) relay forwarded set exactly equals bag received set -> ratio 1.0.
def test_forwarded_equals_bag():
    d = aligned_window_capture_ratio(
        forwarded_seqs=[0, 1, 2, 3, 4],
        bag_received_seqs=[0, 1, 2, 3, 4],
    )
    assert d["aligned_window_forwarded_to_bag_capture_ratio"] == 1.0
    assert d["forwarded_but_not_in_bag_count"] == 0
    assert d["bag_sequences_not_in_forwarded_set"] == []
    assert d["bag_window_covers_full_relay_lifetime"] is True


# (9) relay forwarded some messages that never reached the bag -> ratio < 1
#     over the aligned bag window (real capture loss between relay and bag).
def test_forwarded_not_all_in_bag():
    d = aligned_window_capture_ratio(
        forwarded_seqs=[0, 1, 2, 3, 4, 5],
        bag_received_seqs=[0, 1, 3, 4, 5],  # bag missing seq 2 within [0,5]
    )
    assert d["aligned_window_forwarded_to_bag_capture_ratio"] == pytest.approx(5 / 6)
    assert d["forwarded_but_not_in_bag_count"] == 1


# (9b) bag window narrower than the relay's full forwarded lifetime (a start-
#      boundary effect) must be flagged, not silently treated as full-lifetime
#      capture -- this is the exact D01 scenario (relay fwd 0..433, bag 20..433).
def test_bag_window_narrower_than_relay_lifetime_is_flagged():
    d = aligned_window_capture_ratio(
        forwarded_seqs=list(range(0, 100)),
        bag_received_seqs=list(range(20, 100)),  # bag started recording late
    )
    assert d["aligned_window_forwarded_to_bag_capture_ratio"] == 1.0
    assert d["relay_total_forwarded_count"] == 100
    assert d["forwarded_in_aligned_window_count"] == 80
    assert d["bag_window_covers_full_relay_lifetime"] is False


# (10) Condition-D expected reordering must NOT be a DATA_VALIDITY failure.
def test_reordering_is_valid_data():
    m = analyze_received_stream([12, 10, 11, 13, 15, 14, 16])
    v = classify_data_validity_for_reordering(m)
    assert v["data_validity"] == "VALID"
    assert v["reordering_present_but_not_a_validity_failure"] is True


# Extra: the relay-received-to-forwarded ratio (loss point) for a lossless
# jitter stream is exactly 1.0, and drops reduce it below 1. This is a
# relay-internal ratio (received-by-relay vs forwarded-by-relay), NOT a
# full source-to-relay PDR.
def test_relay_received_to_forwarded_ratio():
    lossless = relay_received_to_forwarded_ratio(
        relay_input_seqs=[0, 1, 2, 3], forwarded_seqs=[0, 1, 2, 3], dropped_seqs=[]
    )
    assert lossless["relay_received_to_forwarded_ratio"] == 1.0
    lossy = relay_received_to_forwarded_ratio(
        relay_input_seqs=[0, 1, 2, 3], forwarded_seqs=[0, 1, 3], dropped_seqs=[2]
    )
    assert lossy["relay_received_to_forwarded_ratio"] == pytest.approx(3 / 4)


# Extra: empty stream -> NOT_MEASURABLE, never guessed as 0.
def test_empty_stream_not_measurable():
    m = analyze_received_stream([])
    assert m["measurable"] is False
    assert "NOT_MEASURABLE" in m["reason"]


if __name__ == "__main__":
    raise SystemExit(pytest.main([os.path.abspath(__file__), "-v"]))
