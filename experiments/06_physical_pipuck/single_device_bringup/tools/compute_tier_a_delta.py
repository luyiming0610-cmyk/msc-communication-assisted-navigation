#!/usr/bin/env python3
"""Tier A (Pi -> WSL application-level state delivery) computed as a
TRIAL-START-vs-TRIAL-END SNAPSHOT DELTA of the expanded bridge's own
cumulative /epuck_bridge/status counters -- never the bridge's all-time
absolute counts directly, per the batch design's explicit process-reuse
policy (the bridge itself is REUSED across all trials in a batch, so its
counters keep accumulating across trial boundaries; only the delta over
a single trial's own start/end snapshots is that trial's own tier-A
result).

state_seq_first is a constant (the bridge's all-time first-ever sequence
number, e.g. 0) and does not change per trial, so it is not part of the
delta. The trial-local "expected count" is the growth in state_seq_last
over the trial; the trial-local "received count" is the growth in
state_unique_received. missing/out_of_order deltas are the growth in
those cumulative counters over the trial window.
"""
import argparse
import json


def compute_delta(start_snapshot: dict, end_snapshot: dict) -> dict:
    def _i(snap, key):
        return int(snap[key])

    seq_last_delta = _i(end_snapshot, "state_seq_last") - _i(start_snapshot, "state_seq_last")
    unique_received_delta = _i(end_snapshot, "state_unique_received") - _i(start_snapshot, "state_unique_received")
    missing_delta = _i(end_snapshot, "state_missing") - _i(start_snapshot, "state_missing")
    out_of_order_delta = _i(end_snapshot, "state_out_of_order") - _i(start_snapshot, "state_out_of_order")
    crc_errors_delta = _i(end_snapshot, "crc_errors") - _i(start_snapshot, "crc_errors")

    # Expected new sequence numbers over the trial = growth in seq_last
    # (seq_first is constant, not part of this trial's own span).
    expected_delta = seq_last_delta
    delivery_ratio_delta = (unique_received_delta / expected_delta) if expected_delta > 0 else None

    return {
        "computation": "trial-start-vs-trial-end snapshot delta of the bridge's own cumulative counters (never the all-time absolute values)",
        "start_snapshot": {
            "state_seq_first": _i(start_snapshot, "state_seq_first"),
            "state_seq_last": _i(start_snapshot, "state_seq_last"),
            "state_unique_received": _i(start_snapshot, "state_unique_received"),
            "state_missing": _i(start_snapshot, "state_missing"),
            "state_out_of_order": _i(start_snapshot, "state_out_of_order"),
            "crc_errors": _i(start_snapshot, "crc_errors"),
        },
        "end_snapshot": {
            "state_seq_first": _i(end_snapshot, "state_seq_first"),
            "state_seq_last": _i(end_snapshot, "state_seq_last"),
            "state_unique_received": _i(end_snapshot, "state_unique_received"),
            "state_missing": _i(end_snapshot, "state_missing"),
            "state_out_of_order": _i(end_snapshot, "state_out_of_order"),
            "crc_errors": _i(end_snapshot, "crc_errors"),
        },
        "trial_seq_last_delta": seq_last_delta,
        "trial_unique_received_delta": unique_received_delta,
        "trial_state_missing_delta": missing_delta,
        "trial_state_out_of_order_delta": out_of_order_delta,
        "trial_crc_errors_delta": crc_errors_delta,
        "trial_expected_count_delta": expected_delta,
        "APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO_delta": delivery_ratio_delta,
        "duplicate_count_SEPARATELY_TRACKED": None,
        "duplicate_note": "NOT_MEASURABLE -- the bridge's own update_sequence_stats() does not distinguish a true duplicate from a generic out-of-order/backward arrival; both increment the same out_of_order counter. Not fabricated here.",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--start-json", required=True)
    parser.add_argument("--end-json", required=True)
    args = parser.parse_args()
    start_snapshot = json.loads(open(args.start_json, encoding="utf-8").read())
    end_snapshot = json.loads(open(args.end_json, encoding="utf-8").read())
    print(json.dumps(compute_delta(start_snapshot, end_snapshot), indent=2))


if __name__ == "__main__":
    main()
