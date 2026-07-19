#!/usr/bin/env python3
"""Reorder-safe, set-based delivery/sequence analyzer (v1).

WHY THIS EXISTS
---------------
The live ``sequence_counter.py`` (a diagnostic streaming observer) computes
its sequence statistics from ADJACENT-ARRIVAL deltas:

  * ``sequence_gap_count += seq - prev_seq - 1``   whenever ``seq > prev+1``
  * ``out_of_order_count += 1``                    whenever ``seq < prev``
  * ``expected_count = last_arrived_seq - first_arrived_seq + 1``

That accounting is correct ONLY for an in-order stream. Under Condition D
(jitter with reordering) it is provably wrong in two ways, both observed in
``objective5_impairment_matrix_v1_condition_D_trial01_attempt01``:

  1. A forward jump (a later sequence arriving before an earlier one) adds to
     ``sequence_gap_count``, but when the skipped-over sequence later arrives
     out of order it only bumps ``out_of_order_count`` -- the earlier gap is
     never reconciled. So a purely-reordered, lossless stream reports a large
     bogus "missing" count (D01 reported 189/192 with zero real loss).
  2. ``first_arrived_seq`` is used as the minimum. If the first message to
     ARRIVE is not the smallest sequence (a lower one arrives later, out of
     order), ``expected_count`` is computed too small and
     ``capture_ratio = unique / expected`` can exceed 1.0 -- mathematically
     impossible for a real delivery ratio (D01 direction epuck2->epuck1
     reported 430/429 = 1.00233).

This analyzer replaces that adjacent-delta accounting with SET-BASED,
reorder-invariant accounting, reconstructed from the actual per-message
sequence sets. Reordering is measured but never conflated with loss, and
every ratio is clamped to [0, 1] with a hard error if it would exceed the
bound (a ratio > 1 means the inputs are inconsistent and MUST NOT yield a
"valid" verdict).

It is offline and read-only: it never runs Webots, never touches the
controller/relay/world, and never modifies the raw evidence. It reads the
already-recorded rosbag and relay CSVs; the ``sequence_counter`` JSON is used
only as a cross-reference, never as the source of truth for missing counts.

The pure-Python functions at the top (no ROS imports) carry all the logic and
are unit-tested in ``test_reorder_safe_delivery_analyzer.py``. The ROS-backed
CLI at the bottom only extracts sequence lists from a bag + CSVs and hands
them to those pure functions.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


class RatioOutOfRangeError(ValueError):
    """Raised when a computed ratio falls outside [0, 1]. A delivery/capture
    ratio outside this range is never a valid measurement -- it signals an
    inconsistency in the inputs (e.g. a wrong minimum sequence), and the
    analyzer refuses to emit a verdict rather than reporting an impossible
    number."""


def _clamp_checked(ratio: float, label: str, tol: float = 1e-9) -> float:
    if ratio < -tol or ratio > 1.0 + tol:
        raise RatioOutOfRangeError(
            f"{label} = {ratio!r} is outside [0, 1] -- inputs are inconsistent; "
            f"refusing to emit a valid ratio."
        )
    return min(1.0, max(0.0, ratio))


def analyze_received_stream(received_seqs_arrival_order: list[int]) -> dict:
    """Set-based, reorder-safe metrics for a single received sequence stream.

    ``received_seqs_arrival_order`` is the list of sequence numbers in the
    ORDER THEY ARRIVED (so reordering and duplicates are observable). All
    counting is set-based except the reordering metrics, which are the only
    ones that legitimately depend on arrival order.

    Key invariants (each asserted / enforced):
      * ``expected_count`` uses the TRUE minimum of the received set, never the
        first arrival -> ``unique_count <= expected_count`` always, so
        ``capture_ratio <= 1``.
      * ``missing_set`` is the set of sequence numbers in [min, max] that were
        never received. Reordered messages are, by definition, received, so
        they are never in ``missing_set``.
      * ``out_of_order`` (both flavours) is measured but is NOT loss.
    """
    arrival = list(received_seqs_arrival_order)
    if not arrival:
        return {
            "measurable": False,
            "reason": "NOT_MEASURABLE (no messages in this stream)",
        }

    received_set = set(arrival)
    unique_count = len(received_set)
    total_arrivals = len(arrival)
    duplicate_count = total_arrivals - unique_count

    min_seq = min(received_set)
    max_seq = max(received_set)
    expected_count = max_seq - min_seq + 1
    expected_set = set(range(min_seq, max_seq + 1))
    missing_set = expected_set - received_set
    actual_missing_count = len(missing_set)

    # Reordering flavour 1: adjacent inversions (a message arrives with a
    # sequence smaller than the immediately preceding arrival).
    adjacent_inversions = sum(
        1 for a, b in zip(arrival, arrival[1:]) if b < a
    )
    # Reordering flavour 2: displaced arrivals (a message arrives with a
    # sequence smaller than the running maximum of everything seen so far).
    # This matches sequence_counter's out_of_order definition on the first
    # occurrence but is computed independently here.
    running_max = None
    displaced_count = 0
    for s in arrival:
        if running_max is not None and s < running_max:
            displaced_count += 1
        running_max = s if running_max is None else max(running_max, s)

    # capture within this stream's own [min,max] window: fraction of the
    # contiguous expected range that was actually received. <= 1 by
    # construction (unique <= expected). This is NOT the delivery ratio
    # against the forwarded set -- see delivery_ratio_forwarded_to_bag.
    within_window_capture_ratio = _clamp_checked(
        unique_count / expected_count,
        "within_window_capture_ratio",
    )

    return {
        "measurable": True,
        "total_arrivals": total_arrivals,
        "unique_count": unique_count,
        "duplicate_count": duplicate_count,
        "min_sequence": min_seq,
        "max_sequence": max_seq,
        "first_arrival_sequence": arrival[0],
        "first_arrival_is_min": arrival[0] == min_seq,
        "expected_count": expected_count,
        "actual_missing_count": actual_missing_count,
        "actual_missing_set": sorted(missing_set),
        "out_of_order_adjacent_inversions": adjacent_inversions,
        "out_of_order_displaced_count": displaced_count,
        "reordering_present": (adjacent_inversions > 0 or displaced_count > 0),
        "within_window_capture_ratio": within_window_capture_ratio,
    }


def aligned_window_capture_ratio(
    forwarded_seqs: list[int], bag_received_seqs: list[int]
) -> dict:
    """Fraction of the relay's FORWARDED messages that were actually captured
    in the bag, computed over the ALIGNED COMMON sequence window -- i.e. the
    bag's own [min, max] coverage -- so that a bag whose recording
    started/stopped partway through the forwarded range is not mistaken for
    loss.

    NAMING (corrected 2026-07-19, statistics-naming-only, no value change):
    the emitted key is ``aligned_window_forwarded_to_bag_capture_ratio``, not
    a bare ``forwarded_to_bag_capture_ratio``, and ``relay_total_forwarded_count``
    is reported alongside it. This ratio proves ONLY that there was no
    capture loss inside the aligned window shared by the relay's forwarded
    stream and the bag; it explicitly does NOT characterize the bag's
    capture rate over the relay's FULL lifetime -- ``relay_total_forwarded_count``
    can differ from the in-window bag count purely because rosbag started
    recording a few messages after the relay began forwarding (a recording
    START BOUNDARY, not a loss). See ``bag_window_covers_full_relay_lifetime``.

    Alignment: restrict the forwarded set to [min(bag), max(bag)] (the bag's
    own coverage window). Every message the bag captured must be one the relay
    forwarded (the bag is downstream of the relay); the ratio is
    |bag ∩ forwarded_in_window| / |forwarded_in_window|. Clamped to [0, 1].
    """
    forwarded_set = set(forwarded_seqs)
    bag_set = set(bag_received_seqs)
    if not bag_set:
        return {
            "measurable": False,
            "reason": "NOT_MEASURABLE (no bag messages on this topic)",
        }
    if not forwarded_set:
        return {
            "measurable": False,
            "reason": "NOT_MEASURABLE (relay forwarded no messages)",
        }

    bag_min, bag_max = min(bag_set), max(bag_set)
    relay_min, relay_max = min(forwarded_set), max(forwarded_set)
    forwarded_in_window = {s for s in forwarded_set if bag_min <= s <= bag_max}
    captured = bag_set & forwarded_in_window
    # A bag sequence not present in the forwarded set would be impossible
    # (the bag is strictly downstream of the relay) -- surface it rather than
    # hide it.
    bag_not_in_forwarded = sorted(bag_set - forwarded_set)

    ratio = _clamp_checked(
        len(captured) / len(forwarded_in_window),
        "aligned_window_forwarded_to_bag_capture_ratio",
    )
    return {
        "measurable": True,
        "aligned_window_min": bag_min,
        "aligned_window_max": bag_max,
        "relay_total_forwarded_window_min": relay_min,
        "relay_total_forwarded_window_max": relay_max,
        "relay_total_forwarded_count": len(forwarded_set),
        "forwarded_in_aligned_window_count": len(forwarded_in_window),
        "bag_captured_in_aligned_window_count": len(captured),
        "aligned_window_forwarded_to_bag_capture_ratio": ratio,
        "forwarded_but_not_in_bag_count": len(forwarded_in_window - bag_set),
        "bag_sequences_not_in_forwarded_set": bag_not_in_forwarded,
        "bag_window_covers_full_relay_lifetime": (bag_min == relay_min and bag_max == relay_max),
        "note": (
            "This ratio is computed over the ALIGNED window shared by the "
            "relay's forwarded stream and the bag's own coverage; it proves "
            "only that no capture loss occurred WITHIN that common window. "
            "It is NOT the bag's capture rate over the relay's full "
            "lifetime -- relay_total_forwarded_count vs "
            "forwarded_in_aligned_window_count differing is expected when "
            "the bag recorder started after the relay began forwarding "
            "(a start-boundary effect, not loss)."
        ),
    }


def relay_received_to_forwarded_ratio(
    relay_input_seqs: list[int], forwarded_seqs: list[int], dropped_seqs: list[int]
) -> dict:
    """Delivery ratio AT THE RELAY ITSELF: forwarded / (forwarded + dropped),
    i.e. the fraction of messages the relay RECEIVED (per its own CSV log)
    that it forwarded rather than dropped. This is the true, planned-vs-
    actual loss point for loss conditions (E/F/G); for a jitter-only
    condition (D) it must be 1.0.

    NAMING (corrected 2026-07-19, statistics-naming-only, no value change):
    the emitted key is ``relay_received_to_forwarded_ratio``, not a bare
    ``source_to_forwarded_delivery_ratio`` -- the denominator here is the
    RELAY'S OWN RECEIVED-MESSAGE COUNT from its CSV log, not the full
    ``/epuckN/state_raw`` SOURCE-side publication lifecycle. Without a
    separate alignment between the source publisher's own sequence set and
    the relay's input sequence set (not attempted by this function), this
    ratio must NOT be described as a complete source-to-relay PDR/end-to-end
    delivery ratio -- only as a relay-received-to-relay-forwarded ratio.
    """
    forwarded_set = set(forwarded_seqs)
    dropped_set = set(dropped_seqs)
    input_set = set(relay_input_seqs) if relay_input_seqs else (forwarded_set | dropped_set)
    total_input = len(forwarded_set | dropped_set)
    if total_input == 0:
        return {
            "measurable": False,
            "reason": "NOT_MEASURABLE (relay received no messages)",
        }
    ratio = _clamp_checked(
        len(forwarded_set) / total_input,
        "relay_received_to_forwarded_ratio",
    )
    return {
        "measurable": True,
        "relay_received_count": len(input_set),
        "forwarded_count": len(forwarded_set),
        "dropped_count": len(dropped_set),
        "relay_received_to_forwarded_ratio": ratio,
        "note": (
            "Denominator is the relay's OWN received-message count from its "
            "CSV log, not the full source/state_raw publication lifecycle. "
            "Not a complete source-to-relay PDR unless the source and relay "
            "sequence sets are separately aligned."
        ),
    }


def classify_data_validity_for_reordering(stream_metrics: dict) -> dict:
    """A reordered-but-lossless stream is VALID data, not a failure. This
    encodes requirement (10): Condition D's expected reordering must never be
    treated as a DATA_VALIDITY failure. DATA_VALIDITY here concerns whether the
    sequence accounting is self-consistent and measurable -- NOT whether
    reordering occurred."""
    if not stream_metrics.get("measurable"):
        return {"data_validity": "INVALID", "reason": stream_metrics.get("reason")}
    problems = []
    if stream_metrics["within_window_capture_ratio"] > 1.0:
        problems.append("capture_ratio > 1 (impossible)")
    if stream_metrics["duplicate_count"] < 0 or stream_metrics["actual_missing_count"] < 0:
        problems.append("negative count")
    return {
        "data_validity": "VALID" if not problems else "INVALID",
        "reason": "reorder-safe set-based accounting self-consistent" if not problems else "; ".join(problems),
        "reordering_present_but_not_a_validity_failure": stream_metrics["reordering_present"],
    }


# --------------------------------------------------------------------------
# ROS-backed CLI (thin): extract sequence lists from a bag + relay CSVs, then
# delegate to the pure functions above. Nothing below is unit-tested with ROS;
# the logic lives in the pure functions.
# --------------------------------------------------------------------------

def _read_relay_csv(path: Path):
    forwarded, dropped, all_input = [], [], []
    with open(path, encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            seq = int(row["received_seq"])
            all_input.append(seq)
            if row["action"] == "forwarded":
                forwarded.append(seq)
            elif row["action"] == "dropped":
                dropped.append(seq)
    return forwarded, dropped, all_input


def _read_bag_sequences(bag_dir: Path, topic: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from epuck2_comm_interfaces.msg import EpuckState

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    seqs = []
    while reader.has_next():
        t, data, _ts = reader.read_next()
        if t == topic:
            seqs.append(int(deserialize_message(data, EpuckState).sequence))
    return seqs


# Direction -> (relay CSV name, delivered bag topic, source-raw bag topic).
# Established empirically for this trial from matching sequence ranges:
#   epuck1_relay forwards /epuck1/state_raw (seq 0..433) onto /epuck1/state,
#   consumed by epuck2's controller  => direction epuck1_to_epuck2.
#   epuck2_relay forwards /epuck2/state_raw (seq 0..440) onto /epuck2/state,
#   consumed by epuck1's controller  => direction epuck2_to_epuck1.
DIRECTIONS = {
    "epuck1_to_epuck2": {
        "relay_csv": "epuck1_relay.csv",
        "delivered_topic": "/epuck1/state",
        "source_topic": "/epuck1/state_raw",
    },
    "epuck2_to_epuck1": {
        "relay_csv": "epuck2_relay.csv",
        "delivered_topic": "/epuck2/state",
        "source_topic": "/epuck2/state_raw",
    },
}


def analyze_trial(trial_dir: Path) -> dict:
    bag_dir = trial_dir / "bag"
    diag_dir = trial_dir / "diag_logs"

    out = {
        "analyzer": "reorder_safe_delivery_analyzer.py v1",
        "audit_type": "OFFLINE_REORDER_SAFE_DELIVERY_AUDIT",
        "webots_run_for_this_audit": False,
        "trial_dir": str(trial_dir),
        "directions": {},
    }
    for direction, cfg in DIRECTIONS.items():
        forwarded, dropped, relay_input = _read_relay_csv(diag_dir / cfg["relay_csv"])
        bag_received = _read_bag_sequences(bag_dir, cfg["delivered_topic"])

        stream = analyze_received_stream(bag_received)
        forwarded_stream = analyze_received_stream(forwarded)
        delivery = aligned_window_capture_ratio(forwarded, bag_received)
        relay_delivery = relay_received_to_forwarded_ratio(relay_input, forwarded, dropped)
        validity = classify_data_validity_for_reordering(stream)

        out["directions"][direction] = {
            "relay_csv": cfg["relay_csv"],
            "delivered_topic": cfg["delivered_topic"],
            "source_topic": cfg["source_topic"],
            "relay_forwarded_set_size": len(set(forwarded)),
            "relay_dropped_set_size": len(set(dropped)),
            "bag_received_set_size": len(set(bag_received)),
            "bag_delivered_stream": stream,
            "relay_forwarded_stream": forwarded_stream,
            "aligned_window_forwarded_to_bag": delivery,
            "relay_received_to_forwarded": relay_delivery,
            "data_validity": validity,
        }
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial-dir", required=True, type=Path,
                        help="path to <...>/bags/<trial_id>/ (containing bag/ and diag_logs/)")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    result = analyze_trial(args.trial_dir)
    text = json.dumps(result, indent=2)
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
