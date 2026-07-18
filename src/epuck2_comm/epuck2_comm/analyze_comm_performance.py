"""Read-only communication-performance metrics for EpuckState traffic.

Objective 5 (Performance Analysis) support tool. Reads an already-recorded
rosbag and computes, per state-publishing topic (one per robot), the
formal communication metrics: packet delivery ratio, sequence gaps,
duplicates, out-of-order arrivals, message age percentiles, actual
publish/receive rate, serialized message size, and bandwidth. This module
never touches the frozen PROTOCOL_VERSION=1 EpuckState message, the frozen
controller, or any existing bag -- it only reads.

Time-measurement rules (see PROTOCOL_FREEZE_20260717.md and this session's
communication-phase instructions):

- CORRECTED (protocol_v1.1_stamp_semantics): the bag's own record
  timestamp (`timestamp_ns` from rosbag2_py's SequentialReader) is NOT
  guaranteed to be ROS/sim time -- `ros2 bag record`'s default recording
  clock is the recorder process's own system/wall clock, a different
  domain from `message.stamp` (which IS ROS/sim time under
  `use_sim_time=true`, since state_publisher.py sets it from
  `self.get_clock().now()`). A prior analysis pass wrongly assumed both
  timestamps shared one clock domain; "message age" computed as
  bag_record_time - message.stamp under that assumption produced
  epoch-scale nonsense (~1.78e9 "seconds"), not real latency, for
  objective5_comm_baseline_zero_impairment_formal_trial01 -- see that
  trial's known_limitations entry in experiment_registry.csv.
  `_state_age_stats()` below now excludes any sample whose computed age
  is negative or implausibly large (see `_MAX_PLAUSIBLE_AGE_S`) rather
  than silently reporting it, and reports how many samples were
  excluded. The live, same-clock-domain alternative is
  sequence_counter.py's own age tracking (message.stamp vs its own
  get_clock().now() at receipt, both under the node's own use_sim_time),
  which does not have this bag-recording-clock mismatch.
- This does NOT generalize to a physical Pi-puck without first verifying
  clock synchronization (NTP/chrony) between devices -- see
  `verify_clock_sync()` below, which is a stub that must be filled in and
  actually run before any cross-device latency claim is made on hardware.
  Do not compute "network delay" from two unsynchronized system clocks.
- CPU/memory overhead cannot be reliably reconstructed from a bag alone
  (it is a runtime-only measurement). This analyzer reports it as
  "not measured" rather than fabricate a number; a live sampling
  companion (e.g. psutil polling during the run) would be needed.

Trial-window handling: the first `warmup_s` and last `cooldown_s` seconds
of each topic's own message stream are excluded from every statistic, to
avoid start-of-recording and shutdown transients (rosbag pre-roll, the
controller's own startup hold, final zero-command settling) contaminating
rate/PDR/latency numbers. Sequence resets (a publisher process restarting
mid-bag) are detected and treated as separate sub-sessions for PDR/gap
accounting, never silently merged across a reset.
"""

import argparse
import csv
import json
import math
from pathlib import Path

STATE_TOPICS = ("/epuck1/state", "/epuck2/state")
SEQUENCE_MODULUS = 2**32
# A sequence reset (publisher restarted) is declared when the new sequence
# is small AND the drop from the previous sequence is far larger than any
# realistic out-of-order jitter (which swaps a message with one of its
# near neighbours, not tens of positions) -- this distinguishes a genuine
# publisher restart from ordinary reordering, and from a real 2**32
# wraparound, which would not realistically occur within one bounded
# trial at any plausible publish rate (years, not minutes, to wrap).
RESET_NEW_SEQ_THRESHOLD = 16
RESET_MIN_DROP_THRESHOLD = 20


def _read_state_bag(bag_path: Path):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "Run this command inside a sourced ROS 2 Humble environment."
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_names = {t.name: t.type for t in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in type_names.items()
        if topic in STATE_TOPICS
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(message_types)))

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(raw, message_types[topic])
        yield topic, message, timestamp_ns, len(raw)


def _percentile(sorted_values, fraction: float):
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    index = fraction * (len(sorted_values) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return sorted_values[int(index)]
    weight = index - lower
    return sorted_values[lower] * (1 - weight) + sorted_values[upper] * weight


def _stamp_ns(message) -> int:
    return int(message.stamp.sec) * 1_000_000_000 + int(message.stamp.nanosec)


# protocol_v1.1_stamp_semantics: see the module docstring's clock-domain
# note. An age this large cannot be genuine transport latency for this
# system and is treated as a clock-domain-mismatch symptom, excluded from
# the reported statistics rather than silently averaged in.
_MAX_PLAUSIBLE_AGE_S = 30.0


def _split_sessions(rows):
    """rows: list of (bag_ns, seq, stamp_ns, size_bytes) sorted by bag_ns.
    Returns list of sessions, each a list of rows, split on a detected
    sequence reset (see module docstring)."""
    sessions = []
    current = []
    previous_seq = None
    for row in rows:
        seq = row[1]
        if (
            previous_seq is not None
            and seq < RESET_NEW_SEQ_THRESHOLD
            and (previous_seq - seq) > RESET_MIN_DROP_THRESHOLD
        ):
            sessions.append(current)
            current = []
        current.append(row)
        previous_seq = seq
    if current:
        sessions.append(current)
    return sessions


def _session_metrics(session_rows, peer_timeout_s: float):
    seqs = [row[1] for row in session_rows]
    first_seq, last_seq = seqs[0], seqs[-1]
    # Wrap-aware expected count across this session (see module docstring:
    # a true 2**32 wrap inside one bounded trial is not realistically
    # expected, but the modulus keeps this correct if it ever happens).
    span = (last_seq - first_seq) % SEQUENCE_MODULUS
    expected_count = span + 1

    seen = {}
    duplicate_count = 0
    out_of_order_count = 0
    previous_seq = None
    for row in session_rows:
        seq = row[1]
        seen[seq] = seen.get(seq, 0) + 1
        if previous_seq is not None and seq < previous_seq:
            out_of_order_count += 1
        previous_seq = seq
    duplicate_count = sum(count - 1 for count in seen.values() if count > 1)
    unique_received = len(seen)
    missing_count = max(0, expected_count - unique_received)
    pdr = unique_received / expected_count if expected_count > 0 else None

    all_ages_s = [(row[0] - row[2]) / 1.0e9 for row in session_rows]
    negative_age_count = sum(1 for age in all_ages_s if age < 0.0)
    anomalous_age_count = sum(1 for age in all_ages_s if age > _MAX_PLAUSIBLE_AGE_S)
    ages_s = [age for age in all_ages_s if 0.0 <= age <= _MAX_PLAUSIBLE_AGE_S]
    ages_sorted = sorted(ages_s)
    # If every sample was excluded (all negative and/or anomalous), there is
    # no valid latency data at all for this session -- likely a clock-domain
    # mismatch between the bag's own record timestamp and message.stamp
    # (see module docstring). Report explicitly rather than defaulting the
    # stats fields to None with no indication of why.
    latency_domain_mismatch_detected = bool(all_ages_s) and not ages_s

    bag_times_s = [row[0] / 1.0e9 for row in session_rows]
    intervals_s = [
        b - a for a, b in zip(bag_times_s, bag_times_s[1:]) if b > a
    ]
    stale_state_events = sum(
        1 for interval in intervals_s if interval > peer_timeout_s
    )

    sizes = [row[3] for row in session_rows]
    duration_s = bag_times_s[-1] - bag_times_s[0] if len(bag_times_s) > 1 else 0.0

    return {
        "message_count": len(session_rows),
        "unique_sequence_count": unique_received,
        "expected_sequence_count": expected_count,
        "first_sequence": first_seq,
        "last_sequence": last_seq,
        "missing_sequence_count": missing_count,
        "duplicate_count": duplicate_count,
        "out_of_order_count": out_of_order_count,
        "packet_delivery_ratio": pdr,
        "negative_age_sample_count": negative_age_count,
        "anomalous_age_sample_count": anomalous_age_count,
        "valid_age_sample_count": len(ages_sorted),
        "latency_domain_mismatch_detected": latency_domain_mismatch_detected,
        "mean_message_age_s": (sum(ages_s) / len(ages_s)) if ages_s else None,
        "p50_message_age_s": _percentile(ages_sorted, 0.50),
        "p95_message_age_s": _percentile(ages_sorted, 0.95),
        "p99_message_age_s": _percentile(ages_sorted, 0.99),
        "max_message_age_s": ages_sorted[-1] if ages_sorted else None,
        "mean_interval_s": (sum(intervals_s) / len(intervals_s)) if intervals_s else None,
        "actual_rate_hz": (
            (len(intervals_s) / sum(intervals_s)) if intervals_s and sum(intervals_s) > 0 else None
        ),
        "stale_state_candidate_events": stale_state_events,
        "mean_message_size_bytes": (sum(sizes) / len(sizes)) if sizes else None,
        "total_bytes": sum(sizes),
        "duration_s": duration_s,
        "mean_bandwidth_bytes_per_s": (sum(sizes) / duration_s) if duration_s > 0 else None,
    }


def _peak_bandwidth(session_rows, window_s: float = 1.0):
    if not session_rows:
        return None
    bag_times_s = [row[0] / 1.0e9 for row in session_rows]
    sizes = [row[3] for row in session_rows]
    start = bag_times_s[0]
    end = bag_times_s[-1]
    if end <= start:
        return float(sum(sizes))
    n_windows = max(1, int(math.ceil((end - start) / window_s)))
    bins = [0] * n_windows
    for t, size in zip(bag_times_s, sizes):
        idx = min(n_windows - 1, int((t - start) / window_s))
        bins[idx] += size
    return max(bins) / window_s


def analyze(
    bag_path: Path,
    warmup_s: float = 2.0,
    cooldown_s: float = 2.0,
    peer_timeout_s: float = 0.5,
):
    per_topic_raw = {topic: [] for topic in STATE_TOPICS}
    first_ns = None
    for topic, message, timestamp_ns, size_bytes in _read_state_bag(bag_path):
        if first_ns is None:
            first_ns = timestamp_ns
        per_topic_raw[topic].append(
            (timestamp_ns, int(message.sequence), _stamp_ns(message), size_bytes)
        )

    if first_ns is None:
        raise RuntimeError("No /epuck1,2/state messages found in this bag.")

    results = {}
    for topic, rows in per_topic_raw.items():
        rows.sort(key=lambda r: r[0])
        if not rows:
            results[topic] = {"message_count": 0, "note": "no messages on this topic"}
            continue

        topic_start_ns = rows[0][0]
        topic_end_ns = rows[-1][0]
        window_start_ns = topic_start_ns + int(warmup_s * 1.0e9)
        window_end_ns = topic_end_ns - int(cooldown_s * 1.0e9)
        windowed = [row for row in rows if window_start_ns <= row[0] <= window_end_ns]
        excluded_count = len(rows) - len(windowed)

        if not windowed:
            results[topic] = {
                "message_count": len(rows),
                "note": (
                    f"trial window (warmup_s={warmup_s}, cooldown_s={cooldown_s}) "
                    "excluded all messages; topic duration too short for this window"
                ),
            }
            continue

        sessions = _split_sessions(windowed)
        session_metrics = [_session_metrics(s, peer_timeout_s) for s in sessions]
        session_peak_bw = [_peak_bandwidth(s) for s in sessions]

        total_expected = sum(m["expected_sequence_count"] for m in session_metrics)
        total_received = sum(m["unique_sequence_count"] for m in session_metrics)

        results[topic] = {
            "message_count_total_in_bag": len(rows),
            "message_count_in_window": len(windowed),
            "excluded_by_window": excluded_count,
            "session_count": len(sessions),
            "sessions": session_metrics,
            "peak_bandwidth_bytes_per_s_per_session": session_peak_bw,
            "overall_packet_delivery_ratio": (
                total_received / total_expected if total_expected > 0 else None
            ),
            "overall_expected_sequence_count": total_expected,
            "overall_received_sequence_count": total_received,
            "cpu_memory_overhead": (
                "NOT MEASURED -- cannot be reliably reconstructed from a bag "
                "alone; requires a live psutil-style sampling companion run "
                "alongside the recorded trial."
            ),
        }

    return {
        "bag_path": str(bag_path),
        "warmup_s": warmup_s,
        "cooldown_s": cooldown_s,
        "peer_timeout_s": peer_timeout_s,
        "topics": results,
        "clock_domain_note": (
            "All timestamps in this analysis are ROS/sim time from a single "
            "shared /clock domain (one Webots session). Valid for latency "
            "measurement as-is. Do NOT reuse this age/latency methodology "
            "across two independently-clocked devices (e.g. sim host vs. "
            "physical Pi-puck) without first verifying clock sync -- see "
            "verify_clock_sync() in this module."
        ),
    }


def verify_clock_sync(host_a: str, host_b: str, max_offset_s: float = 0.010):
    """Stub for the physical-hardware phase: must be implemented and
    actually invoked (e.g. via chronyc tracking / ntpdate -q against a
    shared reference, or a round-trip PTP-style exchange) before any
    single-direction latency number computed between host_a and host_b's
    independent system clocks is reported as a network delay measurement.
    Raises NotImplementedError until a real check is wired in -- this is
    intentional so a cross-device latency claim can never be silently
    computed by accident without an explicit clock-sync verification step.
    """
    raise NotImplementedError(
        "verify_clock_sync() must be implemented against the real physical "
        "Pi-puck deployment (NTP/chrony offset check) before any "
        "cross-device latency figure is trusted. Not applicable to the "
        "current single-machine Webots simulation phase, where publisher "
        "and analyzer share one /clock domain."
    )


def _write_outputs(output_dir: Path, result: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "comm_performance_summary.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    with (output_dir / "comm_performance_summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as fh:
        writer = csv.writer(fh)
        writer.writerow(
            (
                "topic", "session_index", "message_count", "unique_sequence_count",
                "expected_sequence_count", "missing_sequence_count", "duplicate_count",
                "out_of_order_count", "packet_delivery_ratio", "mean_message_age_s",
                "p50_message_age_s", "p95_message_age_s", "p99_message_age_s",
                "max_message_age_s", "actual_rate_hz", "stale_state_candidate_events",
                "mean_message_size_bytes", "mean_bandwidth_bytes_per_s",
                "peak_bandwidth_bytes_per_s",
            )
        )
        for topic, data in result["topics"].items():
            if "sessions" not in data:
                continue
            for idx, (session, peak_bw) in enumerate(
                zip(data["sessions"], data["peak_bandwidth_bytes_per_s_per_session"])
            ):
                writer.writerow(
                    (
                        topic, idx, session["message_count"], session["unique_sequence_count"],
                        session["expected_sequence_count"], session["missing_sequence_count"],
                        session["duplicate_count"], session["out_of_order_count"],
                        session["packet_delivery_ratio"], session["mean_message_age_s"],
                        session["p50_message_age_s"], session["p95_message_age_s"],
                        session["p99_message_age_s"], session["max_message_age_s"],
                        session["actual_rate_hz"], session["stale_state_candidate_events"],
                        session["mean_message_size_bytes"], session["mean_bandwidth_bytes_per_s"],
                        peak_bw,
                    )
                )


def _arguments():
    parser = argparse.ArgumentParser(
        description="Read-only communication-performance metrics for an EpuckState-carrying rosbag."
    )
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--warmup-s", type=float, default=2.0)
    parser.add_argument("--cooldown-s", type=float, default=2.0)
    parser.add_argument("--peer-timeout-s", type=float, default=0.5)
    return parser.parse_args()


def main():
    args = _arguments()
    bag_path = args.bag_path.expanduser().resolve()
    output_dir = args.output_dir or bag_path / "analysis"
    result = analyze(
        bag_path,
        warmup_s=args.warmup_s,
        cooldown_s=args.cooldown_s,
        peer_timeout_s=args.peer_timeout_s,
    )
    _write_outputs(output_dir, result)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"Communication-performance analysis written to: {output_dir}")


if __name__ == "__main__":
    main()
