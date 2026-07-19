#!/usr/bin/env python3
"""Objective 5 step 3: comm baseline pilot acceptance checks.

Verifies, against a real recorded run with zero-impairment relays inserted:
- sequence/PDR algorithm correctness (both state_raw and relayed state);
- message age is non-negative and clock-consistent;
- the zero-delay/zero-loss relay does not change message content;
- direct (state_raw) vs relayed (state) publish/receive frequency agree;
- no mis-triggered stale-state safety stop beyond the expected one-time
  startup transition per robot;
- analyzer-derived counts agree with the relay's own CSV log;
- realtime factor in 0.8-1.2.

Read-only: reads the bag and the relay's own log files, does not touch the
frozen controller or the frozen EpuckState message.
"""
import csv
import json
import os
import re
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from epuck2_comm.analyze_comm_performance import analyze as analyze_comm_performance

BAG_DIR = Path(os.environ["BAG_DIR"])
EXPERIMENT_DIR = Path(os.environ["EXPERIMENT_DIR"])
STEM = os.environ["STEM"]
PRELOAD_FACTOR = float(os.environ["PRELOAD_FACTOR"])
FULL_LOAD_FACTOR = float(os.environ["FULL_LOAD_FACTOR"])
COMPLETE_COUNT = int(os.environ["COMPLETE_COUNT"])
CONTROLLER_LOG = EXPERIMENT_DIR / "logs" / f"{STEM}.log"

RAW_TOPICS = ("/epuck1/state_raw", "/epuck2/state_raw")
RELAYED_TOPICS = ("/epuck1/state", "/epuck2/state")


def read_state_topic(bag_path: Path, topic: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_names = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_names:
        return []
    msg_type = get_message(type_names[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while reader.has_next():
        _, raw, ts = reader.read_next()
        message = deserialize_message(raw, msg_type)
        rows.append((ts, message))
    rows.sort(key=lambda r: r[0])
    return rows


def main():
    reasons_fail = []

    comm_result = analyze_comm_performance(BAG_DIR, warmup_s=2.0, cooldown_s=2.0)
    relayed_summary = comm_result["topics"]

    for topic in RELAYED_TOPICS:
        data = relayed_summary.get(topic, {})
        sessions = data.get("sessions", [])
        if not sessions:
            reasons_fail.append(f"{topic}: no analyzable session (empty or too short)")
            continue
        pdr = data.get("overall_packet_delivery_ratio")
        if pdr is None or pdr < 0.999:
            reasons_fail.append(f"{topic}: overall_packet_delivery_ratio={pdr} (<0.999, PDR algorithm or real loss issue)")
        for idx, session in enumerate(sessions):
            if session["negative_age_sample_count"] != 0:
                reasons_fail.append(
                    f"{topic} session {idx}: {session['negative_age_sample_count']} negative message-age samples (clock inconsistency)"
                )
            if session["duplicate_count"] != 0:
                reasons_fail.append(f"{topic} session {idx}: {session['duplicate_count']} duplicate messages unexpected at zero impairment")

    # Raw-vs-relayed content and frequency comparison.
    raw_rows = {topic: read_state_topic(BAG_DIR, topic) for topic in RAW_TOPICS}
    relayed_rows = {topic: read_state_topic(BAG_DIR, topic) for topic in RELAYED_TOPICS}

    content_mismatches = 0
    for raw_topic, relayed_topic in zip(RAW_TOPICS, RELAYED_TOPICS):
        raw_by_seq = {m.sequence: m for _, m in raw_rows[raw_topic]}
        relayed_by_seq = {m.sequence: m for _, m in relayed_rows[relayed_topic]}
        if not raw_by_seq or not relayed_by_seq:
            reasons_fail.append(f"{raw_topic}/{relayed_topic}: one or both topics have no messages")
            continue
        common = set(raw_by_seq) & set(relayed_by_seq)
        if len(common) < 0.99 * len(raw_by_seq):
            reasons_fail.append(
                f"{raw_topic}->{relayed_topic}: only {len(common)}/{len(raw_by_seq)} raw sequences reached the relayed topic (zero-loss relay should forward ~all)"
            )
        for seq in common:
            a, b = raw_by_seq[seq], relayed_by_seq[seq]
            if (a.x_m, a.y_m, a.yaw_rad, a.sequence, a.robot_id) != (b.x_m, b.y_m, b.yaw_rad, b.sequence, b.robot_id):
                content_mismatches += 1
        raw_rate = relayed_summary.get(raw_topic, {}).get("sessions", [{}])[0].get("actual_rate_hz")
        relayed_rate = relayed_summary.get(relayed_topic, {}).get("sessions", [{}])[0].get("actual_rate_hz")
        if raw_rate and relayed_rate and abs(raw_rate - relayed_rate) / raw_rate > 0.10:
            reasons_fail.append(
                f"{raw_topic} rate={raw_rate:.3f}Hz vs {relayed_topic} rate={relayed_rate:.3f}Hz differ by >10%"
            )
    if content_mismatches:
        reasons_fail.append(f"{content_mismatches} messages had different content between state_raw and relayed state (relay must never mutate content)")

    # Relay's own CSV log cross-check.
    for namespace in ("epuck1", "epuck2"):
        relay_log_path = EXPERIMENT_DIR / "logs" / f"{STEM}_{namespace}_relay.csv"
        if not relay_log_path.exists():
            reasons_fail.append(f"relay log missing: {relay_log_path}")
            continue
        with relay_log_path.open(encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
        forwarded = sum(1 for r in rows if r["action"] == "forwarded")
        dropped = sum(1 for r in rows if r["action"] == "dropped")
        if dropped != 0:
            reasons_fail.append(f"{namespace} relay log shows {dropped} dropped messages at zero drop_probability")
        relayed_topic = f"/{namespace}/state"
        analyzer_count = relayed_summary.get(relayed_topic, {}).get("message_count_total_in_bag")
        if analyzer_count is not None and abs(forwarded - analyzer_count) > max(2, int(0.02 * forwarded)):
            reasons_fail.append(
                f"{namespace}: relay log forwarded_count={forwarded} vs analyzer message_count_total_in_bag={analyzer_count} disagree by more than 2%"
            )

    # Stale-state mis-trigger check: at most one SAFE_STOP_STALE transition
    # per robot is expected (the very first tick, before any message has
    # arrived at all); any more indicates the relay caused a spurious
    # freshness failure.
    if CONTROLLER_LOG.exists():
        log_text = CONTROLLER_LOG.read_text(encoding="utf-8", errors="replace")
        stale_transitions = len(re.findall(r"->SAFE_STOP_STALE", log_text))
        if stale_transitions > 2:
            reasons_fail.append(
                f"{stale_transitions} SAFE_STOP_STALE transitions observed (>2, expected at most 1 per robot at startup) -- possible relay-induced staleness"
            )
        if "COMPLETE: maximum runtime reached" in log_text:
            reasons_fail.append("at least one robot stopped via maximum runtime, not task completion")
    else:
        reasons_fail.append(f"controller log missing: {CONTROLLER_LOG}")

    if COMPLETE_COUNT < 2:
        reasons_fail.append(f"only {COMPLETE_COUNT}/2 robots logged cooperative recovery COMPLETE")

    realtime_ok = 0.8 <= PRELOAD_FACTOR <= 1.2 and 0.8 <= FULL_LOAD_FACTOR <= 1.2
    if not realtime_ok:
        reasons_fail.append(f"realtime factor out of range (preload={PRELOAD_FACTOR}, full_load={FULL_LOAD_FACTOR})")

    verdict = "PASS" if not reasons_fail else "FAIL"

    result = {
        "verdict": verdict,
        "fail_reasons": reasons_fail,
        "complete_count": COMPLETE_COUNT,
        "content_mismatches": content_mismatches,
        "preload_realtime_factor": PRELOAD_FACTOR,
        "full_load_realtime_factor": FULL_LOAD_FACTOR,
        "realtime_factor_ok": realtime_ok,
        "comm_performance_summary_ref": "analysis/comm_performance_summary.json",
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
