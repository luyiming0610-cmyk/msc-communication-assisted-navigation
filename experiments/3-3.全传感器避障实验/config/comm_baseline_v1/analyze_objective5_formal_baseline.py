#!/usr/bin/env python3
"""Objective5 formal zero-impairment baseline acceptance checks.

Builds on analyze_measurement_chain.py's relay-vs-bag isolation logic
(imported directly, not duplicated) and adds the task-level checks needed
for a genuine formal baseline: sequence_counter completeness, controller
completion reason, and communication metrics (PDR, message rate, age,
bandwidth) pulled from analyze_comm_performance's own output. Read-only;
does not touch the frozen controller or the frozen EpuckState message.
"""
import argparse
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_measurement_chain import (  # noqa: E402
    bag_topic_stats,
    extract_relay_config,
    read_relay_csv,
)

RAW_TOPICS = ("/epuck1/state_raw", "/epuck2/state_raw")
RELAYED_TOPICS = ("/epuck1/state", "/epuck2/state")


def _read_counter(diag_log_dir: Path, namespace: str, topic_key: str):
    # sequence_counter.py's JSON is keyed by the un-namespaced topic name
    # it was passed via --topics (e.g. "state", "state_raw"), since the
    # node itself is launched inside a ROS namespace -- not by the fully
    # resolved topic path ("/epuck1/state").
    path = diag_log_dir / f"{namespace}_counter.json"
    if not path.exists():
        return None, f"{namespace}: counter output file missing entirely ({path})"
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return None, f"{namespace}: counter output file is not valid JSON ({exc})"
    if not data.get("complete", False):
        return data, f"{namespace}: counter's own complete flag is false (abnormal shutdown or checkpoint-only data)"
    topic_data = data.get(topic_key)
    if topic_data is None:
        return data, f"{namespace}: counter has no data for topic key '{topic_key}'"
    return data, None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-bag-dir", type=Path, required=True)
    parser.add_argument("--diag-log-dir", type=Path, required=True)
    parser.add_argument("--controller-log", type=Path, required=True)
    parser.add_argument("--state1-log", type=Path, required=True)
    parser.add_argument("--state2-log", type=Path, required=True)
    parser.add_argument("--bag-record-log", type=Path, required=True)
    parser.add_argument("--relay-counter-log", type=Path, required=True)
    parser.add_argument("--preload-factor", type=float, required=True)
    parser.add_argument("--full-load-factor", type=float, required=True)
    parser.add_argument("--complete-count", type=int, required=True)
    parser.add_argument("--output-path", type=Path, required=True)
    args = parser.parse_args()

    reasons_fail = []
    per_robot = {}

    for namespace in ("epuck1", "epuck2"):
        raw_topic = f"/{namespace}/state_raw"
        relayed_topic = f"/{namespace}/state"

        bag_raw = bag_topic_stats(args.native_bag_dir, raw_topic)
        bag_relayed = bag_topic_stats(args.native_bag_dir, relayed_topic)
        relay = read_relay_csv(args.diag_log_dir / f"{namespace}_relay.csv")
        relay_config = extract_relay_config(args.relay_counter_log, namespace)

        if relay_config is None:
            reasons_fail.append(f"{namespace}: could not confirm relay zero-impairment configuration from its log")
        elif not relay_config["immediate_passthrough"] or relay_config["drop_probability"] != 0.0:
            reasons_fail.append(f"{namespace}: relay was NOT configured for zero impairment: {relay_config}")
        if relay["dropped"] != 0:
            reasons_fail.append(f"{namespace}: relay dropped {relay['dropped']} messages at zero drop_probability")

        counter_relayed_data, counter_error = _read_counter(args.diag_log_dir, namespace, "state")
        if counter_error:
            reasons_fail.append(counter_error)
        counter_relayed = (counter_relayed_data or {}).get("state", {})
        if counter_relayed:
            if counter_relayed.get("sequence_gap_count", 0) != 0:
                reasons_fail.append(f"{namespace}: counter observed {counter_relayed['sequence_gap_count']} sequence gaps on relayed topic")
            if counter_relayed.get("duplicate_count", 0) != 0:
                reasons_fail.append(f"{namespace}: counter observed {counter_relayed['duplicate_count']} duplicates on relayed topic")
            if counter_relayed.get("out_of_order_count", 0) != 0:
                reasons_fail.append(f"{namespace}: counter observed {counter_relayed['out_of_order_count']} out-of-order messages on relayed topic")

        bag_relayed_seqs = bag_relayed["sequences"]
        relay_seqs = relay["sequences_forwarded"]
        window_relay = (
            {s for s in relay_seqs if (bag_relayed["first_sequence"] or 0) <= s <= (bag_relayed["last_sequence"] or -1)}
            if bag_relayed_seqs else set()
        )
        aligned_pdr = (len(window_relay & bag_relayed_seqs) / len(window_relay)) if window_relay else None
        if aligned_pdr is None or aligned_pdr < 0.99:
            reasons_fail.append(f"{namespace}: aligned-window PDR (bag vs relay-forwarded) = {aligned_pdr} (<0.99)")

        per_robot[namespace] = {
            "bag_raw": {k: v for k, v in bag_raw.items() if k != "sequences"},
            "bag_relayed": {k: v for k, v in bag_relayed.items() if k != "sequences"},
            "relay": {k: v for k, v in relay.items() if k != "sequences_forwarded"},
            "counter_relayed": counter_relayed,
            "counter_complete": (counter_relayed_data or {}).get("complete"),
            "relay_config": relay_config,
            "aligned_window_pdr_bag_vs_relay": aligned_pdr,
        }

    # Controller-level checks: genuine task completion, not max_runtime/
    # FAILSAFE/TASK_TIMEOUT.
    if args.controller_log.exists():
        log_text = args.controller_log.read_text(encoding="utf-8", errors="replace")
    else:
        log_text = ""
        reasons_fail.append(f"controller log missing: {args.controller_log}")

    if args.complete_count < 2:
        reasons_fail.append(f"only {args.complete_count}/2 robots logged cooperative recovery COMPLETE")
    if "COMPLETE: maximum runtime reached" in log_text:
        reasons_fail.append("at least one robot stopped via maximum runtime, not task completion")
    if re.search(r"failsafe_cause=(?!NONE)[A-Z_]+", log_text) or "LOCAL_ENCOUNTER_FAILSAFE" in log_text:
        reasons_fail.append("FAILSAFE observed in controller log")
    if "SAFE_STOP_INVALID_ODOM" in log_text:
        reasons_fail.append("SENSOR_INVALID/invalid-odom observed in controller log")
    if "TASK_TIMEOUT" in log_text:
        reasons_fail.append("TASK_TIMEOUT observed")

    # rosbag recorder health.
    bag_record_text = args.bag_record_log.read_text(encoding="utf-8", errors="replace") if args.bag_record_log.exists() else ""
    drop_warn_lines = [line for line in bag_record_text.splitlines() if re.search(r"drop|warn|error", line, re.IGNORECASE)]
    if drop_warn_lines:
        reasons_fail.append(f"rosbag record log contains {len(drop_warn_lines)} drop/warn/error line(s)")

    realtime_ok = 0.8 <= args.preload_factor <= 1.2 and 0.8 <= args.full_load_factor <= 1.2
    if not realtime_ok:
        reasons_fail.append(f"realtime factor out of range (preload={args.preload_factor}, full_load={args.full_load_factor})")

    def _or_na(value):
        # dict.get(key, "N/A") only falls back when the key is MISSING; a
        # key present with value None (e.g. every age sample excluded as a
        # clock-domain-mismatch anomaly) would otherwise render as JSON
        # null, not the required "N/A".
        return value if value is not None else "N/A"

    # Communication metrics: pulled from analyze_comm_performance's own
    # output (already run against the same bag before this script, see
    # the orchestration .sh). Never recomputed here; N/A if missing.
    # Latency specifically also cross-checked against sequence_counter's
    # own live, same-clock-domain measurement (message.stamp vs its own
    # get_clock().now() at receipt) -- see protocol_v1.1_stamp_semantics.
    comm_perf_path = args.diag_log_dir / "comm_performance_summary.json"
    comm_metrics = {"note": "comm_performance_summary.json not found -- N/A"}
    if comm_perf_path.exists():
        comm_perf = json.loads(comm_perf_path.read_text(encoding="utf-8"))
        comm_metrics = {}
        for topic in RELAYED_TOPICS:
            namespace = topic.split("/")[1]
            topic_data = comm_perf.get("topics", {}).get(topic, {})
            sessions = topic_data.get("sessions", [])
            session = sessions[0] if sessions else {}
            counter_relayed = per_robot.get(namespace, {}).get("counter_relayed") or {}
            mismatch = session.get("latency_domain_mismatch_detected", False)
            comm_metrics[topic] = {
                "packet_delivery_ratio": _or_na(topic_data.get("overall_packet_delivery_ratio")),
                "actual_rate_hz": _or_na(session.get("actual_rate_hz")),
                "mean_bandwidth_bytes_per_s": _or_na(session.get("mean_bandwidth_bytes_per_s")),
                "bag_based_mean_message_age_s": "N/A" if mismatch else _or_na(session.get("mean_message_age_s")),
                "bag_based_p95_message_age_s": "N/A" if mismatch else _or_na(session.get("p95_message_age_s")),
                "bag_based_latency_domain_mismatch_detected": mismatch,
                "live_counter_mean_message_age_s": _or_na(counter_relayed.get("mean_message_age_s")),
                "live_counter_median_message_age_s": _or_na(counter_relayed.get("median_message_age_s")),
                "live_counter_p95_message_age_s": _or_na(counter_relayed.get("p95_message_age_s")),
                "live_counter_max_message_age_s": _or_na(counter_relayed.get("max_message_age_s")),
                "live_counter_valid_age_sample_count": _or_na(counter_relayed.get("valid_age_sample_count")),
                "live_counter_negative_age_sample_count": _or_na(counter_relayed.get("negative_age_sample_count")),
                "live_counter_anomalous_age_sample_count": _or_na(counter_relayed.get("anomalous_age_sample_count")),
            }

    verdict = "PASS" if not reasons_fail else "FAIL"
    result = {
        "verdict": verdict,
        "fail_reasons": reasons_fail,
        "per_robot": per_robot,
        "complete_count": args.complete_count,
        "communication_metrics": comm_metrics,
        "bag_record_drop_warn_error_lines": drop_warn_lines[:20],
        "preload_realtime_factor": args.preload_factor,
        "full_load_realtime_factor": args.full_load_factor,
        "realtime_factor_ok": realtime_ok,
    }
    args.output_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
