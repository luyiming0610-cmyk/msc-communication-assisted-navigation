#!/usr/bin/env python3
"""Measurement-chain isolation diagnostic analysis.

Four-way comparison per robot/topic: relay CSV, sequence_counter JSON,
rosbag (read directly), and state_publisher's own throttled log line, all
compared over their common/intersection sequence window (never by naive
total-row-count comparison across sources that started/stopped observing
at different times). Read-only: does not touch the frozen controller or
any already-sealed evidence.
"""

import argparse
import csv
import json
import re
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag_topic(bag_dir: Path, topic: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
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
        rows.append((ts, int(message.sequence)))
    rows.sort(key=lambda r: r[0])
    return rows


def bag_topic_stats(bag_dir: Path, topic: str):
    rows = read_bag_topic(bag_dir, topic)
    if not rows:
        return {
            "received_count": 0, "unique_sequence_count": 0,
            "first_sequence": None, "last_sequence": None,
            "duplicate_count": 0, "out_of_order_count": 0,
            "sequences": set(),
        }
    seqs = [r[1] for r in rows]
    seen = {}
    out_of_order = 0
    previous = None
    for s in seqs:
        seen[s] = seen.get(s, 0) + 1
        if previous is not None and s < previous:
            out_of_order += 1
        previous = s
    duplicate_count = sum(c - 1 for c in seen.values() if c > 1)
    return {
        "received_count": len(rows),
        "unique_sequence_count": len(seen),
        "first_sequence": seqs[0],
        "last_sequence": seqs[-1],
        "duplicate_count": duplicate_count,
        "out_of_order_count": out_of_order,
        "sequences": set(seen),
    }


def read_relay_csv(path: Path):
    if not path.exists():
        return {"forwarded": 0, "dropped": 0, "sequences_forwarded": set(), "config_zero_impairment": None}
    with path.open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    forwarded = [int(r["received_seq"]) for r in rows if r["action"] == "forwarded"]
    dropped = sum(1 for r in rows if r["action"] == "dropped")
    return {
        "forwarded": len(forwarded),
        "dropped": dropped,
        "sequences_forwarded": set(forwarded),
        "first_sequence": min(forwarded) if forwarded else None,
        "last_sequence": max(forwarded) if forwarded else None,
    }


def read_counter_json(path: Path, topic_key: str):
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    return data.get(topic_key)


def extract_relay_config(relay_counter_log: Path, namespace: str):
    if not relay_counter_log.exists():
        return None
    text = relay_counter_log.read_text(encoding="utf-8", errors="replace")
    pattern = re.compile(
        rf"\[{namespace}\.network_impairment_relay\].*network_impairment_relay: "
        r"delay_s=([\d.]+) jitter_s=([\d.]+) drop_probability=([\d.]+) seed=(-?\d+) "
        r"immediate_passthrough=(True|False)"
    )
    match = pattern.search(text)
    if not match:
        return None
    return {
        "delay_s": float(match.group(1)),
        "jitter_s": float(match.group(2)),
        "drop_probability": float(match.group(3)),
        "immediate_passthrough": match.group(5) == "True",
    }


def extract_publisher_last_count(state_log: Path):
    if not state_log.exists():
        return None
    text = state_log.read_text(encoding="utf-8", errors="replace")
    matches = re.findall(r"published=(\d+)", text)
    return int(matches[-1]) if matches else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-bag-dir", type=Path, required=True)
    parser.add_argument("--diag-log-dir", type=Path, required=True)
    parser.add_argument("--state1-log", type=Path, required=True)
    parser.add_argument("--state2-log", type=Path, required=True)
    parser.add_argument("--bag-record-log", type=Path, required=True)
    parser.add_argument("--relay-counter-log", type=Path, required=True)
    parser.add_argument("--preload-factor", type=float, required=True)
    parser.add_argument("--full-load-factor", type=float, required=True)
    parser.add_argument("--output-path", type=Path, required=True)
    args = parser.parse_args()

    reasons_fail = []
    per_robot = {}
    candidate_logs = [args.relay_counter_log]

    for namespace, state_log in (("epuck1", args.state1_log), ("epuck2", args.state2_log)):
        raw_topic = f"/{namespace}/state_raw"
        relayed_topic = f"/{namespace}/state"

        bag_raw = bag_topic_stats(args.native_bag_dir, raw_topic)
        bag_relayed = bag_topic_stats(args.native_bag_dir, relayed_topic)
        relay = read_relay_csv(args.diag_log_dir / f"{namespace}_relay.csv")
        counter_raw = read_counter_json(args.diag_log_dir / f"{namespace}_counter.json", raw_topic) or {}
        counter_relayed = read_counter_json(args.diag_log_dir / f"{namespace}_counter.json", relayed_topic) or {}
        publisher_last_count = extract_publisher_last_count(state_log)

        relay_config = None
        for log_path in candidate_logs:
            relay_config = extract_relay_config(log_path, namespace)
            if relay_config:
                break

        if relay_config is None:
            reasons_fail.append(f"{namespace}: could not confirm relay zero-impairment configuration from its log")
        elif not relay_config["immediate_passthrough"] or relay_config["drop_probability"] != 0.0:
            reasons_fail.append(f"{namespace}: relay was NOT configured for zero impairment: {relay_config}")

        if relay["dropped"] != 0:
            reasons_fail.append(f"{namespace}: relay dropped {relay['dropped']} messages at zero drop_probability")

        counter_relayed_seqs = None
        if counter_relayed:
            if counter_relayed.get("out_of_order_count", 0) != 0:
                reasons_fail.append(f"{namespace}: counter observed {counter_relayed['out_of_order_count']} out-of-order on relayed topic")
            if counter_relayed.get("duplicate_count", 0) != 0:
                reasons_fail.append(f"{namespace}: counter observed {counter_relayed['duplicate_count']} duplicates on relayed topic")

        # Aligned-window comparison: bag vs relay's forwarded sequences,
        # over their common intersection (never naive total-count diff).
        bag_relayed_seqs = bag_relayed["sequences"]
        relay_seqs = relay["sequences_forwarded"]
        common = bag_relayed_seqs & relay_seqs if (bag_relayed_seqs and relay_seqs) else set()
        aligned_pdr = (len(bag_relayed_seqs & common) / len(relay_seqs)) if relay_seqs else None
        # More precisely: within the window relay actually forwarded, what
        # fraction did the bag also capture?
        window_relay = {s for s in relay_seqs if (bag_relayed["first_sequence"] or 0) <= s <= (bag_relayed["last_sequence"] or -1)} if bag_relayed_seqs else set()
        aligned_pdr_windowed = (len(window_relay & bag_relayed_seqs) / len(window_relay)) if window_relay else None

        if aligned_pdr_windowed is not None and aligned_pdr_windowed < 0.99:
            reasons_fail.append(
                f"{namespace}: aligned-window PDR (bag vs relay-forwarded, within bag's own sequence span) = "
                f"{aligned_pdr_windowed:.4f} (<0.99)"
            )

        per_robot[namespace] = {
            "bag_raw": {k: v for k, v in bag_raw.items() if k != "sequences"},
            "bag_relayed": {k: v for k, v in bag_relayed.items() if k != "sequences"},
            "relay": {k: v for k, v in relay.items() if k != "sequences_forwarded"},
            "counter_raw": counter_raw,
            "counter_relayed": counter_relayed,
            "publisher_last_logged_count": publisher_last_count,
            "relay_config": relay_config,
            "aligned_window_pdr_bag_vs_relay": aligned_pdr_windowed,
        }

    bag_record_text = args.bag_record_log.read_text(encoding="utf-8", errors="replace") if args.bag_record_log.exists() else ""
    drop_warn_lines = [
        line for line in bag_record_text.splitlines()
        if re.search(r"drop|warn|error", line, re.IGNORECASE)
    ]
    if drop_warn_lines:
        reasons_fail.append(f"rosbag record log contains {len(drop_warn_lines)} drop/warn/error line(s)")

    realtime_ok = 0.8 <= args.preload_factor <= 1.2 and 0.8 <= args.full_load_factor <= 1.2
    if not realtime_ok:
        reasons_fail.append(f"realtime factor out of range (preload={args.preload_factor}, full_load={args.full_load_factor})")

    verdict = "PASS" if not reasons_fail else "FAIL"
    result = {
        "verdict": verdict,
        "fail_reasons": reasons_fail,
        "per_robot": per_robot,
        "bag_record_drop_warn_error_lines": drop_warn_lines[:20],
        "preload_realtime_factor": args.preload_factor,
        "full_load_realtime_factor": args.full_load_factor,
        "realtime_factor_ok": realtime_ok,
    }
    args.output_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
