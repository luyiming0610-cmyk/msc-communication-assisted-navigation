#!/usr/bin/env python3
import csv
import json
import re
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def state_time_factor(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {"/epuck1/state", "/epuck2/state"}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
    samples = {topic: [] for topic in wanted}
    while reader.has_next():
        topic, raw, recorded_ns = reader.read_next()
        message = deserialize_message(raw, get_message(topic_types[topic]))
        stamp_ns = int(message.stamp.sec) * 1_000_000_000 + int(message.stamp.nanosec)
        samples[topic].append((int(recorded_ns), stamp_ns))
    factors = {}
    for topic, values in samples.items():
        if len(values) < 2:
            raise RuntimeError(f"insufficient state samples for {topic}")
        recorded_delta = values[-1][0] - values[0][0]
        sim_delta = values[-1][1] - values[0][1]
        factors[topic] = sim_delta / recorded_delta
    return factors, sum(factors.values()) / len(factors)


def avoidance_onset_skew(commands_path: Path):
    first = {}
    with commands_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            topic = row["topic"]
            angular = float(row["angular_rps"])
            if abs(angular) >= 0.2 and topic not in first:
                first[topic] = float(row["time_s"])
    required = {"/epuck1/cmd_vel", "/epuck2/cmd_vel"}
    if set(first) != required:
        raise RuntimeError(f"avoidance onset missing: {first}")
    return first, abs(first["/epuck1/cmd_vel"] - first["/epuck2/cmd_vel"])


def logged_factors(log_path: Path):
    if not log_path.exists():
        return None, None
    text = log_path.read_text(encoding="utf-8", errors="replace")
    pre = re.search(r"PRELOAD_REALTIME_FACTOR=([0-9.]+)", text)
    full = re.search(r"FULL_LOAD_REALTIME_FACTOR=([0-9.]+)", text)
    return (float(pre.group(1)) if pre else None, float(full.group(1)) if full else None)


base = Path(sys.argv[1])
results = []
for number in range(1, 6):
    trial = f"{number:02d}"
    stem = f"ablation_local_only_head_on_realtime_formal_trial_{trial}"
    bag = base / "bags" / stem
    summary = json.loads((bag / "analysis" / "summary.json").read_text(encoding="utf-8"))
    factors, mean_factor = state_time_factor(bag)
    onset, onset_skew = avoidance_onset_skew(bag / "analysis" / "commands.csv")
    pre, full = logged_factors(base / "logs" / f"{stem}_execution.log")
    results.append(
        {
            "trial": trial,
            "preload_factor": pre,
            "full_load_factor": full,
            "bag_state_time_factor": mean_factor,
            "state_time_factor_by_robot": factors,
            "minimum_center_separation_m": summary["minimum_center_separation_m"],
            "minimum_safety_margin_m": summary["minimum_safety_margin_m"],
            "collision_detected": summary["collision_detected"],
            "invalid_state_total": sum(summary["invalid_state_messages"].values()),
            "motion_start_skew_s": summary["motion_start_skew_s"],
            "avoidance_onset_s": onset,
            "avoidance_onset_skew_s": onset_skew,
            "last_motion_command_skew_s": summary["last_motion_command_skew_s"],
            "epuck1_angular_sign_changes": summary["commands"]["/epuck1/cmd_vel"]["angular_sign_changes"],
            "epuck2_angular_sign_changes": summary["commands"]["/epuck2/cmd_vel"]["angular_sign_changes"],
        }
    )

print(json.dumps(results, indent=2, ensure_ascii=False))
