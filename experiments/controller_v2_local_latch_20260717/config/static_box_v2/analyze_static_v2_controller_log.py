#!/usr/bin/env python3
"""controller_v2_local_latch_20260717 pilot analysis: mode-sequence and
cumulative side-turn checks, specific to validating the new side-lane phase
state machine (TURNING/CAPPED_BYPASS/RECOVERY_ALLOWED/FAILSAFE). Box
clearance/collision is reported separately by the existing
analyze_combined_task.py (reused unmodified against the same bag).
"""

import argparse
import json
import re
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


MODE_RE = re.compile(r"\[(\d+\.\d+)\].*mode=(\S+)")


def parse_mode_transitions(log_path: Path):
    transitions = []
    last_mode = None
    with log_path.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            match = MODE_RE.search(line)
            if not match:
                continue
            t, mode = float(match.group(1)), match.group(2)
            if mode != last_mode:
                transitions.append((t, mode))
                last_mode = mode
    return transitions


def cumulative_side_turn(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if "/epuck1/cmd_vel" not in types:
        return 0.0, 0.0
    msg_type = get_message(types["/epuck1/cmd_vel"])
    prev_t = None
    total = 0.0
    peak_yaw = 0.0
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/epuck1/cmd_vel"]))
    first_ns = None
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if first_ns is None:
            first_ns = t_ns
        t_s = (t_ns - first_ns) / 1.0e9
        msg = deserialize_message(raw, msg_type)
        if prev_t is not None:
            total += abs(float(msg.angular.z)) * (t_s - prev_t)
        prev_t = t_s

    reader2 = rosbag2_py.SequentialReader()
    reader2.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    if "/epuck1/state" in types:
        state_type = get_message(types["/epuck1/state"])
        reader2.set_filter(rosbag2_py.StorageFilter(topics=["/epuck1/state"]))
        while reader2.has_next():
            _, raw, _ = reader2.read_next()
            msg = deserialize_message(raw, state_type)
            peak_yaw = max(peak_yaw, abs(float(msg.yaw_rad)))
    return total, peak_yaw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("controller_log", type=Path)
    args = parser.parse_args()

    transitions = parse_mode_transitions(args.controller_log)
    modes_seen = [mode for _, mode in transitions]

    left_count = modes_seen.count("LOCAL_LEFT_SIDE")
    right_count = modes_seen.count("LOCAL_RIGHT_SIDE")
    clearance_count = modes_seen.count("LOCAL_CLEARANCE")
    side_bypass_occurred = "LOCAL_SIDE_BYPASS" in modes_seen
    recovery_ready_occurred = "LOCAL_RECOVERY_READY" in modes_seen
    local_recover_occurred = "LOCAL_RECOVER" in modes_seen
    failsafe_occurred = "LOCAL_SIDE_ENCOUNTER_FAILSAFE" in modes_seen
    safe_stop_occurred = "SAFE_STOP_LOCAL_SENSORS" in modes_seen
    stale_or_invalid_occurred = any(
        m in ("SAFE_STOP_STALE", "SAFE_STOP_INVALID_ODOM") for m in modes_seen
    )
    task_complete = any(m == "COMPLETE" for m in modes_seen)

    side_trigger_modes = {"LOCAL_LEFT_SIDE", "LOCAL_RIGHT_SIDE", "LOCAL_CLEARANCE"}
    max_consecutive_side_retriggers = 0
    run = 0
    for mode in modes_seen:
        if mode in side_trigger_modes:
            run += 1
            max_consecutive_side_retriggers = max(max_consecutive_side_retriggers, run)
        else:
            run = 0

    cumulative_turn_rad, peak_yaw_rad = cumulative_side_turn(args.bag_path)

    summary = {
        "bag_path": str(args.bag_path),
        "controller_log": str(args.controller_log),
        "mode_transitions": transitions,
        "local_left_side_count": left_count,
        "local_right_side_count": right_count,
        "local_clearance_count": clearance_count,
        "max_consecutive_side_retriggers": max_consecutive_side_retriggers,
        "side_bypass_occurred": side_bypass_occurred,
        "recovery_ready_occurred": recovery_ready_occurred,
        "local_recover_occurred": local_recover_occurred,
        "failsafe_occurred": failsafe_occurred,
        "safe_stop_local_sensors_occurred": safe_stop_occurred,
        "stale_or_invalid_state_occurred": stale_or_invalid_occurred,
        "task_complete_seen": task_complete,
        "cumulative_side_cmd_vel_turn_rad": cumulative_turn_rad,
        "peak_state_yaw_rad": peak_yaw_rad,
        "final_mode": modes_seen[-1] if modes_seen else None,
    }
    output = args.bag_path / "analysis" / "static_v2_controller_log_summary.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"static v2 controller-log analysis written to: {output}")


if __name__ == "__main__":
    main()
