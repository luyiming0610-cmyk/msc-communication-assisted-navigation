#!/usr/bin/env python3
"""Minimum inter-robot distance over a trial's main window, for the
UNSAFE_FAILURE collision heuristic in matrix_verdict.py. Reads
/epuck1/state and /epuck2/state x_m/y_m fields directly from the bag --
same read pattern as analyze_trigger_reason.py's _read_state_bag, kept
separate here so the pure geometry (testable) is not entangled with
rosbag2_py I/O (only importable inside a sourced ROS environment).
"""
from __future__ import annotations

import math


def min_distance_from_paired_positions(positions_a: list, positions_b: list) -> float | None:
    """positions_a/positions_b: lists of (timestamp_ns, x_m, y_m), one
    per robot, NOT required to be time-aligned message-for-message (real
    bags rarely are). For each sample of the FASTER-sampled robot, pairs
    it with the nearest-in-time sample of the other robot (nearest-
    neighbor pairing, not interpolation -- simple and conservative: it
    can only ever report a distance at moments both robots' positions
    were actually observed close in time, never a fabricated
    interpolated value). Returns None if either list is empty (distance
    genuinely not computable, never silently 0.0 or some other default)."""
    if not positions_a or not positions_b:
        return None
    # Iterate the shorter list against the longer one's timestamps via
    # simple linear-scan nearest neighbor -- trial bags are a few
    # thousand samples, this is not a hot path.
    b_times = [p[0] for p in positions_b]
    min_distance = None
    for t_a, x_a, y_a in positions_a:
        # nearest neighbor in positions_b by timestamp
        nearest = min(range(len(positions_b)), key=lambda i: abs(b_times[i] - t_a))
        _, x_b, y_b = positions_b[nearest]
        distance = math.hypot(x_a - x_b, y_a - y_b)
        if min_distance is None or distance < min_distance:
            min_distance = distance
    return min_distance


def _read_bag_positions(bag_path, topic: str, window_start_s: float | None, window_end_s: float | None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

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
        _, raw, ts_ns = reader.read_next()
        if window_start_s is not None and ts_ns / 1.0e9 < window_start_s:
            continue
        if window_end_s is not None and ts_ns / 1.0e9 > window_end_s:
            continue
        message = deserialize_message(raw, msg_type)
        rows.append((ts_ns, float(message.x_m), float(message.y_m)))
    return rows


def compute_min_interrobot_distance(bag_path, window_start_s=None, window_end_s=None) -> float | None:
    positions_1 = _read_bag_positions(bag_path, "/epuck1/state", window_start_s, window_end_s)
    positions_2 = _read_bag_positions(bag_path, "/epuck2/state", window_start_s, window_end_s)
    return min_distance_from_paired_positions(positions_1, positions_2)


def main():
    import argparse
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser()
    parser.add_argument("--native-bag-dir", type=Path, required=True)
    parser.add_argument("--window-start-s", type=float, default=None)
    parser.add_argument("--window-end-s", type=float, default=None)
    args = parser.parse_args()
    result = compute_min_interrobot_distance(args.native_bag_dir, args.window_start_s, args.window_end_s)
    print(json.dumps({"min_interrobot_distance_m": result}))


if __name__ == "__main__":
    main()
