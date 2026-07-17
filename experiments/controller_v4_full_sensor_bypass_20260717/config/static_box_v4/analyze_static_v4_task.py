#!/usr/bin/env python3
"""controller_v4_full_sensor_bypass_20260717 pilot task analysis.

Replaces analyze_combined_task.py (reused unmodified since
cooperative_avoidance_20260716) for v4 static-box pilots specifically to fix
a real bug that corrupted pilot_a3's own "max_epuck1_x_m" verdict: a single
post-shutdown /epuck1/state sample at x=0,y=0,yaw=0 (a state_publisher/
Webots teardown artifact recorded AFTER the controller had already reached
max_runtime_s and everything was being torn down) got included in the max()
and min-clearance calculations, producing a false "epuck1_passed_box=true"
that had nothing to do with the actual run.

This script filters to only the controller's own active window -- from the
first CONTROLLER_LOG line to the last one before its process was stopped --
using the controller log itself as the source of truth for "when was this
node actually running", not the bag's full recording span (which includes
pre-armed startup and post-teardown tail). It also explicitly excludes any
sample that is the exact reset-artifact triple (x=0, y=0, yaw=0) even inside
that window, since the same failure mode could in principle recur from a
mid-run node restart.

Per revision-6.2-derived pilot_a3 diagnostic requirements, this script does
NOT compute or print a boolean "PASS" -- collision truth and the joint
success criteria are reported as raw fields for the run script / human
report to combine explicitly, so "script exited 0" can never again be
silently read as "experiment passed".
"""

import argparse
import json
import math
import re
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


COMPLETE_RE = re.compile(r"\[(?P<t>\d+\.\d+)\].*COMPLETE: (?P<msg>.+)")
FIRST_LOG_RE = re.compile(r"\[(?P<t>\d+\.\d+)\]")


def _is_reset_artifact(x, y, yaw):
    return x == 0.0 and y == 0.0 and yaw == 0.0


def _controller_window(log_path: Path):
    """[first_t, last_t] from the controller's own log timestamps -- the
    node's real active span, immune to bag-recording pre-roll/post-roll."""
    first_t = None
    last_t = None
    with log_path.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            m = FIRST_LOG_RE.search(line)
            if not m:
                continue
            t = float(m.group("t"))
            if first_t is None:
                first_t = t
            last_t = t
    return first_t, last_t


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("controller_log", type=Path)
    parser.add_argument("--box-x", type=float, default=-0.25)
    parser.add_argument("--box-y", type=float, default=0.0)
    parser.add_argument("--box-size", type=float, default=0.06)
    parser.add_argument("--robot-radius", type=float, default=0.035)
    parser.add_argument("--pass-x", type=float, default=-0.175)
    parser.add_argument("--danger-zone-x-min", type=float, default=-0.30)
    parser.add_argument("--danger-zone-x-max", type=float, default=-0.20)
    args = parser.parse_args()

    window_start, window_end = _controller_window(args.controller_log)
    if window_start is None:
        raise SystemExit(f"no timestamped lines found in {args.controller_log}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    topics = ("/epuck1/state", "/epuck2/state")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    half = args.box_size / 2.0

    def clearance(x, y):
        dx = max(abs(x - args.box_x) - half, 0.0)
        dy = max(abs(y - args.box_y) - half, 0.0)
        return math.hypot(dx, dy) - args.robot_radius

    minimum_clearance = {t: math.inf for t in topics}
    minimum_row = {t: None for t in topics}
    last_state = {}
    max_epuck1_x = -math.inf
    excluded_reset_artifacts = 0
    excluded_out_of_window = 0
    epuck1_trajectory = []  # (t_s, x, y, yaw) within window, for return-to-danger-zone check

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t_s = t_ns / 1.0e9
        message = deserialize_message(raw, get_message(types[topic]))
        x_m, y_m, yaw_rad = float(message.x_m), float(message.y_m), float(message.yaw_rad)

        if not (window_start <= t_s <= window_end):
            excluded_out_of_window += 1
            continue
        if _is_reset_artifact(x_m, y_m, yaw_rad):
            excluded_reset_artifacts += 1
            continue

        clr = clearance(x_m, y_m)
        if clr < minimum_clearance[topic]:
            minimum_clearance[topic] = clr
            minimum_row[topic] = {"time_s": t_s - window_start, "x_m": x_m, "y_m": y_m}
        last_state[topic] = {"x_m": x_m, "y_m": y_m, "yaw_rad": yaw_rad}
        if topic == "/epuck1/state":
            max_epuck1_x = max(max_epuck1_x, x_m)
            epuck1_trajectory.append((t_s - window_start, x_m, y_m))

    if set(last_state) != set(topics):
        raise SystemExit("Missing state samples inside the controller's active window")

    epuck1_passed_box = max_epuck1_x >= args.pass_x

    # Return-to-danger-zone check: after first passing the box, does the
    # trajectory ever re-enter the box's own x-span at all (regardless of
    # y)? This is what pilot_a3 needed and pilot_a/pilot_a2's analyzer never
    # checked.
    returned_to_danger_zone = False
    passed_once = False
    for t_s, x_m, y_m in epuck1_trajectory:
        if x_m >= args.pass_x:
            passed_once = True
            continue
        if passed_once and args.danger_zone_x_min <= x_m <= args.danger_zone_x_max:
            returned_to_danger_zone = True
            break

    summary = {
        "bag_path": str(args.bag_path),
        "controller_log": str(args.controller_log),
        "controller_window_s": [window_start, window_end],
        "excluded_reset_artifact_samples": excluded_reset_artifacts,
        "excluded_out_of_window_samples": excluded_out_of_window,
        "box": {"x_m": args.box_x, "y_m": args.box_y, "size_m": args.box_size},
        "robot_radius_m": args.robot_radius,
        "pass_x_m": args.pass_x,
        "max_epuck1_x_m": max_epuck1_x,
        "epuck1_passed_box": epuck1_passed_box,
        "returned_to_danger_zone_after_passing": returned_to_danger_zone,
        "minimum_box_clearance_m": minimum_clearance,
        "minimum_box_clearance_state": minimum_row,
        "box_collision_detected": {
            topic: clearance <= 0.0 for topic, clearance in minimum_clearance.items()
        },
        "final_state": last_state,
    }
    output = args.bag_path / "analysis" / "static_v4_task_summary.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"static v4 task analysis written to: {output}")


if __name__ == "__main__":
    main()
