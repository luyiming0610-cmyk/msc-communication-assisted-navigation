#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


parser = argparse.ArgumentParser()
parser.add_argument("bag_path", type=Path)
parser.add_argument("--box-x", type=float, default=-0.25)
parser.add_argument("--box-y", type=float, default=0.0)
parser.add_argument("--box-size", type=float, default=0.06)
parser.add_argument("--robot-radius", type=float, default=0.035)
parser.add_argument("--pass-x", type=float, default=-0.175)
args = parser.parse_args()

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(args.bag_path), storage_id="sqlite3"),
    rosbag2_py.ConverterOptions("cdr", "cdr"),
)
types = {item.name: item.type for item in reader.get_all_topics_and_types()}
topics = ("/epuck1/state", "/epuck2/state")
reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
minimum_clearance = {topic: math.inf for topic in topics}
minimum_row = {topic: None for topic in topics}
last_state = {}
max_epuck1_x = -math.inf
half = args.box_size / 2.0
first_ns = None

while reader.has_next():
    topic, raw, timestamp_ns = reader.read_next()
    if first_ns is None:
        first_ns = timestamp_ns
    message = deserialize_message(raw, get_message(types[topic]))
    x_m = float(message.x_m)
    y_m = float(message.y_m)
    yaw_rad = float(message.yaw_rad)
    time_s = (timestamp_ns - first_ns) / 1.0e9
    dx = max(abs(x_m - args.box_x) - half, 0.0)
    dy = max(abs(y_m - args.box_y) - half, 0.0)
    clearance = math.hypot(dx, dy) - args.robot_radius
    if clearance < minimum_clearance[topic]:
        minimum_clearance[topic] = clearance
        minimum_row[topic] = {
            "time_s": time_s,
            "x_m": x_m,
            "y_m": y_m,
        }
    last_state[topic] = {"x_m": x_m, "y_m": y_m, "yaw_rad": yaw_rad}
    if topic == "/epuck1/state":
        max_epuck1_x = max(max_epuck1_x, x_m)

if set(last_state) != set(topics):
    raise SystemExit("Missing state samples")

summary = {
    "bag_path": str(args.bag_path),
    "box": {"x_m": args.box_x, "y_m": args.box_y, "size_m": args.box_size},
    "robot_radius_m": args.robot_radius,
    "pass_x_m": args.pass_x,
    "max_epuck1_x_m": max_epuck1_x,
    "epuck1_passed_box": max_epuck1_x >= args.pass_x,
    "minimum_box_clearance_m": minimum_clearance,
    "minimum_box_clearance_state": minimum_row,
    "box_collision_detected": {
        topic: clearance <= 0.0 for topic, clearance in minimum_clearance.items()
    },
    "final_state": last_state,
}
output = args.bag_path / "analysis" / "combined_task_summary.json"
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
print(json.dumps(summary, indent=2, ensure_ascii=False))
print(f"Combined-task analysis written to: {output}")
