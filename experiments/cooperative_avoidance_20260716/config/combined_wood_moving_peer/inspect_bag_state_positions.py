#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


parser = argparse.ArgumentParser()
parser.add_argument("bag_path", type=Path)
parser.add_argument("--every", type=float, default=1.0)
parser.add_argument("--box-x", type=float)
parser.add_argument("--box-y", type=float)
parser.add_argument("--box-size", type=float, default=0.06)
parser.add_argument("--robot-radius", type=float, default=0.035)
args = parser.parse_args()

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(args.bag_path), storage_id="sqlite3"),
    rosbag2_py.ConverterOptions("cdr", "cdr"),
)
types = {item.name: item.type for item in reader.get_all_topics_and_types()}
wanted = {"/epuck1/state", "/epuck2/state"}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
first_ns = None
latest = {}
next_print = 0.0
minimum = (math.inf, None)
box_clearance = {topic: (math.inf, None) for topic in wanted}
while reader.has_next():
    topic, raw, timestamp_ns = reader.read_next()
    if first_ns is None:
        first_ns = timestamp_ns
    time_s = (timestamp_ns - first_ns) / 1.0e9
    message = deserialize_message(raw, get_message(types[topic]))
    latest[topic] = (float(message.x_m), float(message.y_m), float(message.yaw_rad))
    if args.box_x is not None and args.box_y is not None:
        half = args.box_size / 2.0
        x_m, y_m, _ = latest[topic]
        dx = max(abs(x_m - args.box_x) - half, 0.0)
        dy = max(abs(y_m - args.box_y) - half, 0.0)
        clearance = math.hypot(dx, dy) - args.robot_radius
        if clearance < box_clearance[topic][0]:
            box_clearance[topic] = (clearance, (time_s, x_m, y_m))
    if len(latest) != 2:
        continue
    first = latest["/epuck1/state"]
    second = latest["/epuck2/state"]
    separation = math.hypot(first[0] - second[0], first[1] - second[1])
    if separation < minimum[0]:
        minimum = (separation, (time_s, first, second))
    if time_s >= next_print:
        print(
            f"t={time_s:.3f} e1=({first[0]:.3f},{first[1]:.3f},{first[2]:.3f}) "
            f"e2=({second[0]:.3f},{second[1]:.3f},{second[2]:.3f}) sep={separation:.3f}"
        )
        next_print += args.every

distance, row = minimum
time_s, first, second = row
print(
    f"MIN t={time_s:.3f} e1=({first[0]:.6f},{first[1]:.6f},{first[2]:.6f}) "
    f"e2=({second[0]:.6f},{second[1]:.6f},{second[2]:.6f}) sep={distance:.6f}"
)
if args.box_x is not None and args.box_y is not None:
    for topic in sorted(wanted):
        clearance, row = box_clearance[topic]
        time_s, x_m, y_m = row
        print(
            f"BOX_CLEARANCE topic={topic} t={time_s:.3f} "
            f"position=({x_m:.6f},{y_m:.6f}) clearance={clearance:.6f}"
        )
