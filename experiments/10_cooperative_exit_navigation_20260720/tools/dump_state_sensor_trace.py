#!/usr/bin/env python3
"""Read-only: dump every recorded EpuckState sample for one robot within a
time window, including raw front/left/right distances, for root-cause
diagnosis. No modification of any recorded evidence."""
import argparse
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from epuck2_comm_interfaces.msg import EpuckState


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag_dir")
    p.add_argument("topic")
    p.add_argument("--t-min", type=float, default=None)
    p.add_argument("--t-max", type=float, default=None)
    args = p.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    rows = []
    while reader.has_next():
        t, data, _ts = reader.read_next()
        if t == args.topic:
            msg = deserialize_message(data, EpuckState)
            stamp_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) / 1e9
            rows.append((stamp_s, msg))
    rows.sort(key=lambda r: r[0])
    print("t_s,x_m,y_m,yaw_rad,linear_velocity_mps,front_distance_m,left_distance_m,right_distance_m,validity_flags")
    for stamp_s, msg in rows:
        if args.t_min is not None and stamp_s < args.t_min:
            continue
        if args.t_max is not None and stamp_s > args.t_max:
            continue
        print(
            f"{stamp_s:.3f},{msg.x_m:.4f},{msg.y_m:.4f},{msg.yaw_rad:.4f},"
            f"{msg.linear_velocity_mps:.4f},{msg.front_distance_m:.4f},"
            f"{msg.left_distance_m:.4f},{msg.right_distance_m:.4f},{msg.validity_flags}"
        )


if __name__ == "__main__":
    sys.exit(main())
