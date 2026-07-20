#!/usr/bin/env python3
"""Post-hoc check on a closed rosbag: the LAST recorded /epuckN/cmd_vel
sample for each given topic must be (linear.x=0, angular.z=0).

Required after any trial whose stop_reason is TASK_COMPLETE_GOAL or
CONTROLLER_SELF_COMPLETE, to verify the safe-stop path genuinely zeroed
motion rather than the bag simply having been closed mid-command.
Read-only; does not modify the bag.
"""
from __future__ import annotations

import sys


def is_zero_twist(linear_x: float, angular_z: float, tol: float = 1e-6) -> bool:
    """Pure predicate, importable/testable without rosbag2_py/rclpy."""
    return abs(linear_x) < tol and abs(angular_z) < tol


def last_cmd_vel(bag_dir: str, topic: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Twist

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    last = None
    while reader.has_next():
        t, data, _ts = reader.read_next()
        if t == topic:
            last = deserialize_message(data, Twist)
    return last


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if len(argv) < 2:
        print("usage: verify_cmd_vel_zero.py BAG_DIR TOPIC [TOPIC ...]", file=sys.stderr)
        return 2
    bag_dir, topics = argv[0], argv[1:]
    all_zero = True
    for topic in topics:
        msg = last_cmd_vel(bag_dir, topic)
        if msg is None:
            print(f"{topic}: NO_SAMPLES")
            all_zero = False
            continue
        is_zero = is_zero_twist(msg.linear.x, msg.angular.z)
        print(f"{topic}: last linear.x={msg.linear.x:.6f} angular.z={msg.angular.z:.6f} zero={is_zero}")
        if not is_zero:
            all_zero = False
    return 0 if all_zero else 1


if __name__ == "__main__":
    sys.exit(main())
