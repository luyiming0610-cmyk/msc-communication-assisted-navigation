#!/usr/bin/env python3
"""Post-hoc check on a closed rosbag: each robot's LAST recorded
/epuckN/state.linear_velocity_mps must be at or below a small threshold.

Why this exists (and not just checking /epuckN/cmd_vel): cooperative_
avoider.py's frozen stop() method publishes a zero Twist 3x on SIGINT,
then immediately destroy_node()s and exits. Empirically (PILOT05), the
bag's last recorded /epuckN/cmd_vel sample can still show the pre-SIGINT
steady-state command (e.g. 0.025 m/s) -- the 3 zero-Twist publishes can
be lost to a DDS-teardown race between publish() and the publisher's own
node/context destruction moments later, which the controller's own code
comment explicitly acknowledges and tolerates ("Motion is also bounded
by the driver watchdog, so cleanup must remain quiet"). Since nothing
publishes to /epuckN/cmd_vel once the controller process has exited,
extending the bag recording window cannot recover a lost cmd_vel sample
-- checking that raw topic's literal last message is not a reliable
safety signal for an early (non-max-runtime) stop.

/epuckN/state is different: it is published by the still-running
state_publisher node (stopped only AFTER the controller and after a
settle window), reflects Webots' own physical odometry, and is
therefore a genuine, independent measurement of whether the robot
ACTUALLY stopped moving -- not merely whether a specific command
message reached the bag before a process exited.
"""
from __future__ import annotations

import sys


def is_settled(linear_velocity_mps: float, threshold_mps: float = 0.01) -> bool:
    """Pure predicate, importable/testable without rosbag2_py/rclpy."""
    return abs(linear_velocity_mps) <= threshold_mps


def last_linear_velocity(bag_dir: str, topic: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from epuck2_comm_interfaces.msg import EpuckState

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    last = None
    while reader.has_next():
        t, data, _ts = reader.read_next()
        if t == topic:
            last = deserialize_message(data, EpuckState)
    return last


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if len(argv) < 2:
        print("usage: verify_state_velocity_settled.py BAG_DIR TOPIC [TOPIC ...]", file=sys.stderr)
        return 2
    bag_dir, topics = argv[0], argv[1:]
    all_settled = True
    for topic in topics:
        msg = last_linear_velocity(bag_dir, topic)
        if msg is None:
            print(f"{topic}: NO_SAMPLES")
            all_settled = False
            continue
        settled = is_settled(msg.linear_velocity_mps)
        print(f"{topic}: last linear_velocity_mps={msg.linear_velocity_mps:.6f} settled={settled}")
        if not settled:
            all_settled = False
    return 0 if all_settled else 1


if __name__ == "__main__":
    sys.exit(main())
