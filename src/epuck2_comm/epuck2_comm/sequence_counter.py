"""Lightweight, rosbag-independent counting subscriber.

Diagnostic-only tool for isolating where messages are actually lost in the
measurement chain (publisher -> relay -> subscriber -> rosbag). Subscribes
to one or more EpuckState topics and tracks, per topic, purely in memory
(never writes to a rosbag): first_sequence, last_sequence, received_count,
sequence_gap_count, duplicate_count, out_of_order_count, and first/last ROS
time. Writes a JSON summary to --output-path on SIGINT (Ctrl-C / relay-
style graceful shutdown).

This is a third-party observer: comparing its counts against the relay's
own CSV log and against what rosbag actually recorded is what lets a
mismatch be attributed to a specific stage rather than guessed at.
"""

import argparse
import json
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from epuck2_comm_interfaces.msg import EpuckState


class TopicCounter:
    def __init__(self):
        self.first_sequence = None
        self.last_sequence = None
        self.received_count = 0
        self.sequence_gap_count = 0
        self.duplicate_count = 0
        self.out_of_order_count = 0
        self.first_ros_time_s = None
        self.last_ros_time_s = None
        self._seen = set()
        self._previous_sequence = None

    def observe(self, sequence: int, now_s: float) -> None:
        self.received_count += 1
        if self.first_sequence is None:
            self.first_sequence = sequence
            self.first_ros_time_s = now_s
        self.last_sequence = sequence
        self.last_ros_time_s = now_s

        if sequence in self._seen:
            self.duplicate_count += 1
        else:
            self._seen.add(sequence)

        if self._previous_sequence is not None:
            if sequence < self._previous_sequence:
                self.out_of_order_count += 1
            elif sequence > self._previous_sequence + 1:
                self.sequence_gap_count += sequence - self._previous_sequence - 1
        self._previous_sequence = sequence

    def summary(self) -> dict:
        expected_count = None
        if self.first_sequence is not None and self.last_sequence is not None:
            expected_count = self.last_sequence - self.first_sequence + 1
        return {
            "first_sequence": self.first_sequence,
            "last_sequence": self.last_sequence,
            "received_count": self.received_count,
            "unique_sequence_count": len(self._seen),
            "expected_count": expected_count,
            "sequence_gap_count": self.sequence_gap_count,
            "duplicate_count": self.duplicate_count,
            "out_of_order_count": self.out_of_order_count,
            "first_ros_time_s": self.first_ros_time_s,
            "last_ros_time_s": self.last_ros_time_s,
        }


class SequenceCounterNode(Node):
    def __init__(self, topics):
        super().__init__("sequence_counter")
        self.counters = {topic: TopicCounter() for topic in topics}
        for topic in topics:
            self.create_subscription(
                EpuckState, topic, self._make_callback(topic), 20
            )
        self.get_logger().info(f"sequence_counter watching: {list(topics)}")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _make_callback(self, topic):
        def _cb(msg):
            self.counters[topic].observe(int(msg.sequence), self._now_s())
        return _cb

    def write_summary(self, output_path: str) -> None:
        result = {topic: counter.summary() for topic, counter in self.counters.items()}
        with open(output_path, "w", encoding="utf-8") as fh:
            json.dump(result, fh, indent=2, ensure_ascii=False)
            fh.write("\n")
        self.get_logger().info(f"sequence_counter summary written to {output_path}: {json.dumps(result)}")


def _arguments(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--topics", nargs="+", required=True)
    parser.add_argument("--output-path", required=True)
    return parser.parse_args(argv)


def main():
    # launch_ros passes this node its own args followed by ROS-specific
    # ones (--ros-args -r __ns:=... --params-file ...); those must be
    # stripped before argparse sees them, or argparse fails with
    # "unrecognized arguments" and the process exits immediately.
    args = _arguments(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = SequenceCounterNode(args.topics)

    def _on_sigint(signum, frame):
        node.write_summary(args.output_path)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _on_sigint)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the summary is written even if shutdown happened via a
        # path other than SIGINT (defensive; the SIGINT handler above is
        # the primary path used by the diagnostic script).
        try:
            node.write_summary(args.output_path)
        except Exception:  # noqa: BLE001 -- best-effort final write
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
