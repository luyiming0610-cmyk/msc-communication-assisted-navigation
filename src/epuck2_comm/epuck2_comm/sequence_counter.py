"""Lightweight, rosbag-independent counting subscriber.

Diagnostic-only tool for isolating where messages are actually lost in the
measurement chain (publisher -> relay -> subscriber -> rosbag). Subscribes
to one or more EpuckState topics and tracks, per topic, purely in memory
(never writes to a rosbag): first_sequence, last_sequence, received_count,
sequence_gap_count, duplicate_count, out_of_order_count, and first/last ROS
time.

Reliability design (fixed after an earlier version's SIGINT-triggered
write did not reliably fire under ros2 launch's process supervision):
this node does NOT depend on a custom signal handler to ever produce
output. A periodic timer (`checkpoint_period_s`, default 1.0s) writes an
atomic checkpoint (`complete=false`) to --output-path on every tick, so an
abnormal exit (killed, crashed, launch tears it down before a graceful
shutdown) still leaves the most recent checkpoint on disk, at most one
period stale. A normal shutdown (rclpy.spin() returning, whether via
external shutdown or KeyboardInterrupt) writes one final checkpoint with
`complete=true`. The write itself is atomic (write to a temp file in the
same directory, then os.replace) so a reader never sees a partially
written JSON file, even if the process is killed mid-write.

This is a third-party observer: comparing its counts against the relay's
own CSV log and against what rosbag actually recorded is what lets a
mismatch be attributed to a specific stage rather than guessed at.
"""

import argparse
import json
import os
import sys
import tempfile

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


def atomic_write_json(output_path: str, payload: dict) -> None:
    """Write `payload` as JSON to output_path such that a concurrent
    reader never observes a partially-written file: write to a temp file
    in the same directory (so os.replace stays on one filesystem, making
    it atomic), then rename over the destination."""
    directory = os.path.dirname(os.path.abspath(output_path)) or "."
    fd, temp_path = tempfile.mkstemp(dir=directory, prefix=".sequence_counter_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2, ensure_ascii=False)
            fh.write("\n")
        os.replace(temp_path, output_path)
    except BaseException:
        try:
            os.remove(temp_path)
        except OSError:
            pass
        raise


class SequenceCounterNode(Node):
    def __init__(self, topics, output_path: str, checkpoint_period_s: float = 1.0):
        super().__init__("sequence_counter")
        self.output_path = output_path
        self.counters = {topic: TopicCounter() for topic in topics}
        for topic in topics:
            self.create_subscription(
                EpuckState, topic, self._make_callback(topic), 20
            )
        self.get_logger().info(f"sequence_counter watching: {list(topics)}")
        # Periodic checkpoint: never depends on a clean shutdown to
        # produce output at all -- at most one period of data can be lost
        # if the process is killed, and the file on disk is always valid
        # JSON (atomic_write_json), just possibly slightly stale and
        # marked complete=false.
        self.create_timer(checkpoint_period_s, self._checkpoint)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _make_callback(self, topic):
        def _cb(msg):
            self.counters[topic].observe(int(msg.sequence), self._now_s())
        return _cb

    def _payload(self, complete: bool) -> dict:
        result = {topic: counter.summary() for topic, counter in self.counters.items()}
        result["complete"] = complete
        return result

    def _checkpoint(self) -> None:
        atomic_write_json(self.output_path, self._payload(complete=False))

    def write_final_summary(self) -> None:
        payload = self._payload(complete=True)
        atomic_write_json(self.output_path, payload)
        self.get_logger().info(f"sequence_counter FINAL summary written to {self.output_path}: {json.dumps(payload)}")


def _arguments(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--topics", nargs="+", required=True)
    parser.add_argument("--output-path", required=True)
    parser.add_argument("--checkpoint-period-s", type=float, default=1.0)
    return parser.parse_args(argv)


def main():
    # launch_ros passes this node its own args followed by ROS-specific
    # ones (--ros-args -r __ns:=... --params-file ...); those must be
    # stripped before argparse sees them, or argparse fails with
    # "unrecognized arguments" and the process exits immediately.
    args = _arguments(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = SequenceCounterNode(
        args.topics, args.output_path, checkpoint_period_s=args.checkpoint_period_s
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Normal-path final write (complete=true). If the process is
        # killed hard enough that even this never runs, the periodic
        # checkpoint from _checkpoint() has already left a valid,
        # at-most-one-period-stale file with complete=false -- there is
        # no path that leaves no output at all or a corrupt file.
        try:
            node.write_final_summary()
        except Exception:  # noqa: BLE001 -- best-effort final write
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
