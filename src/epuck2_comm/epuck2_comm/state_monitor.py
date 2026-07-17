import csv
import json
import math
import os
import time
from datetime import datetime

import rclpy
from epuck2_comm_interfaces.msg import EpuckState
from rclpy.node import Node
from rclpy.serialization import serialize_message

from .neighbor_cache import NeighborCache


def _finite_or_none(value: float):
    return float(value) if math.isfinite(value) else None


class StateMonitor(Node):
    def __init__(self):
        super().__init__("state_monitor")

        self.declare_parameter("state_topics", ["/epuck1/state", "/epuck2/state"])
        self.declare_parameter("output_dir", "logs")
        self.declare_parameter("summary_period_s", 2.0)
        self.declare_parameter("stale_threshold_s", 0.5)

        topics = [str(value) for value in self.get_parameter("state_topics").value]
        output_dir = os.path.expanduser(str(self.get_parameter("output_dir").value))
        summary_period_s = float(self.get_parameter("summary_period_s").value)
        self.stale_threshold_s = float(self.get_parameter("stale_threshold_s").value)

        os.makedirs(output_dir, exist_ok=True)
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.jsonl_path = os.path.join(output_dir, f"state_monitor_{run_id}.jsonl")
        self.summary_path = os.path.join(output_dir, f"state_monitor_{run_id}_summary.csv")
        self.jsonl_file = open(self.jsonl_path, "a", encoding="utf-8", buffering=1)

        self.cache = NeighborCache()
        self.start_monotonic = time.monotonic()
        self.stale_observations = {}

        # Node.subscriptions is a read-only rclpy property. Keep explicit
        # references under a private name so the subscriptions remain alive.
        self._state_subscriptions = [
            self.create_subscription(EpuckState, topic, self._callback, 20)
            for topic in topics
        ]
        self.timer = self.create_timer(summary_period_s, self._print_summary)

        self.get_logger().info(f"monitoring {', '.join(topics)}")
        self.get_logger().info(f"JSONL output: {self.jsonl_path}")
        self.get_logger().info(f"summary output: {self.summary_path}")

    def _callback(self, msg: EpuckState) -> None:
        receive_monotonic = time.monotonic()
        receive_stamp_ns = self.get_clock().now().nanoseconds
        source_stamp_ns = int(msg.stamp.sec) * 1_000_000_000 + int(msg.stamp.nanosec)
        serialized_bytes = len(serialize_message(msg))

        update = self.cache.update(
            robot_id=int(msg.robot_id),
            sequence=int(msg.sequence),
            source_stamp_ns=source_stamp_ns,
            receive_stamp_ns=receive_stamp_ns,
            receive_monotonic=receive_monotonic,
            serialized_bytes=serialized_bytes,
            state=msg,
        )

        record = {
            "receive_monotonic_s": receive_monotonic,
            "receive_ros_ns": receive_stamp_ns,
            "source_ros_ns": source_stamp_ns,
            "robot_id": int(msg.robot_id),
            "sequence": int(msg.sequence),
            "latency_ms": update.latency_ms,
            "serialized_bytes": serialized_bytes,
            "inferred_gap": update.inferred_gap,
            "duplicate": update.duplicate,
            "out_of_order": update.out_of_order,
            "state": {
                "version": int(msg.version),
                "source": int(msg.source),
                "x_m": float(msg.x_m),
                "y_m": float(msg.y_m),
                "yaw_rad": float(msg.yaw_rad),
                "linear_velocity_mps": float(msg.linear_velocity_mps),
                "angular_velocity_rps": float(msg.angular_velocity_rps),
                "front_distance_m": _finite_or_none(msg.front_distance_m),
                "left_distance_m": _finite_or_none(msg.left_distance_m),
                "right_distance_m": _finite_or_none(msg.right_distance_m),
                "obstacle_status": int(msg.obstacle_status),
                "validity_flags": int(msg.validity_flags),
            },
        }
        self.jsonl_file.write(json.dumps(record, separators=(",", ":")) + "\n")

    def _rows(self):
        now = time.monotonic()
        rows = []
        for entry in sorted(self.cache.entries(), key=lambda item: item.robot_id):
            duration_s = max(0.001, now - entry.first_receive_monotonic)
            receive_age_s = now - entry.receive_monotonic
            if receive_age_s > self.stale_threshold_s:
                self.stale_observations[entry.robot_id] = self.stale_observations.get(entry.robot_id, 0) + 1
            rows.append(
                {
                    "robot_id": entry.robot_id,
                    "received": entry.stats.received,
                    "inferred_lost": entry.stats.inferred_lost,
                    "delivery_ratio": entry.stats.delivery_ratio,
                    "duplicates": entry.stats.duplicates,
                    "out_of_order": entry.stats.out_of_order,
                    "frequency_hz": entry.stats.received / duration_s,
                    "average_latency_ms": entry.stats.average_latency_ms,
                    "maximum_latency_ms": entry.stats.latency_max_ms,
                    "average_serialized_bytes": entry.stats.serialized_bytes / max(1, entry.stats.received),
                    "receive_age_s": receive_age_s,
                    "stale_observations": self.stale_observations.get(entry.robot_id, 0),
                }
            )
        return rows

    def _print_summary(self) -> None:
        rows = self._rows()
        if not rows:
            self.get_logger().info("waiting for state messages")
            return
        text = " | ".join(
            (
                f"r{row['robot_id']}: n={row['received']} "
                f"hz={row['frequency_hz']:.2f} loss={row['inferred_lost']} "
                f"delivery={100.0 * row['delivery_ratio']:.2f}% "
                f"lat={row['average_latency_ms']:.3f}ms "
                f"bytes={row['average_serialized_bytes']:.1f}"
            )
            for row in rows
        )
        self.get_logger().info(text)

    def _write_summary(self) -> None:
        rows = self._rows()
        if not rows:
            return
        with open(self.summary_path, "w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    def destroy_node(self):
        try:
            self._write_summary()
            self.jsonl_file.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
