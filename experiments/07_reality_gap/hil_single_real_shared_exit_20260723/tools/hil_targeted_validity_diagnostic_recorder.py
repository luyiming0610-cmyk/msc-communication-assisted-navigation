#!/usr/bin/env python3
"""Lightweight, read-only, multi-topic recorder for the targeted
validity_flags periodicity follow-up (TARGETED_STATIONARY_DIAGNOSTIC).

Subscribes concurrently to /epuck1/state, /epuck_bridge/status, /odom,
/tof, and the six IR topics FLAG_IR_VALID actually requires (/ps0,
/ps1, /ps2, /ps5, /ps6, /ps7) -- never publishes anything. Each
callback does the minimum possible work (build one small dict, append
to an in-memory list) and returns immediately; ALL CSV writing happens
exactly once, at shutdown, so the recorder's own I/O cannot introduce
a periodic stall that would confound the very periodicity this
diagnostic is trying to characterize.

Every recorded row carries:
  - local_time_ns (time.time_ns(), wall clock)
  - local_monotonic_ns (time.monotonic_ns(), immune to wall-clock steps)
  - topic
  - stamp_sec / stamp_nanosec (message header/EpuckState stamp, where present)
  - validity_flags / sequence (EpuckState only)
  - connected / rx_count / crc_errors / last_rtt_ms / last_state_age_s /
    state_missing / state_out_of_order (bridge status only, parsed from
    its JSON payload)

The pure `build_row()` function below has no ROS/rclpy dependency and
is independently unit-tested (test_hil_targeted_validity_diagnostic_recorder.py).
"""
from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass, field
from typing import Optional

CSV_FIELDS = [
    "local_time_ns",
    "local_monotonic_ns",
    "topic",
    "stamp_sec",
    "stamp_nanosec",
    "validity_flags",
    "sequence",
    "connected",
    "rx_count",
    "crc_errors",
    "last_rtt_ms",
    "last_state_age_s",
    "state_missing",
    "state_out_of_order",
]


def build_row(
    *,
    local_time_ns: int,
    local_monotonic_ns: int,
    topic: str,
    stamp_sec: Optional[int] = None,
    stamp_nanosec: Optional[int] = None,
    validity_flags: Optional[int] = None,
    sequence: Optional[int] = None,
    connected: Optional[bool] = None,
    rx_count: Optional[int] = None,
    crc_errors: Optional[int] = None,
    last_rtt_ms: Optional[float] = None,
    last_state_age_s: Optional[float] = None,
    state_missing: Optional[int] = None,
    state_out_of_order: Optional[int] = None,
) -> dict:
    """Pure row-construction -- no I/O, no ROS. One dict per event,
    always with exactly CSV_FIELDS keys (missing values are None ->
    written as an empty CSV cell, never fabricated)."""
    return {
        "local_time_ns": local_time_ns,
        "local_monotonic_ns": local_monotonic_ns,
        "topic": topic,
        "stamp_sec": stamp_sec,
        "stamp_nanosec": stamp_nanosec,
        "validity_flags": validity_flags,
        "sequence": sequence,
        "connected": connected,
        "rx_count": rx_count,
        "crc_errors": crc_errors,
        "last_rtt_ms": last_rtt_ms,
        "last_state_age_s": last_state_age_s,
        "state_missing": state_missing,
        "state_out_of_order": state_out_of_order,
    }


def parse_bridge_status_json(data: str) -> dict:
    """Pure JSON parse for the bridge status String payload. Returns an
    empty dict (no fields) on malformed JSON, never raises -- a
    malformed status message must not crash the recorder or silently
    fabricate values."""
    try:
        payload = json.loads(data)
    except (json.JSONDecodeError, TypeError):
        return {}
    if not isinstance(payload, dict):
        return {}
    return payload


def write_rows_csv(path: str, rows: list) -> None:
    """Writes every buffered row in ONE call, at shutdown -- the
    recorder never opens/flushes the file mid-run, so its own I/O
    cannot introduce a periodic disturbance."""
    with open(path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


# --------------------------------------------------------------------
# rclpy wrapper -- thin, minimal-work callbacks only.
# --------------------------------------------------------------------
def _build_node():
    import argparse

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Range
    from std_msgs.msg import String

    from epuck2_comm_interfaces.msg import EpuckState

    IR_TOPICS = ("ps0", "ps1", "ps2", "ps5", "ps6", "ps7")

    class TargetedValidityRecorder(Node):
        def __init__(self, args):
            super().__init__("hil_targeted_validity_diagnostic_recorder")
            self.args = args
            self.rows: list = []

            self.create_subscription(EpuckState, args.state_topic, self._on_state, qos_profile_sensor_data)
            self.create_subscription(String, args.bridge_status_topic, self._on_bridge_status, 10)
            self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
            self.create_subscription(Range, "/tof", self._on_tof, qos_profile_sensor_data)
            for name in IR_TOPICS:
                self.create_subscription(
                    Range, f"/{name}",
                    (lambda msg, topic=name: self._on_ir(topic, msg)),
                    qos_profile_sensor_data,
                )
            self.get_logger().warn(
                f"HIL_TARGETED_VALIDITY_RECORDER_START state_topic={args.state_topic} "
                f"bridge_status_topic={args.bridge_status_topic}"
            )

        def _append(self, **kwargs) -> None:
            self.rows.append(build_row(
                local_time_ns=time.time_ns(),
                local_monotonic_ns=time.monotonic_ns(),
                **kwargs,
            ))

        def _on_state(self, msg) -> None:
            self._append(
                topic=self.args.state_topic,
                stamp_sec=int(msg.stamp.sec),
                stamp_nanosec=int(msg.stamp.nanosec),
                validity_flags=int(msg.validity_flags),
                sequence=int(msg.sequence),
            )

        def _on_bridge_status(self, msg) -> None:
            fields = parse_bridge_status_json(msg.data)
            self._append(
                topic=self.args.bridge_status_topic,
                connected=fields.get("connected"),
                rx_count=fields.get("rx_count"),
                crc_errors=fields.get("crc_errors"),
                last_rtt_ms=fields.get("last_rtt_ms"),
                last_state_age_s=fields.get("last_state_age_s"),
                state_missing=fields.get("state_missing"),
                state_out_of_order=fields.get("state_out_of_order"),
            )

        def _on_odom(self, msg) -> None:
            self._append(
                topic="/odom",
                stamp_sec=int(msg.header.stamp.sec),
                stamp_nanosec=int(msg.header.stamp.nanosec),
            )

        def _on_tof(self, msg) -> None:
            self._append(
                topic="/tof",
                stamp_sec=int(msg.header.stamp.sec),
                stamp_nanosec=int(msg.header.stamp.nanosec),
            )

        def _on_ir(self, topic_name, msg) -> None:
            self._append(
                topic=f"/{topic_name}",
                stamp_sec=int(msg.header.stamp.sec),
                stamp_nanosec=int(msg.header.stamp.nanosec),
            )

        def flush(self, path: str) -> int:
            write_rows_csv(path, self.rows)
            return len(self.rows)

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--state-topic", default="/epuck1/state")
        parser.add_argument("--bridge-status-topic", default="/epuck_bridge/status")
        parser.add_argument("--output-csv", required=True)
        parser.add_argument("--duration-s", type=float, default=120.0)
        return parser.parse_args(argv)

    return rclpy, TargetedValidityRecorder, parse_args


def main(argv=None):
    import sys

    rclpy, TargetedValidityRecorder, parse_args = _build_node()
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[])
    node = TargetedValidityRecorder(args)
    stop_at = time.monotonic() + args.duration_s
    try:
        while rclpy.ok() and time.monotonic() < stop_at:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        count = node.flush(args.output_csv)
        node.get_logger().warn(f"HIL_TARGETED_VALIDITY_RECORDER_DONE rows={count} output={args.output_csv}")
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
