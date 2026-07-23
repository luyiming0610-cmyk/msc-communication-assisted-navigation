#!/usr/bin/env python3
"""Bounded, read-only, continuous command-evidence recorder -- Part 3
of the command-evidence-chain work following the two 2026-07-23
UNEXPECTED_PHYSICAL_MOTION incidents
(safety_incident_unexpected_motion_20260723/SUMMARY.md and
safety_incident_unexpected_motion_2_20260723/SUMMARY.md), whose audits
both concluded command origin was NOT_MEASURABLE specifically because
no continuous recording of /cmd_vel or /cmd_vel_unguarded existed. This
tool closes that evidence gap.

Subscribes concurrently to the five topics required by the incident
audits' preconditions for any future powered session:
  - /cmd_vel_unguarded (geometry_msgs/Twist)   -- pre-guard command
  - /cmd_vel           (geometry_msgs/Twist)   -- final, driver-facing command
  - /hil_guard/arm     (std_msgs/Bool)          -- arm/disarm state
  - /epuck1/state      (EpuckState, validity_flags field only)
  - /epuck_bridge/status (std_msgs/String, JSON payload)

Never publishes anything (verified by
test_hil_command_evidence_recorder_zero_publishers.py). Every callback
does the minimum possible work (build one small dict, append to an
in-memory list) and returns immediately; ALL CSV writing happens once,
at shutdown -- mirrors hil_targeted_validity_diagnostic_recorder.py's
established pattern in this project, so the recorder's own I/O cannot
introduce a periodic disturbance or delay delivery of a genuine
command event.

Every recorded row carries local_time_ns (time.time_ns(), wall clock)
and local_monotonic_ns (time.monotonic_ns(), immune to wall-clock
steps) captured at the moment of RECEIPT, independent of whatever
stamp (if any) the message itself carries -- this is what makes the
recording usable as command-origin evidence even if a message's own
header stamp is stale, zero, or absent.
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
    "linear_x",
    "angular_z",
    "arm_state",
    "validity_flags",
    "sequence",
    "bridge_connected",
    "bridge_rx_count",
]

REQUIRED_COMMAND_TOPICS = ("/cmd_vel_unguarded", "/cmd_vel")
REQUIRED_COMMAND_TOPIC_TYPE = "geometry_msgs/msg/Twist"


def build_row(
    *,
    local_time_ns: int,
    local_monotonic_ns: int,
    topic: str,
    linear_x: Optional[float] = None,
    angular_z: Optional[float] = None,
    arm_state: Optional[bool] = None,
    validity_flags: Optional[int] = None,
    sequence: Optional[int] = None,
    bridge_connected: Optional[bool] = None,
    bridge_rx_count: Optional[int] = None,
) -> dict:
    """Pure row construction -- no I/O, no ROS. One dict per event,
    always with exactly CSV_FIELDS keys (missing values are None ->
    written as an empty CSV cell, never fabricated)."""
    return {
        "local_time_ns": local_time_ns,
        "local_monotonic_ns": local_monotonic_ns,
        "topic": topic,
        "linear_x": linear_x,
        "angular_z": angular_z,
        "arm_state": arm_state,
        "validity_flags": validity_flags,
        "sequence": sequence,
        "bridge_connected": bridge_connected,
        "bridge_rx_count": bridge_rx_count,
    }


def parse_bridge_status_json(data: str) -> dict:
    """Pure JSON parse for the bridge status String payload. Returns an
    empty dict on malformed JSON, never raises."""
    try:
        payload = json.loads(data)
    except (json.JSONDecodeError, TypeError):
        return {}
    if not isinstance(payload, dict):
        return {}
    return payload


def write_rows_csv(path: str, rows: list) -> None:
    """Writes every buffered row in ONE call, at shutdown."""
    with open(path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


@dataclass(frozen=True)
class CommandTopicVerifyResult:
    ok: bool
    missing: tuple = field(default_factory=tuple)
    wrong_type: tuple = field(default_factory=tuple)


def verify_required_command_topics_present(
    topic_names_and_types: dict,
    required_topics: tuple = REQUIRED_COMMAND_TOPICS,
    required_type: str = REQUIRED_COMMAND_TOPIC_TYPE,
) -> CommandTopicVerifyResult:
    """Checks that every topic in `required_topics` is registered in
    the ROS graph's topic_names_and_types (as returned by
    Node.get_topic_names_and_types()) with the expected message type.

    Honest limitation, stated plainly: a topic can appear in this dict
    purely because THIS recorder itself just subscribed to it -- this
    does NOT prove any publisher exists yet, which is the expected
    state when the recorder starts first, per the required sequencing
    (recorder starts before guard/controller). This check only proves
    the topic name/type resolves as expected in the current ROS graph,
    catching a typo'd topic name or an unexpected type collision, not
    "someone is currently commanding the robot."
    """
    missing = []
    wrong_type = []
    for topic in required_topics:
        types = topic_names_and_types.get(topic)
        if types is None:
            missing.append(topic)
        elif required_type not in types:
            wrong_type.append((topic, tuple(types)))
    return CommandTopicVerifyResult(
        ok=not missing and not wrong_type,
        missing=tuple(missing),
        wrong_type=tuple(wrong_type),
    )


# --------------------------------------------------------------------
# rclpy wrapper -- thin, minimal-work callbacks only. Never publishes
# anything (no create_publisher call anywhere in this class).
# --------------------------------------------------------------------
def _build_node():
    import argparse

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    from epuck2_comm_interfaces.msg import EpuckState

    class CommandEvidenceRecorder(Node):
        def __init__(self, args):
            super().__init__("hil_command_evidence_recorder")
            self.args = args
            self.rows: list = []

            self.create_subscription(
                Twist, args.upstream_cmd_vel_topic, self._on_upstream_cmd_vel, 20
            )
            self.create_subscription(
                Twist, args.guarded_cmd_vel_topic, self._on_guarded_cmd_vel, 20
            )
            self.create_subscription(Bool, args.arm_topic, self._on_arm, 10)
            self.create_subscription(EpuckState, args.state_topic, self._on_state, 20)
            self.create_subscription(
                String, args.bridge_status_topic, self._on_bridge_status, 10
            )

            self.get_logger().warn(
                "HIL_COMMAND_EVIDENCE_RECORDER_START "
                f"upstream_cmd_vel_topic={args.upstream_cmd_vel_topic} "
                f"guarded_cmd_vel_topic={args.guarded_cmd_vel_topic} "
                f"arm_topic={args.arm_topic} state_topic={args.state_topic} "
                f"bridge_status_topic={args.bridge_status_topic}"
            )

        def verify_command_topics(self) -> CommandTopicVerifyResult:
            # get_topic_names_and_types() returns fully-resolved
            # (absolute, leading-slash) topic names -- the CLI
            # defaults ("cmd_vel_unguarded", "cmd_vel") are relative,
            # so they must be resolved the same way rclpy itself
            # resolves them before comparing, or every check would
            # silently always report "missing".
            required = (
                self.resolve_topic_name(self.args.upstream_cmd_vel_topic),
                self.resolve_topic_name(self.args.guarded_cmd_vel_topic),
            )
            topics = dict(self.get_topic_names_and_types())
            return verify_required_command_topics_present(topics, required_topics=required)

        def _append(self, **kwargs) -> None:
            self.rows.append(
                build_row(
                    local_time_ns=time.time_ns(),
                    local_monotonic_ns=time.monotonic_ns(),
                    **kwargs,
                )
            )

        def _on_upstream_cmd_vel(self, msg) -> None:
            self._append(
                topic=self.args.upstream_cmd_vel_topic,
                linear_x=float(msg.linear.x),
                angular_z=float(msg.angular.z),
            )

        def _on_guarded_cmd_vel(self, msg) -> None:
            self._append(
                topic=self.args.guarded_cmd_vel_topic,
                linear_x=float(msg.linear.x),
                angular_z=float(msg.angular.z),
            )

        def _on_arm(self, msg) -> None:
            self._append(topic=self.args.arm_topic, arm_state=bool(msg.data))

        def _on_state(self, msg) -> None:
            self._append(
                topic=self.args.state_topic,
                validity_flags=int(msg.validity_flags),
                sequence=int(msg.sequence),
            )

        def _on_bridge_status(self, msg) -> None:
            fields = parse_bridge_status_json(msg.data)
            self._append(
                topic=self.args.bridge_status_topic,
                bridge_connected=fields.get("connected"),
                bridge_rx_count=fields.get("rx_count"),
            )

        def flush(self, path: str) -> int:
            write_rows_csv(path, self.rows)
            return len(self.rows)

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--upstream-cmd-vel-topic", default="cmd_vel_unguarded")
        parser.add_argument("--guarded-cmd-vel-topic", default="cmd_vel")
        parser.add_argument("--arm-topic", default="/hil_guard/arm")
        parser.add_argument("--state-topic", default="/epuck1/state")
        parser.add_argument("--bridge-status-topic", default="/epuck_bridge/status")
        parser.add_argument("--output-csv", required=True)
        parser.add_argument("--duration-s", type=float, default=3600.0)
        return parser.parse_args(argv)

    return rclpy, CommandEvidenceRecorder, parse_args


def main(argv=None):
    import sys

    rclpy, CommandEvidenceRecorder, parse_args = _build_node()
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[])
    node = CommandEvidenceRecorder(args)

    verify_result = node.verify_command_topics()
    node.get_logger().warn(
        f"HIL_COMMAND_EVIDENCE_RECORDER_TOPIC_VERIFY ok={verify_result.ok} "
        f"missing={verify_result.missing} wrong_type={verify_result.wrong_type}"
    )
    if not verify_result.ok:
        node.get_logger().error(
            "HIL_COMMAND_EVIDENCE_RECORDER_REFUSING_TO_START: required command "
            f"topic(s) not resolvable as expected: missing={verify_result.missing} "
            f"wrong_type={verify_result.wrong_type}"
        )
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        return 1

    stop_at = time.monotonic() + args.duration_s
    try:
        while rclpy.ok() and time.monotonic() < stop_at:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        count = node.flush(args.output_csv)
        node.get_logger().warn(
            f"HIL_COMMAND_EVIDENCE_RECORDER_DONE rows={count} output={args.output_csv}"
        )
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
