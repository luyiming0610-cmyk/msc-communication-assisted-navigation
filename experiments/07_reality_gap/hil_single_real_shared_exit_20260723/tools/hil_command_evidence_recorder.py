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
  - /epuck1/state      (EpuckState, validity_flags + pose fields)
  - /epuck_bridge/status (std_msgs/String, JSON payload)

State-topic pose capture (state_x_m/state_y_m/state_yaw_rad, added
2026-07-27 for SINGLE_ROBOT_GROUND_REPEATABILITY_BASELINE): purely
additive CSV columns read from the SAME EpuckState subscription this
recorder already holds -- state_publisher.py has always populated
x_m/y_m/yaw_rad from its own real /odom subscription, this recorder
simply did not record them before. No new subscription, no message
type change, no state_publisher change. Every existing column's
meaning is unchanged; a CSV produced before this change (missing these
three columns entirely) remains valid input to every existing reader.

Never publishes anything (verified by
test_hil_command_evidence_recorder_zero_publishers.py).

CSV writing is INCREMENTAL, not batched at shutdown (changed
2026-07-24 after a real powered-session activation attempt found the
file simply did not exist while the recorder was running, contrary to
COMMAND_EVIDENCE_ACTIVATION.md's expectation that it could be watched
"growing" mid-session -- see
command_evidence_activation_20260724/SUMMARY.md for the full incident).
The CSV is created and its header written immediately at startup; each
row is written to the file object as soon as its event arrives, and
the file is flushed to disk at a bounded interval (default 1.0s,
`--flush-interval-s`) rather than after every single row -- this keeps
the recorder's own I/O cost low (no `os.fsync` at all; a plain
`flush()` merely asks the OS to accept the buffered writes, it does
not force a physical disk write) while still guaranteeing the file is
valid and inspectable at essentially any point during a run, not only
at the end. The file remains a valid, parseable CSV after a SIGINT, a
duration timeout, or a normal shutdown, because closing the file
handle in the `finally` block always flushes whatever was buffered
since the last periodic flush.

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
    "state_x_m",
    "state_y_m",
    "state_yaw_rad",
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
    state_x_m: Optional[float] = None,
    state_y_m: Optional[float] = None,
    state_yaw_rad: Optional[float] = None,
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
        "state_x_m": state_x_m,
        "state_y_m": state_y_m,
        "state_yaw_rad": state_yaw_rad,
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


DEFAULT_FLUSH_INTERVAL_S = 1.0


class CommandEvidenceCsvWriter:
    """Incremental CSV writer: opens the file and writes the header
    immediately at construction, writes each row as it arrives, and
    flushes to disk at a bounded interval rather than after every row.

    Never calls os.fsync -- a plain file.flush() is sufficient (it
    hands buffered data to the OS; the OS's own write-back handles the
    physical disk write on its own schedule) and far cheaper than
    forcing a physical write on every single message.

    Deliberately not a dataclass / not frozen: it owns a live file
    handle and a monotonic flush clock, both mutable by design.
    """

    def __init__(self, path: str, flush_interval_s: float = DEFAULT_FLUSH_INTERVAL_S):
        self._path = path
        self._flush_interval_s = float(flush_interval_s)
        self._fh = open(path, "w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._fh, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._fh.flush()
        self._last_flush_monotonic = time.monotonic()
        self._row_count = 0
        self._closed = False

    @property
    def row_count(self) -> int:
        return self._row_count

    @property
    def path(self) -> str:
        return self._path

    def write_row(self, row: dict, *, now_monotonic: Optional[float] = None) -> None:
        if self._closed:
            raise ValueError("cannot write to a closed CommandEvidenceCsvWriter")
        self._writer.writerow(row)
        self._row_count += 1
        now = now_monotonic if now_monotonic is not None else time.monotonic()
        if now - self._last_flush_monotonic >= self._flush_interval_s:
            self._fh.flush()
            self._last_flush_monotonic = now

    def close(self) -> None:
        if self._closed:
            return
        self._fh.flush()
        self._fh.close()
        self._closed = True


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
            # Opens the file and writes the header immediately -- the
            # CSV exists and is a valid (header-only) file from this
            # point forward, not only once the run ends.
            self._csv_writer = CommandEvidenceCsvWriter(
                args.output_csv, flush_interval_s=args.flush_interval_s
            )

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
            self._csv_writer.write_row(
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
                state_x_m=float(msg.x_m),
                state_y_m=float(msg.y_m),
                state_yaw_rad=float(msg.yaw_rad),
            )

        def _on_bridge_status(self, msg) -> None:
            fields = parse_bridge_status_json(msg.data)
            self._append(
                topic=self.args.bridge_status_topic,
                bridge_connected=fields.get("connected"),
                bridge_rx_count=fields.get("rx_count"),
            )

        def close_csv(self) -> int:
            row_count = self._csv_writer.row_count
            self._csv_writer.close()
            return row_count

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--upstream-cmd-vel-topic", default="cmd_vel_unguarded")
        parser.add_argument("--guarded-cmd-vel-topic", default="cmd_vel")
        parser.add_argument("--arm-topic", default="/hil_guard/arm")
        parser.add_argument("--state-topic", default="/epuck1/state")
        parser.add_argument("--bridge-status-topic", default="/epuck_bridge/status")
        parser.add_argument("--output-csv", required=True)
        parser.add_argument("--duration-s", type=float, default=3600.0)
        parser.add_argument(
            "--flush-interval-s",
            type=float,
            default=DEFAULT_FLUSH_INTERVAL_S,
            help="How often (seconds) the CSV is flushed to disk. Never fsync'd; a "
            "conservative periodic flush() is sufficient and far cheaper than "
            "flushing after every row.",
        )
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
        # The CSV writer was already opened (header-only, zero rows) in
        # the constructor, before this check ran -- must still be
        # closed properly even on this early-refusal path.
        try:
            node.close_csv()
        except Exception:
            pass
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
        count = node.close_csv()
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
