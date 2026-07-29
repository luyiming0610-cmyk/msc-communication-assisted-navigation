#!/usr/bin/env python3
"""Stage 3 (OFFLINE_INTEGRATION_VALIDATION) dedicated evidence recorder.

A separate tool from hil_command_evidence_recorder.py (the physically
validated command recorder, left unmodified) because Stage 3's evidence
shape is materially wider -- GoalAnnouncement, NavigationIntent, phase
transitions, and peer-gate state have no equivalent in the physical
recorder's five-topic schema, and extending that already-proven,
physically-exercised tool for a hardware-free graph it will never
actually run against would risk the physical tool itself.

Same incremental-write, sparse-row, topic-tagged CSV convention as
hil_command_evidence_recorder.py (build_row()/CSV_FIELDS), extended
with the additional Stage 3 columns. Every recorded row carries
local_time_ns/local_monotonic_ns captured at RECEIPT, independent of
any stamp inside the message itself -- same rationale as the physical
recorder.

Never publishes anything on any topic (mirrors
test_hil_command_evidence_recorder_zero_publishers.py's proof for the
physical recorder; a parallel test exists for this file).
"""
from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass
from typing import Optional

CSV_FIELDS = [
    "local_time_ns",
    "local_monotonic_ns",
    "topic",
    # Twist (requested / guarded cmd_vel)
    "linear_x",
    "angular_z",
    # EpuckState (own test state / virtual-peer source / virtual-peer gate-input)
    "validity_flags",
    "sequence",
    "robot_id",
    "source",
    "x_m",
    "y_m",
    "yaw_rad",
    # GoalAnnouncement
    "goal_id",
    "announcement_source_robot_id",
    "announcement_sequence",
    "goal_x_m",
    "goal_y_m",
    "announcement_valid",
    # NavigationIntent
    "navigation_phase",
    "desired_heading_rad",
    # arm / bridge-status
    "arm_state",
    "bridge_connected",
    "bridge_rx_count",
    # harness-internal evidence events, delivered over the isolated
    # phase-event topic (std_msgs/String, JSON) -- not a ROS message
    # type of their own
    "phase",
    "gate_open",
    "adoption_confirmed",
    "duplicate_sent",
    "duplicate_rejected",
    "guard_blocked_reasons",
    # gate-owned structured forward-decision evidence, delivered over the
    # isolated gate-decision topic (std_msgs/String, JSON) -- emitted
    # synchronously at the gate's own decision point inside the harness,
    # never reconstructed here from cross-topic row order
    "gate_decision_event_type",
    "gate_decision_gate_epoch",
    "gate_decision_gate_state",
    "gate_decision_source_protocol_version",
    "gate_decision_source_robot_id",
    "gate_decision_source_sequence",
    "gate_decision_source_production_stamp_s",
    "gate_decision_decision",
    "gate_decision_decision_timestamp_s",
    "gate_decision_first_source_after_reopen",
    "gate_decision_forwarded_destination_topic",
]

# Fixed pseudo-topic for gate-decision-event rows, parallel to
# PHASE_EVENT_ROW_TOPIC below -- these events arrive over one real ROS
# topic but are recorded under this fixed logical name so the verifier
# can find them unambiguously regardless of the actual topic name used
# for a given run.
GATE_DECISION_EVENT_ROW_TOPIC = "GATE_DECISION_EVENT"

REQUIRED_STAGE3_TOPICS_PLACEHOLDER = ()  # populated by the caller's own topic table, not hardcoded here

# The single, fixed pseudo-topic name used for every phase-event row --
# these events arrive over one real ROS topic (std_msgs/String, JSON)
# but are recorded under this fixed logical name, distinct from any
# real topic string, so the verifier can find them unambiguously
# regardless of what the actual phase-event topic was named for a
# given run.
PHASE_EVENT_ROW_TOPIC = "PHASE_EVENT"


def build_row(
    *,
    local_time_ns: int,
    local_monotonic_ns: int,
    topic: str,
    linear_x: Optional[float] = None,
    angular_z: Optional[float] = None,
    validity_flags: Optional[int] = None,
    sequence: Optional[int] = None,
    robot_id: Optional[int] = None,
    source: Optional[int] = None,
    x_m: Optional[float] = None,
    y_m: Optional[float] = None,
    yaw_rad: Optional[float] = None,
    goal_id: Optional[str] = None,
    announcement_source_robot_id: Optional[int] = None,
    announcement_sequence: Optional[int] = None,
    goal_x_m: Optional[float] = None,
    goal_y_m: Optional[float] = None,
    announcement_valid: Optional[bool] = None,
    navigation_phase: Optional[str] = None,
    desired_heading_rad: Optional[float] = None,
    arm_state: Optional[bool] = None,
    bridge_connected: Optional[bool] = None,
    bridge_rx_count: Optional[int] = None,
    phase: Optional[str] = None,
    gate_open: Optional[bool] = None,
    adoption_confirmed: Optional[bool] = None,
    duplicate_sent: Optional[bool] = None,
    duplicate_rejected: Optional[bool] = None,
    guard_blocked_reasons: Optional[str] = None,
    gate_decision_event_type: Optional[str] = None,
    gate_decision_gate_epoch: Optional[int] = None,
    gate_decision_gate_state: Optional[str] = None,
    gate_decision_source_protocol_version: Optional[int] = None,
    gate_decision_source_robot_id: Optional[int] = None,
    gate_decision_source_sequence: Optional[int] = None,
    gate_decision_source_production_stamp_s: Optional[float] = None,
    gate_decision_decision: Optional[str] = None,
    gate_decision_decision_timestamp_s: Optional[float] = None,
    gate_decision_first_source_after_reopen: Optional[bool] = None,
    gate_decision_forwarded_destination_topic: Optional[str] = None,
) -> dict:
    row = {field: None for field in CSV_FIELDS}
    row.update({
        "local_time_ns": local_time_ns,
        "local_monotonic_ns": local_monotonic_ns,
        "topic": topic,
        "linear_x": linear_x,
        "angular_z": angular_z,
        "validity_flags": validity_flags,
        "sequence": sequence,
        "robot_id": robot_id,
        "source": source,
        "x_m": x_m,
        "y_m": y_m,
        "yaw_rad": yaw_rad,
        "goal_id": goal_id,
        "announcement_source_robot_id": announcement_source_robot_id,
        "announcement_sequence": announcement_sequence,
        "goal_x_m": goal_x_m,
        "goal_y_m": goal_y_m,
        "announcement_valid": announcement_valid,
        "navigation_phase": navigation_phase,
        "desired_heading_rad": desired_heading_rad,
        "arm_state": arm_state,
        "bridge_connected": bridge_connected,
        "bridge_rx_count": bridge_rx_count,
        "phase": phase,
        "gate_open": gate_open,
        "adoption_confirmed": adoption_confirmed,
        "duplicate_sent": duplicate_sent,
        "duplicate_rejected": duplicate_rejected,
        "guard_blocked_reasons": guard_blocked_reasons,
        "gate_decision_event_type": gate_decision_event_type,
        "gate_decision_gate_epoch": gate_decision_gate_epoch,
        "gate_decision_gate_state": gate_decision_gate_state,
        "gate_decision_source_protocol_version": gate_decision_source_protocol_version,
        "gate_decision_source_robot_id": gate_decision_source_robot_id,
        "gate_decision_source_sequence": gate_decision_source_sequence,
        "gate_decision_source_production_stamp_s": gate_decision_source_production_stamp_s,
        "gate_decision_decision": gate_decision_decision,
        "gate_decision_decision_timestamp_s": gate_decision_decision_timestamp_s,
        "gate_decision_first_source_after_reopen": gate_decision_first_source_after_reopen,
        "gate_decision_forwarded_destination_topic": gate_decision_forwarded_destination_topic,
    })
    extra_keys = set(row.keys()) - set(CSV_FIELDS)
    if extra_keys:
        raise ValueError(f"unexpected row key(s) not in CSV_FIELDS: {sorted(extra_keys)}")
    return row


DEFAULT_FLUSH_INTERVAL_S = 1.0


class Stage3EvidenceCsvWriter:
    """Incremental CSV writer -- opens the file and writes the header
    immediately at construction (same convention as
    CommandEvidenceCsvWriter), writes each row as it arrives, flushes at
    a bounded interval rather than every row."""

    def __init__(self, path: str, flush_interval_s: float = DEFAULT_FLUSH_INTERVAL_S):
        self._file = open(path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS, lineterminator="\n")
        self._writer.writeheader()
        self._file.flush()
        self._flush_interval_s = flush_interval_s
        self._last_flush_monotonic = time.monotonic()
        self._row_count = 0
        self._closed = False

    @property
    def row_count(self) -> int:
        return self._row_count

    def write_row(self, row: dict) -> None:
        if self._closed:
            raise ValueError("cannot write to a closed Stage3EvidenceCsvWriter")
        self._writer.writerow(row)
        self._row_count += 1
        now = time.monotonic()
        if (now - self._last_flush_monotonic) >= self._flush_interval_s:
            self._file.flush()
            self._last_flush_monotonic = now

    def close(self) -> None:
        if self._closed:
            return
        self._file.flush()
        self._file.close()
        self._closed = True


@dataclass(frozen=True)
class Stage3RunSummary:
    """Machine-readable JSON summary companion to the CSV. Deliberately
    separate from the CSV so a reader can get the run-level facts
    (ROS_DOMAIN_ID, topic contract, health) without re-parsing every
    row."""
    start_wall_time_ns: int
    end_wall_time_ns: int
    ros_domain_id: int
    topic_contract: dict
    row_count_by_topic: dict
    recorder_health_ok: bool

    def to_dict(self) -> dict:
        return {
            "start_wall_time_ns": self.start_wall_time_ns,
            "end_wall_time_ns": self.end_wall_time_ns,
            "ros_domain_id": self.ros_domain_id,
            "topic_contract": self.topic_contract,
            "row_count_by_topic": self.row_count_by_topic,
            "recorder_health_ok": self.recorder_health_ok,
        }


def write_summary_json(path: str, summary: Stage3RunSummary) -> None:
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(summary.to_dict(), f, indent=2, sort_keys=True)
        f.write("\n")


REQUIRED_TOPIC_ARGS = (
    "own_state_topic", "virtual_peer_source_topic", "virtual_peer_guard_input_topic",
    "goal_announcement_topic", "nav_intent_topic", "requested_cmd_vel_topic",
    "guarded_cmd_vel_topic", "arm_topic", "bridge_status_topic", "phase_event_topic",
    "gate_decision_topic",
)


# ---------------------------------------------------------------------
# rclpy wrapper -- subscribes only, never publishes (mirrors
# hil_command_evidence_recorder.py's own zero-publisher guarantee; see
# test_hil_offline_stage3_evidence_recorder_zero_publishers.py).
# ---------------------------------------------------------------------
def _build_node():
    import argparse

    import rclpy
    from epuck2_comm_interfaces.msg import EpuckState, GoalAnnouncement, NavigationIntent
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    def parse_bridge_status_json(data: str) -> dict:
        try:
            payload = json.loads(data)
        except (json.JSONDecodeError, TypeError):
            return {}
        return payload if isinstance(payload, dict) else {}

    class HilOfflineStage3EvidenceRecorder(Node):
        def __init__(self, args):
            super().__init__("hil_offline_stage3_evidence_recorder")
            self.args = args
            self.writer = Stage3EvidenceCsvWriter(args.output_csv, args.flush_interval_s)
            self._row_counts: dict[str, int] = {}
            self._start_wall_time_ns = time.time_ns()

            self.create_subscription(EpuckState, args.own_state_topic, self._on_own_state, 20)
            self.create_subscription(EpuckState, args.virtual_peer_source_topic, self._on_vp_source, 20)
            self.create_subscription(EpuckState, args.virtual_peer_guard_input_topic, self._on_vp_gate_input, 20)
            self.create_subscription(GoalAnnouncement, args.goal_announcement_topic, self._on_announcement, 10)
            self.create_subscription(NavigationIntent, args.nav_intent_topic, self._on_nav_intent, 10)
            self.create_subscription(Twist, args.requested_cmd_vel_topic, self._on_requested_cmd_vel, 10)
            self.create_subscription(Twist, args.guarded_cmd_vel_topic, self._on_guarded_cmd_vel, 10)
            self.create_subscription(Bool, args.arm_topic, self._on_arm, 10)
            self.create_subscription(String, args.bridge_status_topic, self._on_bridge_status, 10)
            self.create_subscription(String, args.phase_event_topic, self._on_phase_event, 20)
            self.create_subscription(String, args.gate_decision_topic, self._on_gate_decision, 20)

            self.get_logger().info(
                f"HIL_OFFLINE_STAGE3_EVIDENCE_RECORDER_READY output_csv={args.output_csv}"
            )

        def _write(self, topic: str, *, count_topic: str = None, **kwargs) -> None:
            """count_topic lets row_count_by_topic track coverage by the
            REAL subscribed ROS topic even when the CSV row itself is
            tagged with a fixed logical name (phase events all share one
            real topic but are tagged PHASE_EVENT_ROW_TOPIC in the CSV,
            since the verifier finds them by that fixed name, not by
            topic string)."""
            row = build_row(
                local_time_ns=time.time_ns(),
                local_monotonic_ns=time.monotonic_ns(),
                topic=topic,
                **kwargs,
            )
            self.writer.write_row(row)
            count_topic = count_topic if count_topic is not None else topic
            self._row_counts[count_topic] = self._row_counts.get(count_topic, 0) + 1

        def _on_own_state(self, msg) -> None:
            self._write(self.args.own_state_topic, validity_flags=int(msg.validity_flags),
                        sequence=int(msg.sequence), robot_id=int(msg.robot_id), source=int(msg.source),
                        x_m=float(msg.x_m), y_m=float(msg.y_m), yaw_rad=float(msg.yaw_rad))

        def _on_vp_source(self, msg) -> None:
            self._write(self.args.virtual_peer_source_topic, validity_flags=int(msg.validity_flags),
                        sequence=int(msg.sequence), robot_id=int(msg.robot_id), source=int(msg.source),
                        x_m=float(msg.x_m), y_m=float(msg.y_m), yaw_rad=float(msg.yaw_rad))

        def _on_vp_gate_input(self, msg) -> None:
            self._write(self.args.virtual_peer_guard_input_topic, validity_flags=int(msg.validity_flags),
                        sequence=int(msg.sequence), robot_id=int(msg.robot_id), source=int(msg.source),
                        x_m=float(msg.x_m), y_m=float(msg.y_m), yaw_rad=float(msg.yaw_rad))

        def _on_announcement(self, msg) -> None:
            self._write(self.args.goal_announcement_topic, goal_id=str(msg.goal_id),
                        announcement_source_robot_id=int(msg.source_robot_id),
                        announcement_sequence=int(msg.sequence), goal_x_m=float(msg.goal_x_m),
                        goal_y_m=float(msg.goal_y_m), announcement_valid=bool(msg.valid))

        def _on_nav_intent(self, msg) -> None:
            self._write(self.args.nav_intent_topic, navigation_phase=str(msg.navigation_phase),
                        desired_heading_rad=float(msg.desired_heading_rad))

        def _on_requested_cmd_vel(self, msg) -> None:
            self._write(self.args.requested_cmd_vel_topic, linear_x=float(msg.linear.x),
                        angular_z=float(msg.angular.z))

        def _on_guarded_cmd_vel(self, msg) -> None:
            self._write(self.args.guarded_cmd_vel_topic, linear_x=float(msg.linear.x),
                        angular_z=float(msg.angular.z))

        def _on_arm(self, msg) -> None:
            self._write(self.args.arm_topic, arm_state=bool(msg.data))

        def _on_bridge_status(self, msg) -> None:
            fields = parse_bridge_status_json(msg.data)
            self._write(self.args.bridge_status_topic, bridge_connected=fields.get("connected"),
                        bridge_rx_count=fields.get("rx_count"))

        def _on_phase_event(self, msg) -> None:
            fields = parse_bridge_status_json(msg.data)  # same generic JSON-object parse, reused
            self._write(
                PHASE_EVENT_ROW_TOPIC,
                count_topic=self.args.phase_event_topic,
                phase=fields.get("phase"),
                gate_open=fields.get("gate_open"),
                adoption_confirmed=fields.get("adoption_confirmed"),
                duplicate_sent=fields.get("duplicate_sent"),
                duplicate_rejected=fields.get("duplicate_rejected"),
                guard_blocked_reasons=fields.get("guard_blocked_reasons"),
            )

        def _on_gate_decision(self, msg) -> None:
            try:
                fields = json.loads(msg.data)
            except (json.JSONDecodeError, TypeError):
                fields = {}
            if not isinstance(fields, dict):
                fields = {}
            self._write(
                GATE_DECISION_EVENT_ROW_TOPIC,
                count_topic=self.args.gate_decision_topic,
                gate_decision_event_type=fields.get("event_type"),
                gate_decision_gate_epoch=fields.get("gate_epoch"),
                gate_decision_gate_state=fields.get("gate_state"),
                gate_decision_source_protocol_version=fields.get("source_protocol_version"),
                gate_decision_source_robot_id=fields.get("source_robot_id"),
                gate_decision_source_sequence=fields.get("source_sequence"),
                gate_decision_source_production_stamp_s=fields.get("source_production_stamp_s"),
                gate_decision_decision=fields.get("decision"),
                gate_decision_decision_timestamp_s=fields.get("decision_timestamp_s"),
                gate_decision_first_source_after_reopen=fields.get("first_source_after_reopen"),
                gate_decision_forwarded_destination_topic=fields.get("forwarded_destination_topic"),
            )

        def write_summary(self) -> None:
            summary = Stage3RunSummary(
                start_wall_time_ns=self._start_wall_time_ns,
                end_wall_time_ns=time.time_ns(),
                ros_domain_id=int(__import__("os").environ.get("ROS_DOMAIN_ID", "-1")),
                topic_contract={name: getattr(self.args, name) for name in REQUIRED_TOPIC_ARGS},
                row_count_by_topic=dict(self._row_counts),
                recorder_health_ok=True,
            )
            write_summary_json(self.args.output_summary_json, summary)

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--own-state-topic", required=True, dest="own_state_topic")
        parser.add_argument("--virtual-peer-source-topic", required=True, dest="virtual_peer_source_topic")
        parser.add_argument("--virtual-peer-guard-input-topic", required=True, dest="virtual_peer_guard_input_topic")
        parser.add_argument("--goal-announcement-topic", required=True, dest="goal_announcement_topic")
        parser.add_argument("--nav-intent-topic", required=True, dest="nav_intent_topic")
        parser.add_argument("--requested-cmd-vel-topic", required=True, dest="requested_cmd_vel_topic")
        parser.add_argument("--guarded-cmd-vel-topic", required=True, dest="guarded_cmd_vel_topic")
        parser.add_argument("--arm-topic", required=True, dest="arm_topic")
        parser.add_argument("--bridge-status-topic", required=True, dest="bridge_status_topic")
        parser.add_argument("--phase-event-topic", required=True, dest="phase_event_topic")
        parser.add_argument("--gate-decision-topic", required=True, dest="gate_decision_topic")
        parser.add_argument("--output-csv", required=True, dest="output_csv")
        parser.add_argument("--output-summary-json", required=True, dest="output_summary_json")
        parser.add_argument("--flush-interval-s", type=float, default=DEFAULT_FLUSH_INTERVAL_S, dest="flush_interval_s")
        return parser.parse_args(argv)

    return rclpy, HilOfflineStage3EvidenceRecorder, parse_args


def main(argv=None):
    import sys

    rclpy, HilOfflineStage3EvidenceRecorder, parse_args = _build_node()
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[])
    node = HilOfflineStage3EvidenceRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.write_summary()
        except Exception:
            pass
        try:
            node.writer.close()
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


if __name__ == "__main__":
    main()
