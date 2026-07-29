#!/usr/bin/env python3
"""Limited, hardware-free recorder-verifier integration test -- NOT the
final Stage 3 multi-process graph. Requires a sourced ROS 2 workspace,
run under an explicit isolated ROS_DOMAIN_ID (93 for this preparation
pass, distinct from 0/77/89/91/92) AND ROS_LOCALHOST_ONLY=1. Starts only
the Stage 3 evidence recorder as a real rclpy node plus a handful of
plain publisher utilities standing in for every upstream node -- never
cooperative_avoider.py, never hil_topic_adapter.py/hil_virtual_peer.py/
hil_cmd_vel_guard.py together as the real graph, never a physical
bridge, never a production cmd_vel-producing node of any kind.

Every topic used here lives under
/hil_offline_stage3_preparation_test/... -- a namespace distinct from
both the real Stage 3 namespace (/hil_offline_stage3/...) and every
production topic.

Evidence is written to a pytest/tempfile-managed TemporaryDirectory,
deleted automatically at the end of the test -- never a path under the
repository, never project evidence.

Invoke as:
  ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_offline_stage3_recorder_verifier_integration.py -v
Also contains FullBehaviouralIntegrationTest -- a second, additional test
in this same file (still one of the nine authorised Stage 3 preparation
paths; no new file) that DOES instantiate the real production behavioral
chain (HilOfflineStage3Harness, Stage3AutomaticRunner, the real
adapter-hosted HilGoalAnnouncementEvidenceNavigator/GoalNavigator,
NavigationTargetState.receive_announcement, the real
DuplicateAnnouncementController, and the real virtual peer
HilVirtualPeer) alongside the real recorder and verifier, under
ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1 and topics under
/hil_offline_stage3_behavior_test/... -- still never cooperative_avoider,
never hil_cmd_vel_guard.py, never a physical bridge, never Webots, never
the final Stage 3 multi-process graph.
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor

from epuck2_comm_interfaces.msg import EpuckState, GoalAnnouncement, NavigationIntent
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from hil_offline_stage3_evidence_recorder import GATE_DECISION_EVENT_ROW_TOPIC, PHASE_EVENT_ROW_TOPIC, _build_node
from hil_offline_stage3_post_run_verifier import DEFAULT_VIRTUAL_PEER_TIMEOUT_S, run_verifier

_COOP_EXIT_TOOLS_DIR = str(
    Path(__file__).resolve().parents[3] / "10_cooperative_exit_navigation_20260720" / "tools"
)
if _COOP_EXIT_TOOLS_DIR not in sys.path:
    sys.path.insert(0, _COOP_EXIT_TOOLS_DIR)

import hil_virtual_peer  # noqa: E402  (path adjusted above by design)
import goal_navigator  # noqa: E402

from hil_goal_announcement_evidence import build_evidence_navigator_class  # noqa: E402
from hil_offline_stage3_harness import (  # noqa: E402
    DuplicateOrderingError,
    FORBIDDEN_ROS_DOMAIN_IDS,
    EXPECTED_STAGE3_ROS_DOMAIN_ID,
    Stage3AutomaticRunner,
    Stage3Phase,
)
from hil_offline_stage3_harness import _build_node as _build_harness_node  # noqa: E402

NS = "/hil_offline_stage3_preparation_test"
OWN_STATE_TOPIC = f"{NS}/epuck1/state"
VP_SOURCE_TOPIC = f"{NS}/virtual_peer/source_state"
VP_GATE_INPUT_TOPIC = f"{NS}/virtual_peer/guard_input_state"
GOAL_ANNOUNCEMENT_TOPIC = f"{NS}/goal_announcement"
NAV_INTENT_TOPIC = f"{NS}/epuck1/nav_intent"
REQUESTED_CMD_VEL_TOPIC = f"{NS}/cmd_vel_unguarded_test_only"
GUARDED_CMD_VEL_TOPIC = f"{NS}/cmd_vel_guarded_test_only"
ARM_TOPIC = f"{NS}/guard_arm_test_only"
BRIDGE_STATUS_TOPIC = f"{NS}/bridge_status_test_only"
PHASE_EVENT_TOPIC = f"{NS}/phase_event_test_only"
GATE_DECISION_TOPIC = f"{NS}/gate_decision_test_only"

INTEGRATION_TEST_ROS_DOMAIN_ID = 93


def _spin_for(executor, seconds, step=0.02):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=step)


def wait_until_wall_clock_at_least(target_ns: int, *, timeout_s: float = 5.0, poll_interval_s: float = 0.005) -> int:
    """Blocks until time.time_ns() >= target_ns -- the SAME clock
    hil_offline_stage3_evidence_recorder.py itself uses to stamp
    local_time_ns at receipt (see its own `local_time_ns=time.time_ns()`
    call) -- so a caller who derives `target_ns` from an
    already-observed wall-clock reference point gets a deterministic,
    causally-grounded guarantee about ordering relative to that
    reference, not a guess about how long "enough" wall-clock delay
    happens to be. Returns the actual time.time_ns() value observed at
    return (>= target_ns). Bounded by `timeout_s`: raises TimeoutError
    with a clear reason if target_ns is never reached (e.g. a clock
    regression), rather than hanging indefinitely.
    """
    deadline = time.monotonic() + timeout_s
    while True:
        now_ns = time.time_ns()
        if now_ns >= target_ns:
            return now_ns
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"wait_until_wall_clock_at_least: target_ns={target_ns} not reached "
                f"within timeout_s={timeout_s} (last observed time.time_ns()={now_ns})"
            )
        time.sleep(poll_interval_s)


class WaitUntilWallClockHelperTest(unittest.TestCase):
    """Pure, deterministic, no-ROS-dependency proof of
    wait_until_wall_clock_at_least() itself -- the causal-condition wait
    helper that replaced the previous test's fixed time.sleep(1.1)."""

    def test_returns_only_once_target_is_actually_reached(self):
        target = time.time_ns() + int(0.05 * 1e9)
        observed = wait_until_wall_clock_at_least(target, timeout_s=5.0)
        self.assertGreaterEqual(observed, target)
        self.assertGreaterEqual(time.time_ns(), target)

    def test_does_not_return_early(self):
        target = time.time_ns() + int(0.2 * 1e9)
        before = time.monotonic()
        wait_until_wall_clock_at_least(target, timeout_s=5.0)
        elapsed = time.monotonic() - before
        self.assertGreaterEqual(elapsed, 0.19, "must not return before the target wall-clock time")

    def test_raises_clear_timeout_when_target_is_unreachable_within_budget(self):
        target = time.time_ns() + int(10 * 1e9)  # 10s away
        with self.assertRaises(TimeoutError) as ctx:
            wait_until_wall_clock_at_least(target, timeout_s=0.05, poll_interval_s=0.01)
        self.assertIn("not reached", str(ctx.exception))
        self.assertIn(str(target), str(ctx.exception))

    def test_returns_immediately_if_target_already_in_the_past(self):
        target = time.time_ns() - int(1 * 1e9)
        before = time.monotonic()
        wait_until_wall_clock_at_least(target, timeout_s=5.0)
        elapsed = time.monotonic() - before
        self.assertLess(elapsed, 1.0, "a past target must not incur any wait")


class RecorderVerifierIntegrationTest(unittest.TestCase):
    """EVIDENCE_PIPELINE_INTEGRATION_ONLY scope: proves the real
    HilOfflineStage3EvidenceRecorder + run_verifier() correctly ingest
    and evaluate a well-formed evidence stream. The announcement,
    adoption, duplicate, and gate-decision events below are hand-published
    by this test's own stimulus node -- they do NOT exercise the real
    HilOfflineStage3Harness, Stage3AutomaticRunner, adapter-hosted
    navigator, NavigationTargetState.receive_announcement, or
    DuplicateAnnouncementController. See FullBehaviouralIntegrationTest
    below for the test that does exercise those real production paths."""

    def setUp(self):
        import os
        from hil_offline_stage3_harness import FORBIDDEN_ROS_DOMAIN_IDS, EXPECTED_STAGE3_ROS_DOMAIN_ID

        self.assertEqual(
            os.environ.get("ROS_LOCALHOST_ONLY"), "1",
            "This live ROS test must be invoked with ROS_LOCALHOST_ONLY=1",
        )
        current_domain = int(os.environ.get("ROS_DOMAIN_ID", -1))
        # This test is DESIGNED to be run standalone under
        # ROS_DOMAIN_ID=93 (see the runbook and the Stage 3 preparation
        # report). It still must never run under a forbidden/production
        # domain (0/77/89) or the real Stage 3 domain (91) even when
        # invoked as part of a larger combined-directory test run that
        # does not itself set ROS_DOMAIN_ID=93 -- that combination is
        # rejected here rather than silently reusing an unrelated domain.
        self.assertNotIn(
            current_domain, FORBIDDEN_ROS_DOMAIN_IDS | {EXPECTED_STAGE3_ROS_DOMAIN_ID},
            f"ROS_DOMAIN_ID={current_domain} is forbidden or reserved for the real Stage 3 domain",
        )
        self.effective_domain = current_domain
        self._tmpdir = tempfile.TemporaryDirectory()
        self.csv_path = str(Path(self._tmpdir.name) / "evidence.csv")
        self.summary_path = str(Path(self._tmpdir.name) / "summary.json")

        rclpy.init(args=[])
        _, HilOfflineStage3EvidenceRecorder, parse_args = _build_node()
        args = parse_args([
            "--own-state-topic", OWN_STATE_TOPIC,
            "--virtual-peer-source-topic", VP_SOURCE_TOPIC,
            "--virtual-peer-guard-input-topic", VP_GATE_INPUT_TOPIC,
            "--goal-announcement-topic", GOAL_ANNOUNCEMENT_TOPIC,
            "--nav-intent-topic", NAV_INTENT_TOPIC,
            "--requested-cmd-vel-topic", REQUESTED_CMD_VEL_TOPIC,
            "--guarded-cmd-vel-topic", GUARDED_CMD_VEL_TOPIC,
            "--arm-topic", ARM_TOPIC,
            "--bridge-status-topic", BRIDGE_STATUS_TOPIC,
            "--phase-event-topic", PHASE_EVENT_TOPIC,
            "--gate-decision-topic", GATE_DECISION_TOPIC,
            "--output-csv", self.csv_path,
            "--output-summary-json", self.summary_path,
            "--flush-interval-s", "0.05",
        ])
        self.recorder = HilOfflineStage3EvidenceRecorder(args)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.recorder)

        self.stim = rclpy.create_node("stage3_prep_test_stimulus")
        self.executor.add_node(self.stim)
        self.own_state_pub = self.stim.create_publisher(EpuckState, OWN_STATE_TOPIC, 10)
        self.vp_source_pub = self.stim.create_publisher(EpuckState, VP_SOURCE_TOPIC, 10)
        self.vp_gate_input_pub = self.stim.create_publisher(EpuckState, VP_GATE_INPUT_TOPIC, 10)
        self.announcement_pub = self.stim.create_publisher(GoalAnnouncement, GOAL_ANNOUNCEMENT_TOPIC, 10)
        self.nav_intent_pub = self.stim.create_publisher(NavigationIntent, NAV_INTENT_TOPIC, 10)
        self.requested_pub = self.stim.create_publisher(Twist, REQUESTED_CMD_VEL_TOPIC, 10)
        self.guarded_pub = self.stim.create_publisher(Twist, GUARDED_CMD_VEL_TOPIC, 10)
        self.arm_pub = self.stim.create_publisher(Bool, ARM_TOPIC, 10)
        self.bridge_status_pub = self.stim.create_publisher(String, BRIDGE_STATUS_TOPIC, 10)
        self.phase_event_pub = self.stim.create_publisher(String, PHASE_EVENT_TOPIC, 10)
        self.gate_decision_pub = self.stim.create_publisher(String, GATE_DECISION_TOPIC, 10)

    def tearDown(self):
        try:
            self.recorder.write_summary()
        except Exception:
            pass
        try:
            self.recorder.writer.close()
        except Exception:
            pass
        self.executor.remove_node(self.stim)
        self.executor.remove_node(self.recorder)
        self.stim.destroy_node()
        self.recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._tmpdir.cleanup()

    def _publish_state(self, pub, *, robot_id, sequence, source, validity_flags=7, x=0.0, y=0.0, yaw=0.0):
        msg = EpuckState()
        msg.version = EpuckState.PROTOCOL_VERSION
        msg.robot_id = robot_id
        msg.sequence = sequence
        msg.stamp = self.stim.get_clock().now().to_msg()
        msg.source = source
        msg.x_m, msg.y_m, msg.yaw_rad = x, y, yaw
        msg.validity_flags = validity_flags
        pub.publish(msg)

    def _publish_phase_event(self, **fields):
        msg = String()
        msg.data = json.dumps(fields)
        self.phase_event_pub.publish(msg)

    def _publish_gate_decision(self, **fields):
        now_s = time.time()
        event = {
            "event_type": "GATE_DECISION",
            "gate_epoch": 0,
            "gate_state": "OPEN",
            "source_protocol_version": 1,
            "source_robot_id": 2,
            "source_sequence": 0,
            "source_production_stamp_s": now_s,
            "decision": "FORWARDED",
            "decision_timestamp_s": now_s,
            "first_source_after_reopen": False,
            "forwarded_destination_topic": None,
        }
        event.update(fields)
        msg = String()
        msg.data = json.dumps(event)
        self.gate_decision_pub.publish(msg)

    def _publish_goal_announcement(self, *, sequence, goal_id="shared_exit"):
        ann = GoalAnnouncement()
        ann.protocol_version = GoalAnnouncement.PROTOCOL_VERSION
        ann.source_robot_id = 2
        ann.sequence = sequence
        ann.production_stamp = self.stim.get_clock().now().to_msg()
        ann.goal_id = goal_id
        ann.goal_x_m, ann.goal_y_m = 5.0, 5.0
        ann.valid = True
        self.announcement_pub.publish(ann)

    def test_full_recorder_verifier_round_trip_produces_valid_success(self):
        # Own state, continuously fresh/valid.
        self._publish_state(self.own_state_pub, robot_id=1, sequence=1, source=EpuckState.SOURCE_HARDWARE)
        _spin_for(self.executor, 0.05)

        # Original GoalAnnouncement + adoption -- exactly one.
        self._publish_goal_announcement(sequence=1)
        self._publish_phase_event(phase="ANNOUNCEMENT_ADOPTED")
        _spin_for(self.executor, 0.05)

        # Virtual peer source continues throughout; gate-input forwarded
        # before closure.
        self._publish_state(self.vp_source_pub, robot_id=2, sequence=1, source=EpuckState.SOURCE_VIRTUAL)
        self._publish_state(self.vp_gate_input_pub, robot_id=2, sequence=1, source=EpuckState.SOURCE_VIRTUAL)
        _spin_for(self.executor, 0.05)

        self._publish_phase_event(gate_open=False, phase="PEER_GATE_CLOSED")
        _spin_for(self.executor, 0.05)
        # The recorder's own PEER_GATE_CLOSED callback runs synchronously
        # inside this single-threaded executor's spin above (same
        # process, same thread) -- so it has already executed and
        # recorded local_time_ns=time.time_ns() by the time control
        # returns here. This wall-clock reading is therefore a strict,
        # causally-grounded upper bound on that recorded close_ts (not a
        # guess): close_ts happened before this read, in this thread.
        close_processed_wall_ns = time.time_ns()

        # Source keeps publishing during the closed interval; gate-input
        # does NOT (suppressed) -- proves continuation vs. suppression.
        # The gate itself also emits its own structured decision event
        # for each source message it processes while closed -- this is
        # the gate-owned evidence the verifier now relies on, never a
        # cross-topic row-order inference.
        self._publish_state(self.vp_source_pub, robot_id=2, sequence=2, source=EpuckState.SOURCE_VIRTUAL)
        self._publish_gate_decision(
            gate_epoch=0, gate_state="CLOSED", source_sequence=2, decision="REJECTED_GATE_CLOSED",
            forwarded_destination_topic=None,
        )
        _spin_for(self.executor, 0.3)
        self._publish_state(self.vp_source_pub, robot_id=2, sequence=3, source=EpuckState.SOURCE_VIRTUAL)
        self._publish_gate_decision(
            gate_epoch=0, gate_state="CLOSED", source_sequence=3, decision="REJECTED_GATE_CLOSED",
            forwarded_destination_topic=None,
        )

        # Peer-timeout (DEFAULT_VIRTUAL_PEER_TIMEOUT_S, the same
        # threshold the verifier itself checks against) must have
        # definitely elapsed before STALE_ZERO_CONFIRMED is published --
        # waited for via the actual causal condition (wall-clock time
        # since the already-processed PEER_GATE_CLOSED event), never a
        # fixed sleep whose margin against the verifier's threshold can
        # be eaten by scheduling jitter under load.
        wait_until_wall_clock_at_least(
            close_processed_wall_ns + int(DEFAULT_VIRTUAL_PEER_TIMEOUT_S * 1e9)
        )
        self._publish_phase_event(phase="STALE_ZERO_CONFIRMED")
        _spin_for(self.executor, 0.05)

        self._publish_phase_event(gate_open=True, phase="PEER_GATE_REOPENED")
        _spin_for(self.executor, 0.05)

        # Fresh source + forwarded gate-input after reopening, without
        # backlog -- and the gate's own FORWARDED, first-after-reopen
        # decision event for this exact message.
        self._publish_state(self.vp_source_pub, robot_id=2, sequence=4, source=EpuckState.SOURCE_VIRTUAL)
        self._publish_state(self.vp_gate_input_pub, robot_id=2, sequence=4, source=EpuckState.SOURCE_VIRTUAL)
        self._publish_gate_decision(
            gate_epoch=1, gate_state="OPEN", source_sequence=4, decision="FORWARDED",
            first_source_after_reopen=True, forwarded_destination_topic=VP_GATE_INPUT_TOPIC,
        )
        _spin_for(self.executor, 0.05)
        self._publish_phase_event(phase="RECOVERY_CONFIRMED")
        _spin_for(self.executor, 0.05)

        # NavigationIntent, requested/guarded commands, arm, bridge status.
        nav = NavigationIntent()
        nav.protocol_version = NavigationIntent.PROTOCOL_VERSION
        nav.source_robot_id = 1
        nav.sequence = 1
        nav.production_stamp = self.stim.get_clock().now().to_msg()
        nav.desired_heading_rad = 0.0
        nav.desired_linear_speed_mps = 0.01
        nav.navigation_phase = "GO_TO_EXIT"
        nav.valid = True
        self.nav_intent_pub.publish(nav)

        requested = Twist()
        requested.linear.x = 0.01
        requested.angular.z = 0.02
        self.requested_pub.publish(requested)
        guarded = Twist()
        guarded.linear.x = 0.01
        guarded.angular.z = 0.02
        self.guarded_pub.publish(guarded)

        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)

        bridge_msg = String()
        bridge_msg.data = json.dumps({"connected": True, "rx_count": 5})
        self.bridge_status_pub.publish(bridge_msg)
        _spin_for(self.executor, 0.05)

        # The one permitted duplicate GoalAnnouncement -- sent only after
        # the single adoption above -- followed by a rejected SECOND
        # duplicate attempt. Both must be individually recorded and
        # correctly ordered relative to adoption/each other.
        self._publish_goal_announcement(sequence=999999)
        self._publish_phase_event(duplicate_sent=True)
        _spin_for(self.executor, 0.05)
        self._publish_phase_event(duplicate_rejected=True, guard_blocked_reasons="duplicate already sent")
        _spin_for(self.executor, 0.05)

        self._publish_phase_event(phase="COMPLETE")
        _spin_for(self.executor, 0.2)

        # Finalize evidence before verifying (mirrors main()'s finally block).
        self.recorder.write_summary()
        self.recorder.writer.close()

        with open(self.summary_path, encoding="utf-8") as f:
            summary = json.load(f)
        self.assertEqual(summary["ros_domain_id"], self.effective_domain)
        for key in (
            "own_state_topic", "virtual_peer_source_topic", "virtual_peer_guard_input_topic",
            "goal_announcement_topic", "nav_intent_topic", "requested_cmd_vel_topic",
            "guarded_cmd_vel_topic", "arm_topic", "bridge_status_topic", "phase_event_topic",
            "gate_decision_topic",
        ):
            self.assertGreater(summary["row_count_by_topic"].get(summary["topic_contract"][key], 0), 0, key)

        with open(self.csv_path, newline="", encoding="utf-8") as f:
            import csv as csv_module
            rows = list(csv_module.DictReader(f))

        announcement_rows = [r for r in rows if r["topic"] == GOAL_ANNOUNCEMENT_TOPIC]
        self.assertEqual(len(announcement_rows), 2, "exactly one original + one duplicate announcement")
        self.assertEqual(
            sorted(int(r["announcement_sequence"]) for r in announcement_rows), [1, 999999],
        )

        adoption_rows = [r for r in rows if r.get("phase") == "ANNOUNCEMENT_ADOPTED"]
        self.assertEqual(len(adoption_rows), 1, "exactly one adoption event, never a second")

        duplicate_sent_rows = [r for r in rows if r.get("duplicate_sent") == "True"]
        self.assertEqual(len(duplicate_sent_rows), 1, "exactly one duplicate-sent event, never a second")

        duplicate_rejected_rows = [r for r in rows if r.get("duplicate_rejected") == "True"]
        self.assertEqual(len(duplicate_rejected_rows), 1, "the second duplicate attempt must be recorded as rejected")

        # Ordering: duplicate-sent strictly after adoption; duplicate-
        # rejected strictly after duplicate-sent.
        self.assertLess(
            int(adoption_rows[0]["local_time_ns"]), int(duplicate_sent_rows[0]["local_time_ns"]),
        )
        self.assertLess(
            int(duplicate_sent_rows[0]["local_time_ns"]), int(duplicate_rejected_rows[0]["local_time_ns"]),
        )

        gate_decision_rows = [r for r in rows if r["topic"] == GATE_DECISION_EVENT_ROW_TOPIC]
        self.assertGreaterEqual(len(gate_decision_rows), 3)
        rejected_while_closed = [
            r for r in gate_decision_rows if r["gate_decision_gate_state"] == "CLOSED"
        ]
        self.assertTrue(rejected_while_closed)
        self.assertTrue(all(
            r["gate_decision_decision"] == "REJECTED_GATE_CLOSED" for r in rejected_while_closed
        ), "every closed-gate decision must be REJECTED_GATE_CLOSED")
        first_after_reopen_rows = [
            r for r in gate_decision_rows if r.get("gate_decision_first_source_after_reopen") == "True"
        ]
        self.assertEqual(len(first_after_reopen_rows), 1)
        self.assertEqual(first_after_reopen_rows[0]["gate_decision_decision"], "FORWARDED")

        phase_rows = [r for r in rows if r["topic"] == PHASE_EVENT_ROW_TOPIC]
        self.assertTrue(any(r.get("phase") == "COMPLETE" for r in phase_rows))

        result = run_verifier(
            csv_path=self.csv_path, summary_json_path=self.summary_path,
            residual_process_detected=False,
            test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
            expected_domain_id=self.effective_domain,
        )
        self.assertEqual(result["DATA_VALIDITY"], "VALID", result["data_validity_reasons"])
        self.assertEqual(result["TASK_OUTCOME"], "SUCCESS", result["task_outcome_reasons"])


BEHAVIOR_NS = "/hil_offline_stage3_behavior_test"
BEHAVIOR_OWN_STATE_TOPIC = f"{BEHAVIOR_NS}/epuck1/state"
BEHAVIOR_VP_SOURCE_TOPIC = f"{BEHAVIOR_NS}/virtual_peer/source_state"
BEHAVIOR_VP_GATE_INPUT_TOPIC = f"{BEHAVIOR_NS}/virtual_peer/guard_input_state"
BEHAVIOR_GOAL_ANNOUNCEMENT_TOPIC = f"{BEHAVIOR_NS}/goal_announcement"
BEHAVIOR_NAV_INTENT_TOPIC = f"{BEHAVIOR_NS}/epuck1/nav_intent"
BEHAVIOR_REQUESTED_CMD_VEL_TOPIC = f"{BEHAVIOR_NS}/cmd_vel_unguarded_test_only"
BEHAVIOR_GUARDED_CMD_VEL_TOPIC = f"{BEHAVIOR_NS}/cmd_vel_guarded_test_only"
BEHAVIOR_ARM_TOPIC = f"{BEHAVIOR_NS}/guard_arm_test_only"
BEHAVIOR_BRIDGE_STATUS_TOPIC = f"{BEHAVIOR_NS}/bridge_status_test_only"
BEHAVIOR_PHASE_EVENT_TOPIC = f"{BEHAVIOR_NS}/phase_event_test_only"
BEHAVIOR_GATE_DECISION_TOPIC = f"{BEHAVIOR_NS}/gate_decision_test_only"

VIRTUAL_PEER_TARGET = (2.0, 3.0)
ORIGINAL_GOAL_ID = "shared_exit"
# Sequence the harness's own request_duplicate_publication() always uses --
# a fixed sentinel, read from hil_offline_stage3_harness.py's source, never
# supplied by any caller. Documented here, not invented for this test.
HARNESS_DUPLICATE_SEQUENCE = 999999


class FullBehaviouralIntegrationTest(unittest.TestCase):
    """FULL_BEHAVIOURAL_INTEGRATION scope. Instantiates and runs together,
    over one shared isolated topic set, the real:
      - HilOfflineStage3Harness + Stage3AutomaticRunner (hil_offline_stage3_harness.py);
      - HilVirtualPeer, the original GoalAnnouncement source (hil_virtual_peer.py);
      - HilGoalAnnouncementEvidenceNavigator -- exactly the class
        hil_topic_adapter.py's own main() constructs (a thin GoalNavigator
        subclass, hil_goal_announcement_evidence.py), so
        NavigationTargetState.receive_announcement() and the real
        adoption/idempotent-duplicate-rejection logic in
        navigation_target_state.py both execute for real;
      - HilOfflineStage3EvidenceRecorder + run_verifier() (the same real
        production evidence/verification tools used everywhere else).

    Duplicate-identity contract (read from source, not assumed): a message
    is "the duplicate" of an already-adopted GoalAnnouncement if and only
    if it is a second, independently-published, valid GoalAnnouncement
    processed by the SAME receiving NavigationTargetState instance AFTER
    that instance's switched_to_goal has already latched True from a
    first, real adoption. NavigationTargetState.receive_announcement()
    (navigation_target_state.py) -- the sole idempotency authority -- takes
    only (goal_x_m, goal_y_m, valid); it does not inspect goal_id or
    sequence at all, so the contract does not require them to match the
    original either. This test nonetheless publishes the duplicate with
    the SAME goal_id ("shared_exit") and SAME coordinates as the original
    -- a genuine retransmission of the same announcement, the realistic
    case this mechanism exists to guard against -- rather than an
    unrelated announcement that happens to also be rejected. The one field
    that necessarily differs is `sequence`: hil_offline_stage3_harness.py's
    own request_duplicate_publication() always stamps the fixed sentinel
    999999 on the message it constructs (read directly from that method's
    source, not a choice made by this test), distinguishing it on the wire
    as a distinct message from the scout's own incrementing sequence --
    consistent with the contract, since sequence is not part of it either.
    The proof that the second copy was correctly rejected is therefore
    never "the coordinates differ" -- it is the real
    receive_announcement() call sequence (True then False) and the real
    adopted target being unchanged, both captured directly below.

    Only two minimal synthetic Twist stimuli are used, since no real
    hil_cmd_vel_guard.py runs in this hardware-free test -- a zero Twist
    kept flowing on the guarded-cmd-vel topic (so the runner's zero/
    bounded phase checks have something real to observe) and one Twist
    on the requested-cmd-vel topic (recorder evidence-coverage only).
    Neither stimulus is used to claim cooperative_avoider, the production
    guard graph, or physical command delivery was tested. Adoption,
    duplicate handling, state-gate decisions, recording, and verification
    are all real. Never cooperative_avoider, never hil_cmd_vel_guard.py,
    never a physical bridge, never Webots, never the final Stage 3
    multi-process graph.

    Invoke as:
      ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_offline_stage3_recorder_verifier_integration.py::FullBehaviouralIntegrationTest -v
    """

    def setUp(self):
        self.assertEqual(
            os.environ.get("ROS_LOCALHOST_ONLY"), "1",
            "This live ROS test must be invoked with ROS_LOCALHOST_ONLY=1",
        )
        domain_raw = os.environ.get("ROS_DOMAIN_ID")
        self.assertIsNotNone(domain_raw, "ROS_DOMAIN_ID must be set explicitly for this live test")
        domain = int(domain_raw)
        self.assertNotIn(domain, FORBIDDEN_ROS_DOMAIN_IDS)
        self.assertNotEqual(domain, EXPECTED_STAGE3_ROS_DOMAIN_ID)
        self.effective_domain = domain

        self._tmpdir = tempfile.TemporaryDirectory()
        self.csv_path = str(Path(self._tmpdir.name) / "evidence.csv")
        self.summary_path = str(Path(self._tmpdir.name) / "summary.json")

        rclpy.init(args=[])

        _, HilOfflineStage3Harness, harness_parse_args = _build_harness_node()
        harness_args = harness_parse_args([
            "--own-state-topic", BEHAVIOR_OWN_STATE_TOPIC,
            "--bridge-status-topic", BEHAVIOR_BRIDGE_STATUS_TOPIC,
            "--arm-topic", BEHAVIOR_ARM_TOPIC,
            "--phase-event-topic", BEHAVIOR_PHASE_EVENT_TOPIC,
            "--goal-announcement-topic", BEHAVIOR_GOAL_ANNOUNCEMENT_TOPIC,
            "--virtual-peer-source-topic", BEHAVIOR_VP_SOURCE_TOPIC,
            "--virtual-peer-guard-input-topic", BEHAVIOR_VP_GATE_INPUT_TOPIC,
            "--gate-decision-topic", BEHAVIOR_GATE_DECISION_TOPIC,
            "--nav-intent-topic", BEHAVIOR_NAV_INTENT_TOPIC,
            "--guarded-cmd-vel-topic", BEHAVIOR_GUARDED_CMD_VEL_TOPIC,
            "--own-robot-id", "1",
            "--max-runtime-s", "300",
        ])
        self.harness = HilOfflineStage3Harness(harness_args)

        _, HilVirtualPeer, vp_parse_args = hil_virtual_peer._build_node()
        vp_args = vp_parse_args([
            "--robot-id", "2",
            "--state-topic", BEHAVIOR_VP_SOURCE_TOPIC,
            "--announcement-topic", BEHAVIOR_GOAL_ANNOUNCEMENT_TOPIC,
            "--start-x-m", str(VIRTUAL_PEER_TARGET[0]),
            "--start-y-m", str(VIRTUAL_PEER_TARGET[1]),
            "--start-yaw-rad", "0.0",
            "--target-x-m", str(VIRTUAL_PEER_TARGET[0]),
            "--target-y-m", str(VIRTUAL_PEER_TARGET[1]),
            "--cruise-linear-mps", "0.05",
            "--arrival-radius-m", "0.5",
            "--max-angular-rps", "0.2",
            "--rate-hz", "20",
        ])
        self.virtual_peer = HilVirtualPeer(vp_args)

        EvidenceNavigator = build_evidence_navigator_class()
        nav_args = goal_navigator.parse_args([
            "--robot-id", "1",
            "--state-topic", BEHAVIOR_OWN_STATE_TOPIC,
            "--nav-intent-topic", BEHAVIOR_NAV_INTENT_TOPIC,
            "--mode", "search",
            "--waypoints", "0.0:0.0,1.0:1.0",
            "--goal-announcement-topic", BEHAVIOR_GOAL_ANNOUNCEMENT_TOPIC,
            "--nominal-speed-mps", "0.05",
            "--exit-center-x", "5.0",
            "--exit-center-y", "5.0",
            "--exit-radius", "0.1",
            "--parking-x", "9.0",
            "--parking-y", "9.0",
            "--parking-radius", "0.1",
            "--goal-hold-time-s", "1.0",
        ])
        self.receiver = EvidenceNavigator(nav_args)

        _, HilOfflineStage3EvidenceRecorder, rec_parse_args = _build_node()
        rec_args = rec_parse_args([
            "--own-state-topic", BEHAVIOR_OWN_STATE_TOPIC,
            "--virtual-peer-source-topic", BEHAVIOR_VP_SOURCE_TOPIC,
            "--virtual-peer-guard-input-topic", BEHAVIOR_VP_GATE_INPUT_TOPIC,
            "--goal-announcement-topic", BEHAVIOR_GOAL_ANNOUNCEMENT_TOPIC,
            "--nav-intent-topic", BEHAVIOR_NAV_INTENT_TOPIC,
            "--requested-cmd-vel-topic", BEHAVIOR_REQUESTED_CMD_VEL_TOPIC,
            "--guarded-cmd-vel-topic", BEHAVIOR_GUARDED_CMD_VEL_TOPIC,
            "--arm-topic", BEHAVIOR_ARM_TOPIC,
            "--bridge-status-topic", BEHAVIOR_BRIDGE_STATUS_TOPIC,
            "--phase-event-topic", BEHAVIOR_PHASE_EVENT_TOPIC,
            "--gate-decision-topic", BEHAVIOR_GATE_DECISION_TOPIC,
            "--output-csv", self.csv_path,
            "--output-summary-json", self.summary_path,
            "--flush-interval-s", "0.05",
        ])
        self.recorder = HilOfflineStage3EvidenceRecorder(rec_args)

        # Minimal synthetic stimulus: only the two cmd_vel-shaped topics a
        # real (not-run-here) guard would otherwise produce.
        self.stim = rclpy.create_node("stage3_behavior_test_stimulus")
        self.requested_pub = self.stim.create_publisher(Twist, BEHAVIOR_REQUESTED_CMD_VEL_TOPIC, 10)
        self.guarded_pub = self.stim.create_publisher(Twist, BEHAVIOR_GUARDED_CMD_VEL_TOPIC, 10)

        self.executor = SingleThreadedExecutor()
        for node in (self.harness, self.virtual_peer, self.receiver, self.recorder, self.stim):
            self.executor.add_node(node)

    def tearDown(self):
        try:
            self.recorder.write_summary()
        except Exception:
            pass
        try:
            self.recorder.writer.close()
        except Exception:
            pass
        for node in (self.stim, self.recorder, self.receiver, self.virtual_peer, self.harness):
            try:
                self.executor.remove_node(node)
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
        self._tmpdir.cleanup()

    def test_real_production_chain_produces_valid_success(self):
        # --- instrumentation: wraps-style spies around real production
        # methods. Each spy calls the real bound method first and returns
        # its real, unaltered result -- it only counts/observes calls,
        # never replaces behaviour. Both are safe to attach here because
        # each is looked up FRESH on every call from within the real
        # calling method (self.target_state.receive_announcement(...) and
        # self.duplicate_controller.authorize_duplicate_publication()),
        # not captured once at construction time the way an rclpy
        # subscription callback reference would be.
        receive_calls = []
        real_receive_announcement = self.receiver.target_state.receive_announcement

        def _spy_receive_announcement(goal_x_m, goal_y_m, valid):
            result = real_receive_announcement(goal_x_m, goal_y_m, valid)
            receive_calls.append(((goal_x_m, goal_y_m, valid), result))
            return result

        self.receiver.target_state.receive_announcement = _spy_receive_announcement

        authorize_calls = []
        real_authorize = self.harness.duplicate_controller.authorize_duplicate_publication

        def _spy_authorize():
            real_authorize()
            authorize_calls.append(True)

        self.harness.duplicate_controller.authorize_duplicate_publication = _spy_authorize

        synthetic_phase_stimuli_used = set()
        requested_published = {"done": False}

        def spin_once():
            self.executor.spin_once(timeout_sec=0.02)
            self.guarded_pub.publish(Twist())
            synthetic_phase_stimuli_used.add(BEHAVIOR_GUARDED_CMD_VEL_TOPIC)
            if not requested_published["done"]:
                self.requested_pub.publish(Twist())
                synthetic_phase_stimuli_used.add(BEHAVIOR_REQUESTED_CMD_VEL_TOPIC)
                requested_published["done"] = True

        runner = Stage3AutomaticRunner(
            self.harness,
            per_phase_timeout_s=10.0, overall_timeout_s=40.0,
            test_only_linear_bound_mps=0.3, test_only_angular_bound_rps=3.0,
            peer_timeout_s=1.1, duplicate_source_robot_id=2,
            # Same goal_id and coordinates as the original -- a genuine
            # retransmission per the documented duplicate-identity
            # contract above, not an unrelated announcement.
            duplicate_goal_x_m=VIRTUAL_PEER_TARGET[0], duplicate_goal_y_m=VIRTUAL_PEER_TARGET[1],
            duplicate_goal_id=ORIGINAL_GOAL_ID,
        )
        runner.run(spin_once)

        # Let the (rejected) duplicate reach the real navigator and the
        # recorder finish writing its rows.
        _spin_for(self.executor, 0.3)

        # --- A: real harness + real runner drove all 11 phases ---
        self.assertTrue(self.harness.phase_machine.is_complete)
        self.assertEqual(self.harness.phase_machine.phase, Stage3Phase.COMPLETE)

        # --- D/E/F: real adoption, real duplicate controller, real
        # idempotent duplicate rejection -- observed directly on the real
        # production objects' own state and call history, never inferred
        # from a CSV row's field name. ---
        self.assertEqual(len(receive_calls), 2, receive_calls)
        (first_args, first_result), (second_args, second_result) = receive_calls
        self.assertTrue(first_result, "first receive_announcement() call must adopt")
        self.assertFalse(second_result, "second (duplicate) call must be an idempotent no-op")
        # Both calls carry the SAME (goal_x_m, goal_y_m) -- the duplicate is
        # a genuine retransmission per the documented contract, not a
        # distinguishable different goal. The rejection proof below is
        # therefore the real return-value sequence and the real receiver
        # state, never a coordinate mismatch.
        self.assertEqual(first_args[:2], VIRTUAL_PEER_TARGET)
        self.assertEqual(second_args[:2], VIRTUAL_PEER_TARGET)
        self.assertTrue(self.receiver.target_state.switched_to_goal)
        self.assertEqual(
            self.receiver.target_state.current_target, VIRTUAL_PEER_TARGET,
            "the duplicate must not have altered the real adopted target",
        )

        self.assertEqual(len(authorize_calls), 1, "duplicate authorized exactly once")
        self.assertEqual(self.harness.duplicate_controller.adoption_count, 1)
        self.assertTrue(self.harness.duplicate_controller.duplicate_sent)

        # --- B: real state-gate callback executed for real virtual-peer
        # traffic -- proven by the real recorder's own row count for the
        # real gate-decision topic (these rows can only exist if the
        # harness's own _on_virtual_peer_source() actually ran and
        # published them; this is not a hand-published event). ---
        gate_decision_row_count = self.recorder._row_counts.get(BEHAVIOR_GATE_DECISION_TOPIC, 0)
        self.assertGreater(gate_decision_row_count, 1, "expected closed- and reopened-epoch gate-decision rows")

        # --- E/F continued: a SECOND request_duplicate_publication() call
        # (the run has already reached COMPLETE) must be rejected by the
        # real DuplicateAnnouncementController, and no additional
        # GoalAnnouncement may reach the real navigator or be recorded. ---
        announcement_row_count_before_second_attempt = self.recorder._row_counts.get(
            BEHAVIOR_GOAL_ANNOUNCEMENT_TOPIC, 0
        )
        with self.assertRaises(DuplicateOrderingError):
            self.harness.request_duplicate_publication(
                2, VIRTUAL_PEER_TARGET[0], VIRTUAL_PEER_TARGET[1], ORIGINAL_GOAL_ID,
            )
        _spin_for(self.executor, 0.3)

        self.assertEqual(len(authorize_calls), 1, "the second authorization attempt must not succeed")
        self.assertEqual(len(receive_calls), 2, "no second duplicate reached the real navigator")
        self.assertEqual(
            self.recorder._row_counts.get(BEHAVIOR_GOAL_ANNOUNCEMENT_TOPIC, 0),
            announcement_row_count_before_second_attempt,
            "no additional GoalAnnouncement was recorded from the rejected second request",
        )
        self.assertTrue(self.harness.duplicate_controller.run_complete)

        # --- G/H: real recorder + real verifier ---
        self.recorder.write_summary()
        self.recorder.writer.close()

        with open(self.summary_path, encoding="utf-8") as f:
            summary = json.load(f)
        self.assertGreater(sum(summary["row_count_by_topic"].values()), 0)

        result = run_verifier(
            csv_path=self.csv_path, summary_json_path=self.summary_path,
            residual_process_detected=False,
            test_only_angular_bound_rps=3.0, test_only_linear_bound_mps=0.3,
            virtual_peer_timeout_s=1.0,
            expected_domain_id=self.effective_domain,
        )
        self.assertEqual(result["DATA_VALIDITY"], "VALID", result["data_validity_reasons"])
        self.assertEqual(result["TASK_OUTCOME"], "SUCCESS", result["task_outcome_reasons"])

        self.assertEqual(synthetic_phase_stimuli_used, {BEHAVIOR_GUARDED_CMD_VEL_TOPIC, BEHAVIOR_REQUESTED_CMD_VEL_TOPIC})

        self.real_method_invocation_counts = {
            "receive_announcement": len(receive_calls),
            "duplicate_authorization": len(authorize_calls),
            "gate_decision_rows_recorded": gate_decision_row_count,
        }


if __name__ == "__main__":
    unittest.main()
