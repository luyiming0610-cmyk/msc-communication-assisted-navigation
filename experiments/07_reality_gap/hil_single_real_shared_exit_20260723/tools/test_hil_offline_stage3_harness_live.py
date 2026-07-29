#!/usr/bin/env python3
"""Live rclpy tests for hil_offline_stage3_harness.py -- require a sourced
ROS 2 workspace, run under an explicit isolated ROS_DOMAIN_ID (92 for
this preparation pass) AND ROS_LOCALHOST_ONLY=1, no hardware, no
production topics.

These tests construct the harness node directly (never via main(), so
no rclpy.init()/shutdown() ownership conflict) and drive it against
private /hil_offline_stage3/... test topics only.

Invoke as:
  ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_offline_stage3_harness_live.py -v
"""
from __future__ import annotations

import os
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor

from epuck2_comm_interfaces.msg import EpuckState, GoalAnnouncement, NavigationIntent
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from hil_offline_stage3_harness import FORBIDDEN_ROS_DOMAIN_IDS, PHASE_ORDER, _build_node

OWN_STATE_TOPIC = "/hil_offline_stage3/epuck1/state"
BRIDGE_STATUS_TOPIC = "/hil_offline_stage3/bridge_status_test_only"
ARM_TOPIC = "/hil_offline_stage3/guard_arm_test_only"
PHASE_EVENT_TOPIC = "/hil_offline_stage3/phase_event_test_only"
GOAL_ANNOUNCEMENT_TOPIC = "/hil_offline_stage3/goal_announcement"
VP_SOURCE_TOPIC = "/hil_offline_stage3/virtual_peer/source_state"
VP_GATE_INPUT_TOPIC = "/hil_offline_stage3/virtual_peer/guard_input_state"
GATE_DECISION_TOPIC = "/hil_offline_stage3/gate_decision_test_only"
NAV_INTENT_TOPIC = "/hil_offline_stage3/epuck1/nav_intent"
GUARDED_CMD_VEL_TOPIC = "/hil_offline_stage3/cmd_vel_guarded_test_only"


def _spin_for(executor, seconds, step=0.02):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=step)


class HarnessLiveTestBase(unittest.TestCase):
    def setUp(self):
        self.assertEqual(
            os.environ.get("ROS_LOCALHOST_ONLY"), "1",
            "This live ROS test must be invoked with ROS_LOCALHOST_ONLY=1",
        )
        domain_raw = os.environ.get("ROS_DOMAIN_ID")
        self.assertIsNotNone(domain_raw, "ROS_DOMAIN_ID must be set explicitly for this live test")
        domain = int(domain_raw)
        self.assertNotIn(domain, FORBIDDEN_ROS_DOMAIN_IDS)
        self.assertNotEqual(domain, 91, "ROS_DOMAIN_ID=91 is reserved for the real Stage 3 run, not preparation")

        rclpy.init(args=[])
        _, HilOfflineStage3Harness, parse_args = _build_node()
        args = parse_args([
            "--own-state-topic", OWN_STATE_TOPIC,
            "--bridge-status-topic", BRIDGE_STATUS_TOPIC,
            "--arm-topic", ARM_TOPIC,
            "--phase-event-topic", PHASE_EVENT_TOPIC,
            "--goal-announcement-topic", GOAL_ANNOUNCEMENT_TOPIC,
            "--virtual-peer-source-topic", VP_SOURCE_TOPIC,
            "--virtual-peer-guard-input-topic", VP_GATE_INPUT_TOPIC,
            "--gate-decision-topic", GATE_DECISION_TOPIC,
            "--nav-intent-topic", NAV_INTENT_TOPIC,
            "--guarded-cmd-vel-topic", GUARDED_CMD_VEL_TOPIC,
            "--max-runtime-s", "300",
        ])
        self.harness = HilOfflineStage3Harness(args)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.harness)

    def tearDown(self):
        self.executor.remove_node(self.harness)
        self.harness.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class PeriodicOwnStatePublicationTest(HarnessLiveTestBase):
    def test_publishes_multiple_messages_with_flags_7_and_increasing_sequence(self):
        received = []
        probe = rclpy.create_node("probe_own_state")
        probe.create_subscription(EpuckState, OWN_STATE_TOPIC, received.append, 20)
        self.executor.add_node(probe)
        _spin_for(self.executor, 0.6)
        self.executor.remove_node(probe)
        probe.destroy_node()

        self.assertGreaterEqual(len(received), 3, "expected several periodic own-state publishes within 0.6s")
        for msg in received:
            self.assertEqual(int(msg.validity_flags), 7)
            self.assertEqual(int(msg.version), EpuckState.PROTOCOL_VERSION)
        sequences = [int(m.sequence) for m in received]
        self.assertEqual(sequences, sorted(sequences))
        self.assertTrue(all(b > a for a, b in zip(sequences, sequences[1:])))
        stamps_ns = [m.stamp.sec * 1_000_000_000 + m.stamp.nanosec for m in received]
        self.assertEqual(stamps_ns, sorted(stamps_ns))


class BridgeStatusPublicationTest(HarnessLiveTestBase):
    def test_publishes_valid_json_with_expected_keys(self):
        import json
        received = []
        probe = rclpy.create_node("probe_bridge_status")
        probe.create_subscription(String, BRIDGE_STATUS_TOPIC, received.append, 10)
        self.executor.add_node(probe)
        _spin_for(self.executor, 1.2)
        self.executor.remove_node(probe)
        probe.destroy_node()

        self.assertGreaterEqual(len(received), 1)
        payload = json.loads(received[0].data)
        self.assertIn("connected", payload)
        self.assertIn("rx_count", payload)


class ArmPublicationTest(HarnessLiveTestBase):
    def test_set_arm_true_then_false_publishes_expected_values(self):
        received = []
        probe = rclpy.create_node("probe_arm")
        probe.create_subscription(Bool, ARM_TOPIC, received.append, 10)
        self.executor.add_node(probe)
        self.harness.set_arm(True)
        _spin_for(self.executor, 0.2)
        self.harness.set_arm(False)
        _spin_for(self.executor, 0.2)
        self.executor.remove_node(probe)
        probe.destroy_node()

        self.assertGreaterEqual(len(received), 2)
        self.assertTrue(received[0].data)
        self.assertFalse(received[-1].data)


class VirtualPeerGateLiveTest(HarnessLiveTestBase):
    def _publish_vp_state(self, publisher, sequence):
        msg = EpuckState()
        msg.version = EpuckState.PROTOCOL_VERSION
        msg.robot_id = 2
        msg.sequence = sequence
        msg.stamp = self.harness.get_clock().now().to_msg()
        msg.source = EpuckState.SOURCE_VIRTUAL
        msg.x_m, msg.y_m, msg.yaw_rad = 1.0, 2.0, 0.5
        msg.validity_flags = EpuckState.FLAG_ODOM_VALID
        publisher.publish(msg)
        return msg

    def test_gate_forwards_exactly_while_open(self):
        received = []
        probe = rclpy.create_node("probe_gate_open")
        probe.create_subscription(EpuckState, VP_GATE_INPUT_TOPIC, received.append, 20)
        vp_pub = probe.create_publisher(EpuckState, VP_SOURCE_TOPIC, 20)
        self.executor.add_node(probe)

        self._publish_vp_state(vp_pub, sequence=1)
        _spin_for(self.executor, 0.3)

        self.executor.remove_node(probe)
        probe.destroy_node()
        self.assertGreaterEqual(len(received), 1)
        self.assertEqual(int(received[0].sequence), 1)
        self.assertEqual(int(received[0].robot_id), 2)
        self.assertEqual(float(received[0].x_m), 1.0)

    def test_gate_closed_forwards_nothing(self):
        received = []
        probe = rclpy.create_node("probe_gate_closed")
        probe.create_subscription(EpuckState, VP_GATE_INPUT_TOPIC, received.append, 20)
        vp_pub = probe.create_publisher(EpuckState, VP_SOURCE_TOPIC, 20)
        self.executor.add_node(probe)

        self.harness.close_gate()
        self._publish_vp_state(vp_pub, sequence=1)
        self._publish_vp_state(vp_pub, sequence=2)
        _spin_for(self.executor, 0.3)

        self.executor.remove_node(probe)
        probe.destroy_node()
        self.assertEqual(len(received), 0)

    def test_gate_reopen_forwards_newest_without_backlog_replay(self):
        received = []
        probe = rclpy.create_node("probe_gate_reopen")
        probe.create_subscription(EpuckState, VP_GATE_INPUT_TOPIC, received.append, 20)
        vp_pub = probe.create_publisher(EpuckState, VP_SOURCE_TOPIC, 20)
        self.executor.add_node(probe)

        self.harness.close_gate()
        self._publish_vp_state(vp_pub, sequence=10)
        self._publish_vp_state(vp_pub, sequence=11)
        _spin_for(self.executor, 0.3)
        self.assertEqual(len(received), 0, "nothing should have been queued/forwarded while closed")

        self.harness.open_gate()
        self._publish_vp_state(vp_pub, sequence=99)
        _spin_for(self.executor, 0.3)

        self.executor.remove_node(probe)
        probe.destroy_node()
        self.assertEqual(len(received), 1, "only the newest post-reopen message should be forwarded")
        self.assertEqual(int(received[0].sequence), 99)


class GateDecisionEventLiveTest(HarnessLiveTestBase):
    def _publish_vp_state(self, publisher, sequence):
        msg = EpuckState()
        msg.version = EpuckState.PROTOCOL_VERSION
        msg.robot_id = 2
        msg.sequence = sequence
        msg.stamp = self.harness.get_clock().now().to_msg()
        msg.source = EpuckState.SOURCE_VIRTUAL
        msg.x_m, msg.y_m, msg.yaw_rad = 1.0, 2.0, 0.5
        msg.validity_flags = EpuckState.FLAG_ODOM_VALID
        publisher.publish(msg)

    def test_forwarded_and_rejected_decisions_emitted_at_the_gate_itself(self):
        import json

        received = []
        probe = rclpy.create_node("probe_gate_decision")
        probe.create_subscription(String, GATE_DECISION_TOPIC, received.append, 20)
        vp_pub = probe.create_publisher(EpuckState, VP_SOURCE_TOPIC, 20)
        self.executor.add_node(probe)

        self._publish_vp_state(vp_pub, sequence=1)
        _spin_for(self.executor, 0.3)
        self.harness.close_gate()
        self._publish_vp_state(vp_pub, sequence=2)
        _spin_for(self.executor, 0.3)

        self.executor.remove_node(probe)
        probe.destroy_node()

        self.assertGreaterEqual(len(received), 2)
        events = [json.loads(m.data) for m in received]
        forwarded = [e for e in events if e["source_sequence"] == 1]
        rejected = [e for e in events if e["source_sequence"] == 2]
        self.assertEqual(len(forwarded), 1)
        self.assertEqual(forwarded[0]["decision"], "FORWARDED")
        self.assertEqual(forwarded[0]["forwarded_destination_topic"], VP_GATE_INPUT_TOPIC)
        self.assertEqual(len(rejected), 1)
        self.assertEqual(rejected[0]["decision"], "REJECTED_GATE_CLOSED")
        self.assertIsNone(rejected[0]["forwarded_destination_topic"])
        for e in events:
            self.assertEqual(e["event_type"], "GATE_DECISION")

    def test_first_source_after_reopen_is_marked_and_epoch_increments(self):
        import json

        received = []
        probe = rclpy.create_node("probe_gate_decision_reopen")
        probe.create_subscription(String, GATE_DECISION_TOPIC, received.append, 20)
        vp_pub = probe.create_publisher(EpuckState, VP_SOURCE_TOPIC, 20)
        self.executor.add_node(probe)

        self.harness.close_gate()
        self._publish_vp_state(vp_pub, sequence=10)
        _spin_for(self.executor, 0.2)
        self.harness.open_gate()
        self._publish_vp_state(vp_pub, sequence=11)
        self._publish_vp_state(vp_pub, sequence=12)
        _spin_for(self.executor, 0.3)

        self.executor.remove_node(probe)
        probe.destroy_node()

        events = [json.loads(m.data) for m in received]
        first_after_reopen = [e for e in events if e.get("first_source_after_reopen")]
        self.assertEqual(len(first_after_reopen), 1)
        self.assertEqual(first_after_reopen[0]["source_sequence"], 11)
        self.assertEqual(first_after_reopen[0]["decision"], "FORWARDED")
        closed_epoch = next(e["gate_epoch"] for e in events if e["source_sequence"] == 10)
        reopened_epoch = first_after_reopen[0]["gate_epoch"]
        self.assertGreater(reopened_epoch, closed_epoch)


class AdoptionGateTest(HarnessLiveTestBase):
    def _publish_nav_intent(self, publisher, phase):
        msg = NavigationIntent()
        msg.protocol_version = NavigationIntent.PROTOCOL_VERSION
        msg.source_robot_id = 1
        msg.sequence = 1
        msg.production_stamp = self.harness.get_clock().now().to_msg()
        msg.desired_heading_rad = 0.0
        msg.desired_linear_speed_mps = 0.0
        msg.navigation_phase = phase
        msg.valid = True
        publisher.publish(msg)

    def test_adoption_confirmed_false_until_go_to_exit_phase_observed(self):
        probe = rclpy.create_node("probe_nav_intent")
        nav_pub = probe.create_publisher(NavigationIntent, NAV_INTENT_TOPIC, 10)
        self.executor.add_node(probe)

        self.assertFalse(self.harness.adoption_confirmed())
        self._publish_nav_intent(nav_pub, "SEARCH")
        _spin_for(self.executor, 0.2)
        self.assertFalse(self.harness.adoption_confirmed())

        self._publish_nav_intent(nav_pub, "GO_TO_EXIT")
        _spin_for(self.executor, 0.2)
        self.executor.remove_node(probe)
        probe.destroy_node()
        self.assertTrue(self.harness.adoption_confirmed())


class DuplicateAnnouncementOrderingLiveTest(HarnessLiveTestBase):
    def _publish_nav_intent(self, publisher, phase):
        msg = NavigationIntent()
        msg.protocol_version = NavigationIntent.PROTOCOL_VERSION
        msg.source_robot_id = 1
        msg.sequence = 1
        msg.production_stamp = self.harness.get_clock().now().to_msg()
        msg.desired_heading_rad = 0.0
        msg.desired_linear_speed_mps = 0.0
        msg.navigation_phase = phase
        msg.valid = True
        publisher.publish(msg)

    def test_duplicate_before_adoption_raises_and_publishes_nothing(self):
        from hil_offline_stage3_harness import DuplicateOrderingError

        received = []
        probe = rclpy.create_node("probe_duplicate_before_adoption")
        probe.create_subscription(GoalAnnouncement, GOAL_ANNOUNCEMENT_TOPIC, received.append, 10)
        self.executor.add_node(probe)

        with self.assertRaises(DuplicateOrderingError):
            self.harness.request_duplicate_publication(2, 5.0, 5.0, "shared_exit")
        _spin_for(self.executor, 0.2)

        self.executor.remove_node(probe)
        probe.destroy_node()
        self.assertEqual(len(received), 0)

    def test_duplicate_after_adoption_sends_exactly_once_then_rejects_second(self):
        from hil_offline_stage3_harness import DuplicateOrderingError

        received = []
        probe = rclpy.create_node("probe_duplicate_after_adoption")
        probe.create_subscription(GoalAnnouncement, GOAL_ANNOUNCEMENT_TOPIC, received.append, 10)
        nav_pub = probe.create_publisher(NavigationIntent, NAV_INTENT_TOPIC, 10)
        self.executor.add_node(probe)

        self._publish_nav_intent(nav_pub, "GO_TO_EXIT")
        _spin_for(self.executor, 0.2)
        self.assertTrue(self.harness.adoption_confirmed())

        self.harness.request_duplicate_publication(2, 5.0, 5.0, "shared_exit")  # must not raise
        with self.assertRaises(DuplicateOrderingError):
            self.harness.request_duplicate_publication(2, 5.0, 5.0, "shared_exit")
        _spin_for(self.executor, 0.3)

        self.executor.remove_node(probe)
        probe.destroy_node()
        self.assertEqual(len(received), 1)

    def test_adoption_count_greater_than_one_raises(self):
        from hil_offline_stage3_harness import AdoptionCountExceededError

        probe = rclpy.create_node("probe_double_adoption")
        nav_pub = probe.create_publisher(NavigationIntent, NAV_INTENT_TOPIC, 10)
        self.executor.add_node(probe)

        self._publish_nav_intent(nav_pub, "GO_TO_EXIT")
        _spin_for(self.executor, 0.2)
        self.assertEqual(self.harness.duplicate_controller.adoption_count, 1)

        # Force a second rising-edge directly (bypassing the nav_intent
        # subscription, since the real message stream cannot toggle back
        # to SEARCH by design) to prove the controller itself aborts --
        # this is testing the controller's own defence-in-depth, not
        # claiming the real navigation stream can produce this input.
        with self.assertRaises(AdoptionCountExceededError):
            self.harness.duplicate_controller.record_adoption_event()

        self.executor.remove_node(probe)
        probe.destroy_node()

    def test_duplicate_after_completion_raises_even_with_prior_adoption(self):
        from hil_offline_stage3_harness import DuplicateOrderingError

        probe = rclpy.create_node("probe_duplicate_after_completion")
        nav_pub = probe.create_publisher(NavigationIntent, NAV_INTENT_TOPIC, 10)
        self.executor.add_node(probe)

        self._publish_nav_intent(nav_pub, "GO_TO_EXIT")
        _spin_for(self.executor, 0.2)
        for expected in PHASE_ORDER[:-1]:
            self.harness.advance_phase(expected)
        self.assertTrue(self.harness.phase_machine.is_complete)

        with self.assertRaises(DuplicateOrderingError):
            self.harness.request_duplicate_publication(2, 5.0, 5.0, "shared_exit")

        self.executor.remove_node(probe)
        probe.destroy_node()


class Stage3AutomaticRunnerLiveTest(HarnessLiveTestBase):
    """Exercises Stage3AutomaticRunner against the real, live harness
    node over real (isolated, non-production) ROS topics -- a synthetic
    probe node stands in for the future real guard/navigation stack,
    publishing NavigationIntent(GO_TO_EXIT) for adoption and a plain
    Twist on the guarded-cmd-vel topic for the zero/bounded checks, and
    the real virtual-peer source topic for the gate to forward."""

    def _publish_nav_intent(self, publisher, phase):
        msg = NavigationIntent()
        msg.protocol_version = NavigationIntent.PROTOCOL_VERSION
        msg.source_robot_id = 1
        msg.sequence = 1
        msg.production_stamp = self.harness.get_clock().now().to_msg()
        msg.desired_heading_rad = 0.0
        msg.desired_linear_speed_mps = 0.0
        msg.navigation_phase = phase
        msg.valid = True
        publisher.publish(msg)

    def _publish_vp_state(self, publisher, sequence):
        msg = EpuckState()
        msg.version = EpuckState.PROTOCOL_VERSION
        msg.robot_id = 2
        msg.sequence = sequence
        msg.stamp = self.harness.get_clock().now().to_msg()
        msg.source = EpuckState.SOURCE_VIRTUAL
        msg.x_m, msg.y_m, msg.yaw_rad = 1.0, 2.0, 0.5
        msg.validity_flags = EpuckState.FLAG_ODOM_VALID
        publisher.publish(msg)

    def test_runner_drives_the_real_harness_through_all_11_phases(self):
        from hil_offline_stage3_harness import Stage3AutomaticRunner, Stage3Phase

        probe = rclpy.create_node("probe_runner_live")
        nav_pub = probe.create_publisher(NavigationIntent, NAV_INTENT_TOPIC, 10)
        guarded_pub = probe.create_publisher(Twist, GUARDED_CMD_VEL_TOPIC, 10)
        vp_pub = probe.create_publisher(EpuckState, VP_SOURCE_TOPIC, 20)
        self.executor.add_node(probe)

        state = {"adopted": False, "vp_seq": 0}

        def spin_once():
            self.executor.spin_once(timeout_sec=0.02)
            if not state["adopted"]:
                self._publish_nav_intent(nav_pub, "GO_TO_EXIT")
                state["adopted"] = True
            zero = Twist()
            guarded_pub.publish(zero)
            state["vp_seq"] += 1
            self._publish_vp_state(vp_pub, sequence=state["vp_seq"])

        runner = Stage3AutomaticRunner(
            self.harness,
            per_phase_timeout_s=5.0, overall_timeout_s=20.0,
            test_only_linear_bound_mps=0.3, test_only_angular_bound_rps=3.0,
            peer_timeout_s=0.2, duplicate_source_robot_id=2,
            duplicate_goal_x_m=0.0, duplicate_goal_y_m=0.0, duplicate_goal_id="dup_live",
        )
        runner.run(spin_once)

        self.executor.remove_node(probe)
        probe.destroy_node()

        self.assertTrue(self.harness.phase_machine.is_complete)
        self.assertEqual(self.harness.phase_machine.phase, Stage3Phase.COMPLETE)
        self.assertEqual(self.harness.duplicate_controller.adoption_count, 1)
        self.assertTrue(self.harness.duplicate_controller.duplicate_sent)


if __name__ == "__main__":
    unittest.main()
