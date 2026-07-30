#!/usr/bin/env python3
"""Hardware-free, live rclpy proof of the automatic GoalAnnouncement
chain: virtual peer reaches its scripted arrival condition -> publishes
one GoalAnnouncement -> the adapter-hosted receiving navigator
(HilGoalAnnouncementEvidenceNavigator, exactly what hil_topic_adapter.py
constructs) receives it -> adopts the goal exactly once -> a duplicate
announcement is rejected (idempotent, matching
NavigationTargetState.receive_announcement()'s own contract, reused
here rather than reimplemented).

Isolation: every topic name used below is a literal, private test-only
string (e.g. "/pytest_isolated/..."), passed directly as a CLI argument
to each node -- neither node has any hardcoded topic default, so there
is no remapping mechanism to bypass and no path by which this test can
touch a real robot topic. No Pi, no bridge, no hil_cmd_vel_guard.py, no
/cmd_vel of any kind is constructed or referenced anywhere in this file.

Requires a sourced ROS 2 workspace (rclpy + epuck2_comm_interfaces) --
same requirement as test_hil_command_evidence_recorder_zero_publishers.py
and test_hil_topic_contract.py. Does not require Webots, a bridge, or
any hardware.
"""
from __future__ import annotations

import json
import sys
import time
import unittest
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[3] / "10_cooperative_exit_navigation_20260720" / "tools"),
)

from epuck2_comm_interfaces.msg import EpuckState, GoalAnnouncement, NavigationIntent  # noqa: E402

import hil_virtual_peer  # noqa: E402
from hil_goal_announcement_evidence import (  # noqa: E402
    STAGE4_ADOPTION_EVIDENCE_SCHEMA_VERSION,
    STAGE4_ADOPTION_EVIDENCE_TOPIC,
    build_evidence_navigator_class,
)
import goal_navigator  # noqa: E402

STATE_TOPIC = "/pytest_isolated/epuck1/state"
NAV_INTENT_TOPIC = "/pytest_isolated/epuck1/nav_intent"
VIRTUAL_PEER_STATE_TOPIC = "/pytest_isolated/virtual_peer/state"
GOAL_ANNOUNCEMENT_TOPIC = "/pytest_isolated/goal_announcement"

VIRTUAL_PEER_TARGET = (2.0, 3.0)
DUPLICATE_GOAL = (99.0, -99.0)


def _spin_until(executor, predicate, timeout_s=5.0, step_s=0.05):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=step_s)
        if predicate():
            return True
    return False


class GoalAnnouncementAdoptionTest(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])

        _, HilVirtualPeer, parse_vp_args = hil_virtual_peer._build_node()
        vp_args = parse_vp_args([
            "--robot-id", "2",
            "--state-topic", VIRTUAL_PEER_STATE_TOPIC,
            "--announcement-topic", GOAL_ANNOUNCEMENT_TOPIC,
            "--start-x-m", str(VIRTUAL_PEER_TARGET[0]),
            "--start-y-m", str(VIRTUAL_PEER_TARGET[1]),
            "--start-yaw-rad", "0.0",
            "--target-x-m", str(VIRTUAL_PEER_TARGET[0]),
            "--target-y-m", str(VIRTUAL_PEER_TARGET[1]),
            "--cruise-linear-mps", "0.05",
            "--arrival-radius-m", "0.5",
            "--max-angular-rps", "0.2",
            "--rate-hz", "50",
        ])
        self.virtual_peer = HilVirtualPeer(vp_args)

        EvidenceNavigator = build_evidence_navigator_class()
        nav_args = goal_navigator.parse_args([
            "--robot-id", "1",
            "--state-topic", STATE_TOPIC,
            "--nav-intent-topic", NAV_INTENT_TOPIC,
            "--mode", "search",
            "--waypoints", "0.0:0.0,1.0:1.0",
            "--goal-announcement-topic", GOAL_ANNOUNCEMENT_TOPIC,
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

        self.nav_intent_messages = []
        self.receiver.create_subscription(
            NavigationIntent, NAV_INTENT_TOPIC, self.nav_intent_messages.append, 10,
        )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.virtual_peer)
        self.executor.add_node(self.receiver)

    def tearDown(self):
        self.executor.remove_node(self.receiver)
        self.executor.remove_node(self.virtual_peer)
        self.receiver.destroy_node()
        self.virtual_peer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_virtual_peer_becomes_a_real_rclpy_node_with_expected_publisher_type(self):
        publisher_names_and_types = self.virtual_peer.get_publisher_names_and_types_by_node(
            self.virtual_peer.get_name(), self.virtual_peer.get_namespace()
        )
        topic_names = [name for name, _ in publisher_names_and_types]
        self.assertIn(VIRTUAL_PEER_STATE_TOPIC, topic_names)
        self.assertIn(GOAL_ANNOUNCEMENT_TOPIC, topic_names)

    def test_automatic_announcement_is_produced_and_adopted_exactly_once(self):
        adopted = _spin_until(self.executor, lambda: self.receiver.target_state.switched_to_goal)
        self.assertTrue(adopted, "GoalAnnouncement was never adopted within the timeout")
        self.assertTrue(self.virtual_peer.announced)
        self.assertEqual(self.receiver.target_state.current_target, VIRTUAL_PEER_TARGET)

    def test_duplicate_announcement_on_the_same_topic_is_rejected(self):
        _spin_until(self.executor, lambda: self.receiver.target_state.switched_to_goal)
        adopted_target_before = self.receiver.target_state.current_target

        duplicate_pub = self.virtual_peer.create_publisher(GoalAnnouncement, GOAL_ANNOUNCEMENT_TOPIC, 10)
        duplicate_msg = GoalAnnouncement()
        duplicate_msg.protocol_version = GoalAnnouncement.PROTOCOL_VERSION
        duplicate_msg.source_robot_id = 2
        duplicate_msg.sequence = 999
        duplicate_msg.production_stamp = self.virtual_peer.get_clock().now().to_msg()
        duplicate_msg.goal_id = "shared_exit"
        duplicate_msg.goal_x_m = DUPLICATE_GOAL[0]
        duplicate_msg.goal_y_m = DUPLICATE_GOAL[1]
        duplicate_msg.valid = True
        duplicate_pub.publish(duplicate_msg)

        for _ in range(20):
            self.executor.spin_once(timeout_sec=0.05)

        self.assertEqual(self.receiver.target_state.current_target, adopted_target_before)
        self.assertNotEqual(self.receiver.target_state.current_target, DUPLICATE_GOAL)
        self.virtual_peer.destroy_publisher(duplicate_pub)

    def test_navigation_intent_stamps_are_independently_generated_not_copied_from_announcement(self):
        """No forwarding/re-stamping contract exists between GoalAnnouncement
        and NavigationIntent -- they are two independent message streams.
        This asserts that fact directly rather than assuming it.

        GoalNavigator only publishes NavigationIntent once its own
        own_position has been set via its EpuckState subscription (see
        _tick()'s `if self.own_position is not None` gate) -- that is
        real, existing, unmodified behaviour, not something this test
        should work around silently. One EpuckState sample is published
        here for exactly that reason, unrelated to the GoalAnnouncement
        chain itself."""
        own_state_pub = self.receiver.create_publisher(EpuckState, STATE_TOPIC, 10)
        own_state_msg = EpuckState()
        own_state_msg.version = EpuckState.PROTOCOL_VERSION
        own_state_msg.robot_id = 1
        own_state_msg.sequence = 1
        own_state_msg.stamp = self.receiver.get_clock().now().to_msg()
        own_state_msg.source = EpuckState.SOURCE_HARDWARE
        own_state_msg.x_m = 0.0
        own_state_msg.y_m = 0.0
        own_state_msg.yaw_rad = 0.0
        own_state_msg.validity_flags = (
            EpuckState.FLAG_ODOM_VALID | EpuckState.FLAG_IR_VALID | EpuckState.FLAG_TOF_VALID
        )
        own_state_pub.publish(own_state_msg)

        _spin_until(self.executor, lambda: self.receiver.target_state.switched_to_goal)
        _spin_until(self.executor, lambda: len(self.nav_intent_messages) >= 2, timeout_s=3.0)
        self.assertGreaterEqual(len(self.nav_intent_messages), 2)
        stamps = {(m.production_stamp.sec, m.production_stamp.nanosec) for m in self.nav_intent_messages[:2]}
        self.assertEqual(len(stamps), 2, "expected each NavigationIntent tick to carry its own fresh stamp")
        self.receiver.destroy_publisher(own_state_pub)

    def test_receiver_publishes_no_cmd_vel_topic_of_any_name(self):
        publisher_names_and_types = self.receiver.get_publisher_names_and_types_by_node(
            self.receiver.get_name(), self.receiver.get_namespace()
        )
        for name, _types in publisher_names_and_types:
            self.assertNotIn("cmd_vel", name)

    def test_adoption_evidence_message_is_published_machine_readably_on_acceptance(self):
        """Stage 4 addition: the Stage 4 motion supervisor's online adoption
        gate must never scrape the HIL_GOAL_ANNOUNCEMENT_EVIDENCE log line.
        This asserts the machine-readable replacement actually exists, is
        published exactly once for the one real announcement, and carries
        goal_id/coordinates/accepted/duplicate matching the log line's own
        values."""
        received = []
        self.receiver.create_subscription(
            String, STAGE4_ADOPTION_EVIDENCE_TOPIC, lambda m: received.append(json.loads(m.data)), 10,
        )

        adopted = _spin_until(self.executor, lambda: self.receiver.target_state.switched_to_goal)
        self.assertTrue(adopted)
        _spin_until(self.executor, lambda: len(received) >= 1, timeout_s=3.0)

        self.assertEqual(len(received), 1)
        record = received[0]
        self.assertEqual(record["schema_version"], STAGE4_ADOPTION_EVIDENCE_SCHEMA_VERSION)
        self.assertIn("adapter_receive_monotonic_s", record)
        self.assertIsInstance(record["adapter_receive_monotonic_s"], float)
        self.assertEqual(record["goal_id"], "shared_exit")
        self.assertEqual(record["source_robot_id"], 2)
        self.assertTrue(record["valid"])
        self.assertTrue(record["accepted"])
        self.assertFalse(record["duplicate"])
        self.assertAlmostEqual(record["target_x_m"], VIRTUAL_PEER_TARGET[0], places=6)
        self.assertAlmostEqual(record["target_y_m"], VIRTUAL_PEER_TARGET[1], places=6)

    def test_adoption_evidence_message_marks_duplicate_as_not_accepted(self):
        received = []
        self.receiver.create_subscription(
            String, STAGE4_ADOPTION_EVIDENCE_TOPIC, lambda m: received.append(json.loads(m.data)), 10,
        )
        _spin_until(self.executor, lambda: self.receiver.target_state.switched_to_goal)
        _spin_until(self.executor, lambda: len(received) >= 1, timeout_s=3.0)

        duplicate_pub = self.virtual_peer.create_publisher(GoalAnnouncement, GOAL_ANNOUNCEMENT_TOPIC, 10)
        duplicate_msg = GoalAnnouncement()
        duplicate_msg.protocol_version = GoalAnnouncement.PROTOCOL_VERSION
        duplicate_msg.source_robot_id = 2
        duplicate_msg.sequence = 999
        duplicate_msg.production_stamp = self.virtual_peer.get_clock().now().to_msg()
        duplicate_msg.goal_id = "shared_exit"
        duplicate_msg.goal_x_m = DUPLICATE_GOAL[0]
        duplicate_msg.goal_y_m = DUPLICATE_GOAL[1]
        duplicate_msg.valid = True
        duplicate_pub.publish(duplicate_msg)

        self.assertTrue(_spin_until(self.executor, lambda: len(received) >= 2, timeout_s=3.0))
        self.assertFalse(received[1]["accepted"])
        self.assertTrue(received[1]["duplicate"])
        self.virtual_peer.destroy_publisher(duplicate_pub)


if __name__ == "__main__":
    unittest.main()
