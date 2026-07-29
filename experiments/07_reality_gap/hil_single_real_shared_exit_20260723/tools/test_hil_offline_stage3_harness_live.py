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

import math
import os
import signal
import subprocess
import time
import unittest
from dataclasses import dataclass, replace

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.executors import SingleThreadedExecutor

from epuck2_comm_interfaces.msg import EpuckState, GoalAnnouncement, NavigationIntent
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from hil_offline_stage3_harness import (
    FORBIDDEN_ROS_DOMAIN_IDS,
    PHASE_ORDER,
    SYNTHETIC_CLEAR_SENSOR_FIXTURE_FIELDS,
    _build_node,
)

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


def resolve_cooperative_avoider_executable() -> str:
    """Resolves the real, installed cooperative_avoider executable via
    the standard, committed, reproducible ROS 2 package-prefix
    mechanism (ament_index_python.packages.get_package_prefix --
    confirmed live to return the exact same path as `ros2 pkg prefix
    epuck2_comm`), never a hardcoded machine-specific path.

    Launching this file directly (never via `ros2 run epuck2_comm
    cooperative_avoider`) avoids the ros2-run wrapper/child ambiguity:
    it is a plain console_scripts entry-point script
    (`#!/usr/bin/python3`, `EASY-INSTALL-ENTRY-SCRIPT`) whose body calls
    `load_entry_point('epuck2-comm==0.1.0', 'console_scripts',
    'cooperative_avoider')()` in-process -- confirmed by direct read of
    the installed file and cross-checked against
    src/epuck2_comm/setup.py:29's own
    `"cooperative_avoider = epuck2_comm.cooperative_avoider:main"` entry
    point declaration. Live-verified (this same authorised test suite)
    that invoking it directly produces exactly one OS process for its
    entire lifetime -- no wrapper, no forked child -- eliminating the
    PID/PGID ambiguity `ros2 run` introduced.
    """
    prefix = get_package_prefix("epuck2_comm")
    exe = os.path.join(prefix, "lib", "epuck2_comm", "cooperative_avoider")
    if not os.path.isfile(exe) or not os.access(exe, os.X_OK):
        raise AssertionError(f"resolved cooperative_avoider executable is missing or not executable: {exe}")
    return exe


@dataclass(frozen=True)
class OwnedProcessIdentity:
    """Captures enough identity evidence about a process THIS test/run
    itself started (via Popen, immediately after launch) to later prove
    -- before ever signalling anything -- that the target is still the
    exact same process, not a PID that has since been reused by an
    unrelated process. Never derived from name-based process discovery
    (pgrep or otherwise)."""

    pid: int
    pgid: int
    start_time: str
    exe_path: str
    cmdline: str


def _read_proc_stat_start_time(pid: int) -> str:
    with open(f"/proc/{pid}/stat", encoding="utf-8") as f:
        content = f.read()
    # comm (field 2) may itself contain spaces or parentheses; split
    # after the LAST ')' to reliably reach the fixed-format fields that
    # follow it. starttime is field 22 overall == index 19 in this
    # post-comm split (field 3 = state = index 0).
    after_comm = content[content.rfind(")") + 2:]
    fields = after_comm.split()
    return fields[19]


def _read_proc_exe(pid: int) -> str:
    return os.readlink(f"/proc/{pid}/exe")


def _read_proc_cmdline(pid: int) -> str:
    with open(f"/proc/{pid}/cmdline", "rb") as f:
        raw = f.read()
    return raw.replace(b"\x00", b" ").decode(errors="replace").strip()


def capture_owned_process_identity(proc: subprocess.Popen) -> OwnedProcessIdentity:
    """Must be called immediately after Popen() returns, while the
    process is guaranteed to still be the one this call just started."""
    pid = proc.pid
    pgid = os.getpgid(pid)
    return OwnedProcessIdentity(
        pid=pid, pgid=pgid,
        start_time=_read_proc_stat_start_time(pid),
        exe_path=_read_proc_exe(pid),
        cmdline=_read_proc_cmdline(pid),
    )


def _owned_identity_still_matches(identity: OwnedProcessIdentity) -> bool:
    """True only if PID `identity.pid` still exists AND its start time,
    resolved executable, and process group all still match exactly what
    was recorded at launch -- the guard against PID reuse and against
    ever mistaking an unrelated process for the owned one."""
    try:
        current_start = _read_proc_stat_start_time(identity.pid)
        current_exe = _read_proc_exe(identity.pid)
        current_pgid = os.getpgid(identity.pid)
    except (FileNotFoundError, ProcessLookupError, OSError):
        return False
    return (
        current_start == identity.start_time
        and current_exe == identity.exe_path
        and current_pgid == identity.pgid
    )


def terminate_owned_process_group_and_verify(
    identity: OwnedProcessIdentity | None, *, sigint_wait_s=15, sigkill_wait_s=5, poll_interval_s=0.2,
) -> dict:
    """Signals ONLY the exact owned process group recorded in
    `identity`, after independently re-verifying PID existence,
    /proc start time, /proc/<pid>/exe, and PGID all still match what was
    captured at launch. Never uses `pgrep`, `pkill`, or any other
    name-based process discovery to choose what to signal -- if the
    identity no longer matches (already exited, or -- vanishingly
    unlikely but handled -- the PID was reused by something else), this
    function does nothing rather than risk signalling an unrelated
    process that merely shares the executable name.

    Sequence when the identity DOES still match: SIGINT to the owned
    process group -> bounded wait -> re-verify identity -> only if it
    is STILL the same owned process (not merely "something is still
    alive with that PID") escalate to SIGKILL on that same verified
    group -> bounded wait -> final verify. Raises AssertionError (never
    silently returns) if the owned process cannot be confirmed stopped
    after the full sequence.

    Returns {"escalated_to_sigkill": bool, "cleanup_classification":
    "normal"|"abnormal"} -- classification is "abnormal" whenever
    SIGKILL escalation was required, per this project's own cleanup
    discipline.
    """
    if identity is None or not _owned_identity_still_matches(identity):
        return {"escalated_to_sigkill": False, "cleanup_classification": "normal"}

    try:
        os.killpg(identity.pgid, signal.SIGINT)
    except ProcessLookupError:
        return {"escalated_to_sigkill": False, "cleanup_classification": "normal"}

    deadline = time.monotonic() + sigint_wait_s
    while time.monotonic() < deadline:
        if not _owned_identity_still_matches(identity):
            return {"escalated_to_sigkill": False, "cleanup_classification": "normal"}
        time.sleep(poll_interval_s)

    escalated = False
    if _owned_identity_still_matches(identity):
        escalated = True
        try:
            os.killpg(identity.pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        kill_deadline = time.monotonic() + sigkill_wait_s
        while time.monotonic() < kill_deadline:
            if not _owned_identity_still_matches(identity):
                break
            time.sleep(poll_interval_s)

    if _owned_identity_still_matches(identity):
        raise AssertionError(
            f"owned process pid={identity.pid} pgid={identity.pgid} exe={identity.exe_path} "
            f"did not exit after SIGINT+SIGKILL escalation on its verified owned process group"
        )

    return {"escalated_to_sigkill": escalated, "cleanup_classification": "abnormal" if escalated else "normal"}


def self_match_safe_residual_audit(pattern: str = "[c]ooperative_avoider") -> str:
    """READ-ONLY audit helper: reports processes matching `pattern` by
    command line, for logging/diagnostic purposes only. The bracket
    trick makes the pattern not match its own `pgrep` invocation. This
    function's output must NEVER be used to select a signal target --
    see terminate_owned_process_group_and_verify(), which signals only
    a verified owned PGID and never anything discovered by name."""
    result = subprocess.run(["pgrep", "-af", pattern], capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else ""


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


class SyntheticClearSensorFixtureLiveTest(HarnessLiveTestBase):
    """Proves the SYNTHETIC_CLEAR_SENSOR_FIXTURE correction over a real
    ROS publish/subscribe round trip -- the real harness publishes the
    corrected EpuckState and a real, independent subscriber receives
    positive infinity in every required field. No production topic is
    touched; only /hil_offline_stage3/... isolated topics are used
    (inherited from HarnessLiveTestBase). No cooperative_avoider,
    hil_cmd_vel_guard.py, adapter, or virtual peer is started here --
    this proves serialization only, not the final graph.

    Invoke as:
      ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_offline_stage3_harness_live.py::SyntheticClearSensorFixtureLiveTest -v
    """

    def test_real_harness_publishes_positive_infinity_in_every_required_field(self):
        received = []
        probe = rclpy.create_node("probe_clear_sensor_fixture")
        probe.create_subscription(EpuckState, OWN_STATE_TOPIC, received.append, 20)
        self.executor.add_node(probe)
        _spin_for(self.executor, 0.5)
        self.executor.remove_node(probe)
        probe.destroy_node()

        self.assertGreaterEqual(len(received), 1, "expected at least one real own-state publish")
        for msg in received:
            self.assertEqual(int(msg.validity_flags), 7)
            for field_name in SYNTHETIC_CLEAR_SENSOR_FIXTURE_FIELDS:
                value = float(getattr(msg, field_name))
                self.assertTrue(
                    math.isinf(value) and value > 0,
                    f"{field_name}={value} did not survive real ROS publish/subscribe as +Inf",
                )


class CooperativeAvoiderComponentCompatibilityTest(unittest.TestCase):
    """Narrowly-scoped, hardware-free component test: the REAL, unmodified
    cooperative_avoider.py (launched by directly invoking its resolved,
    installed console_scripts executable -- never `ros2 run`, never
    modified, never stubbed) receiving the real, corrected harness
    own-state fixture, proving it does not lock permanently into
    LOCAL_FRONT_DANGER. This is explicitly NOT the final Stage 3 graph --
    no guard, no adapter, no virtual peer, no bridge, no Pi, no Webots.
    enable_peer_avoidance/enable_dynamic_heading/enable_dynamic_speed are
    disabled here purely to keep this component test minimal and
    self-contained (they are exercised by other, already-reviewed
    physical-launcher parameter choices, not by this test); only
    enable_local_avoidance and require_local_sensors -- the two settings
    this correction is actually about -- remain enabled. Only
    /hil_offline_stage3_fixture_test/... topics are used.

    Cleanup signals ONLY the exact owned process (captured immediately
    after Popen() via capture_owned_process_identity()) -- never a
    process discovered by name.

    Invoke as:
      ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_offline_stage3_harness_live.py::CooperativeAvoiderComponentCompatibilityTest -v
    """

    NS = "/hil_offline_stage3_fixture_test"
    FIXTURE_OWN_STATE_TOPIC = f"{NS}/epuck1/state"
    FIXTURE_CMD_VEL_TOPIC = f"{NS}/cmd_vel_unguarded_test_only"

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
            "--own-state-topic", self.FIXTURE_OWN_STATE_TOPIC,
            "--bridge-status-topic", f"{self.NS}/bridge_status_test_only",
            "--arm-topic", f"{self.NS}/guard_arm_test_only",
            "--phase-event-topic", f"{self.NS}/phase_event_test_only",
            "--goal-announcement-topic", f"{self.NS}/goal_announcement",
            "--virtual-peer-source-topic", f"{self.NS}/virtual_peer/source_state",
            "--virtual-peer-guard-input-topic", f"{self.NS}/virtual_peer/guard_input_state",
            "--gate-decision-topic", f"{self.NS}/gate_decision_test_only",
            "--nav-intent-topic", f"{self.NS}/epuck1/nav_intent",
            "--guarded-cmd-vel-topic", f"{self.NS}/cmd_vel_guarded_test_only",
            "--max-runtime-s", "300",
        ])
        self.harness = HilOfflineStage3Harness(args)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.harness)
        self.coop_proc = None
        self.coop_identity = None

    def tearDown(self):
        terminate_owned_process_group_and_verify(self.coop_identity)
        self.executor.remove_node(self.harness)
        self.harness.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_real_cooperative_avoider_leaves_local_front_danger_with_corrected_fixture(self):
        exe = resolve_cooperative_avoider_executable()
        cmd = [
            exe, "--ros-args",
            "-r", f"state:={self.FIXTURE_OWN_STATE_TOPIC}",
            "-r", f"cmd_vel:={self.FIXTURE_CMD_VEL_TOPIC}",
            "-r", f"nav_intent:={self.NS}/epuck1/nav_intent",
            "-p", "robot_id:=1",
            "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=false",
            "-p", "enable_dynamic_heading:=false",
            "-p", "enable_dynamic_speed:=false",
            "-p", "enable_local_avoidance:=true",
            "-p", "require_local_sensors:=true",
            "-p", "use_sim_time:=false",
            "-p", "startup_hold_s:=0.3",
            "-p", "max_runtime_s:=30.0",
        ]
        self.coop_proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            start_new_session=True,  # own session/process group -- direct executable, no ros2-run wrapper/child split to worry about
        )
        self.coop_identity = capture_owned_process_identity(self.coop_proc)

        probe = rclpy.create_node("probe_coop_component_test")
        received = []
        probe.create_subscription(Twist, self.FIXTURE_CMD_VEL_TOPIC, received.append, 20)
        self.executor.add_node(probe)

        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and len(received) < 3:
            self.executor.spin_once(timeout_sec=0.1)

        self.executor.remove_node(probe)
        probe.destroy_node()

        self.assertIsNone(self.coop_proc.poll(), "cooperative_avoider exited unexpectedly")
        self.assertGreaterEqual(len(received), 1, "expected at least one real cooperative_avoider cmd_vel command")
        # A permanent LOCAL_FRONT_DANGER lock always commands zero linear
        # with a nonzero, sign-constant angular in-place turn on every
        # tick -- impossible to also see angular.z == 0.0. At least one
        # received command with angular.z == 0.0 is therefore direct
        # proof this run left that state (with enable_dynamic_heading/
        # enable_peer_avoidance disabled here, the only other source of a
        # nonzero angular command is local avoidance itself).
        zero_angular_seen = any(abs(m.angular.z) < 1e-9 for m in received)
        self.assertTrue(
            zero_angular_seen,
            [(round(m.linear.x, 4), round(m.angular.z, 4)) for m in received],
        )


class CooperativeAvoiderCleanupPathTest(unittest.TestCase):
    """Durable, hardware-free proof that
    terminate_owned_process_group_and_verify() correctly manages the
    real, directly-invoked cooperative_avoider executable under every
    abnormal exit path this project's discipline requires, not only the
    golden path already covered by
    CooperativeAvoiderComponentCompatibilityTest, WITHOUT ever using
    name-based process discovery (pgrep/pkill) to choose a signal
    target:
      1. normal completion (the ordinary, non-exceptional return path)
      2. a mid-test assertion failure (cleanup invoked from `finally`)
      3. a readiness-wait timeout (cleanup invoked while the process is
         still inside its own startup_hold_s, before any readiness signal)
      4. an early subprocess exit / crash (malformed --ros-args, so the
         process is already dead before cleanup runs)
      5. a setup failure occurring immediately after Popen(), before any
         further setup work has executed

    Plus dedicated proofs that:
      - a PID/start-time mismatch prevents signalling entirely;
      - an executable-path mismatch prevents signalling entirely;
      - SIGKILL escalation, when it does occur, remains scoped to the
        verified owned process group;
      - a deliberately-launched, unrelated second cooperative_avoider
        process (the "sentinel") is NEVER touched by any of the above,
        even though its command line matches the same executable name --
        this is the direct proof that cleanup is owned-identity-based,
        not name-based.

    Invoke as:
      ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_offline_stage3_harness_live.py::CooperativeAvoiderCleanupPathTest -v
    """

    NS = "/hil_offline_stage3_fixture_test/cleanup_path_test"

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

        self.exe = resolve_cooperative_avoider_executable()
        # An unrelated, independently-owned second cooperative_avoider
        # process, deliberately never passed to any cleanup call under
        # test below -- proves owned-identity cleanup never touches it,
        # even though a name-based `pgrep cooperative_avoider` would
        # match it too.
        self.sentinel_proc = self._launch("sentinel", startup_hold_s=0.3, max_runtime_s=60.0)
        self.sentinel_identity = capture_owned_process_identity(self.sentinel_proc)

    def tearDown(self):
        terminate_owned_process_group_and_verify(self.sentinel_identity)

    def _launch(self, namespace_suffix, extra_ros_args=None, startup_hold_s=0.3, max_runtime_s=30.0):
        ns = f"{self.NS}/{namespace_suffix}"
        cmd = [
            self.exe, "--ros-args",
            "-r", f"state:={ns}/epuck1/state",
            "-r", f"cmd_vel:={ns}/cmd_vel_unguarded_test_only",
            "-r", f"nav_intent:={ns}/epuck1/nav_intent",
            "-p", "robot_id:=1",
            "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=false",
            "-p", "enable_dynamic_heading:=false",
            "-p", "enable_dynamic_speed:=false",
            "-p", "enable_local_avoidance:=true",
            "-p", "require_local_sensors:=true",
            "-p", "use_sim_time:=false",
            "-p", f"startup_hold_s:={startup_hold_s}",
            "-p", f"max_runtime_s:={max_runtime_s}",
        ]
        if extra_ros_args:
            cmd += extra_ros_args
        return subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            start_new_session=True,
        )

    def _assert_sentinel_untouched(self):
        self.assertTrue(
            _owned_identity_still_matches(self.sentinel_identity),
            "the unrelated sentinel cooperative_avoider process must never be signalled by owned-process cleanup",
        )

    def test_cleanup_after_normal_completion(self):
        # "Normal completion" means the ordinary, non-exceptional return
        # path: the test body runs to its end with no exception and
        # cleanup is invoked directly, contrasted with the
        # exception-driven scenarios below. It does NOT mean the
        # subprocess exits on its own: cooperative_avoider.py's own
        # _complete() (reached once elapsed >= max_runtime_s, see
        # cooperative_avoider.py lines 710-716) only commands zero
        # velocity and sets a logical "complete" state -- it never calls
        # rclpy.shutdown() or exits the process. max_runtime_s is a
        # behavioural watchdog, not a process-lifetime one.
        proc = self._launch("under_test", startup_hold_s=0.1, max_runtime_s=1.0)
        identity = capture_owned_process_identity(proc)
        time.sleep(1.5)  # let it actually reach and pass its own max_runtime_s watchdog
        self.assertTrue(_owned_identity_still_matches(identity), "process expected to still be running (behavioural, not process-lifetime, max_runtime_s)")
        result = terminate_owned_process_group_and_verify(identity)
        self.assertFalse(_owned_identity_still_matches(identity))
        self.assertEqual(result["cleanup_classification"], "normal")
        self._assert_sentinel_untouched()

    def test_cleanup_after_mid_test_assertion_failure(self):
        proc = self._launch("under_test", startup_hold_s=0.3, max_runtime_s=30.0)
        identity = capture_owned_process_identity(proc)
        time.sleep(0.5)  # let it actually start running, not just be forked
        cleanup_error = None
        try:
            try:
                raise AssertionError("simulated mid-test assertion failure")
            finally:
                try:
                    terminate_owned_process_group_and_verify(identity)
                except AssertionError as exc:
                    cleanup_error = exc
        except AssertionError as simulated:
            self.assertEqual(str(simulated), "simulated mid-test assertion failure")
        if cleanup_error is not None:
            raise AssertionError(f"owned cleanup failed after a simulated assertion failure: {cleanup_error}")
        self.assertFalse(_owned_identity_still_matches(identity))
        self._assert_sentinel_untouched()

    def test_cleanup_during_readiness_wait_timeout(self):
        # startup_hold_s is deliberately long relative to the immediate
        # cleanup call below, so cleanup runs while the node is still
        # inside its own startup hold -- simulating a caller whose
        # readiness-wait loop gave up and timed out before any readiness
        # signal was observed.
        proc = self._launch("under_test", startup_hold_s=10.0, max_runtime_s=30.0)
        identity = capture_owned_process_identity(proc)
        terminate_owned_process_group_and_verify(identity)
        self.assertFalse(_owned_identity_still_matches(identity))
        self._assert_sentinel_untouched()

    def test_cleanup_after_subprocess_early_exit(self):
        # A malformed --ros-args remap (a dangling -r with no value)
        # makes rclpy's own argument parser raise immediately during
        # rclpy.init(), so the process is already dead (nonzero exit) by
        # the time cleanup is invoked -- proving the "already exited"
        # branch (identity no longer matches -> do nothing) is
        # exercised, not merely the still-running branch.
        proc = self._launch("under_test", extra_ros_args=["-r"])
        identity = capture_owned_process_identity(proc)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.fail("expected the malformed direct invocation to exit immediately")
        self.assertNotEqual(proc.returncode, 0, "malformed invocation was expected to exit with a nonzero status")
        result = terminate_owned_process_group_and_verify(identity)
        self.assertEqual(result, {"escalated_to_sigkill": False, "cleanup_classification": "normal"})
        self._assert_sentinel_untouched()

    def test_cleanup_after_setup_failure_immediately_following_popen(self):
        proc = self._launch("under_test", startup_hold_s=0.3, max_runtime_s=30.0)
        identity = capture_owned_process_identity(proc)
        cleanup_error = None
        try:
            try:
                raise RuntimeError("simulated setup failure occurring immediately after Popen()")
            finally:
                try:
                    terminate_owned_process_group_and_verify(identity)
                except AssertionError as exc:
                    cleanup_error = exc
        except RuntimeError as simulated:
            self.assertEqual(str(simulated), "simulated setup failure occurring immediately after Popen()")
        if cleanup_error is not None:
            raise AssertionError(f"owned cleanup failed after a simulated setup failure: {cleanup_error}")
        self.assertFalse(_owned_identity_still_matches(identity))
        self._assert_sentinel_untouched()

    def test_pid_start_time_mismatch_prevents_signalling(self):
        proc = self._launch("under_test", startup_hold_s=0.3, max_runtime_s=30.0)
        real_identity = capture_owned_process_identity(proc)
        forged_identity = replace(real_identity, start_time="0")
        result = terminate_owned_process_group_and_verify(forged_identity)
        self.assertEqual(result, {"escalated_to_sigkill": False, "cleanup_classification": "normal"})
        self.assertTrue(
            _owned_identity_still_matches(real_identity),
            "the real process must be untouched when the forged identity's start_time does not match",
        )
        self._assert_sentinel_untouched()
        # Real cleanup with the correct identity, so this test does not leak the process it started.
        terminate_owned_process_group_and_verify(real_identity)
        self.assertFalse(_owned_identity_still_matches(real_identity))

    def test_executable_identity_mismatch_prevents_signalling(self):
        proc = self._launch("under_test", startup_hold_s=0.3, max_runtime_s=30.0)
        real_identity = capture_owned_process_identity(proc)
        forged_identity = replace(real_identity, exe_path="/bin/false")
        result = terminate_owned_process_group_and_verify(forged_identity)
        self.assertEqual(result, {"escalated_to_sigkill": False, "cleanup_classification": "normal"})
        self.assertTrue(
            _owned_identity_still_matches(real_identity),
            "the real process must be untouched when the forged identity's exe_path does not match",
        )
        self._assert_sentinel_untouched()
        terminate_owned_process_group_and_verify(real_identity)
        self.assertFalse(_owned_identity_still_matches(real_identity))

    def test_escalation_to_sigkill_remains_limited_to_the_verified_owned_group(self):
        # An artificially tiny sigint_wait_s forces escalation
        # deterministically (rather than depending on the real node
        # ignoring SIGINT, which it does not) -- the point under test is
        # that escalation, when it happens, is still scoped to the
        # exact verified owned process group and classified abnormal,
        # never touching the unrelated sentinel.
        proc = self._launch("under_test", startup_hold_s=0.3, max_runtime_s=30.0)
        identity = capture_owned_process_identity(proc)
        time.sleep(0.5)
        result = terminate_owned_process_group_and_verify(
            identity, sigint_wait_s=0.002, sigkill_wait_s=5, poll_interval_s=0.001,
        )
        self.assertTrue(result["escalated_to_sigkill"])
        self.assertEqual(result["cleanup_classification"], "abnormal")
        self.assertFalse(_owned_identity_still_matches(identity))
        self._assert_sentinel_untouched()

    def test_read_only_residual_audit_reports_clean_after_owned_cleanup_but_never_selected_the_target(self):
        # self_match_safe_residual_audit() is READ-ONLY -- it must never
        # be the thing that decides what gets signalled. This test
        # proves the owned-identity cleanup alone (never the audit
        # function) is what stops the process: the audit report is
        # empty afterward, purely as an observation.
        proc = self._launch("under_test", startup_hold_s=0.3, max_runtime_s=30.0)
        identity = capture_owned_process_identity(proc)
        terminate_owned_process_group_and_verify(identity)
        self.assertFalse(_owned_identity_still_matches(identity))
        # The sentinel is still alive and its cmdline still matches the
        # audit pattern -- proving the audit call below reports on
        # whatever is actually running, not on what cleanup targeted.
        audit_report = self_match_safe_residual_audit()
        self.assertIn(str(self.sentinel_proc.pid), audit_report)
        self._assert_sentinel_untouched()


if __name__ == "__main__":
    unittest.main()
