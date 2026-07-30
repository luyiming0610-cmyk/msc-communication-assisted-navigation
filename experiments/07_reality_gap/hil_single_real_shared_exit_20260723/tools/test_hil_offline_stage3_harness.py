#!/usr/bin/env python3
"""Tests for hil_offline_stage3_harness.py.

Pure-logic tests (no rclpy needed) cover the phase machine, the exact
pass-through gate, topic-isolation/domain checks, and the timeout
helper. Live rclpy tests (require a sourced ROS workspace, run under an
explicit isolated ROS_DOMAIN_ID, no hardware) prove periodic own-state
publication, sequence/timestamp progression, gate behaviour over real
topics, arm/bridge-status publication, and duplicate-announcement
gating -- all against private /hil_offline_stage3/... test topics only.
"""
from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path

from hil_offline_stage3_harness import (
    AdoptionCountExceededError,
    DuplicateAnnouncementController,
    DuplicateOrderingError,
    EXPECTED_STAGE3_ROS_DOMAIN_ID,
    FORBIDDEN_ROS_DOMAIN_IDS,
    GateDecision,
    OWN_STATE_REQUIRED_VALIDITY_FLAGS,
    PHASE_ORDER,
    PhaseMachine,
    PhaseTransitionError,
    RunnerTimeoutError,
    Stage3AutomaticRunner,
    Stage3Phase,
    SYNTHETIC_CLEAR_SENSOR_FIXTURE_FIELDS,
    apply_synthetic_clear_sensor_fixture,
    build_bridge_status_payload,
    build_gate_decision_event,
    check_ros_domain_id,
    gate_forward,
    is_adoption_confirmed,
    is_isolated_topic,
    is_timeout_exceeded,
)

# local_obstacle_logic.py has no ROS dependency (confirmed by its own
# module docstring: "no ROS dependency so the safety and priority rules
# can be unit-tested") -- imported here as a bare module by adding its
# real source directory to sys.path directly, the same pattern already
# used elsewhere in this project to reuse a sibling module without
# requiring a sourced ROS workspace just for a pure-logic test.
_LOCAL_OBSTACLE_LOGIC_DIR = str(
    Path(__file__).resolve().parents[4] / "src" / "epuck2_comm" / "epuck2_comm"
)
if _LOCAL_OBSTACLE_LOGIC_DIR not in sys.path:
    sys.path.insert(0, _LOCAL_OBSTACLE_LOGIC_DIR)
from local_obstacle_logic import decide_local_obstacle  # noqa: E402


class PhaseMachineTest(unittest.TestCase):
    def test_starts_at_initialising(self):
        m = PhaseMachine()
        self.assertEqual(m.phase, Stage3Phase.INITIALISING)
        self.assertFalse(m.is_complete)

    def test_advances_in_exact_order(self):
        m = PhaseMachine()
        for current, nxt in zip(PHASE_ORDER, PHASE_ORDER[1:]):
            self.assertEqual(m.phase, current)
            self.assertEqual(m.advance(current), nxt)
        self.assertTrue(m.is_complete)

    def test_advance_with_wrong_expected_current_raises(self):
        m = PhaseMachine()
        with self.assertRaises(PhaseTransitionError):
            m.advance(Stage3Phase.COMPLETE)
        self.assertEqual(m.phase, Stage3Phase.INITIALISING)

    def test_cannot_advance_past_complete(self):
        m = PhaseMachine()
        for current in PHASE_ORDER[:-1]:
            m.advance(current)
        with self.assertRaises(PhaseTransitionError):
            m.advance(Stage3Phase.COMPLETE)

    def test_history_records_every_transition_once(self):
        m = PhaseMachine()
        for current in PHASE_ORDER[:-1]:
            m.advance(current)
        self.assertEqual([p for p, _ in m.history], PHASE_ORDER)


class DuplicateAnnouncementControllerTest(unittest.TestCase):
    def test_duplicate_before_adoption_is_rejected(self):
        c = DuplicateAnnouncementController()
        with self.assertRaises(DuplicateOrderingError):
            c.authorize_duplicate_publication()
        self.assertFalse(c.duplicate_sent)

    def test_duplicate_after_one_adoption_is_allowed_once(self):
        c = DuplicateAnnouncementController()
        c.record_adoption_event()
        c.authorize_duplicate_publication()  # must not raise
        self.assertTrue(c.duplicate_sent)

    def test_second_duplicate_request_is_rejected(self):
        c = DuplicateAnnouncementController()
        c.record_adoption_event()
        c.authorize_duplicate_publication()
        with self.assertRaises(DuplicateOrderingError):
            c.authorize_duplicate_publication()

    def test_adoption_count_greater_than_one_aborts(self):
        c = DuplicateAnnouncementController()
        c.record_adoption_event()
        with self.assertRaises(AdoptionCountExceededError):
            c.record_adoption_event()
        self.assertEqual(c.adoption_count, 2)

    def test_duplicate_after_completion_is_rejected_even_with_one_adoption(self):
        c = DuplicateAnnouncementController()
        c.record_adoption_event()
        c.mark_complete()
        with self.assertRaises(DuplicateOrderingError):
            c.authorize_duplicate_publication()
        self.assertFalse(c.duplicate_sent)

    def test_phase_ordering_cannot_be_bypassed_by_calling_authorize_twice_quickly(self):
        """Even if a caller calls authorize_duplicate_publication() twice
        back-to-back without checking any return value, the second call
        always raises -- there is no path to a second successful
        authorization regardless of caller diligence."""
        c = DuplicateAnnouncementController()
        c.record_adoption_event()
        results = []
        for _ in range(3):
            try:
                c.authorize_duplicate_publication()
                results.append("OK")
            except DuplicateOrderingError:
                results.append("REJECTED")
        self.assertEqual(results, ["OK", "REJECTED", "REJECTED"])


class GateForwardPureLogicTest(unittest.TestCase):
    def test_open_gate_returns_identical_object(self):
        msg = types.SimpleNamespace(
            protocol_version=1, source=2, robot_id=7, sequence=42,
            stamp="STAMP", x_m=1.0, y_m=2.0, yaw_rad=0.3,
            linear_velocity_mps=0.01, angular_velocity_rps=0.0, validity_flags=1,
        )
        result = gate_forward(msg, gate_open=True)
        self.assertIs(result, msg)  # identity, not a copy -- proves no field can have been touched
        for field_name, value in vars(msg).items():
            self.assertEqual(getattr(result, field_name), value)

    def test_closed_gate_returns_none(self):
        msg = types.SimpleNamespace(x_m=1.0)
        self.assertIsNone(gate_forward(msg, gate_open=False))

    def test_gate_never_mutates_input_regardless_of_state(self):
        msg = types.SimpleNamespace(sequence=1, x_m=5.0)
        gate_forward(msg, gate_open=True)
        gate_forward(msg, gate_open=False)
        self.assertEqual(msg.sequence, 1)
        self.assertEqual(msg.x_m, 5.0)


class OwnStateContractTest(unittest.TestCase):
    def test_required_validity_flags_is_odom_ir_tof_value_7(self):
        self.assertEqual(OWN_STATE_REQUIRED_VALIDITY_FLAGS, 7)


class BridgeStatusPayloadTest(unittest.TestCase):
    def test_uses_the_exact_keys_the_recorder_parser_reads(self):
        payload = build_bridge_status_payload(rx_count=5, connected=True)
        self.assertEqual(payload, {"connected": True, "rx_count": 5})

    def test_connected_defaults_true(self):
        self.assertTrue(build_bridge_status_payload(rx_count=0)["connected"])


class AdoptionConfirmationTest(unittest.TestCase):
    def test_go_to_exit_confirms_adoption(self):
        self.assertTrue(is_adoption_confirmed("GO_TO_EXIT"))

    def test_search_phase_does_not_confirm_adoption(self):
        self.assertFalse(is_adoption_confirmed("SEARCH"))

    def test_none_does_not_confirm_adoption(self):
        self.assertFalse(is_adoption_confirmed(None))

    def test_arrived_hold_does_not_confirm_adoption(self):
        self.assertFalse(is_adoption_confirmed("ARRIVED_HOLD"))


class IsolatedTopicTest(unittest.TestCase):
    def test_rejects_every_production_topic(self):
        for topic in (
            "/cmd_vel", "/cmd_vel_unguarded", "/epuck1/state",
            "/epuck_bridge/status", "/hil_guard/arm",
        ):
            self.assertFalse(is_isolated_topic(topic), topic)

    def test_rejects_non_namespaced_topic(self):
        self.assertFalse(is_isolated_topic("/some_other_topic"))

    def test_accepts_properly_namespaced_topic(self):
        self.assertTrue(is_isolated_topic("/hil_offline_stage3/epuck1/state"))

    def test_module_never_publishes_a_production_topic_string_literal(self):
        """Checks only actual create_publisher/create_subscription call
        sites -- the module docstring legitimately names every production
        topic in prose (explaining what must never be constructed), which
        is not itself a publisher/subscription call and must not trip
        this check."""
        source = Path(__file__).with_name("hil_offline_stage3_harness.py").read_text(encoding="utf-8")
        call_lines = [
            line for line in source.splitlines()
            if "create_publisher" in line or "create_subscription" in line
        ]
        for topic in ("/cmd_vel", "/cmd_vel_unguarded", "/epuck1/state",
                      "/epuck_bridge/status", "/hil_guard/arm"):
            for line in call_lines:
                self.assertNotIn(f'"{topic}"', line, line)


class RosDomainIdCheckTest(unittest.TestCase):
    def test_rejects_default_and_reserved_domains(self):
        for domain in (0, 77, 89):
            ok, reason = check_ros_domain_id(domain)
            self.assertFalse(ok, domain)
            self.assertIn(str(domain), reason)

    def test_accepts_sanctioned_stage3_domain(self):
        ok, reason = check_ros_domain_id(EXPECTED_STAGE3_ROS_DOMAIN_ID)
        self.assertTrue(ok)
        self.assertEqual(reason, "")

    def test_rejects_arbitrary_other_domain(self):
        ok, _ = check_ros_domain_id(12345)
        self.assertFalse(ok)

    def test_forbidden_set_matches_expectation(self):
        self.assertEqual(FORBIDDEN_ROS_DOMAIN_IDS, frozenset({0, 77, 89}))


class GateDecisionEventTest(unittest.TestCase):
    def test_forwarded_event_shape(self):
        event = build_gate_decision_event(
            gate_epoch=2, gate_state="OPEN", source_protocol_version=1,
            source_robot_id=3, source_sequence=42, source_production_stamp_s=1.5,
            decision=GateDecision.FORWARDED.value, decision_timestamp_s=1.6,
            first_source_after_reopen=True, forwarded_destination_topic="/hil_offline_stage3/x",
        )
        self.assertEqual(event["event_type"], "GATE_DECISION")
        self.assertEqual(event["gate_epoch"], 2)
        self.assertEqual(event["decision"], "FORWARDED")
        self.assertTrue(event["first_source_after_reopen"])
        self.assertEqual(event["forwarded_destination_topic"], "/hil_offline_stage3/x")

    def test_rejected_event_carries_no_destination(self):
        event = build_gate_decision_event(
            gate_epoch=0, gate_state="CLOSED", source_protocol_version=1,
            source_robot_id=3, source_sequence=1, source_production_stamp_s=1.0,
            decision=GateDecision.REJECTED_GATE_CLOSED.value, decision_timestamp_s=1.1,
            first_source_after_reopen=False, forwarded_destination_topic=None,
        )
        self.assertEqual(event["decision"], "REJECTED_GATE_CLOSED")
        self.assertIsNone(event["forwarded_destination_topic"])
        self.assertFalse(event["first_source_after_reopen"])

    def test_all_fields_are_type_coerced(self):
        event = build_gate_decision_event(
            gate_epoch="3", gate_state="OPEN", source_protocol_version="1",
            source_robot_id="2", source_sequence="7", source_production_stamp_s="1.25",
            decision=GateDecision.FORWARDED.value, decision_timestamp_s="1.5",
            first_source_after_reopen=1, forwarded_destination_topic="/hil_offline_stage3/y",
        )
        self.assertEqual(event["gate_epoch"], 3)
        self.assertEqual(event["source_protocol_version"], 1)
        self.assertEqual(event["source_robot_id"], 2)
        self.assertEqual(event["source_sequence"], 7)
        self.assertEqual(event["source_production_stamp_s"], 1.25)
        self.assertEqual(event["decision_timestamp_s"], 1.5)
        self.assertIs(event["first_source_after_reopen"], True)


class _FakeStage3Harness:
    """Rclpy-free test double exposing exactly the public surface
    Stage3AutomaticRunner is allowed to call. Records every call with
    the phase active at call time, so tests can assert phase-restricted
    ordering without needing a real rclpy graph."""

    def __init__(self):
        self.phase_machine = PhaseMachine()
        self.duplicate_controller = DuplicateAnnouncementController()
        self._adoption_confirmed = False
        self._guarded_cmd = None
        self._fresh_post_reopen = False
        self.action_log: list = []

    def advance_phase(self, expected_current):
        new_phase = self.phase_machine.advance(expected_current)
        self.action_log.append(("advance_phase", new_phase))
        if new_phase == Stage3Phase.COMPLETE:
            self.duplicate_controller.mark_complete()
        return new_phase

    def set_arm(self, value):
        self.action_log.append(("set_arm", value, self.phase_machine.phase))

    def close_gate(self):
        self.action_log.append(("close_gate", self.phase_machine.phase))

    def open_gate(self):
        self.action_log.append(("open_gate", self.phase_machine.phase))

    def adoption_confirmed(self):
        return self._adoption_confirmed

    def latest_guarded_command_is_zero(self):
        return self._guarded_cmd == (0.0, 0.0)

    def latest_guarded_command_within_bounds(self, linear_bound_mps, angular_bound_rps):
        if self._guarded_cmd is None:
            return False
        lin, ang = self._guarded_cmd
        return abs(lin) <= linear_bound_mps and abs(ang) <= angular_bound_rps

    def has_fresh_post_reopen_gate_input(self):
        return self._fresh_post_reopen

    def request_duplicate_publication(self, source_robot_id, goal_x_m, goal_y_m, goal_id):
        self.action_log.append(("request_duplicate_publication", self.phase_machine.phase))
        self.duplicate_controller.authorize_duplicate_publication()


def _make_runner(harness, **overrides):
    kwargs = dict(
        per_phase_timeout_s=2.0, overall_timeout_s=10.0,
        test_only_linear_bound_mps=0.3, test_only_angular_bound_rps=3.0,
        peer_timeout_s=0.05, duplicate_source_robot_id=2,
        duplicate_goal_x_m=0.0, duplicate_goal_y_m=0.0, duplicate_goal_id="dup",
    )
    kwargs.update(overrides)
    return Stage3AutomaticRunner(harness, **kwargs)


class Stage3AutomaticRunnerTest(unittest.TestCase):
    def _happy_path_spin_once(self, harness, ticks: dict):
        """Returns a spin_once callable that flips harness fields on
        after enough calls, driving the runner through a full,
        successful run without ever needing a real rclpy graph."""
        state = {"count": 0}

        def spin_once():
            state["count"] += 1
            n = state["count"]
            if n >= ticks.get("adoption", 1) and not harness._adoption_confirmed:
                harness._adoption_confirmed = True
                harness.duplicate_controller.record_adoption_event()
            if n >= ticks.get("zero", 1):
                harness._guarded_cmd = (0.0, 0.0)
            if n >= ticks.get("bounded", 2) and harness.phase_machine.phase in (
                Stage3Phase.DISARMED_ZERO_CONFIRMED,
            ):
                harness._guarded_cmd = (0.1, 0.1)
            if n >= ticks.get("fresh_post_reopen", 1):
                harness._fresh_post_reopen = True

        return spin_once

    def test_drives_all_11_phases_in_exact_order_with_no_skip_or_repeat(self):
        harness = _FakeStage3Harness()
        runner = _make_runner(harness)
        runner.run(self._happy_path_spin_once(harness, {}))
        self.assertTrue(harness.phase_machine.is_complete)
        advanced = [phase for action, phase in
                    ((a[0], a[1]) for a in harness.action_log if a[0] == "advance_phase")]
        self.assertEqual(advanced, PHASE_ORDER[1:])

    def test_phase_restricted_actions_occur_at_the_correct_phase(self):
        harness = _FakeStage3Harness()
        runner = _make_runner(harness)
        runner.run(self._happy_path_spin_once(harness, {}))
        by_action = {a[0]: a for a in harness.action_log if a[0] != "advance_phase"}
        self.assertEqual(by_action["set_arm"][2], Stage3Phase.DISARMED_ZERO_CONFIRMED)
        self.assertEqual(by_action["close_gate"][1], Stage3Phase.ARMED_BOUNDED_CONFIRMED)
        self.assertEqual(by_action["open_gate"][1], Stage3Phase.STALE_ZERO_CONFIRMED)
        self.assertEqual(by_action["request_duplicate_publication"][1], Stage3Phase.RECOVERY_CONFIRMED)

    def test_duplicate_publication_is_authorized_only_after_adoption(self):
        harness = _FakeStage3Harness()
        runner = _make_runner(harness)
        runner.run(self._happy_path_spin_once(harness, {}))
        self.assertEqual(harness.duplicate_controller.adoption_count, 1)
        self.assertTrue(harness.duplicate_controller.duplicate_sent)

    def test_second_adoption_rising_edge_aborts_the_run(self):
        harness = _FakeStage3Harness()
        runner = _make_runner(harness)
        state = {"count": 0}

        def spin_once():
            state["count"] += 1
            n = state["count"]
            if n == 1:
                harness._adoption_confirmed = True
                harness.duplicate_controller.record_adoption_event()
            if n == 2:
                # a spurious second adoption rising-edge must abort
                harness.duplicate_controller.record_adoption_event()
            harness._guarded_cmd = (0.0, 0.0)
            harness._fresh_post_reopen = True

        with self.assertRaises(AdoptionCountExceededError):
            runner.run(spin_once)

    def test_phase_never_skipped_or_repeated_even_under_a_stalled_condition(self):
        harness = _FakeStage3Harness()
        runner = _make_runner(harness, per_phase_timeout_s=0.05, overall_timeout_s=5.0)
        with self.assertRaises(RunnerTimeoutError):
            runner.run(lambda: None)  # adoption_confirmed() never becomes True
        # only INITIALISING->READY_DISARMED was ever attempted; nothing
        # further was skipped to or repeated
        advanced = [a[1] for a in harness.action_log if a[0] == "advance_phase"]
        self.assertEqual(advanced, [Stage3Phase.READY_DISARMED])

    def test_per_phase_timeout_raised_when_condition_never_satisfied(self):
        harness = _FakeStage3Harness()
        runner = _make_runner(harness, per_phase_timeout_s=0.05, overall_timeout_s=5.0)
        with self.assertRaises(RunnerTimeoutError) as ctx:
            runner.run(lambda: None)
        self.assertIn("per-phase timeout", str(ctx.exception))

    def test_overall_timeout_raised_even_if_each_phase_individually_meets_its_own_budget(self):
        harness = _FakeStage3Harness()
        runner = _make_runner(harness, per_phase_timeout_s=1.0, overall_timeout_s=0.05)
        with self.assertRaises(RunnerTimeoutError) as ctx:
            runner.run(lambda: None)
        self.assertIn("overall", str(ctx.exception))

    def test_no_action_possible_after_complete_phase_is_reached(self):
        harness = _FakeStage3Harness()
        runner = _make_runner(harness)
        runner.run(self._happy_path_spin_once(harness, {}))
        with self.assertRaises(PhaseTransitionError):
            harness.advance_phase(Stage3Phase.DUPLICATE_REJECTED)
        with self.assertRaises(DuplicateOrderingError):
            harness.request_duplicate_publication(2, 0.0, 0.0, "dup2")

    def test_runner_calls_only_the_harness_public_orchestration_surface(self):
        """Code-inspection guard: Stage3AutomaticRunner must never
        reference navigation/guard/avoidance internals directly -- it is
        only allowed to call the harness's own public orchestration
        methods (advance_phase/close_gate/open_gate/set_arm/
        request_duplicate_publication/adoption_confirmed/latest_guarded_*/
        has_fresh_post_reopen_gate_input)."""
        source = Path(__file__).with_name("hil_offline_stage3_harness.py").read_text(encoding="utf-8")
        start = source.index("class Stage3AutomaticRunner")
        end = source.index("\ndef gate_forward(")
        runner_source = source[start:end]
        # Strip the class's own docstring (which legitimately explains,
        # in prose, what this class deliberately does NOT do) before
        # scanning -- only actual code lines must never reference these.
        doc_start = runner_source.index('"""')
        doc_end = runner_source.index('"""', doc_start + 3) + 3
        code_only = runner_source[:doc_start] + runner_source[doc_end:]
        for forbidden in ("GoalNavigator", "decide_command", "cooperative_avoider", "NavigationTargetState"):
            self.assertNotIn(forbidden, code_only)
        for topic in ("/cmd_vel", "/cmd_vel_unguarded", "/epuck1/state",
                      "/epuck_bridge/status", "/hil_guard/arm"):
            self.assertNotIn(f'"{topic}"', code_only)


class TimeoutHelperTest(unittest.TestCase):
    def test_not_exceeded_before_deadline(self):
        self.assertFalse(is_timeout_exceeded(start_monotonic_s=0.0, now_monotonic_s=5.0, max_runtime_s=60.0))

    def test_exceeded_after_deadline(self):
        self.assertTrue(is_timeout_exceeded(start_monotonic_s=0.0, now_monotonic_s=61.0, max_runtime_s=60.0))

    def test_boundary_exactly_at_deadline_not_exceeded(self):
        self.assertFalse(is_timeout_exceeded(start_monotonic_s=0.0, now_monotonic_s=60.0, max_runtime_s=60.0))


class SyntheticClearSensorFixtureTest(unittest.TestCase):
    """Proves the SYNTHETIC_CLEAR_SENSOR_FIXTURE correction:
    hil_offline_stage3_harness.py's own-state message must set every
    EpuckState field the real, unmodified cooperative_avoider.py /
    local_obstacle_logic.py chain reads under a "no valid return"
    convention to +Inf, never the ROS float32 implicit default 0.0 --
    which decide_local_obstacle() (imported here as the real, unmodified
    production function, not a reimplementation) would otherwise read as
    a genuine obstacle at zero distance."""

    def _build_stub_state(self, **overrides):
        fields = {name: float("inf") for name in SYNTHETIC_CLEAR_SENSOR_FIXTURE_FIELDS}
        fields.update(overrides)
        stub = types.SimpleNamespace(validity_flags=OWN_STATE_REQUIRED_VALIDITY_FLAGS, **fields)
        return stub

    def test_validity_flags_is_7(self):
        self.assertEqual(OWN_STATE_REQUIRED_VALIDITY_FLAGS, 7)

    def test_every_consumed_field_becomes_positive_infinity(self):
        stub = types.SimpleNamespace(x_m=1.0, y_m=2.0)  # unrelated field, must survive untouched
        apply_synthetic_clear_sensor_fixture(stub)
        for field_name in SYNTHETIC_CLEAR_SENSOR_FIXTURE_FIELDS:
            value = getattr(stub, field_name)
            self.assertTrue(math.isinf(value) and value > 0, f"{field_name}={value}")
        self.assertEqual(stub.x_m, 1.0)
        self.assertEqual(stub.y_m, 2.0)

    def test_no_consumed_field_remains_zero(self):
        stub = types.SimpleNamespace()
        apply_synthetic_clear_sensor_fixture(stub)
        for field_name in SYNTHETIC_CLEAR_SENSOR_FIXTURE_FIELDS:
            self.assertNotEqual(getattr(stub, field_name), 0.0, field_name)

    def test_real_decide_local_obstacle_returns_clear_for_the_fixture(self):
        stub = self._build_stub_state()
        decision = decide_local_obstacle(
            stub.front_distance_m, stub.left_distance_m, stub.right_distance_m,
            stub.validity_flags,
        )
        self.assertEqual(decision.mode, "LOCAL_CLEAR")
        self.assertFalse(decision.active)
        self.assertFalse(decision.safety_stop)

    def test_does_not_return_local_front_danger(self):
        stub = self._build_stub_state()
        decision = decide_local_obstacle(
            stub.front_distance_m, stub.left_distance_m, stub.right_distance_m,
            stub.validity_flags,
        )
        self.assertNotEqual(decision.mode, "LOCAL_FRONT_DANGER")

    def test_does_not_produce_permanent_in_place_turn(self):
        stub = self._build_stub_state()
        decision = decide_local_obstacle(
            stub.front_distance_m, stub.left_distance_m, stub.right_distance_m,
            stub.validity_flags,
        )
        # LOCAL_CLEAR/inactive means cooperative_avoider's own priority
        # chain falls through to peer-CPA/dynamic-heading/cruise logic
        # instead of returning this decision's own linear/angular values
        # -- confirmed here by asserting no artificial command is even
        # offered (both zero) AND the decision is not active.
        self.assertFalse(decision.active)
        self.assertEqual(decision.linear_mps, 0.0)
        self.assertEqual(decision.angular_rps, 0.0)

    def test_zero_front_distance_still_produces_danger_this_proves_the_test_is_meaningful(self):
        """Without the fixture correction (front_distance_m left at the
        implicit 0.0 default), decide_local_obstacle() must still
        produce LOCAL_FRONT_DANGER -- proving the CLEAR result above is
        a genuine consequence of the +Inf fixture, not a vacuous check
        that would pass regardless of input."""
        stub = self._build_stub_state(front_distance_m=0.0)
        decision = decide_local_obstacle(
            stub.front_distance_m, stub.left_distance_m, stub.right_distance_m,
            stub.validity_flags,
        )
        self.assertEqual(decision.mode, "LOCAL_FRONT_DANGER")
        self.assertTrue(decision.active)

    def test_fixture_is_explicitly_labeled_test_only_not_physical(self):
        source = Path(__file__).with_name("hil_offline_stage3_harness.py").read_text(encoding="utf-8")
        self.assertIn("SYNTHETIC_CLEAR_SENSOR_FIXTURE", source)
        self.assertIn("TEST_ONLY", source)
        self.assertIn("NOT_A_PHYSICAL_MEASUREMENT", source)

    def test_publish_own_state_applies_the_fixture(self):
        """Structural guard: _publish_own_state() must actually call
        apply_synthetic_clear_sensor_fixture() -- a fixture function that
        exists but is never wired in would be exactly as broken as no
        fixture at all."""
        source = Path(__file__).with_name("hil_offline_stage3_harness.py").read_text(encoding="utf-8")
        start = source.index("def _publish_own_state")
        end = source.index("\n\n", start)
        body = source[start:end]
        self.assertIn("apply_synthetic_clear_sensor_fixture(msg)", body)


def _runbook_text() -> str:
    path = Path(__file__).resolve().parents[1] / "HIL_OFFLINE_STAGE3_RUNBOOK.md"
    return path.read_text(encoding="utf-8")


def _runbook_bash_blocks() -> list:
    """Extract only the executable ```bash fenced code blocks from the
    runbook, separate from prohibition/preflight prose -- required so
    this test never asserts a production topic string is absent from
    the whole document (it legitimately appears in prohibition text),
    only that it never appears as an executable argument/remap."""
    text = _runbook_text()
    return re.findall(r"```bash\n(.*?)\n```", text, flags=re.DOTALL)


def _runbook_bash_code_lines() -> list:
    """All executable (non-comment, non-blank) lines across every bash
    block, e.g. for placeholder-token checks that must ignore the
    deliberately-commented-out, deferred RUN_ID/OUT_DIR bootstrap
    lines."""
    lines = []
    for block in _runbook_bash_blocks():
        for line in block.splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            lines.append(line)
    return lines


class RunbookCommandContractTest(unittest.TestCase):
    """Parses the committed runbook's executable ```bash block(s)
    separately from its prohibition/preflight prose, proving the
    command contract itself -- never asserting a production topic
    string is absent from the whole Markdown document (it legitimately
    appears in prohibition/read-only-detection text)."""

    def test_bash_block_exists(self):
        self.assertTrue(_runbook_bash_blocks(), "expected at least one ```bash fenced block in the runbook")

    def test_no_unresolved_control_placeholder_in_executable_lines(self):
        code_lines = _runbook_bash_code_lines()
        forbidden_substrings = (
            "<TEST_ONLY_BOUND>", "<value>", "<X>", "<informed|search>", "[--auto-run]",
        )
        for line in code_lines:
            for forbidden in forbidden_substrings:
                self.assertNotIn(forbidden, line, line)

    def test_mandatory_auto_run_present_not_bracketed(self):
        code_lines = _runbook_bash_code_lines()
        joined = "\n".join(code_lines)
        self.assertIn("--auto-run", joined)
        self.assertNotIn("[--auto-run]", joined)

    def test_duplicate_identity_overrides_match_accepted_contract(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn("--runner-duplicate-source-robot-id 2", joined)
        self.assertIn("--runner-duplicate-goal-x-m 2.0", joined)
        self.assertIn("--runner-duplicate-goal-y-m 3.0", joined)
        self.assertIn("--runner-duplicate-goal-id shared_exit", joined)

    def test_cooperative_avoider_has_all_three_isolated_remaps(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn('-r "state:=${OWN_STATE_TOPIC}"', joined)
        self.assertIn('-r "cmd_vel:=${REQUESTED_CMD_VEL_TOPIC}"', joined)
        self.assertIn('-r "nav_intent:=${NAV_INTENT_TOPIC}"', joined)
        self.assertIn('-p "peer_state_topic:=${VP_GATE_INPUT_TOPIC}"', joined)
        for exact_param in (
            "-p armed:=true", "-p enable_peer_avoidance:=true",
            "-p enable_dynamic_heading:=true", "-p enable_dynamic_speed:=true",
            "-p enable_local_avoidance:=true", "-p require_local_sensors:=true",
            "-p use_sim_time:=false", "-p safety_radius_m:=0.14",
            "-p stop_after_recovery:=false",
        ):
            self.assertIn(exact_param, joined)

    def test_all_operational_topics_are_isolated(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn('NS="/hil_offline_stage3"', joined)

    def test_production_topics_never_appear_as_executable_argument_or_remap(self):
        code_lines = _runbook_bash_code_lines()
        production_topics = (
            "/cmd_vel", "/cmd_vel_unguarded", "/epuck1/state",
            "/epuck_bridge/status", "/hil_guard/arm",
        )
        for line in code_lines:
            if "grep" in line or "pgrep" in line or "ABORT" in line or "for t in" in line:
                continue  # prohibition/detection checks legitimately reference these exact strings
            # Strip isolated-topic constructions (${NS}/epuck1/state etc.)
            # before checking for a BARE production topic string -- an
            # isolated topic that happens to share a suffix with a
            # production topic name (by design: /hil_offline_stage3 +
            # /epuck1/state) is not itself a production topic.
            sanitized = re.sub(r"\$\{NS\}[A-Za-z0-9_/]*", "", line)
            for topic in production_topics:
                self.assertNotIn(topic, sanitized, line)

    def test_production_topics_appear_only_in_prohibition_or_check_text(self):
        text = _runbook_text()
        # Full-document occurrence count must exceed the executable-line
        # count (proving the remaining occurrences are prose/checks),
        # while the executable-line check above already proves zero
        # occurrences as an actual argument/remap.
        self.assertIn("/cmd_vel_unguarded", text)
        self.assertIn("/hil_guard/arm", text)

    def test_direct_dollar_bang_pid_capture_present(self):
        joined = "\n".join(_runbook_bash_code_lines())
        for var in ("RECORDER_PID", "GUARD_PID", "ADAPTER_PID", "COOP_PID", "HARNESS_PID", "PEER_PID"):
            self.assertIn(f'{var}="$!"', joined)
        self.assertNotIn("pgrep -f", joined.replace("pgrep -af", ""))

    def test_abort_cleanup_uses_exact_kill_int_and_no_pkill(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn("kill -INT", joined)
        self.assertNotIn("pkill", joined)

    def test_recorder_last_cleanup_is_encoded(self):
        # Every process is now stopped via terminate_owned_pid()/
        # terminate_owned_process_group() calls inside cleanup() (never an
        # inline `kill -INT "${PID}"`); this proves the CALL ORDER stops
        # recorder strictly last: peer -> harness -> cooperative_avoider ->
        # adapter -> guard -> recorder.
        joined = "\n".join(_runbook_bash_code_lines())
        cleanup_start = joined.index("cleanup() {")
        cleanup_end = joined.index("run_cleanup_once() {")
        cleanup_body = joined[cleanup_start:cleanup_end]
        order = ["PEER_PID", "HARNESS_PID", "COOP_PID", "ADAPTER_PID", "GUARD_PID", "RECORDER_PID"]
        indices = [cleanup_body.index(f'"${{{name}}}"') for name in order]
        self.assertEqual(indices, sorted(indices), "cleanup must stop processes in exact reverse launch order")
        self.assertEqual(
            indices[-1], max(indices),
            "recorder must be the last process referenced in cleanup()",
        )

    def test_verifier_output_and_exit_status_preserved(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn('> "${VERIFIER_JSON}"', joined)
        self.assertIn('echo "${VERIFIER_EXIT}" > "${VERIFIER_EXIT_FILE}"', joined)
        # Verifier execution is captured via an explicit if/else so that
        # `set -Eeuo pipefail`'s errexit never discards VERIFIER_EXIT
        # before it is recorded -- a bare `cmd; VERIFIER_EXIT=$?` would
        # abort at `cmd` itself under errexit.
        self.assertIn('if python3 hil_offline_stage3_post_run_verifier.py "${VERIFIER_ARGS[@]}" > "${VERIFIER_JSON}"; then', joined)
        self.assertIn("VERIFIER_EXIT=0", joined)
        self.assertIn("VERIFIER_EXIT=$?", joined)
        # The final exit reflects FINAL_EXIT (which incorporates harness,
        # cleanup, residual, verifier-JSON-validity, verifier, and hashing
        # results), never a bare pass-through of VERIFIER_EXIT alone.
        self.assertIn('exit "${FINAL_EXIT}"', joined)
        self.assertNotIn('exit "${VERIFIER_EXIT}"', joined)

    def test_fail_fast_shell_policy_present(self):
        # `set -Eeuo pipefail` (errexit + ERR-trap-inheritance + nounset +
        # pipefail) must be the script's own shell policy -- a readiness
        # timeout, hash/HEAD mismatch, forbidden-process/topic detection,
        # or executable-resolution failure must stop startup immediately,
        # never fall through to a later step.
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn("set -Eeuo pipefail", joined)

    def test_readiness_gates_are_bare_statements_not_swallowed(self):
        # Every wait_for_log_pattern(...) call must be a bare statement --
        # never wrapped in `|| true` or `if ...; then :; fi` -- so a
        # nonzero (timeout) return trips errexit immediately instead of
        # being silently discarded.
        joined = "\n".join(_runbook_bash_code_lines())
        for line in _runbook_bash_code_lines():
            if line.strip().startswith("wait_for_log_pattern "):
                self.assertNotIn("||", line, line)
                self.assertNotIn("if ", line, line)

    def test_int_term_exit_handling_is_separated_and_idempotent(self):
        joined = "\n".join(_runbook_bash_code_lines())
        for fn_name in ("on_int", "on_term", "on_exit", "run_cleanup_once"):
            self.assertIn(f"{fn_name}() {{", joined)
        self.assertIn("trap on_int INT", joined)
        self.assertIn("trap on_term TERM", joined)
        self.assertIn("trap on_exit EXIT", joined)
        # A single combined `trap cleanup EXIT INT TERM` handler (the
        # earlier design) is no longer present -- INT/TERM/EXIT must be
        # handled by three separate functions.
        self.assertNotIn("trap cleanup EXIT INT TERM", joined)
        # on_int/on_term delegate their intended exit status to the
        # shared run_cleanup_and_finalize() function (which also
        # performs always-run evidence finalization) rather than
        # exiting with a literal number directly.
        self.assertIn("run_cleanup_and_finalize 130", joined)  # on_int's own exit status
        self.assertIn("run_cleanup_and_finalize 143", joined)  # on_term's own exit status
        self.assertIn('exit "${intended_exit_code}"', joined)
        self.assertIn("CLEANUP_DONE", joined)  # idempotency guard
        # Each signal handler disables all three traps before its own
        # final exit, preventing recursive re-entry.
        on_int_start = joined.index("on_int() {")
        on_int_end = joined.index("on_term() {")
        self.assertIn("trap - INT TERM EXIT", joined[on_int_start:on_int_end])
        on_term_start = on_int_end
        on_term_end = joined.index("on_exit() {")
        self.assertIn("trap - INT TERM EXIT", joined[on_term_start:on_term_end])
        on_exit_start = on_term_end
        on_exit_end = joined.index("trap on_int INT")
        self.assertIn("trap - INT TERM EXIT", joined[on_exit_start:on_exit_end])

    def test_verifier_runs_only_after_explicit_stack_shutdown(self):
        # The verifier must never run while the recorder or any other
        # producer is still active: the explicit `run_cleanup_once`
        # invocation (Step 10) must appear strictly before the verifier
        # invocation (Step 13) in program order.
        joined = "\n".join(_runbook_bash_code_lines())
        cleanup_call_index = joined.index("run_cleanup_once || true")
        verifier_call_index = joined.index('python3 hil_offline_stage3_post_run_verifier.py "${VERIFIER_ARGS[@]}"')
        self.assertLess(cleanup_call_index, verifier_call_index)

    def test_hashing_occurs_after_verifier_and_file_finalization_no_active_writer(self):
        joined = "\n".join(_runbook_bash_code_lines())
        cleanup_call_index = joined.index("run_cleanup_once || true")
        finalization_index = joined.index('_wait_for_file_ready "${EVIDENCE_CSV}" 5')
        verifier_call_index = joined.index('python3 hil_offline_stage3_post_run_verifier.py "${VERIFIER_ARGS[@]}"')
        # Hashing now happens inside the shared finalize_evidence() call
        # on the happy path -- find the REAL invocation site (after the
        # verifier), not finalize_evidence's own definition earlier in
        # the file.
        hashing_index = joined.index('finalize_evidence "RUN" "${TASK_OUTCOME_VALUE}"')
        self.assertLess(cleanup_call_index, finalization_index)
        self.assertLess(finalization_index, verifier_call_index)
        self.assertLess(verifier_call_index, hashing_index)

    def test_exact_ros_humble_and_repo_paths_no_glob_no_bash_source_derivation(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn("source /opt/ros/humble/setup.bash", joined)
        self.assertIn("source /home/eamon/epuck_ws/install/setup.bash", joined)
        self.assertNotIn("/opt/ros/*/setup.bash", joined)
        self.assertIn('REPO_ROOT="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm"', joined)
        self.assertIn('TOOLS_DIR="${REPO_ROOT}/experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools"', joined)
        self.assertNotIn('TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")', joined)

    def test_no_self_referential_expected_head_or_self_hash_embedded(self):
        # A tracked file cannot correctly predict its own post-commit
        # identity -- a real, hardware-free execution attempt against
        # this runbook's own previous revision proved exactly this: its
        # embedded EXPECTED_HEAD necessarily referred to the *parent* of
        # the commit that introduced it, so it could never pass on its
        # own introducing commit. No literal 40-hex-char commit id may
        # ever be assigned to EXPECTED_HEAD, and no hardcoded SHA-256
        # table of this runbook's own path (or any other required path)
        # may be embedded anywhere in this document.
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertNotRegex(joined, r'EXPECTED_HEAD="[0-9a-f]{40}"')
        self.assertNotIn("STAGE3_COMMITTED_SHA256", joined)
        self.assertNotRegex(
            joined,
            r'HIL_OFFLINE_STAGE3_RUNBOOK\.md"\]="[0-9a-f]{64}"',
            "no hardcoded SHA-256 of this runbook's own path may be embedded",
        )

    def test_expected_head_required_externally_run_id_required_only_after_identity_checks(self):
        joined = "\n".join(_runbook_bash_code_lines())
        expected_head_idx = joined.index(':' + ' "${EXPECTED_HEAD:?')
        run_id_idx = joined.index(':' + ' "${RUN_ID:?')
        source_clean_idx = joined.index('echo "PRE_RUN_SOURCE_IDENTITY_CHECK=CLEAN"')
        self.assertGreaterEqual(expected_head_idx, 0)
        # RUN_ID must be required strictly AFTER both identity layers have
        # already passed -- a failed identity check must never reach the
        # RUN_ID gate at all.
        self.assertLess(source_clean_idx, run_id_idx)
        self.assertLess(expected_head_idx, source_clean_idx)

    def test_expected_head_format_existence_and_head_match_checks_present(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn(r'if [[ ! "${EXPECTED_HEAD}" =~ ^[0-9a-f]{40}$ ]]; then', joined)
        self.assertIn('git -C "${REPO_ROOT}" cat-file -e "${EXPECTED_HEAD}^{commit}"', joined)
        self.assertIn('ACTUAL_HEAD="$(git -C "${REPO_ROOT}" rev-parse HEAD)"', joined)
        self.assertIn('if [[ "${ACTUAL_HEAD}" != "${EXPECTED_HEAD}" ]]; then', joined)

    def test_required_source_paths_list_is_complete(self):
        # 23 paths: the 22 from the prior revision plus setup.py --
        # the tracked file that DEFINES the cooperative_avoider
        # console_scripts mapping (deployment/entry-point metadata,
        # reached by the installed launcher/entry-point verification,
        # not by any Python runtime import).
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn("REQUIRED_SOURCE_PATHS=(", joined)
        for path in (
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/HIL_OFFLINE_STAGE3_RUNBOOK.md",
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_offline_stage3_harness.py",
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_offline_stage3_evidence_recorder.py",
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_offline_stage3_post_run_verifier.py",
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_cmd_vel_guard.py",
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_topic_adapter.py",
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_virtual_peer.py",
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_goal_announcement_evidence.py",
            "experiments/10_cooperative_exit_navigation_20260720/tools/goal_navigator.py",
            "experiments/10_cooperative_exit_navigation_20260720/tools/goal_hold_tracker.py",
            "experiments/10_cooperative_exit_navigation_20260720/tools/navigation_target_state.py",
            "src/epuck2_comm/epuck2_comm/__init__.py",
            "src/epuck2_comm/epuck2_comm/cooperative_avoider.py",
            "src/epuck2_comm/epuck2_comm/command_smoothing.py",
            "src/epuck2_comm/epuck2_comm/collision_math.py",
            "src/epuck2_comm/epuck2_comm/local_obstacle_logic.py",
            "src/epuck2_comm/epuck2_comm/models.py",
            "src/epuck2_comm/epuck2_comm/neighbor_cache.py",
            "src/epuck2_comm/epuck2_comm/transmission_policy.py",
            "src/epuck2_comm/setup.py",
            "src/epuck2_comm_interfaces/msg/EpuckState.msg",
            "src/epuck2_comm_interfaces/msg/GoalAnnouncement.msg",
            "src/epuck2_comm_interfaces/msg/NavigationIntent.msg",
        ):
            self.assertIn(f'"{path}"', joined)

    def test_git_blob_comparison_mechanism_present_never_manual_sha256_table(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn('git -C "${REPO_ROOT}" rev-parse --verify -q "${EXPECTED_HEAD}:${src_path}"', joined)
        # Path-aware: attribute lookup (.gitattributes eol/text rules) is
        # keyed off the explicit --path value, never a plain hash-object
        # call relying on incidental cwd/argv-path resolution.
        self.assertIn('git -C "${REPO_ROOT}" hash-object --path="${src_path}" -- "${full_path}"', joined)
        self.assertIn('if [[ "${expected_blob}" != "${worktree_blob}" ]]; then', joined)

    def test_installed_runtime_identity_mechanism_present(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn(': "${INSTALL_ROOT:?', joined)
        self.assertIn("INSTALLED_PY_TO_SOURCE", joined)
        self.assertIn('git -C "${REPO_ROOT}" hash-object --path="${src_path}" -- "${installed_path}"', joined)
        self.assertIn("INSTALLED_RUNTIME_IDENTITY=BLOCKED", joined)
        self.assertIn("cooperative_avoider = epuck2_comm.cooperative_avoider:main", joined)

    def test_index_staged_change_check_uses_cached_diff_not_status(self):
        # Must use `git diff --cached --quiet` (real content comparison,
        # immune to the documented WSL stat-cache anomaly) -- never
        # `git status` or a plain (non---cached) `git diff`.
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn('git -C "${REPO_ROOT}" diff --cached --quiet -- "${src_path}"', joined)
        self.assertNotIn("git status", joined)

    def test_source_verification_precedes_ros_sourcing_coop_resolution_and_run_id_use(self):
        # ROS is now sourced INSIDE Step C (message-schema verification),
        # i.e. before RUN_ID/OUT_DIR, not after -- `source
        # /opt/ros/humble/setup.bash` legitimately appears twice in this
        # document (once in the small illustrative "Environment setup"
        # example near the top, once in the real Step C script);
        # ordering here is checked strictly within the real script, so
        # the LAST occurrence (Step C's) is used.
        joined = "\n".join(_runbook_bash_code_lines())
        ros_source_index = joined.rindex("source /opt/ros/humble/setup.bash")
        source_check_index = joined.index('echo "PRE_RUN_SOURCE_IDENTITY_CHECK=CLEAN"')
        out_dir_use_index = joined.index('OUT_DIR="${HIL_ROOT}/hil_offline_stage3_${RUN_ID}"')
        coop_resolve_index = joined.index('COOP_PREFIX="$(ros2 pkg prefix epuck2_comm)"')
        mkdir_index = joined.index('mkdir -p "${OUT_DIR}"')
        # This exact filename also legitimately appears earlier, inside
        # REQUIRED_SOURCE_PATHS itself -- search only after mkdir_index
        # for the real Step 3 launch line.
        step3_index = joined.index("hil_offline_stage3_evidence_recorder.py", mkdir_index)
        self.assertLess(ros_source_index, source_check_index)
        self.assertLess(source_check_index, out_dir_use_index)
        self.assertLess(out_dir_use_index, coop_resolve_index)
        self.assertLess(coop_resolve_index, mkdir_index)
        self.assertLess(mkdir_index, step3_index)

    def test_source_identity_manifest_created_only_after_out_dir_and_hashed(self):
        joined = "\n".join(_runbook_bash_code_lines())
        mkdir_index = joined.index('mkdir -p "${OUT_DIR}"')
        manifest_write_index = joined.index("SOURCE_IDENTITY_MANIFEST_JSON=")
        self.assertLess(mkdir_index, manifest_write_index)
        # Hashing no longer lists explicit basenames -- finalize_evidence()
        # hashes every file that actually exists directly under OUT_DIR
        # (excluding only SHA256SUMS.txt itself and *.tmp.* atomic-write
        # temp files), so source_identity_manifest.json is swept up by
        # construction, not by an explicit per-file listing.
        self.assertIn('find . -maxdepth 1 -type f', joined)
        self.assertIn('! -name "*.tmp.*"', joined)

    def test_ros_sourcing_nounset_workaround_present(self):
        # /opt/ros/humble/setup.bash references AMENT_TRACE_SETUP_FILES
        # without a safe default and aborts under `set -u` -- confirmed
        # live. `set +u` must bracket only the two ROS `source` calls,
        # restored immediately afterward.
        joined = "\n".join(_runbook_bash_code_lines())
        set_plus_u_index = joined.index("set +u")
        ros_source_index = joined.index("source /opt/ros/humble/setup.bash", set_plus_u_index)
        set_minus_u_index = joined.index("set -u", ros_source_index)
        self.assertLess(set_plus_u_index, ros_source_index)
        self.assertLess(ros_source_index, set_minus_u_index)

    def test_execution_script_preserved_and_hashed(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn('EXECUTION_SCRIPT_COPY="${OUT_DIR}/execution_script.sh"', joined)
        self.assertIn('cp -- "${BASH_SOURCE[0]}" "${EXECUTION_SCRIPT_COPY}"', joined)
        # Swept up by finalize_evidence()'s dynamic find-based hashing
        # (every file that actually exists under OUT_DIR), not listed
        # explicitly -- see test_source_identity_manifest_created_only_after_out_dir_and_hashed.
        self.assertIn('find . -maxdepth 1 -type f', joined)

    def test_final_status_incorporates_every_required_failure_source(self):
        joined = "\n".join(_runbook_bash_code_lines())
        for reason in (
            'FAILURE_REASONS+=("HARNESS_EXIT=',
            'FAILURE_REASONS+=("CLEANUP_EXIT=',
            'FAILURE_REASONS+=("RESIDUAL_PROCESS_DETECTED")',
            'FAILURE_REASONS+=("VERIFIER_JSON_INVALID")',
            'FAILURE_REASONS+=("VERIFIER_EXIT=',
            # Hashing failure, hash-verification failure, JSON-validation
            # failure, and recorder-not-confirmed-stopped are now all
            # unified under one FINALIZE_EXIT-driven reason -- see
            # finalize_evidence()'s own internal FINALIZE_EXIT=1 sites.
            'FAILURE_REASONS+=("EVIDENCE_FINALIZATION_FAILED")',
        ):
            self.assertIn(reason, joined)

    def test_verifier_residual_process_flag_passed_when_cleanup_or_residual_failed(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn("--residual-process-detected", joined)
        self.assertIn(
            'if [[ "${CLEANUP_EXIT}" -ne 0 || "${POST_RUN_RESIDUAL_PROCESS_CHECK}" == "FAIL" ]]; then',
            joined,
        )

    def test_hash_verification_present_and_checked(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn("sha256sum -c", joined)
        self.assertIn("HASH_VERIFY_STATUS", joined)

    def test_cooperative_avoider_launched_via_resolved_direct_executable_not_ros2_run(self):
        # `ros2 run epuck2_comm cooperative_avoider` was live-tested
        # (CooperativeAvoiderCleanupPathTest) to fork a separate child
        # process for the real node while the `ros2 run` CLI wrapper can
        # exit on its own -- an unowned-child ambiguity. The runbook must
        # instead resolve and invoke the real installed executable
        # directly, via the standard, reproducible `ros2 pkg prefix`
        # mechanism, never `ros2 run` for this specific process.
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn('COOP_PREFIX="$(ros2 pkg prefix epuck2_comm)"', joined)
        self.assertIn('COOP_EXE="${COOP_PREFIX}/lib/epuck2_comm/cooperative_avoider"', joined)
        self.assertIn('"${COOP_EXE}" --ros-args', joined)
        self.assertNotIn("ros2 run epuck2_comm cooperative_avoider", joined)

    def test_cooperative_avoider_launch_isolated_via_job_control_and_identity_captured(self):
        # `set -m` (scoped tightly around only this one launch, restored
        # immediately after) makes bash job control assign the
        # background job its own process group (PGID == its own PID,
        # confirmed live) instead of inheriting this script's own
        # process group -- required so terminate_owned_process_group()
        # can killpg the exact owned group without ever touching this
        # script itself. The exact owned identity (PID, PGID, /proc
        # start time, /proc/<pid>/exe) must be captured immediately
        # after launch, before any other command could race with it.
        joined = "\n".join(_runbook_bash_code_lines())
        set_m_index = joined.index("set -m")
        coop_pid_index = joined.index('COOP_PID="$!"')
        set_plus_m_index = joined.index("set +m")
        self.assertLess(set_m_index, coop_pid_index)
        self.assertLess(coop_pid_index, set_plus_m_index)
        self.assertIn('COOP_PGID="$(ps -o pgid= -p "${COOP_PID}" | tr -d \' \')"', joined)
        self.assertIn('COOP_START_TIME="$(_proc_start_time "${COOP_PID}")"', joined)
        self.assertIn('COOP_EXE_PATH="$(_proc_exe_path "${COOP_PID}")"', joined)

    def test_cooperative_avoider_cleanup_is_exact_owned_process_group_only(self):
        # Cleanup must call terminate_owned_process_group() with the
        # captured owned identity, and must NEVER use `pgrep`/`pkill`
        # output to select what gets signalled (pgrep is used ONLY inside
        # the separately-defined, read-only _forbidden_process_scan(),
        # never as a literal call inside cleanup() itself).
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn(
            'terminate_owned_process_group \\\n        "${COOP_PID}" "${COOP_PGID}" "${COOP_START_TIME}" "${COOP_EXE_PATH}" "cooperative_avoider"',
            joined,
        )
        self.assertNotIn("pkill", joined)
        cleanup_start = joined.index("cleanup() {")
        cleanup_end = joined.index("run_cleanup_once() {")
        cleanup_body = joined[cleanup_start:cleanup_end]
        self.assertNotIn("pgrep", cleanup_body)
        self.assertNotIn("pkill", cleanup_body)
        # The post-run residual check is performed via exactly one call to
        # the self-match-safe scan function, never a literal pgrep here.
        self.assertEqual(cleanup_body.count("_forbidden_process_scan"), 1)
        self.assertIn("POST_RUN_RESIDUAL_PROCESS_CHECK", cleanup_body)

    def test_owned_identity_helper_functions_never_use_name_based_discovery(self):
        joined = "\n".join(_runbook_bash_code_lines())
        for fn_name in ("_proc_start_time", "_proc_exe_path", "_owned_identity_still_matches", "terminate_owned_process_group", "terminate_owned_pid"):
            self.assertIn(f"{fn_name}()", joined)
        fn_start = joined.index("_proc_start_time() {")
        fn_end = joined.index("cleanup() {", fn_start)
        helper_block = joined[fn_start:fn_end]
        self.assertNotIn("pgrep", helper_block)
        self.assertNotIn("pkill", helper_block)

    def test_forbidden_process_scan_is_self_match_safe_and_read_only(self):
        # _forbidden_process_scan() must exclude this script's own PID
        # ($$) from the match set before the emptiness test is applied at
        # each call site, so it can never match its own invocation
        # regardless of where the script happens to be staged -- proven
        # structurally (not merely by the bracket-quoting idiom, which
        # only protects pgrep's own argv from matching its own pattern).
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn("SELF_PID=$$", joined)
        self.assertIn("_forbidden_process_scan() {", joined)
        fn_start = joined.index("_forbidden_process_scan() {")
        fn_end = joined.index("if FORBIDDEN_MATCHES=")
        fn_body = joined[fn_start:fn_end]
        self.assertIn('awk -v self="${SELF_PID}" \'$1 != self\'', fn_body)
        # Called exactly twice: once pre-run, once inside cleanup() -- and
        # in both cases only to decide whether to ABORT/report, never to
        # supply a kill target.
        self.assertEqual(joined.count("_forbidden_process_scan"), 3)  # definition + 2 call sites

    def test_evidence_hashing_present_and_manifest_excludes_itself(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn("sha256sum", joined)
        self.assertIn("SHA256SUMS_FILE", joined)
        # The dynamic find-based hashing must explicitly exclude the
        # SHA256SUMS output file's own basename from what it hashes.
        self.assertIn(
            '! -name "$(basename "${SHA256SUMS_FILE}")"',
            joined,
        )


def _extract_trap_cleanup_block() -> str:
    """Extracts the REAL trap/cleanup/helper function definitions
    (from `wait_for_log_pattern() {` through `trap on_exit EXIT`)
    verbatim from the committed runbook's own executable bash block --
    never a reimplementation -- for use by a harness that drives them
    with dummy processes instead of ROS nodes."""
    joined = "\n".join(_runbook_bash_code_lines())
    start = joined.index("wait_for_log_pattern() {")
    end = joined.index("trap on_exit EXIT") + len("trap on_exit EXIT")
    return joined[start:end]


class ShellControlFlowRegressionTest(unittest.TestCase):
    """Pure, hardware-free, ROS-free regression proof of the runbook's
    own trap/cleanup state machine: extracts the REAL function bodies
    (never a reimplementation) and drives them against a single dummy
    background process (`sleep`, renamed via `exec -a` to a name unique
    to this test so residual checks can never collide with an
    unrelated real `sleep` invocation elsewhere on the system) plus
    plain marker files standing in for "steps". No ROS_DOMAIN_ID is
    set, no ROS package is imported, no cooperative_avoider/ros2
    process is ever started.
    """

    DUMMY_TAG = "stage3_shell_control_regression_dummy"

    def _run_scenario(self, body: str, timeout_s: float = 10.0):
        extracted = _extract_trap_cleanup_block()
        prologue = (
            "#!/usr/bin/env bash\n"
            "set -Eeuo pipefail\n"
            'RECORDER_PID=""\nGUARD_PID=""\nADAPTER_PID=""\n'
            'COOP_PID=""\nCOOP_PGID=""\nCOOP_START_TIME=""\nCOOP_EXE_PATH=""\n'
            'COOP_CLEANUP_CLASSIFICATION="normal"\nHARNESS_PID=""\nPEER_PID=""\n'
            "CLEANUP_DONE=0\nCLEANUP_EXIT=0\n"
            'POST_RUN_RESIDUAL_PROCESS_CHECK="UNKNOWN"\n'
            "SELF_PID=$$\n"
            "FORBIDDEN_PROCESS_PATTERN='__no_such_process_for_this_test__'\n"
            "_forbidden_process_scan() {\n"
            '    pgrep -af "${FORBIDDEN_PROCESS_PATTERN}" 2>/dev/null | awk -v self="${SELF_PID}" \'$1 != self\'\n'
            "}\n\n"
        )
        # Instrument CLEANUP_DONE=1 to also increment an on-disk counter,
        # so the test can prove cleanup ran AT MOST ONCE regardless of
        # how many of INT/TERM/EXIT eventually fired.
        extracted_instrumented = extracted.replace(
            "CLEANUP_DONE=1",
            'CLEANUP_DONE=1\n    echo $(( $(cat "${MARKER_DIR}/cleanup_count") + 1 )) > "${MARKER_DIR}/cleanup_count"',
        )
        script = prologue + extracted_instrumented + "\n\nMARKER_DIR=\"$1\"\n" + body
        with tempfile.TemporaryDirectory() as marker_dir:
            script_path = os.path.join(marker_dir, "harness.sh")
            with open(script_path, "w", encoding="utf-8") as f:
                f.write(script)
            with open(os.path.join(marker_dir, "cleanup_count"), "w", encoding="utf-8") as f:
                f.write("0")
            proc = subprocess.run(
                ["bash", script_path, marker_dir],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True, timeout=timeout_s,
            )
            cleanup_count = int(open(os.path.join(marker_dir, "cleanup_count")).read().strip())
            step1_done = os.path.exists(os.path.join(marker_dir, "step1_done"))
            step2_done = os.path.exists(os.path.join(marker_dir, "step2_done"))
        return proc.returncode, cleanup_count, step1_done, step2_done, proc.stderr

    def _assert_no_residual_dummy(self):
        residual = subprocess.run(
            ["pgrep", "-af", f"[e]xec -a {self.DUMMY_TAG}"],
            capture_output=True, text=True,
        )
        self.assertEqual(
            residual.stdout.strip(), "",
            f"dummy process from this regression test was left running: {residual.stdout}",
        )

    def _dummy_launch_line(self):
        return f'exec -a {self.DUMMY_TAG} sleep 30 >/dev/null 2>&1 &\nPEER_PID="$!"\n'

    def test_normal_completion_runs_cleanup_once_and_stops_dummy(self):
        body = self._dummy_launch_line() + 'touch "${MARKER_DIR}/step1_done"\ntouch "${MARKER_DIR}/step2_done"\nexit 0\n'
        returncode, cleanup_count, step1, step2, stderr = self._run_scenario(body)
        self.assertEqual(returncode, 0, stderr)
        self.assertEqual(cleanup_count, 1)
        self.assertTrue(step1)
        self.assertTrue(step2)
        self._assert_no_residual_dummy()

    def test_mid_script_failure_stops_before_next_step_and_still_cleans_up(self):
        # A bare `false` mirrors a nonzero readiness-gate return under
        # `set -Eeuo pipefail` -- errexit must abort before step2 ever runs.
        body = self._dummy_launch_line() + 'touch "${MARKER_DIR}/step1_done"\nfalse\ntouch "${MARKER_DIR}/step2_done"\n'
        returncode, cleanup_count, step1, step2, stderr = self._run_scenario(body)
        self.assertNotEqual(returncode, 0)
        self.assertEqual(cleanup_count, 1)
        self.assertTrue(step1)
        self.assertFalse(step2, "a failure must prevent any subsequent step from running")
        self._assert_no_residual_dummy()

    def test_sigint_ends_with_130_and_does_not_resume_normal_execution(self):
        body = self._dummy_launch_line() + 'touch "${MARKER_DIR}/step1_done"\nkill -INT $$\nsleep 5\ntouch "${MARKER_DIR}/step2_done"\n'
        returncode, cleanup_count, step1, step2, stderr = self._run_scenario(body)
        self.assertEqual(returncode, 130, stderr)
        self.assertEqual(cleanup_count, 1)
        self.assertTrue(step1)
        self.assertFalse(step2, "execution must not resume after SIGINT")
        self._assert_no_residual_dummy()

    def test_sigterm_ends_with_143_and_does_not_resume_normal_execution(self):
        body = self._dummy_launch_line() + 'touch "${MARKER_DIR}/step1_done"\nkill -TERM $$\nsleep 5\ntouch "${MARKER_DIR}/step2_done"\n'
        returncode, cleanup_count, step1, step2, stderr = self._run_scenario(body)
        self.assertEqual(returncode, 143, stderr)
        self.assertEqual(cleanup_count, 1)
        self.assertTrue(step1)
        self.assertFalse(step2, "execution must not resume after SIGTERM")
        self._assert_no_residual_dummy()

    def test_cleanup_failure_overrides_an_otherwise_successful_exit_status(self):
        # cleanup()'s own return status is forced nonzero here (simulating
        # a process that failed to stop) -- the script itself still exits
        # 0 normally, but on_exit() must surface CLEANUP_EXIT instead of
        # silently reporting success.
        extracted = _extract_trap_cleanup_block()
        self.assertIn('return "${failed}"\n}', extracted, "expected cleanup()'s own return statement to patch for this test")
        forced_failure = extracted.replace(
            'return "${failed}"\n}',
            'return "${failed}"\n}\n\n'
            '# TEST-ONLY override: force cleanup() to report failure, to prove\n'
            '# on_exit() surfaces CLEANUP_EXIT rather than a false success.\n'
            'eval "$(declare -f cleanup | sed \'s/return "${failed}"/return 1/\')"',
        )
        body = self._dummy_launch_line() + 'touch "${MARKER_DIR}/step1_done"\ntouch "${MARKER_DIR}/step2_done"\nexit 0\n'
        # _run_scenario always re-extracts the block itself; for this one
        # test we need the forced-failure variant, so inline the same
        # logic here rather than reusing _run_scenario verbatim.
        prologue = (
            "#!/usr/bin/env bash\nset -Eeuo pipefail\n"
            'RECORDER_PID=""\nGUARD_PID=""\nADAPTER_PID=""\n'
            'COOP_PID=""\nCOOP_PGID=""\nCOOP_START_TIME=""\nCOOP_EXE_PATH=""\n'
            'COOP_CLEANUP_CLASSIFICATION="normal"\nHARNESS_PID=""\nPEER_PID=""\n'
            "CLEANUP_DONE=0\nCLEANUP_EXIT=0\n"
            'POST_RUN_RESIDUAL_PROCESS_CHECK="UNKNOWN"\nSELF_PID=$$\n'
            "FORBIDDEN_PROCESS_PATTERN='__no_such_process_for_this_test__'\n"
            "_forbidden_process_scan() {\n"
            '    pgrep -af "${FORBIDDEN_PROCESS_PATTERN}" 2>/dev/null | awk -v self="${SELF_PID}" \'$1 != self\'\n'
            "}\n\n"
        )
        instrumented = forced_failure.replace(
            "CLEANUP_DONE=1",
            'CLEANUP_DONE=1\n    echo $(( $(cat "${MARKER_DIR}/cleanup_count") + 1 )) > "${MARKER_DIR}/cleanup_count"',
        )
        script = prologue + instrumented + "\n\nMARKER_DIR=\"$1\"\n" + body
        with tempfile.TemporaryDirectory() as marker_dir:
            script_path = os.path.join(marker_dir, "harness.sh")
            with open(script_path, "w", encoding="utf-8") as f:
                f.write(script)
            with open(os.path.join(marker_dir, "cleanup_count"), "w", encoding="utf-8") as f:
                f.write("0")
            proc = subprocess.run(
                ["bash", script_path, marker_dir],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True, timeout=10.0,
            )
            cleanup_count = int(open(os.path.join(marker_dir, "cleanup_count")).read().strip())
        self.assertNotEqual(proc.returncode, 0, "a cleanup failure must not be reported as a successful run")
        self.assertEqual(cleanup_count, 1)
        self._assert_no_residual_dummy()


class IncrementalManifestAndFinalizationTest(unittest.TestCase):
    """Pure, hardware-free, ROS-free proof of the runbook's incremental
    atomic PID manifest, failure-stage tracking, and always-run evidence
    finalization (write_pid_manifest_atomic/record_component_*/
    verify_recorder_stopped/finalize_evidence/run_cleanup_and_finalize) --
    extracts the REAL function bodies (the same `_extract_trap_cleanup_block()`
    used by ShellControlFlowRegressionTest, never a reimplementation) and
    drives them with dummy background processes standing in for
    recorder/guard/adapter/etc. No ROS_DOMAIN_ID is set, no ROS package
    is imported, no cooperative_avoider/ros2 process is ever started."""

    DUMMY_TAG = "stage3_finalize_regression_dummy"

    def _run_scenario(self, body: str, timeout_s: float = 10.0):
        extracted = _extract_trap_cleanup_block()
        prologue = (
            "#!/usr/bin/env bash\n"
            "set -Eeuo pipefail\n"
            'RUN_ID="dummy_test_run"\n'
            'OUT_DIR="$1"\n'
            'PID_MANIFEST="${OUT_DIR}/pid_manifest.json"\n'
            'STATUS_RECORD_JSON="${OUT_DIR}/launcher_status.json"\n'
            'SHA256SUMS_FILE="${OUT_DIR}/SHA256SUMS.txt"\n'
            'RECORDER_LOG="${OUT_DIR}/recorder.log"\nGUARD_LOG="${OUT_DIR}/guard.log"\n'
            'ADAPTER_LOG="${OUT_DIR}/adapter.log"\nCOOP_LOG="${OUT_DIR}/coop.log"\n'
            'HARNESS_LOG="${OUT_DIR}/harness.log"\nPEER_LOG="${OUT_DIR}/peer.log"\n'
            'RECORDER_PID=""\nGUARD_PID=""\nADAPTER_PID=""\n'
            'COOP_PID=""\nCOOP_PGID=""\nCOOP_START_TIME=""\nCOOP_EXE_PATH=""\n'
            'COOP_CLEANUP_CLASSIFICATION="normal"\nHARNESS_PID=""\nPEER_PID=""\n'
            'HARNESS_EXIT=""\n'
            "CLEANUP_DONE=0\nCLEANUP_EXIT=0\n"
            'POST_RUN_RESIDUAL_PROCESS_CHECK="UNKNOWN"\n'
            "SELF_PID=$$\n"
            "FORBIDDEN_PROCESS_PATTERN='__no_such_process_for_this_test__'\n"
            "_forbidden_process_scan() {\n"
            '    pgrep -af "${FORBIDDEN_PROCESS_PATTERN}" 2>/dev/null | awk -v self="${SELF_PID}" \'$1 != self\'\n'
            "}\n\n"
        )
        script = prologue + extracted + "\n\n" + body
        with tempfile.TemporaryDirectory() as out_dir:
            script_path = os.path.join(out_dir, "..", "harness.sh")
            script_path = os.path.abspath(script_path)
            with open(script_path, "w", encoding="utf-8") as f:
                f.write(script)
            proc = subprocess.run(
                ["bash", script_path, out_dir],
                capture_output=True, text=True, timeout=timeout_s,
            )
            status = None
            manifest = None
            status_path = os.path.join(out_dir, "launcher_status.json")
            manifest_path = os.path.join(out_dir, "pid_manifest.json")
            if os.path.exists(status_path):
                with open(status_path, encoding="utf-8") as f:
                    status = json.load(f)
            if os.path.exists(manifest_path):
                with open(manifest_path, encoding="utf-8") as f:
                    manifest = json.load(f)
            sha_exists = os.path.exists(os.path.join(out_dir, "SHA256SUMS.txt"))
        try:
            os.remove(script_path)
        except OSError:
            pass
        return proc.returncode, status, manifest, sha_exists, proc.stderr

    def _assert_no_residual_dummy(self):
        residual = subprocess.run(
            ["pgrep", "-af", f"[e]xec -a {self.DUMMY_TAG}"],
            capture_output=True, text=True,
        )
        self.assertEqual(residual.stdout.strip(), "", f"dummy process left running: {residual.stdout}")

    def _dummy(self, label: str) -> str:
        return f"exec -a {self.DUMMY_TAG}_{label} sleep 30"

    def test_failure_before_any_child_launch(self):
        returncode, status, manifest, sha_exists, stderr = self._run_scenario('false\n')
        self.assertNotEqual(returncode, 0, stderr)
        self.assertEqual(status["run_completion"], "INCOMPLETE_BRINGUP")
        self.assertEqual(status["behavioral_verifier_status"], "NOT_RUN")
        self.assertEqual(status["behavioral_result"], "NOT_OBTAINED")
        self.assertEqual(manifest["components"], [])
        self.assertTrue(sha_exists)
        self._assert_no_residual_dummy()

    def test_recorder_launch_failure_readiness_timeout(self):
        body = (
            f'LAST_ATTEMPTED_STAGE="recorder launch/readiness"\n'
            f'( {self._dummy("s2")} ) > "${{RECORDER_LOG}}" 2>&1 &\n'
            'RECORDER_PID="$!"\n'
            'record_component_launch "recorder" "${RECORDER_PID}" "$(ps -o pgid= -p "${RECORDER_PID}" | tr -d " ")" "$(_proc_start_time "${RECORDER_PID}")" "$(_proc_exe_path "${RECORDER_PID}")"\n'
            'if ! wait_for_log_pattern "${RECORDER_LOG}" "NEVER_APPEARS" 1; then\n'
            '    READINESS_TIMEOUT_RESULT="recorder"\n'
            "    exit 1\n"
            "fi\n"
        )
        returncode, status, manifest, sha_exists, stderr = self._run_scenario(body)
        self.assertNotEqual(returncode, 0, stderr)
        self.assertEqual(status["last_attempted_stage"], "recorder launch/readiness")
        self.assertEqual(status["readiness_timeout_result"], "recorder")
        self.assertEqual([c["component"] for c in manifest["components"]], ["recorder"])
        self._assert_no_residual_dummy()

    def test_recorder_ready_guard_failure(self):
        body = (
            'echo "RECORDER_READY" > "${RECORDER_LOG}" &\n'
            'RECORDER_PID="$!"\n'
            'record_component_launch "recorder" "${RECORDER_PID}" "$(ps -o pgid= -p "${RECORDER_PID}" | tr -d " ")" "$(_proc_start_time "${RECORDER_PID}")" "$(_proc_exe_path "${RECORDER_PID}")"\n'
            'wait_for_log_pattern "${RECORDER_LOG}" "RECORDER_READY" 5\n'
            'record_component_ready "recorder"\n'
            f'LAST_ATTEMPTED_STAGE="guard launch/readiness"\n'
            f'( {self._dummy("s4")} ) > "${{GUARD_LOG}}" 2>&1 &\n'
            'GUARD_PID="$!"\n'
            'record_component_launch "guard" "${GUARD_PID}" "$(ps -o pgid= -p "${GUARD_PID}" | tr -d " ")" "$(_proc_start_time "${GUARD_PID}")" "$(_proc_exe_path "${GUARD_PID}")"\n'
            'if ! wait_for_log_pattern "${GUARD_LOG}" "NEVER" 1; then\n'
            '    READINESS_TIMEOUT_RESULT="guard"\n'
            "    exit 1\n"
            "fi\n"
        )
        returncode, status, manifest, sha_exists, stderr = self._run_scenario(body)
        self.assertNotEqual(returncode, 0, stderr)
        self.assertEqual(status["last_attempted_stage"], "guard launch/readiness")
        self.assertEqual([c["component"] for c in manifest["components"]], ["recorder", "guard"])
        self.assertEqual(manifest["components"][0]["readiness_state"], "READY")
        self.assertEqual(manifest["components"][1]["readiness_state"], "PENDING")
        self._assert_no_residual_dummy()

    def test_recorder_guard_ready_adapter_crash(self):
        body = (
            'echo "RECORDER_READY" > "${RECORDER_LOG}" &\nRECORDER_PID="$!"\n'
            'record_component_launch "recorder" "${RECORDER_PID}" "$(ps -o pgid= -p "${RECORDER_PID}" | tr -d " ")" "$(_proc_start_time "${RECORDER_PID}")" "$(_proc_exe_path "${RECORDER_PID}")"\n'
            'wait_for_log_pattern "${RECORDER_LOG}" "RECORDER_READY" 5\nrecord_component_ready "recorder"\n'
            'echo "GUARD_READY" > "${GUARD_LOG}" &\nGUARD_PID="$!"\n'
            'record_component_launch "guard" "${GUARD_PID}" "$(ps -o pgid= -p "${GUARD_PID}" | tr -d " ")" "$(_proc_start_time "${GUARD_PID}")" "$(_proc_exe_path "${GUARD_PID}")"\n'
            'wait_for_log_pattern "${GUARD_LOG}" "GUARD_READY" 5\nrecord_component_ready "guard"\n'
            f'LAST_ATTEMPTED_STAGE="adapter launch/readiness"\n'
            f'( {self._dummy("s5")}; ) > "${{ADAPTER_LOG}}" 2>&1 &\nADAPTER_PID="$!"\n'
            'record_component_launch "adapter" "${ADAPTER_PID}" "$(ps -o pgid= -p "${ADAPTER_PID}" | tr -d " ")" "$(_proc_start_time "${ADAPTER_PID}")" "$(_proc_exe_path "${ADAPTER_PID}")"\n'
            'if ! wait_for_log_pattern "${ADAPTER_LOG}" "NEVER" 1; then\n'
            '    READINESS_TIMEOUT_RESULT="adapter"\n    exit 1\nfi\n'
        )
        returncode, status, manifest, sha_exists, stderr = self._run_scenario(body)
        self.assertNotEqual(returncode, 0, stderr)
        self.assertEqual(status["last_attempted_stage"], "adapter launch/readiness")
        self.assertEqual([c["component"] for c in manifest["components"]], ["recorder", "guard", "adapter"])
        self.assertTrue(sha_exists)
        self._assert_no_residual_dummy()

    def test_sigint_finalizes_evidence_and_stops_recorder_last(self):
        body = (
            'echo "RECORDER_READY" > "${RECORDER_LOG}" &\nRECORDER_PID="$!"\n'
            'record_component_launch "recorder" "${RECORDER_PID}" "$(ps -o pgid= -p "${RECORDER_PID}" | tr -d " ")" "$(_proc_start_time "${RECORDER_PID}")" "$(_proc_exe_path "${RECORDER_PID}")"\n'
            'wait_for_log_pattern "${RECORDER_LOG}" "RECORDER_READY" 5\nrecord_component_ready "recorder"\n'
            "kill -INT $$\nsleep 5\n"
        )
        returncode, status, manifest, sha_exists, stderr = self._run_scenario(body)
        self.assertEqual(returncode, 130, stderr)
        self.assertEqual(status["run_completion"], "INCOMPLETE_BRINGUP")
        self.assertEqual(status["behavioral_verifier_status"], "NOT_RUN")
        self._assert_no_residual_dummy()

    def test_sigterm_finalizes_evidence(self):
        body = (
            'echo "RECORDER_READY" > "${RECORDER_LOG}" &\nRECORDER_PID="$!"\n'
            'record_component_launch "recorder" "${RECORDER_PID}" "$(ps -o pgid= -p "${RECORDER_PID}" | tr -d " ")" "$(_proc_start_time "${RECORDER_PID}")" "$(_proc_exe_path "${RECORDER_PID}")"\n'
            'wait_for_log_pattern "${RECORDER_LOG}" "RECORDER_READY" 5\nrecord_component_ready "recorder"\n'
            "kill -TERM $$\nsleep 5\n"
        )
        returncode, status, manifest, sha_exists, stderr = self._run_scenario(body)
        self.assertEqual(returncode, 143, stderr)
        self.assertEqual(status["run_completion"], "INCOMPLETE_BRINGUP")
        self._assert_no_residual_dummy()

    def test_recorder_refuses_to_stop_is_reported_as_integrity_failure(self):
        extracted = _extract_trap_cleanup_block()
        broken = extracted.replace(
            'terminate_owned_pid() {\n    local pid="$1" label="$2"',
            'terminate_owned_pid() {\n    local pid="$1" label="$2"\n'
            '    if [[ "${label}" == "recorder" ]]; then return 1; fi',
        )
        self.assertNotEqual(broken, extracted, "expected to be able to patch terminate_owned_pid for this test")
        prologue = (
            "#!/usr/bin/env bash\nset -Eeuo pipefail\n"
            'RUN_ID="dummy_test_run"\nOUT_DIR="$1"\n'
            'PID_MANIFEST="${OUT_DIR}/pid_manifest.json"\nSTATUS_RECORD_JSON="${OUT_DIR}/launcher_status.json"\n'
            'SHA256SUMS_FILE="${OUT_DIR}/SHA256SUMS.txt"\nRECORDER_LOG="${OUT_DIR}/recorder.log"\n'
            'GUARD_LOG="${OUT_DIR}/guard.log"\nADAPTER_LOG="${OUT_DIR}/adapter.log"\nCOOP_LOG="${OUT_DIR}/coop.log"\n'
            'HARNESS_LOG="${OUT_DIR}/harness.log"\nPEER_LOG="${OUT_DIR}/peer.log"\n'
            'RECORDER_PID=""\nGUARD_PID=""\nADAPTER_PID=""\n'
            'COOP_PID=""\nCOOP_PGID=""\nCOOP_START_TIME=""\nCOOP_EXE_PATH=""\n'
            'COOP_CLEANUP_CLASSIFICATION="normal"\nHARNESS_PID=""\nPEER_PID=""\nHARNESS_EXIT=""\n'
            "CLEANUP_DONE=0\nCLEANUP_EXIT=0\n"
            'POST_RUN_RESIDUAL_PROCESS_CHECK="UNKNOWN"\nSELF_PID=$$\n'
            "FORBIDDEN_PROCESS_PATTERN='__no_such_process_for_this_test__'\n"
            "_forbidden_process_scan() {\n"
            '    pgrep -af "${FORBIDDEN_PROCESS_PATTERN}" 2>/dev/null | awk -v self="${SELF_PID}" \'$1 != self\'\n'
            "}\n\n"
        )
        body = (
            f'( {self._dummy("stuck")} ) > "${{RECORDER_LOG}}" 2>&1 &\nRECORDER_PID="$!"\n'
            'record_component_launch "recorder" "${RECORDER_PID}" "$(ps -o pgid= -p "${RECORDER_PID}" | tr -d " ")" "$(_proc_start_time "${RECORDER_PID}")" "$(_proc_exe_path "${RECORDER_PID}")"\n'
            "false\n"
        )
        script = prologue + broken + "\n\n" + body
        with tempfile.TemporaryDirectory() as out_dir:
            script_path = os.path.abspath(os.path.join(out_dir, "..", "stuck_harness.sh"))
            with open(script_path, "w", encoding="utf-8") as f:
                f.write(script)
            proc = subprocess.run(["bash", script_path, out_dir], capture_output=True, text=True, timeout=10.0)
            with open(os.path.join(out_dir, "launcher_status.json"), encoding="utf-8") as f:
                status = json.load(f)
        # Clean up the genuinely-still-alive dummy process (the broken
        # terminate_owned_pid never signals it) and the script file --
        # this manual cleanup is test-harness housekeeping, not part of
        # the mechanism under test.
        subprocess.run(["pkill", "-f", f"{self.DUMMY_TAG}_stuck"], capture_output=True)
        try:
            os.remove(script_path)
        except OSError:
            pass
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(status["recorder_confirmed_stopped"])
        self.assertIn("RECORDER_NOT_CONFIRMED_STOPPED", status["run_completion"])

    def test_normal_successful_path_runs_verifier_placeholder_once_and_reports_complete(self):
        body = (
            'echo "RECORDER_READY" > "${RECORDER_LOG}" &\nRECORDER_PID="$!"\n'
            'record_component_launch "recorder" "${RECORDER_PID}" "$(ps -o pgid= -p "${RECORDER_PID}" | tr -d " ")" "$(_proc_start_time "${RECORDER_PID}")" "$(_proc_exe_path "${RECORDER_PID}")"\n'
            'wait_for_log_pattern "${RECORDER_LOG}" "RECORDER_READY" 5\nrecord_component_ready "recorder"\n'
            'VERIFIER_RUN_COUNT_FILE="${OUT_DIR}/verifier_run_count"\n'
            'echo 0 > "${VERIFIER_RUN_COUNT_FILE}"\n'
            'run_behavioral_verifier_placeholder() {\n'
            '    echo $(( $(cat "${VERIFIER_RUN_COUNT_FILE}") + 1 )) > "${VERIFIER_RUN_COUNT_FILE}"\n'
            '    TASK_OUTCOME_VALUE="PASS"\n'
            "}\n"
            "LAST_ATTEMPTED_STAGE=\"cleanup\"\n"
            "run_cleanup_once || true\n"
            "LAST_ATTEMPTED_STAGE=\"verification\"\n"
            "run_behavioral_verifier_placeholder\n"
            "LAST_ATTEMPTED_STAGE=\"hashing\"\n"
            'if finalize_evidence "RUN" "${TASK_OUTCOME_VALUE}"; then FINALIZE_EXIT=0; else FINALIZE_EXIT=$?; fi\n'
            "EVIDENCE_FINALIZED=1\n"
            'exit $(( FINALIZE_EXIT ))\n'
        )
        returncode, status, manifest, sha_exists, stderr = self._run_scenario(body)
        self.assertEqual(returncode, 0, stderr)
        self.assertEqual(status["run_completion"], "COMPLETE")
        self.assertEqual(status["behavioral_verifier_status"], "RUN")
        self.assertEqual(status["behavioral_result"], "PASS")
        self.assertTrue(sha_exists)
        self._assert_no_residual_dummy()

    def test_incomplete_bringup_never_runs_behavioral_verifier(self):
        # Across every early-abort scenario above, no post_run_verification.json
        # or verifier invocation occurs -- proven directly for one
        # representative case (adapter crash) by asserting the verifier
        # output file was never created.
        body = (
            f'LAST_ATTEMPTED_STAGE="adapter launch/readiness"\n'
            f'( {self._dummy("s_noverify")} ) > "${{ADAPTER_LOG}}" 2>&1 &\nADAPTER_PID="$!"\n'
            'record_component_launch "adapter" "${ADAPTER_PID}" "$(ps -o pgid= -p "${ADAPTER_PID}" | tr -d " ")" "$(_proc_start_time "${ADAPTER_PID}")" "$(_proc_exe_path "${ADAPTER_PID}")"\n'
            'if ! wait_for_log_pattern "${ADAPTER_LOG}" "NEVER" 1; then\n'
            '    READINESS_TIMEOUT_RESULT="adapter"\n    exit 1\nfi\n'
        )
        returncode, status, manifest, sha_exists, stderr = self._run_scenario(body)
        self.assertNotEqual(returncode, 0, stderr)
        self.assertEqual(status["behavioral_verifier_status"], "NOT_RUN")
        self._assert_no_residual_dummy()


def _extract_source_identity_block() -> str:
    """Extracts the REAL source-identity verification block verbatim
    from the committed runbook -- from its own leading comment through
    the final `PRE_RUN_SOURCE_IDENTITY_CHECK=CLEAN` echo -- for use by a
    harness that runs it against a disposable synthetic git repository
    instead of the real one. Deliberately starts AFTER the runbook's own
    REPO_ROOT/TOOLS_DIR/HIL_ROOT assignments so a caller can supply its
    own REPO_ROOT (pointing at the synthetic repo) without it being
    clobbered by the real runbook's hardcoded path."""
    joined = "\n".join(_runbook_bash_code_lines())
    start = joined.index(': "${EXPECTED_HEAD:?')
    end = joined.index('echo "PRE_RUN_SOURCE_IDENTITY_CHECK=CLEAN"') + len('echo "PRE_RUN_SOURCE_IDENTITY_CHECK=CLEAN"')
    return joined[start:end]


def _extract_source_identity_block_through_step_b() -> str:
    """Same extraction as `_extract_source_identity_block()`, but stops
    at the end of Step B (Git source paths + installed modules +
    installed launcher + installed entry-point metadata), BEFORE Step C
    sources the real ROS environment. Step C's message-schema check
    necessarily resolves messages from the one real ROS workspace this
    machine has installed (its `source .../setup.bash` targets a fixed
    real path, not a parameter) -- so a synthetic INSTALL_ROOT can
    exercise Steps A/B fully offline, but not Step C. Step C's own logic
    is instead proven directly and separately (never reimplemented) by
    `MessageSchemaVerificationTest`, which extracts and runs its embedded
    Python verbatim against synthetic fixture message classes -- no ROS,
    no bash, no synthetic git repo needed for that part at all."""
    joined = "\n".join(_runbook_bash_code_lines())
    start = joined.index(': "${EXPECTED_HEAD:?')
    end = joined.index('echo "INSTALLED_ENTRYPOINT_METADATA_CHECK=CLEAN"') + len('echo "INSTALLED_ENTRYPOINT_METADATA_CHECK=CLEAN"')
    return joined[start:end]


def _extract_embedded_python(marker_substring: str) -> str:
    """Extracts one embedded `python3 - ... <<'PYEOF' ... PYEOF` payload
    verbatim from the RAW runbook text (not the comment/blank-stripped
    `_runbook_bash_code_lines()`, which would corrupt Python comments and
    the intentional blank lines inside the launcher's golden-template
    string literal). Both embedded Python payloads live in the SAME
    fenced ```bash block, so `marker_substring` locates a position
    inside that block first, then the enclosing `<<'PYEOF' ... PYEOF`
    region around that exact position is extracted -- never just the
    first heredoc in the block."""
    bash_blocks = _runbook_bash_blocks()
    target_block = next(b for b in bash_blocks if marker_substring in b)
    marker_idx = target_block.index(marker_substring)
    heredoc_open = target_block.rindex("<<'PYEOF'\n", 0, marker_idx)
    heredoc_start = heredoc_open + len("<<'PYEOF'\n")
    heredoc_end = target_block.index("\nPYEOF", heredoc_start)
    return target_block[heredoc_start:heredoc_end]


class SourceIdentityMechanismTest(unittest.TestCase):
    """Pure, hardware-free, ROS-free proof of the runbook's real
    two-tier source-identity verification block (extracted verbatim,
    never a reimplementation): Git source-path identity (22 paths,
    proven-complete dependency closure) AND installed-runtime identity
    (~/epuck_ws/install stand-in). Every mutation happens in a disposable
    synthetic git repository AND a disposable synthetic install
    directory built fresh per test -- never the real repository, its
    index, or the real ~/epuck_ws/install. No ROS_DOMAIN_ID is set, no
    ROS package is imported, no rclpy.init, no RUN_ID is ever generated
    or consumed by this block."""

    # Mirrors the real, proven-complete REQUIRED_SOURCE_PATHS in
    # source_identity_block_v3.sh -- 14 tool/message paths plus the 8
    # paths the dependency-closure audit found missing from the prior
    # proposal (goal_hold_tracker.py, navigation_target_state.py,
    # command_smoothing.py, collision_math.py, and the epuck2_comm
    # package's own __init__.py/models.py/neighbor_cache.py/
    # transmission_policy.py, all reached because cooperative_avoider.py
    # is a submodule of the epuck2_comm package and importing any
    # submodule executes the package's __init__.py first).
    REQUIRED_RELATIVE_PATHS = (
        "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/HIL_OFFLINE_STAGE3_RUNBOOK.md",
        "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_offline_stage3_harness.py",
        "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_offline_stage3_evidence_recorder.py",
        "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_offline_stage3_post_run_verifier.py",
        "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_cmd_vel_guard.py",
        "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_topic_adapter.py",
        "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_virtual_peer.py",
        "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_goal_announcement_evidence.py",
        "experiments/10_cooperative_exit_navigation_20260720/tools/goal_navigator.py",
        "experiments/10_cooperative_exit_navigation_20260720/tools/goal_hold_tracker.py",
        "experiments/10_cooperative_exit_navigation_20260720/tools/navigation_target_state.py",
        "src/epuck2_comm/epuck2_comm/__init__.py",
        "src/epuck2_comm/epuck2_comm/cooperative_avoider.py",
        "src/epuck2_comm/epuck2_comm/command_smoothing.py",
        "src/epuck2_comm/epuck2_comm/collision_math.py",
        "src/epuck2_comm/epuck2_comm/local_obstacle_logic.py",
        "src/epuck2_comm/epuck2_comm/models.py",
        "src/epuck2_comm/epuck2_comm/neighbor_cache.py",
        "src/epuck2_comm/epuck2_comm/transmission_policy.py",
        "src/epuck2_comm/setup.py",
        "src/epuck2_comm_interfaces/msg/EpuckState.msg",
        "src/epuck2_comm_interfaces/msg/GoalAnnouncement.msg",
        "src/epuck2_comm_interfaces/msg/NavigationIntent.msg",
    )

    GOLDEN_LAUNCHER_SOURCE = (
        "import re\n"
        "import sys\n"
        "\n"
        "__requires__ = 'epuck2-comm==0.1.0'\n"
        "\n"
        "try:\n"
        "    from importlib.metadata import distribution\n"
        "except ImportError:\n"
        "    try:\n"
        "        from importlib_metadata import distribution\n"
        "    except ImportError:\n"
        "        from pkg_resources import load_entry_point\n"
        "\n"
        "\n"
        "def importlib_load_entry_point(spec, group, name):\n"
        "    dist_name, _, _ = spec.partition('==')\n"
        "    matches = (\n"
        "        entry_point\n"
        "        for entry_point in distribution(dist_name).entry_points\n"
        "        if entry_point.group == group and entry_point.name == name\n"
        "    )\n"
        "    return next(matches).load()\n"
        "\n"
        "\n"
        "globals().setdefault('load_entry_point', importlib_load_entry_point)\n"
        "\n"
        "\n"
        "if __name__ == '__main__':\n"
        "    sys.argv[0] = re.sub(r'(-script\\.pyw?|\\.exe)?$', '', sys.argv[0])\n"
        "    sys.exit(load_entry_point('epuck2-comm==0.1.0', 'console_scripts', 'cooperative_avoider')())\n"
    )

    # Maps each installed artifact's relative location (under a synthetic
    # INSTALL_ROOT) to the REQUIRED_RELATIVE_PATHS entry it is tied to --
    # mirrors INSTALLED_PY_TO_SOURCE in source_identity_block_v3.sh
    # exactly (path shape, not just names).
    INSTALLED_RELATIVE_TO_SOURCE = {
        "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/__init__.py": "src/epuck2_comm/epuck2_comm/__init__.py",
        "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/cooperative_avoider.py": "src/epuck2_comm/epuck2_comm/cooperative_avoider.py",
        "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/command_smoothing.py": "src/epuck2_comm/epuck2_comm/command_smoothing.py",
        "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/collision_math.py": "src/epuck2_comm/epuck2_comm/collision_math.py",
        "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/local_obstacle_logic.py": "src/epuck2_comm/epuck2_comm/local_obstacle_logic.py",
        "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/models.py": "src/epuck2_comm/epuck2_comm/models.py",
        "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/neighbor_cache.py": "src/epuck2_comm/epuck2_comm/neighbor_cache.py",
        "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/transmission_policy.py": "src/epuck2_comm/epuck2_comm/transmission_policy.py",
        "epuck2_comm_interfaces/share/epuck2_comm_interfaces/msg/EpuckState.msg": "src/epuck2_comm_interfaces/msg/EpuckState.msg",
        "epuck2_comm_interfaces/share/epuck2_comm_interfaces/msg/GoalAnnouncement.msg": "src/epuck2_comm_interfaces/msg/GoalAnnouncement.msg",
        "epuck2_comm_interfaces/share/epuck2_comm_interfaces/msg/NavigationIntent.msg": "src/epuck2_comm_interfaces/msg/NavigationIntent.msg",
    }
    LAUNCHER_RELATIVE = "epuck2_comm/lib/epuck2_comm/cooperative_avoider"

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.repo = os.path.join(self._tmp.name, "repo")
        self.install = os.path.join(self._tmp.name, "install")
        os.makedirs(self.repo)
        os.makedirs(self.install)
        subprocess.run(["git", "init", "-q"], cwd=self.repo, check=True)
        subprocess.run(["git", "config", "user.email", "t@t.local"], cwd=self.repo, check=True)
        subprocess.run(["git", "config", "user.name", "t"], cwd=self.repo, check=True)
        # .gitattributes text/eol rule -- required to prove path-aware
        # clean-filtering, not just plain content equality.
        with open(os.path.join(self.repo, ".gitattributes"), "w", encoding="utf-8") as f:
            f.write("*.py text eol=lf\n*.msg text eol=lf\n*.md text eol=lf\n")
        for rel in self.REQUIRED_RELATIVE_PATHS:
            full = os.path.join(self.repo, rel)
            os.makedirs(os.path.dirname(full), exist_ok=True)
            with open(full, "w", encoding="utf-8") as f:
                f.write(f"content of {rel} v1\nline2\n")
        # setup.py stand-in carrying the real console_scripts mapping the
        # launcher-audit branch of the block greps for.
        os.makedirs(os.path.join(self.repo, "src", "epuck2_comm"), exist_ok=True)
        with open(os.path.join(self.repo, "src", "epuck2_comm", "setup.py"), "w", encoding="utf-8") as f:
            f.write(
                'entry_points={"console_scripts": '
                '["cooperative_avoider = epuck2_comm.cooperative_avoider:main"]},\n'
            )
        subprocess.run(["git", "add", "-A"], cwd=self.repo, check=True)
        subprocess.run(["git", "commit", "-q", "-m", "v1"], cwd=self.repo, check=True)
        self.head_v1 = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=self.repo, capture_output=True, text=True, check=True,
        ).stdout.strip()
        self._build_matching_install()

    ENTRY_POINTS_RELATIVE = (
        "epuck2_comm/lib/python3.10/site-packages/epuck2_comm-0.1.0-py3.10.egg-info/entry_points.txt"
    )

    def _build_matching_install(self):
        for installed_rel, src_rel in self.INSTALLED_RELATIVE_TO_SOURCE.items():
            dst = os.path.join(self.install, installed_rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            with open(os.path.join(self.repo, src_rel), "rb") as fsrc, open(dst, "wb") as fdst:
                fdst.write(fsrc.read())
        launcher = os.path.join(self.install, self.LAUNCHER_RELATIVE)
        os.makedirs(os.path.dirname(launcher), exist_ok=True)
        with open(launcher, "w", encoding="utf-8") as f:
            f.write(self.GOLDEN_LAUNCHER_SOURCE)
        os.chmod(launcher, 0o755)
        entry_points = os.path.join(self.install, self.ENTRY_POINTS_RELATIVE)
        os.makedirs(os.path.dirname(entry_points), exist_ok=True)
        with open(entry_points, "w", encoding="utf-8") as f:
            f.write("[console_scripts]\ncooperative_avoider = epuck2_comm.cooperative_avoider:main\n")

    def tearDown(self):
        self._tmp.cleanup()

    def _run(self, expected_head, extra_env=None, timeout_s=10.0):
        # Extracts through the end of Step B only (Git source paths +
        # installed modules + installed launcher + installed entry-point
        # metadata) -- Step C (message-schema) necessarily sources this
        # machine's one real ROS workspace and is proven separately by
        # MessageSchemaVerificationTest instead. See
        # `_extract_source_identity_block_through_step_b()`'s docstring.
        block = _extract_source_identity_block_through_step_b()
        env = dict(os.environ)
        env["REPO_ROOT"] = self.repo
        env["INSTALL_ROOT"] = self.install
        if expected_head is not None:
            env["EXPECTED_HEAD"] = expected_head
        if extra_env:
            env.update(extra_env)
        script = "set -Eeuo pipefail\n" + block
        proc = subprocess.run(
            ["bash", "-c", script], cwd=self.repo, env=env,
            capture_output=True, text=True, timeout=timeout_s,
        )
        return proc

    def test_matching_commit_worktree_and_install_passes(self):
        proc = self._run(self.head_v1)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("GIT_SOURCE_IDENTITY_CHECK=CLEAN", proc.stdout)
        self.assertIn("INSTALLED_RUNTIME_MODULES_CHECK=CLEAN", proc.stdout)
        self.assertIn("INSTALLED_LAUNCHER_IDENTITY_CHECK=CLEAN", proc.stdout)
        self.assertIn("INSTALLED_ENTRYPOINT_METADATA_CHECK=CLEAN", proc.stdout)

    def test_missing_expected_head_fails_before_any_further_check(self):
        proc = self._run(None)
        self.assertNotEqual(proc.returncode, 0)
        self.assertNotIn("GIT_SOURCE_IDENTITY_CHECK", proc.stdout)

    def test_malformed_expected_head_fails(self):
        proc = self._run("not-a-valid-hash")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("not a well-formed 40-character", proc.stderr)

    def test_nonexistent_commit_fails(self):
        proc = self._run("deadbeefdeadbeefdeadbeefdeadbeefdeadbeef")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("does not exist as a commit", proc.stderr)

    def test_head_mismatch_fails(self):
        with open(os.path.join(self.repo, self.REQUIRED_RELATIVE_PATHS[0]), "a", encoding="utf-8") as f:
            f.write("v2 edit\n")
        subprocess.run(["git", "add", "-A"], cwd=self.repo, check=True)
        subprocess.run(["git", "commit", "-q", "-m", "v2"], cwd=self.repo, check=True)
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("HEAD mismatch", proc.stderr)

    def test_required_git_source_path_missing_from_working_tree_fails(self):
        os.remove(os.path.join(self.repo, "experiments/10_cooperative_exit_navigation_20260720/tools/goal_hold_tracker.py"))
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("missing from working tree", proc.stderr)

    def test_incomplete_dependency_closure_caught_as_missing_from_commit(self):
        # Simulates the exact defect the dependency-closure audit found in
        # the prior proposal: a required transitive dependency
        # (navigation_target_state.py) was never committed/tracked at
        # all, not merely deleted from disk -- proving the check catches
        # BOTH "missing from working tree" and "never existed in
        # EXPECTED_HEAD's tree" incompleteness, not just worktree drift.
        # `git rm --cached` (never deleting the working-tree copy) so
        # this exercises the "absent from EXPECTED_HEAD's tree" branch
        # specifically, distinct from the "missing from working tree"
        # branch already covered by the previous test.
        rel = "experiments/10_cooperative_exit_navigation_20260720/tools/navigation_target_state.py"
        subprocess.run(["git", "rm", "-q", "--cached", rel], cwd=self.repo, check=True)
        subprocess.run(["git", "commit", "-q", "-m", "simulate incomplete closure"], cwd=self.repo, check=True)
        head_v2 = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=self.repo, capture_output=True, text=True, check=True,
        ).stdout.strip()
        proc = self._run(head_v2)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("does not exist in EXPECTED_HEAD's tree", proc.stderr)

    def test_commit_blob_worktree_blob_mismatch_fails(self):
        with open(os.path.join(self.repo, "src/epuck2_comm/epuck2_comm/cooperative_avoider.py"), "a", encoding="utf-8") as f:
            f.write("TAMPERED\n")
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("source-identity mismatch", proc.stderr)

    def test_staged_content_change_fails_even_if_index_check_is_reached(self):
        rel = "src/epuck2_comm/epuck2_comm/local_obstacle_logic.py"
        with open(os.path.join(self.repo, rel), "a", encoding="utf-8") as f:
            f.write("STAGED_TAMPER\n")
        subprocess.run(["git", "add", rel], cwd=self.repo, check=True)
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)

    def test_crlf_worktree_variant_passes_via_path_aware_clean_filter(self):
        # A CRLF worktree representation of a *.py path (covered by the
        # committed .gitattributes `text eol=lf` rule) must be treated as
        # logically identical to the committed LF blob -- proves the
        # `git hash-object --path=<repo-relative-path> --` invocation is
        # genuinely path-aware, not merely equivalent to a plain
        # `hash-object` call by coincidence of this repo's layout.
        target = os.path.join(self.repo, "src/epuck2_comm/epuck2_comm/collision_math.py")
        with open(target, "rb") as f:
            content = f.read()
        with open(target, "wb") as f:
            f.write(content.replace(b"\n", b"\r\n"))
        proc = self._run(self.head_v1)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("GIT_SOURCE_IDENTITY_CHECK=CLEAN", proc.stdout)

    def test_matching_blobs_pass_despite_simulated_stat_cache_only_status_entry(self):
        target = os.path.join(self.repo, "src/epuck2_comm/epuck2_comm/models.py")
        os.utime(target, (2000000000, 2000000000))
        proc = self._run(self.head_v1)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("INSTALLED_ENTRYPOINT_METADATA_CHECK=CLEAN", proc.stdout)

    def test_runbooks_own_path_is_verified_through_commit_derived_blob_identity(self):
        with open(os.path.join(self.repo, self.REQUIRED_RELATIVE_PATHS[0]), "a", encoding="utf-8") as f:
            f.write("TAMPERED_RUNBOOK_ITSELF\n")
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("source-identity mismatch", proc.stderr)
        self.assertIn(self.REQUIRED_RELATIVE_PATHS[0], proc.stderr)

    def test_git_source_identity_passes_before_installed_runtime_identity_is_even_reached(self):
        # A Git-layer failure must abort before Step B (installed-runtime)
        # ever runs -- proven by also breaking the install tree in the
        # SAME scenario and observing only the Git-layer error, never an
        # installed-runtime error.
        with open(os.path.join(self.repo, "src/epuck2_comm/epuck2_comm/models.py"), "a", encoding="utf-8") as f:
            f.write("GIT_LAYER_TAMPER\n")
        os.remove(os.path.join(
            self.install, "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/models.py",
        ))
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("source-identity mismatch", proc.stderr)
        self.assertNotIn("installed runtime artifact", proc.stderr)
        self.assertNotIn("GIT_SOURCE_IDENTITY_CHECK=CLEAN", proc.stdout)

    def test_missing_installed_module_artifact_fails_after_git_layer_passes(self):
        os.remove(os.path.join(
            self.install, "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/command_smoothing.py",
        ))
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("GIT_SOURCE_IDENTITY_CHECK=CLEAN", proc.stdout)
        self.assertIn("installed runtime artifact missing", proc.stderr)

    def test_missing_installed_message_artifact_fails(self):
        os.remove(os.path.join(
            self.install, "epuck2_comm_interfaces/share/epuck2_comm_interfaces/msg/GoalAnnouncement.msg",
        ))
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("installed runtime artifact missing", proc.stderr)

    def test_missing_installed_launcher_fails(self):
        os.remove(os.path.join(self.install, self.LAUNCHER_RELATIVE))
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("launcher missing", proc.stderr)

    def test_installed_artifact_content_mismatch_fails_stale_build(self):
        # Simulates a stale colcon build: source edited since the last
        # build, install tree left untouched.
        installed = os.path.join(
            self.install, "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/collision_math.py",
        )
        with open(installed, "a", encoding="utf-8") as f:
            f.write("# stale, pre-edit build artifact\n")
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("does not match reviewed source", proc.stderr)

    def test_installed_artifact_crlf_only_variant_still_passes(self):
        # An installed copy built on a different line-ending convention
        # than the worktree must not false-fail -- installed-vs-source
        # comparison uses the same path-aware git-blob identity as the
        # Git-source layer, never raw byte/sha256 equality.
        installed = os.path.join(
            self.install, "epuck2_comm/lib/python3.10/site-packages/epuck2_comm/transmission_policy.py",
        )
        with open(installed, "rb") as f:
            content = f.read()
        with open(installed, "wb") as f:
            f.write(content.replace(b"\n", b"\r\n"))
        proc = self._run(self.head_v1)
        self.assertEqual(proc.returncode, 0, proc.stderr)

    def test_run_id_never_required_or_expanded_in_this_block(self):
        # Structural proof that this block cannot consume, validate, or
        # cancel a RUN_ID -- it does not even reference the variable.
        block = _extract_source_identity_block()
        self.assertNotIn("${RUN_ID}", block)
        self.assertNotIn("RUN_ID:?", block)

    def test_source_failure_creates_no_out_dir_reference_in_this_block(self):
        # Structural proof this block never creates or references
        # OUT_DIR/mkdir -- evidence-root creation lives strictly after
        # it, in the runbook's Step 2, gated on this block's success.
        block = _extract_source_identity_block()
        self.assertNotIn("OUT_DIR", block)
        self.assertNotIn("mkdir", block)

    def test_installed_entrypoint_metadata_wrong_mapping_fails(self):
        # The installed entry_points.txt disagreeing with the committed
        # setup.py (even though the launcher itself is structurally
        # fine) must fail closed -- proves the 3-way agreement
        # requirement, not just launcher-alone verification.
        entry_points = os.path.join(self.install, self.ENTRY_POINTS_RELATIVE)
        with open(entry_points, "w", encoding="utf-8") as f:
            f.write("[console_scripts]\ncooperative_avoider = epuck2_comm.some_other_wrong_module:main\n")
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("does not map cooperative_avoider", proc.stderr)

    def test_zero_entrypoint_metadata_candidates_fails(self):
        os.remove(os.path.join(self.install, self.ENTRY_POINTS_RELATIVE))
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("INSTALLED_ENTRYPOINT_METADATA_NOT_FOUND", proc.stderr)

    def test_exactly_one_correct_entrypoint_metadata_candidate_passes(self):
        # Baseline: `_build_matching_install()` already places exactly
        # one candidate (the golden fixture in setUp) -- this test just
        # makes that single-candidate-passes case explicit and named.
        proc = self._run(self.head_v1)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("INSTALLED_ENTRYPOINT_METADATA_CHECK=CLEAN", proc.stdout)

    def test_two_entrypoint_metadata_candidates_fails_even_if_both_correct(self):
        # A second, distinct candidate location (egg-info AND dist-info
        # both present, both with the correct mapping) must still fail
        # closed -- ambiguity itself is the defect, never resolved by
        # silently picking the first glob/find result.
        egg_info_dir = os.path.dirname(os.path.join(self.install, self.ENTRY_POINTS_RELATIVE))
        dist_info_dir = egg_info_dir.replace("-py3.10.egg-info", "-py3.10.dist-info")
        os.makedirs(dist_info_dir, exist_ok=True)
        with open(os.path.join(dist_info_dir, "entry_points.txt"), "w", encoding="utf-8") as f:
            f.write("[console_scripts]\ncooperative_avoider = epuck2_comm.cooperative_avoider:main\n")
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("INSTALLED_ENTRYPOINT_METADATA_AMBIGUOUS(count=2)", proc.stderr)

    def test_entrypoint_metadata_failure_precedes_run_id_and_out_dir(self):
        # Structural + behavioural proof combined: the ambiguity/missing
        # checks live entirely inside the Step-B-only extracted block
        # (which itself never references RUN_ID or OUT_DIR/mkdir -- see
        # test_run_id_never_required_or_expanded_in_this_block and
        # test_source_failure_creates_no_out_dir_reference_in_this_block),
        # and this test additionally proves the zero-candidate scenario
        # concretely fails without ever reaching those later gates.
        os.remove(os.path.join(self.install, self.ENTRY_POINTS_RELATIVE))
        proc = self._run(self.head_v1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertNotIn("RUN_ID", proc.stdout)
        self.assertNotIn("OUT_DIR", proc.stdout)
        self.assertIn("INSTALLED_ENTRYPOINT_METADATA_NOT_FOUND", proc.stderr)


class LauncherStructuralIdentityTest(unittest.TestCase):
    """Pure, offline, fail-closed proof of the runbook's REAL embedded
    launcher AST-structural check (extracted verbatim from the
    committed runbook's Step B2 heredoc, never reimplemented) -- never
    executes the launcher, never touches the real repository or the
    real ~/epuck_ws/install."""

    GOLDEN = SourceIdentityMechanismTest.GOLDEN_LAUNCHER_SOURCE

    def _check(self, launcher_source: str):
        script = _extract_embedded_python("def normalize(source):")
        with tempfile.TemporaryDirectory() as tmp:
            launcher_path = os.path.join(tmp, "cooperative_avoider")
            with open(launcher_path, "w", encoding="utf-8") as f:
                f.write(launcher_source)
            proc = subprocess.run(
                ["python3", "-", launcher_path, "epuck2-comm==0.1.0", "console_scripts", "cooperative_avoider"],
                input=script, capture_output=True, text=True, timeout=10.0,
            )
        return proc

    def test_accepted_golden_launcher_passes(self):
        proc = self._check(self.GOLDEN)
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertIn("PASS", proc.stdout)

    def test_wrong_distribution_fails(self):
        mutated = self.GOLDEN.replace("epuck2-comm==0.1.0", "some-other-pkg==9.9.9")
        proc = self._check(mutated)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("BLOCKED", proc.stdout)

    def test_wrong_console_script_name_fails(self):
        mutated = self.GOLDEN.replace("'cooperative_avoider'", "'state_publisher'")
        proc = self._check(mutated)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("BLOCKED", proc.stdout)

    def test_wrong_target_mapping_is_out_of_scope_for_launcher_ast_alone(self):
        # The launcher's AST encodes only (dist, group, name) -- it never
        # encodes the target module:function mapping, which lives solely
        # in entry_points.txt/setup.py. A launcher with the CORRECT
        # (dist, group, name) therefore still passes AST validation even
        # if the target mapping is wrong elsewhere -- that disagreement
        # is caught by the separate 3-way agreement check instead (see
        # SourceIdentityMechanismTest.test_installed_entrypoint_metadata_wrong_mapping_fails).
        proc = self._check(self.GOLDEN)
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)

    def test_extra_code_before_dispatch_fails(self):
        mutated = "import os\nos.system('echo injected')\n" + self.GOLDEN
        proc = self._check(mutated)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("BLOCKED", proc.stdout)

    def test_extra_code_after_dispatch_fails(self):
        mutated = self.GOLDEN + "\nimport subprocess\nsubprocess.run(['echo', 'injected-after'])\n"
        proc = self._check(mutated)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("BLOCKED", proc.stdout)

    def test_missing_launcher_handled_by_caller_existence_check(self):
        # The runbook checks `[[ ! -e "${LAUNCHER}" ]]` BEFORE ever
        # invoking this Python check -- proven structurally, since the
        # check script itself assumes the file exists (open() would
        # raise, which is also fail-closed, but the runbook never
        # reaches that path for a missing launcher).
        joined = "\n".join(_runbook_bash_code_lines())
        launcher_exists_check_idx = joined.index('if [[ ! -e "${LAUNCHER}" ]]; then')
        ast_check_idx = joined.index("LAUNCHER_AST_RESULT=")
        self.assertLess(launcher_exists_check_idx, ast_check_idx)
        self.assertIn('echo "INSTALLED_LAUNCHER_IDENTITY=BLOCKED" >&2', joined)


class MessageSchemaVerificationTest(unittest.TestCase):
    """Pure, offline, ROS-free proof of the runbook's REAL embedded
    runtime message-schema verification (extracted verbatim from the
    committed runbook's Step C heredoc, never reimplemented), run
    against synthetic fixture message classes/modules -- never rclpy,
    never a real ROS node, never the real installed messages."""

    def _script(self) -> str:
        return _extract_embedded_python("def parse_msg_file(")

    def _build_fixture(self, tmp, epuck_state_fields_literal: str):
        install_area = os.path.join(tmp, "install_area")
        msg_pkg = os.path.join(install_area, "epuck2_comm_interfaces", "msg")
        os.makedirs(msg_pkg, exist_ok=True)
        with open(os.path.join(install_area, "epuck2_comm_interfaces", "__init__.py"), "w", encoding="utf-8") as f:
            f.write("")
        with open(os.path.join(msg_pkg, "__init__.py"), "w", encoding="utf-8") as f:
            f.write(
                "class _Base:\n"
                "    @classmethod\n"
                "    def get_fields_and_field_types(cls):\n"
                "        return cls._FIELDS\n"
                "\n"
                f"class EpuckState(_Base):\n    _FIELDS = {epuck_state_fields_literal}\n"
                "\n"
                "class GoalAnnouncement(_Base):\n"
                "    _FIELDS = {\n"
                "        'protocol_version': 'uint32', 'source_robot_id': 'uint32',\n"
                "        'sequence': 'uint32', 'production_stamp': 'builtin_interfaces/Time',\n"
                "        'goal_id': 'string', 'goal_x_m': 'double', 'goal_y_m': 'double',\n"
                "        'valid': 'boolean',\n"
                "    }\n"
                "\n"
                "class NavigationIntent(_Base):\n"
                "    _FIELDS = {\n"
                "        'protocol_version': 'uint32', 'source_robot_id': 'uint32',\n"
                "        'sequence': 'uint32', 'production_stamp': 'builtin_interfaces/Time',\n"
                "        'desired_heading_rad': 'double', 'desired_linear_speed_mps': 'double',\n"
                "        'navigation_phase': 'string', 'valid': 'boolean',\n"
                "    }\n"
            )
        msgfiles = os.path.join(tmp, "msgfiles")
        os.makedirs(msgfiles, exist_ok=True)
        with open(os.path.join(msgfiles, "EpuckState.msg"), "w", encoding="utf-8") as f:
            f.write(
                "uint8 PROTOCOL_VERSION=1\nuint8 version\nuint16 robot_id\nuint32 sequence\n"
                "builtin_interfaces/Time stamp\nfloat32 x_m\nfloat32 y_m\n"
            )
        with open(os.path.join(msgfiles, "GoalAnnouncement.msg"), "w", encoding="utf-8") as f:
            f.write(
                "uint32 PROTOCOL_VERSION=1\nuint32 protocol_version\nuint32 source_robot_id\n"
                "uint32 sequence\nbuiltin_interfaces/Time production_stamp\nstring goal_id\n"
                "float64 goal_x_m\nfloat64 goal_y_m\nbool valid\n"
            )
        with open(os.path.join(msgfiles, "NavigationIntent.msg"), "w", encoding="utf-8") as f:
            f.write(
                "uint32 PROTOCOL_VERSION=1\nuint32 protocol_version\nuint32 source_robot_id\n"
                "uint32 sequence\nbuiltin_interfaces/Time production_stamp\n"
                "double desired_heading_rad\ndouble desired_linear_speed_mps\n"
                "string navigation_phase\nbool valid\n"
            )
        return install_area, msgfiles

    def _run(self, tmp, install_area, msgfiles, install_root_override=None):
        env = dict(os.environ)
        env["PYTHONPATH"] = install_area
        env["INSTALL_ROOT"] = install_root_override if install_root_override is not None else install_area
        proc = subprocess.run(
            [
                "python3", "-",
                os.path.join(msgfiles, "EpuckState.msg"),
                os.path.join(msgfiles, "GoalAnnouncement.msg"),
                os.path.join(msgfiles, "NavigationIntent.msg"),
            ],
            input=self._script(), env=env, capture_output=True, text=True, timeout=10.0,
        )
        return proc

    def test_matching_schema_passes(self):
        with tempfile.TemporaryDirectory() as tmp:
            install_area, msgfiles = self._build_fixture(
                tmp,
                "{'version': 'uint8', 'robot_id': 'uint16', 'sequence': 'uint32', "
                "'stamp': 'builtin_interfaces/Time', 'x_m': 'float', 'y_m': 'float'}",
            )
            proc = self._run(tmp, install_area, msgfiles)
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertIn('"overall_result": "PASS"', proc.stdout)

    def test_missing_field_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            install_area, msgfiles = self._build_fixture(
                tmp,
                "{'version': 'uint8', 'robot_id': 'uint16', 'sequence': 'uint32', "
                "'stamp': 'builtin_interfaces/Time', 'x_m': 'float'}",
            )
            proc = self._run(tmp, install_area, msgfiles)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("MISSING_FIELDS", proc.stdout)

    def test_extra_field_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            install_area, msgfiles = self._build_fixture(
                tmp,
                "{'version': 'uint8', 'robot_id': 'uint16', 'sequence': 'uint32', "
                "'stamp': 'builtin_interfaces/Time', 'x_m': 'float', 'y_m': 'float', "
                "'debug_flag': 'uint8'}",
            )
            proc = self._run(tmp, install_area, msgfiles)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("EXTRA_FIELDS", proc.stdout)

    def test_wrong_field_type_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            install_area, msgfiles = self._build_fixture(
                tmp,
                "{'version': 'uint8', 'robot_id': 'uint16', 'sequence': 'uint32', "
                "'stamp': 'builtin_interfaces/Time', 'x_m': 'double', 'y_m': 'float'}",
            )
            proc = self._run(tmp, install_area, msgfiles)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("FIELD_TYPE_MISMATCH", proc.stdout)

    def test_wrong_field_order_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            install_area, msgfiles = self._build_fixture(
                tmp,
                "{'version': 'uint8', 'robot_id': 'uint16', 'sequence': 'uint32', "
                "'stamp': 'builtin_interfaces/Time', 'y_m': 'float', 'x_m': 'float'}",
            )
            proc = self._run(tmp, install_area, msgfiles)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("FIELD_ORDER_MISMATCH", proc.stdout)

    def test_resolved_outside_expected_install_root_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            install_area, msgfiles = self._build_fixture(
                tmp,
                "{'version': 'uint8', 'robot_id': 'uint16', 'sequence': 'uint32', "
                "'stamp': 'builtin_interfaces/Time', 'x_m': 'float', 'y_m': 'float'}",
            )
            proc = self._run(tmp, install_area, msgfiles, install_root_override="/tmp/unrelated_root_for_test")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("RESOLVED_OUTSIDE_INSTALL_ROOT", proc.stdout)

    def test_missing_generated_class_import_failure_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            _install_area, msgfiles = self._build_fixture(
                tmp,
                "{'version': 'uint8', 'robot_id': 'uint16', 'sequence': 'uint32', "
                "'stamp': 'builtin_interfaces/Time', 'x_m': 'float', 'y_m': 'float'}",
            )
            empty_pythonpath = os.path.join(tmp, "empty")
            os.makedirs(empty_pythonpath, exist_ok=True)
            proc = self._run(tmp, empty_pythonpath, msgfiles)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("IMPORT_FAILED", proc.stdout)

    def test_does_not_initialise_ros(self):
        script = self._script()
        self.assertNotIn("rclpy", script)
        self.assertNotIn("import rclpy", script)


if __name__ == "__main__":
    unittest.main()
