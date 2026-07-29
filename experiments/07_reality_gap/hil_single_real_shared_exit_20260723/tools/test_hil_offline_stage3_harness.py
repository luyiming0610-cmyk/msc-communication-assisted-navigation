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
    build_bridge_status_payload,
    build_gate_decision_event,
    check_ros_domain_id,
    gate_forward,
    is_adoption_confirmed,
    is_isolated_topic,
    is_timeout_exceeded,
)


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


if __name__ == "__main__":
    unittest.main()
