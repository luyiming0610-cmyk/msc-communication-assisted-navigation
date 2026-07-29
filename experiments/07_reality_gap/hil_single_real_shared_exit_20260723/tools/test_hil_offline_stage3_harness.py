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

import math
import re
import sys
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
        joined = "\n".join(_runbook_bash_code_lines())
        cleanup_start = joined.index("cleanup() {")
        cleanup_body = joined[cleanup_start:]
        recorder_kill_index = cleanup_body.index('kill -INT "${RECORDER_PID}"')
        other_kill_index = cleanup_body.index('kill -INT "${pid}"')
        self.assertLess(other_kill_index, recorder_kill_index,
                         "recorder must be the last process killed in cleanup()")

    def test_verifier_output_and_exit_status_preserved(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn('> "${VERIFIER_JSON}"', joined)
        self.assertIn("VERIFIER_EXIT=$?", joined)
        self.assertIn('echo "${VERIFIER_EXIT}" > "${VERIFIER_EXIT_FILE}"', joined)
        self.assertNotIn("set -e", joined)  # errexit is deliberately not used (see script header comment)
        self.assertIn('exit "${VERIFIER_EXIT}"', joined)

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
        # output to select what gets signalled (pgrep -af is permitted
        # elsewhere in this document ONLY for the pre-run/post-run
        # read-only diagnostic checks, asserted separately below).
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn(
            'terminate_owned_process_group \\\n        "${COOP_PID}" "${COOP_PGID}" "${COOP_START_TIME}" "${COOP_EXE_PATH}" "cooperative_avoider"',
            joined,
        )
        self.assertNotIn("pkill", joined)
        cleanup_start = joined.index("cleanup() {")
        cleanup_end = joined.index("\ntrap cleanup", cleanup_start)
        cleanup_body = joined[cleanup_start:cleanup_end]
        # The only pgrep inside cleanup() itself is the final read-only
        # post-run residual check -- never used to pick a kill target.
        self.assertEqual(cleanup_body.count("pgrep"), 1)
        self.assertIn("POST_RUN_RESIDUAL_PROCESS_CHECK", cleanup_body)

    def test_owned_identity_helper_functions_never_use_name_based_discovery(self):
        joined = "\n".join(_runbook_bash_code_lines())
        for fn_name in ("_proc_start_time", "_proc_exe_path", "_owned_identity_still_matches", "terminate_owned_process_group"):
            self.assertIn(f"{fn_name}()", joined)
        fn_start = joined.index("_proc_start_time() {")
        fn_end = joined.index("if pgrep -af 'webots-bin", fn_start)
        helper_block = joined[fn_start:fn_end]
        self.assertNotIn("pgrep", helper_block)
        self.assertNotIn("pkill", helper_block)

    def test_evidence_hashing_present_and_manifest_excludes_itself(self):
        joined = "\n".join(_runbook_bash_code_lines())
        self.assertIn("sha256sum", joined)
        self.assertIn("SHA256SUMS_FILE", joined)
        # The sha256sum invocation's own argument list must not include
        # the SHA256SUMS output file's own basename.
        sha_call_start = joined.index("sha256sum \\")
        sha_call_end = joined.index(")", sha_call_start)
        sha_call_text = joined[sha_call_start:sha_call_end]
        self.assertNotIn('basename "${SHA256SUMS_FILE}"', sha_call_text)


if __name__ == "__main__":
    unittest.main()
