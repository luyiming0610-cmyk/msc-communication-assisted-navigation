#!/usr/bin/env python3
"""Stage 3 (OFFLINE_INTEGRATION_VALIDATION) hardware-free test-orchestration
harness. Not a physical HIL trial, not a Webots experiment, not a
navigation/guard/virtual-peer algorithm reimplementation.

This module ONLY orchestrates: it publishes deterministic, purely
synthetic test stimuli (own-robot EpuckState, bridge-status,
guard-arm), forwards the virtual peer's real EpuckState through a
controllable, exact-pass-through test gate, tracks a one-way phase
state machine, and sends one duplicate GoalAnnouncement only after the
real (unmodified) navigation logic's own NavigationIntent stream shows
adoption has already occurred. It contains no navigation, guard, or
avoidance decision logic of any kind -- those all remain in
goal_navigator.py, cooperative_avoider.py, and hil_cmd_vel_guard.py,
untouched.

Guard freshness contract this harness satisfies (read directly from
hil_cmd_vel_guard.decide_command(), not assumed):
  - physical_state_timeout_s defaults to 0.5s -- own-robot EpuckState
    must be re-published more often than that, continuously, for the
    whole run. OWN_STATE_PUBLISH_PERIOD_S below (0.1s) is well inside
    that margin.
  - required_validity_flags defaults to
    FLAG_ODOM_VALID | FLAG_IR_VALID | FLAG_TOF_VALID == 7 -- every
    own-state publish below sets exactly that value, never a partial
    or invented one.
  - physical_state_protocol_ok requires msg.version ==
    EpuckState.PROTOCOL_VERSION exactly.
  - virtual_peer_timeout_s defaults to 1.0s -- this is the freshness
    window the peer-state gate (below) uses to create a genuine stale
    interval by simply pausing forwarding, not by killing/restarting
    hil_virtual_peer.py (which would reset its own sequence/announced
    state, an unrelated variable this test must not introduce).
  - decide_command() does not itself require EpuckState.sequence to
    increase -- only the recorder's own evidence quality benefits from
    a monotonically increasing sequence, which this harness still
    provides.

All test-only topics used by this harness live under
/hil_offline_stage3/... -- see HIL_OFFLINE_STAGE3_RUNBOOK.md for the
full topic table. This module never constructs a publisher on any
production topic (/cmd_vel, /cmd_vel_unguarded, /epuck1/state,
/epuck_bridge/status, /hil_guard/arm).
"""
from __future__ import annotations

import time
from enum import Enum


class Stage3Phase(str, Enum):
    INITIALISING = "INITIALISING"
    READY_DISARMED = "READY_DISARMED"
    ANNOUNCEMENT_ADOPTED = "ANNOUNCEMENT_ADOPTED"
    DISARMED_ZERO_CONFIRMED = "DISARMED_ZERO_CONFIRMED"
    ARMED_BOUNDED_CONFIRMED = "ARMED_BOUNDED_CONFIRMED"
    PEER_GATE_CLOSED = "PEER_GATE_CLOSED"
    STALE_ZERO_CONFIRMED = "STALE_ZERO_CONFIRMED"
    PEER_GATE_REOPENED = "PEER_GATE_REOPENED"
    RECOVERY_CONFIRMED = "RECOVERY_CONFIRMED"
    DUPLICATE_REJECTED = "DUPLICATE_REJECTED"
    COMPLETE = "COMPLETE"


PHASE_ORDER = [
    Stage3Phase.INITIALISING,
    Stage3Phase.READY_DISARMED,
    Stage3Phase.ANNOUNCEMENT_ADOPTED,
    Stage3Phase.DISARMED_ZERO_CONFIRMED,
    Stage3Phase.ARMED_BOUNDED_CONFIRMED,
    Stage3Phase.PEER_GATE_CLOSED,
    Stage3Phase.STALE_ZERO_CONFIRMED,
    Stage3Phase.PEER_GATE_REOPENED,
    Stage3Phase.RECOVERY_CONFIRMED,
    Stage3Phase.DUPLICATE_REJECTED,
    Stage3Phase.COMPLETE,
]


class PhaseTransitionError(Exception):
    pass


class PhaseMachine:
    """Pure, rclpy-free, strictly-forward phase tracker. This class does
    not observe ROS state itself -- callers must independently confirm
    the real event/condition for the CURRENT phase (a message received,
    a field value observed, a topic count checked) before calling
    advance(); the machine only enforces that phases occur in the fixed
    order above, exactly once each, never skipped, never reversed."""

    def __init__(self):
        self._index = 0
        self.history: list[tuple[Stage3Phase, float]] = [(PHASE_ORDER[0], time.monotonic())]

    @property
    def phase(self) -> Stage3Phase:
        return PHASE_ORDER[self._index]

    @property
    def is_complete(self) -> bool:
        return self.phase == Stage3Phase.COMPLETE

    def advance(self, expected_current: Stage3Phase) -> Stage3Phase:
        if self.phase != expected_current:
            raise PhaseTransitionError(
                f"phase mismatch: expected current={expected_current.value}, "
                f"actual current={self.phase.value}"
            )
        if self._index >= len(PHASE_ORDER) - 1:
            raise PhaseTransitionError("already at terminal phase COMPLETE, cannot advance further")
        self._index += 1
        self.history.append((self.phase, time.monotonic()))
        return self.phase


class DuplicateOrderingError(Exception):
    """Raised whenever duplicate-announcement publication is requested
    out of the required order. Fail-closed: the caller must not catch
    this and publish anyway."""


class AdoptionCountExceededError(Exception):
    """Raised the instant a second adoption-rising-edge is observed.
    This aborts the run -- an adoption count greater than one is never a
    thing to tolerate or average over."""


class DuplicateAnnouncementController:
    """Pure, rclpy-free enforcement of the duplicate-announcement
    ordering contract. This class does not implement or duplicate any
    GoalAnnouncement acceptance/adoption logic -- that stays entirely in
    goal_navigator.py/NavigationTargetState. It only tracks: how many
    times a genuine adoption rising-edge has been recorded (0 or 1 in
    any correct run), whether the one permitted duplicate has already
    been sent, and whether the run has reached COMPLETE. It enforces
    the ordering itself; it does not rely on any external caller to get
    the order right."""

    def __init__(self):
        self.adoption_count = 0
        self.duplicate_sent = False
        self.run_complete = False

    def record_adoption_event(self) -> None:
        """Call exactly once per observed adoption rising-edge (i.e. the
        transition of is_adoption_confirmed() from False to True, never
        once per tick it stays True)."""
        self.adoption_count += 1
        if self.adoption_count > 1:
            raise AdoptionCountExceededError(
                f"adoption_count={self.adoption_count} exceeds 1 -- aborting run"
            )

    def mark_complete(self) -> None:
        self.run_complete = True

    def authorize_duplicate_publication(self) -> None:
        """Raises DuplicateOrderingError (fail closed, never returns a
        False/None the caller could silently ignore) unless exactly one
        adoption has been recorded, the run has not completed, and no
        duplicate has been sent yet. Returns normally (no value) only
        when publication is authorized, and marks duplicate_sent=True as
        part of authorizing it -- so a second call always fails closed
        even if the first call's actual publish somehow did not happen."""
        if self.run_complete:
            raise DuplicateOrderingError("duplicate publication requested after run COMPLETE")
        if self.duplicate_sent:
            raise DuplicateOrderingError("duplicate publication already sent -- refusing a second one")
        if self.adoption_count != 1:
            raise DuplicateOrderingError(
                f"duplicate publication requested before exactly one adoption was confirmed "
                f"(adoption_count={self.adoption_count})"
            )
        self.duplicate_sent = True


class RunnerTimeoutError(Exception):
    """Raised the instant a per-phase or overall deadline elapses without
    the required condition becoming true. Fails closed -- the runner
    never proceeds to the next phase on a timeout, and never silently
    treats a timeout as success."""


class Stage3AutomaticRunner:
    """Drives the harness through the complete, fixed Stage3Phase
    sequence automatically, using ONLY the harness's own observable
    evidence (adoption confirmation derived from the real, unmodified
    NavigationIntent stream; gate state; and the raw guarded-Twist
    values the harness separately subscribes to purely for numeric
    bound/zero checking -- comparing a number against a declared bound
    is not guard decision logic, which remains exclusively inside
    hil_cmd_vel_guard.decide_command(), untouched). This class contains
    no navigation, GoalAnnouncement-acceptance, guard-decision, or
    virtual-peer motion logic of its own -- it only calls the harness's
    existing public methods (advance_phase, close_gate, open_gate,
    set_arm, request_duplicate_publication) in the one required order,
    waiting for each phase's own confirmable condition via the caller-
    supplied spin_once(), and fails closed with RunnerTimeoutError if a
    per-phase or the overall deadline elapses first.

    The guarded-cmd-vel topic this runner observes may be produced by
    a real hil_cmd_vel_guard.py process (a real, later, separately
    authorised Stage 3 run) or by a synthetic test stimulus (this
    preparation pass) -- the runner itself does not know or care which,
    since it only reads numbers off the topic, never the guard's
    internal state.
    """

    def __init__(
        self, harness, *,
        per_phase_timeout_s: float,
        overall_timeout_s: float,
        test_only_linear_bound_mps: float,
        test_only_angular_bound_rps: float,
        peer_timeout_s: float,
        duplicate_source_robot_id: int,
        duplicate_goal_x_m: float,
        duplicate_goal_y_m: float,
        duplicate_goal_id: str,
    ):
        self.harness = harness
        self.per_phase_timeout_s = per_phase_timeout_s
        self.overall_timeout_s = overall_timeout_s
        self.test_only_linear_bound_mps = test_only_linear_bound_mps
        self.test_only_angular_bound_rps = test_only_angular_bound_rps
        self.peer_timeout_s = peer_timeout_s
        self.duplicate_source_robot_id = duplicate_source_robot_id
        self.duplicate_goal_x_m = duplicate_goal_x_m
        self.duplicate_goal_y_m = duplicate_goal_y_m
        self.duplicate_goal_id = duplicate_goal_id
        self._run_start_monotonic: float | None = None

    def _wait_for(self, condition, spin_once, phase_label: str) -> None:
        phase_deadline = time.monotonic() + self.per_phase_timeout_s
        while True:
            if condition():
                return
            now = time.monotonic()
            if now > phase_deadline:
                raise RunnerTimeoutError(f"per-phase timeout waiting for condition of phase {phase_label}")
            if self._run_start_monotonic is not None and now - self._run_start_monotonic > self.overall_timeout_s:
                raise RunnerTimeoutError(f"overall Stage 3 preparation runner timeout during phase {phase_label}")
            spin_once()

    def _sleep_at_least(self, seconds: float, spin_once) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            spin_once()

    def run(self, spin_once) -> None:
        """spin_once must be a zero-argument callable that lets the ROS
        executor process one round of callbacks (e.g. a short
        executor.spin_once(timeout_sec=...)); this method drives phase
        transitions but never owns or constructs the executor itself."""
        self._run_start_monotonic = time.monotonic()
        h = self.harness

        # INITIALISING -> READY_DISARMED: the harness is already fully
        # constructed (all publishers/subscriptions/timers exist) by the
        # time run() is called, so this transition requires no further
        # external condition.
        h.advance_phase(Stage3Phase.INITIALISING)

        # READY_DISARMED -> ANNOUNCEMENT_ADOPTED: wait for the real,
        # unmodified navigation stream to report adoption.
        self._wait_for(h.adoption_confirmed, spin_once, "READY_DISARMED")
        h.advance_phase(Stage3Phase.READY_DISARMED)

        # ANNOUNCEMENT_ADOPTED -> DISARMED_ZERO_CONFIRMED: while disarmed,
        # the observed guarded command must be exactly zero.
        self._wait_for(h.latest_guarded_command_is_zero, spin_once, "ANNOUNCEMENT_ADOPTED")
        h.advance_phase(Stage3Phase.ANNOUNCEMENT_ADOPTED)

        # DISARMED_ZERO_CONFIRMED -> ARMED_BOUNDED_CONFIRMED: arm, then
        # wait for a bounded (possibly zero) observed guarded command.
        h.set_arm(True)
        self._wait_for(
            lambda: h.latest_guarded_command_within_bounds(
                self.test_only_linear_bound_mps, self.test_only_angular_bound_rps
            ),
            spin_once, "DISARMED_ZERO_CONFIRMED",
        )
        h.advance_phase(Stage3Phase.DISARMED_ZERO_CONFIRMED)

        # ARMED_BOUNDED_CONFIRMED -> PEER_GATE_CLOSED: close the gate --
        # the closing action IS the phase transition's own condition.
        h.close_gate()
        h.advance_phase(Stage3Phase.ARMED_BOUNDED_CONFIRMED)

        # PEER_GATE_CLOSED -> STALE_ZERO_CONFIRMED: the peer timeout must
        # genuinely elapse (never shortcut) before this phase is allowed
        # to be confirmed.
        self._sleep_at_least(self.peer_timeout_s, spin_once)
        h.advance_phase(Stage3Phase.PEER_GATE_CLOSED)

        # STALE_ZERO_CONFIRMED -> PEER_GATE_REOPENED: reopen the gate.
        h.open_gate()
        h.advance_phase(Stage3Phase.STALE_ZERO_CONFIRMED)

        # PEER_GATE_REOPENED -> RECOVERY_CONFIRMED: wait for a fresh
        # post-reopen forwarded state to actually arrive.
        self._wait_for(h.has_fresh_post_reopen_gate_input, spin_once, "PEER_GATE_REOPENED")
        h.advance_phase(Stage3Phase.PEER_GATE_REOPENED)

        # RECOVERY_CONFIRMED -> DUPLICATE_REJECTED: publish the one
        # permitted duplicate, which the harness itself refuses unless
        # exactly one adoption has already been confirmed (enforced by
        # DuplicateAnnouncementController, not by this runner).
        h.request_duplicate_publication(
            self.duplicate_source_robot_id, self.duplicate_goal_x_m,
            self.duplicate_goal_y_m, self.duplicate_goal_id,
        )
        h.advance_phase(Stage3Phase.RECOVERY_CONFIRMED)

        # DUPLICATE_REJECTED -> COMPLETE.
        h.advance_phase(Stage3Phase.DUPLICATE_REJECTED)


def gate_forward(msg, gate_open: bool):
    """Pure exact-pass-through gate. Returns the identical message object
    unchanged when gate_open is True; returns None (nothing to publish)
    when False. Never reads or mutates any field on msg -- this is what
    guarantees protocol_version/source/robot_id/sequence/stamp/pose/
    velocity/validity fields all survive byte-for-byte while the gate is
    open, and that nothing at all is forwarded while it is closed."""
    return msg if gate_open else None


class GateDecision(str, Enum):
    FORWARDED = "FORWARDED"
    REJECTED_GATE_CLOSED = "REJECTED_GATE_CLOSED"


def build_gate_decision_event(
    *,
    gate_epoch: int,
    gate_state: str,
    source_protocol_version: int,
    source_robot_id: int,
    source_sequence: int,
    source_production_stamp_s: float,
    decision: str,
    decision_timestamp_s: float,
    first_source_after_reopen: bool,
    forwarded_destination_topic: str | None,
) -> dict:
    """Pure builder for the gate-owned structured decision event. This is
    created at the gate's own decision point (inside the same callback
    that decides FORWARDED/REJECTED_GATE_CLOSED), never reconstructed
    afterward from two independently-scheduled subscriber streams --
    that is the whole point of this event existing: recorder callbacks
    for the source and guard-input topics can be scheduled/written in
    either order, so cross-topic CSV row order is not causal proof of
    anything, but this one event, emitted synchronously at the decision
    itself, is."""
    return {
        "event_type": "GATE_DECISION",
        "gate_epoch": int(gate_epoch),
        "gate_state": gate_state,
        "source_protocol_version": int(source_protocol_version),
        "source_robot_id": int(source_robot_id),
        "source_sequence": int(source_sequence),
        "source_production_stamp_s": float(source_production_stamp_s),
        "decision": decision,
        "decision_timestamp_s": float(decision_timestamp_s),
        "first_source_after_reopen": bool(first_source_after_reopen),
        "forwarded_destination_topic": forwarded_destination_topic,
    }


# Guard contract constants (see module docstring) -- not invented here,
# copied from hil_cmd_vel_guard.py's own defaults for the sole purpose
# of satisfying that existing, unmodified contract.
OWN_STATE_PUBLISH_PERIOD_S = 0.1
OWN_STATE_REQUIRED_VALIDITY_FLAGS = 7  # FLAG_ODOM_VALID | FLAG_IR_VALID | FLAG_TOF_VALID
BRIDGE_STATUS_PUBLISH_PERIOD_S = 0.5

# Any angular bound value the harness or an operator supplies to the
# guard for this offline run is a test-only numerical input, never a
# physical measurement or calibration result.
TEST_ONLY_SOFTWARE_BOUND_NOT_A_PHYSICAL_LIMIT = "TEST_ONLY_SOFTWARE_BOUND_NOT_A_PHYSICAL_LIMIT"


def build_bridge_status_payload(rx_count: int, connected: bool = True) -> dict:
    """Pure helper -- exact keys read by hil_command_evidence_recorder.py's
    own parse_bridge_status_json() ("connected", "rx_count"), reused here
    verbatim rather than inventing a different schema."""
    return {"connected": bool(connected), "rx_count": int(rx_count)}


def is_adoption_confirmed(navigation_phase: str | None) -> bool:
    """True once the real, unmodified NavigationTargetState.navigation_phase
    (published verbatim inside NavigationIntent by goal_navigator.py's own
    _tick()) has left "SEARCH" for "GO_TO_EXIT" -- which only happens once
    NavigationTargetState.heading_to_final_target becomes True, which for
    a search-mode robot only happens via switched_to_goal (announcement
    adoption) or reaching the final waypoint. Combined with this harness's
    own scripted waypoints/virtual-peer target (never touching the final
    waypoint by construction), "GO_TO_EXIT" here is solely attributable to
    announcement adoption. This function reads an existing, unmodified
    field; it does not reimplement or guess at navigation state."""
    return navigation_phase == "GO_TO_EXIT"


PRODUCTION_TOPICS = frozenset({
    "/cmd_vel",
    "/cmd_vel_unguarded",
    "/epuck1/state",
    "/epuck_bridge/status",
    "/hil_guard/arm",
})


def is_isolated_topic(topic: str) -> bool:
    """A topic is acceptable for Stage 3 use only if it is not a literal
    production topic name AND lives under the /hil_offline_stage3/
    namespace. Both conditions are checked independently -- a topic
    could avoid the production-name blocklist by accident while still
    not being properly namespaced, and vice versa."""
    if topic in PRODUCTION_TOPICS:
        return False
    return topic.startswith("/hil_offline_stage3/")


def is_timeout_exceeded(start_monotonic_s: float, now_monotonic_s: float, max_runtime_s: float) -> bool:
    """Pure bounded-timeout check, no rclpy/timer dependency."""
    return (now_monotonic_s - start_monotonic_s) > max_runtime_s


FORBIDDEN_ROS_DOMAIN_IDS = frozenset({0, 77, 89})
EXPECTED_STAGE3_ROS_DOMAIN_ID = 91


def check_ros_domain_id(domain_id: int) -> tuple[bool, str]:
    """Returns (ok, reason). Fails closed for the default domain (0) and
    every other project-reserved test domain (77 = Stage 2 verification,
    89 = the project's standing pytest-isolation domain from
    src/epuck2_comm/test/conftest.py), and requires the exact sanctioned
    Stage 3 domain (91) for anything claiming to be a real Stage 3 run."""
    if domain_id in FORBIDDEN_ROS_DOMAIN_IDS:
        return False, f"ROS_DOMAIN_ID={domain_id} is reserved/forbidden for Stage 3"
    if domain_id != EXPECTED_STAGE3_ROS_DOMAIN_ID:
        return False, f"ROS_DOMAIN_ID={domain_id} does not match the sanctioned Stage 3 domain {EXPECTED_STAGE3_ROS_DOMAIN_ID}"
    return True, ""


# ---------------------------------------------------------------------
# rclpy wrapper -- only importable/constructed once rclpy and the
# message packages are available, same lazy pattern as every other
# hil_*.py tool in this directory.
# ---------------------------------------------------------------------
def _build_node():
    import argparse
    import json

    import rclpy
    from epuck2_comm_interfaces.msg import EpuckState, GoalAnnouncement, NavigationIntent
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    class HilOfflineStage3Harness(Node):
        def __init__(self, args):
            super().__init__("hil_offline_stage3_harness")
            self.args = args
            self.phase_machine = PhaseMachine()
            self.duplicate_controller = DuplicateAnnouncementController()
            self._own_seq = 0
            self._gate_open = True
            self._gate_epoch = 0
            self._first_source_after_reopen_pending = False
            self._was_adoption_confirmed = False
            self._latest_nav_intent_phase: str | None = None
            self._latest_guarded_cmd_vel: Twist | None = None
            self._latest_gate_input_seq: int | None = None
            self._fresh_post_reopen_gate_input_seen = False
            self._start_monotonic = time.monotonic()

            self.own_state_pub = self.create_publisher(EpuckState, args.own_state_topic, 10)
            self.bridge_status_pub = self.create_publisher(String, args.bridge_status_topic, 10)
            self.arm_pub = self.create_publisher(Bool, args.arm_topic, 10)
            self.phase_event_pub = self.create_publisher(String, args.phase_event_topic, 10)
            self.duplicate_announcement_pub = self.create_publisher(
                GoalAnnouncement, args.goal_announcement_topic, 10
            )
            self.gate_pub = self.create_publisher(EpuckState, args.virtual_peer_guard_input_topic, 20)
            self.gate_decision_pub = self.create_publisher(String, args.gate_decision_topic, 20)

            self.create_subscription(
                EpuckState, args.virtual_peer_source_topic, self._on_virtual_peer_source, 20
            )
            self.create_subscription(NavigationIntent, args.nav_intent_topic, self._on_nav_intent, 10)
            self.create_subscription(Twist, args.guarded_cmd_vel_topic, self._on_guarded_cmd_vel, 10)

            self.create_timer(OWN_STATE_PUBLISH_PERIOD_S, self._publish_own_state)
            self.create_timer(BRIDGE_STATUS_PUBLISH_PERIOD_S, self._publish_bridge_status)
            self.create_timer(0.5, self._check_timeout)

            self.get_logger().info(
                f"HIL_OFFLINE_STAGE3_HARNESS_READY phase={self.phase_machine.phase.value} "
                f"own_state_topic={args.own_state_topic} "
                f"virtual_peer_guard_input_topic={args.virtual_peer_guard_input_topic} "
                f"gate_decision_topic={args.gate_decision_topic}"
            )

        def _publish_own_state(self) -> None:
            self._own_seq += 1
            msg = EpuckState()
            msg.version = EpuckState.PROTOCOL_VERSION
            msg.robot_id = self.args.own_robot_id
            msg.sequence = self._own_seq % (2 ** 32)
            msg.stamp = self.get_clock().now().to_msg()
            msg.source = EpuckState.SOURCE_HARDWARE
            msg.x_m = self.args.own_x_m
            msg.y_m = self.args.own_y_m
            msg.yaw_rad = self.args.own_yaw_rad
            msg.validity_flags = OWN_STATE_REQUIRED_VALIDITY_FLAGS
            self.own_state_pub.publish(msg)

        def _publish_bridge_status(self) -> None:
            payload = build_bridge_status_payload(rx_count=self._own_seq)
            msg = String()
            msg.data = json.dumps(payload)
            self.bridge_status_pub.publish(msg)

        def _publish_phase_event(self, **fields) -> None:
            msg = String()
            msg.data = json.dumps(fields)
            self.phase_event_pub.publish(msg)

        def advance_phase(self, expected_current: "Stage3Phase") -> "Stage3Phase":
            """The only way this harness's phase advances. Wraps
            PhaseMachine.advance() (which already fails closed on any
            out-of-order call) and publishes the new phase as evidence.
            Also marks the duplicate controller complete once COMPLETE
            is reached, so any subsequent duplicate-publication attempt
            fails closed regardless of caller behaviour."""
            new_phase = self.phase_machine.advance(expected_current)
            self._publish_phase_event(phase=new_phase.value)
            if new_phase == Stage3Phase.COMPLETE:
                self.duplicate_controller.mark_complete()
            return new_phase

        def set_arm(self, value: bool) -> None:
            msg = Bool()
            msg.data = bool(value)
            self.arm_pub.publish(msg)

        def close_gate(self) -> None:
            self._gate_open = False
            self._publish_phase_event(gate_open=False)
            self.get_logger().info("HIL_STAGE3_PEER_GATE_STATE gate_open=False")

        def open_gate(self) -> None:
            self._gate_open = True
            self._gate_epoch += 1
            self._first_source_after_reopen_pending = True
            self._fresh_post_reopen_gate_input_seen = False
            self._publish_phase_event(gate_open=True, gate_epoch=self._gate_epoch)
            self.get_logger().info(
                f"HIL_STAGE3_PEER_GATE_STATE gate_open=True gate_epoch={self._gate_epoch}"
            )

        def _stamp_to_seconds(self, stamp) -> float:
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9

        def _on_virtual_peer_source(self, msg) -> None:
            """This is the gate's own decision point: the FORWARDED/
            REJECTED_GATE_CLOSED decision and the structured evidence
            event for it are produced synchronously, right here, in the
            same callback -- never reconstructed afterward by comparing
            this topic's recorded rows against the separate guard-input
            topic's recorded rows, which are written by an independently
            scheduled subscriber callback and carry no causal ordering
            guarantee relative to this one."""
            gate_was_open = self._gate_open
            decision = GateDecision.FORWARDED if gate_was_open else GateDecision.REJECTED_GATE_CLOSED
            is_first_after_reopen = gate_was_open and self._first_source_after_reopen_pending
            if gate_was_open:
                self._first_source_after_reopen_pending = False

            event = build_gate_decision_event(
                gate_epoch=self._gate_epoch,
                gate_state="OPEN" if gate_was_open else "CLOSED",
                source_protocol_version=msg.version,
                source_robot_id=msg.robot_id,
                source_sequence=msg.sequence,
                source_production_stamp_s=self._stamp_to_seconds(msg.stamp),
                decision=decision.value,
                decision_timestamp_s=self._stamp_to_seconds(self.get_clock().now().to_msg()),
                first_source_after_reopen=is_first_after_reopen,
                forwarded_destination_topic=(
                    self.args.virtual_peer_guard_input_topic if decision == GateDecision.FORWARDED else None
                ),
            )
            decision_msg = String()
            decision_msg.data = json.dumps(event)
            self.gate_decision_pub.publish(decision_msg)

            forwarded = gate_forward(msg, gate_was_open)
            if forwarded is not None:
                self.gate_pub.publish(forwarded)
                if is_first_after_reopen:
                    self._fresh_post_reopen_gate_input_seen = True

        def _on_guarded_cmd_vel(self, msg) -> None:
            self._latest_guarded_cmd_vel = msg

        def latest_guarded_command_is_zero(self) -> bool:
            msg = self._latest_guarded_cmd_vel
            if msg is None:
                return False
            return (
                msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.linear.z == 0.0
                and msg.angular.x == 0.0 and msg.angular.y == 0.0 and msg.angular.z == 0.0
            )

        def latest_guarded_command_within_bounds(
            self, linear_bound_mps: float, angular_bound_rps: float
        ) -> bool:
            msg = self._latest_guarded_cmd_vel
            if msg is None:
                return False
            return (
                abs(msg.linear.x) <= linear_bound_mps
                and abs(msg.angular.z) <= angular_bound_rps
            )

        def has_fresh_post_reopen_gate_input(self) -> bool:
            """True only once this harness's own gate has actually
            forwarded a message it marked first_source_after_reopen=true
            for the current epoch -- set exclusively inside
            _on_virtual_peer_source(), never inferred from timing."""
            return self._fresh_post_reopen_gate_input_seen

        def _on_nav_intent(self, msg) -> None:
            self._latest_nav_intent_phase = msg.navigation_phase
            now_confirmed = is_adoption_confirmed(self._latest_nav_intent_phase)
            if now_confirmed and not self._was_adoption_confirmed:
                self.duplicate_controller.record_adoption_event()
                self._publish_phase_event(adoption_confirmed=True)
            self._was_adoption_confirmed = now_confirmed

        def adoption_confirmed(self) -> bool:
            return is_adoption_confirmed(self._latest_nav_intent_phase)

        def request_duplicate_publication(
            self, source_robot_id: int, goal_x_m: float, goal_y_m: float, goal_id: str
        ) -> None:
            """Publishes the duplicate GoalAnnouncement ONLY if
            duplicate_controller.authorize_duplicate_publication() does
            not raise -- i.e. exactly one adoption has already been
            confirmed, the run has not completed, and no duplicate has
            been sent yet. A DuplicateOrderingError/AdoptionCountExceededError
            propagates to the caller and no message is published; this
            is the ordering enforcement itself, not a suggestion a
            caller could bypass."""
            try:
                self.duplicate_controller.authorize_duplicate_publication()
            except (DuplicateOrderingError, AdoptionCountExceededError) as exc:
                self._publish_phase_event(duplicate_rejected=True, guard_blocked_reasons=str(exc))
                raise

            msg = GoalAnnouncement()
            msg.protocol_version = GoalAnnouncement.PROTOCOL_VERSION
            msg.source_robot_id = source_robot_id
            msg.sequence = 999999
            msg.production_stamp = self.get_clock().now().to_msg()
            msg.goal_id = goal_id
            msg.goal_x_m = float(goal_x_m)
            msg.goal_y_m = float(goal_y_m)
            msg.valid = True
            self.duplicate_announcement_pub.publish(msg)
            self._publish_phase_event(duplicate_sent=True)

        def _check_timeout(self) -> None:
            if is_timeout_exceeded(self._start_monotonic, time.monotonic(), self.args.max_runtime_s):
                self.get_logger().error("HIL_OFFLINE_STAGE3_TIMEOUT_ABORT")
                raise SystemExit(1)

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--own-state-topic", required=True)
        parser.add_argument("--bridge-status-topic", required=True)
        parser.add_argument("--arm-topic", required=True)
        parser.add_argument("--phase-event-topic", required=True)
        parser.add_argument("--goal-announcement-topic", required=True)
        parser.add_argument("--virtual-peer-source-topic", required=True)
        parser.add_argument("--virtual-peer-guard-input-topic", required=True)
        parser.add_argument("--gate-decision-topic", required=True)
        parser.add_argument("--nav-intent-topic", required=True)
        parser.add_argument("--guarded-cmd-vel-topic", required=True)
        parser.add_argument("--own-robot-id", type=int, default=1)
        parser.add_argument("--own-x-m", type=float, default=0.0)
        parser.add_argument("--own-y-m", type=float, default=0.0)
        parser.add_argument("--own-yaw-rad", type=float, default=0.0)
        parser.add_argument("--max-runtime-s", type=float, default=60.0)
        parser.add_argument("--auto-run", action="store_true",
                             help="Drive all 11 phases automatically via Stage3AutomaticRunner "
                                  "instead of waiting for an external caller to invoke advance_phase().")
        parser.add_argument("--runner-per-phase-timeout-s", type=float, default=20.0)
        parser.add_argument("--runner-overall-timeout-s", type=float, default=55.0)
        parser.add_argument("--runner-linear-bound-mps", type=float, default=0.3)
        parser.add_argument("--runner-angular-bound-rps", type=float, default=3.0)
        parser.add_argument("--runner-peer-timeout-s", type=float, default=1.2)
        parser.add_argument("--runner-duplicate-source-robot-id", type=int, default=2)
        parser.add_argument("--runner-duplicate-goal-x-m", type=float, default=0.0)
        parser.add_argument("--runner-duplicate-goal-y-m", type=float, default=0.0)
        parser.add_argument("--runner-duplicate-goal-id", type=str, default="hil_offline_stage3_duplicate")
        return parser.parse_args(argv)

    return rclpy, HilOfflineStage3Harness, parse_args


def main(argv=None):
    import sys

    rclpy, HilOfflineStage3Harness, parse_args = _build_node()
    args = parse_args(argv if argv is not None else sys.argv[1:])

    # Fail closed before touching rclpy at all -- and therefore before any
    # publisher/subscriber can be created and before any message can be
    # published -- if any topic this harness will use is a literal
    # production topic name or falls outside the /hil_offline_stage3/
    # namespace. This check runs strictly before rclpy.init(), before
    # HilOfflineStage3Harness (and therefore every create_publisher/
    # create_subscription call inside it) is constructed, and therefore
    # before any message on any of these topics can possibly be published.
    for topic in (
        args.own_state_topic, args.bridge_status_topic, args.arm_topic, args.phase_event_topic,
        args.goal_announcement_topic, args.virtual_peer_source_topic,
        args.virtual_peer_guard_input_topic, args.gate_decision_topic,
        args.nav_intent_topic, args.guarded_cmd_vel_topic,
    ):
        if not is_isolated_topic(topic):
            raise SystemExit(f"HIL_OFFLINE_STAGE3_ABORT_NON_ISOLATED_TOPIC({topic})")

    rclpy.init(args=[])
    node = HilOfflineStage3Harness(args)
    try:
        if args.auto_run:
            runner = Stage3AutomaticRunner(
                node,
                per_phase_timeout_s=args.runner_per_phase_timeout_s,
                overall_timeout_s=args.runner_overall_timeout_s,
                test_only_linear_bound_mps=args.runner_linear_bound_mps,
                test_only_angular_bound_rps=args.runner_angular_bound_rps,
                peer_timeout_s=args.runner_peer_timeout_s,
                duplicate_source_robot_id=args.runner_duplicate_source_robot_id,
                duplicate_goal_x_m=args.runner_duplicate_goal_x_m,
                duplicate_goal_y_m=args.runner_duplicate_goal_y_m,
                duplicate_goal_id=args.runner_duplicate_goal_id,
            )
            runner.run(lambda: rclpy.spin_once(node, timeout_sec=0.05))
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
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
