#!/usr/bin/env python3
"""Stage 4 motion supervisor: the sole arm/disarm authority for the
one-real-robot + one-virtual-scout minimal physical HIL validation.

Why this file exists (do not re-derive this reasoning elsewhere): the
existing hil_cmd_vel_guard.py is a *stateless-per-tick*, fail-closed
clamp -- it never inspects the full causal chain (event -> announcement
-> adoption -> first command), never enforces a hard total-motion-
duration cutoff, and never self-disarms. Stage 4 needs exactly one new,
narrowly-scoped component that:

  1. proves, in order, that the virtual scout was released, a matching
     GoalAnnouncement was observed, the real GoalNavigator adopted that
     exact goal_id/coordinates, and only THEN inspects the first raw
     command from cooperative_avoider;
  2. rejects (not clamps) the entire raw Twist if any of its six
     components is outside the frozen tolerances;
  3. is the only process that ever publishes to /hil_guard/arm;
  4. enforces an internal 6.50s cutoff (a bounded margin under the
     verifier's 6.67s hard maximum) via a monotonic clock, never a
     shell sleep;
  5. is a one-shot state machine: COMPLETE or FAILED are permanent,
     latched, and never allow a second motion window in the same
     process.

hil_cmd_vel_guard.py, cooperative_avoider.py, goal_navigator.py, and
hil_virtual_peer.py are all reused completely unmodified. The guard
remains the sole /cmd_vel publisher and an independent second backstop
(started with --max-angular-speed-rps 0.0) -- this supervisor is a
defense-in-depth addition upstream of it, not a replacement for it.

The state machine below (Stage4MotionSupervisor) is a pure, rclpy-free
class: every transition is driven by an explicit method call carrying
already-extracted data, and time is read through an injectable
callable, so the full rehearsal matrix in
test_hil_stage4_motion_supervisor.py runs with no ROS dependency at
all. The rclpy wrapper at the bottom of this file (only importable once
rclpy/message packages are on the path, matching hil_cmd_vel_guard.py's
own established split) is a thin adapter: it does no additional safety
reasoning of its own, it only turns ROS callbacks into calls on the
pure engine and turns the engine's outputs into publishes.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Callable, Optional

# --------------------------------------------------------------------
# Frozen parameters (Stage 4 design review, 2026-07-30). Do not change
# without a fresh committed-evidence justification -- these are safety
# bounds, not tuning knobs.
# --------------------------------------------------------------------
ZERO_TOLERANCE = 1e-6
MIN_ACTIVE_LINEAR_MPS = 0.001
MAX_LINEAR_MPS = 0.015

EVENT_TIMEOUT_S = 30.0
ADOPTION_TIMEOUT_S = 5.0
RAW_COMMAND_TIMEOUT_S = 5.0

INTERNAL_ACTIVE_CUTOFF_S = 6.50
HARD_MAX_NONZERO_DURATION_S = 6.67  # verifier bound only; not a software timer target

DEFAULT_GOAL_COORDINATE_TOLERANCE_M = 1e-3

#: Must stay byte-identical to hil_goal_announcement_evidence.
#: STAGE4_ADOPTION_EVIDENCE_SCHEMA_VERSION -- any other value is
#: rejected fail-closed, including a newer/compatible-looking one.
ADOPTION_EVIDENCE_SCHEMA_VERSION = "1.0.0"

#: Frozen freshness bound (design review, 2026-07-30, revision 3): an
#: adoption-evidence message whose own recorded adapter_receive_time_s
#: is more than this many seconds away from the supervisor's current
#: ROS time (either direction -- a message claiming to be from the
#: future is exactly as suspect as a stale one) is rejected rather than
#: acted on.
ADOPTION_EVIDENCE_MAX_AGE_S = 2.0

_ADOPTION_EVIDENCE_REQUIRED_FIELDS = {
    "schema_version": str,
    "goal_id": str,
    "source_robot_id": int,
    "source_sequence": int,
    "accepted": bool,
    "duplicate": bool,
    "target_x_m": float,
    "target_y_m": float,
    "adapter_receive_time_s": float,
    "adapter_receive_monotonic_s": float,
}


def _field_type_ok(value, expected_type) -> bool:
    if expected_type is bool:
        return isinstance(value, bool)
    if expected_type is int:
        return isinstance(value, int) and not isinstance(value, bool)
    if expected_type is float:
        # JSON numbers may decode as int (e.g. "1") even where a float is
        # semantically required -- accept int here (not bool) and let the
        # caller coerce with float(); NaN/Inf are checked separately.
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if expected_type is str:
        return isinstance(value, str)
    return isinstance(value, expected_type)


@dataclass(frozen=True)
class AdoptionEvidence:
    schema_version: str
    goal_id: str
    source_robot_id: int
    source_sequence: int
    accepted: bool
    duplicate: bool
    target_x_m: float
    target_y_m: float
    adapter_receive_time_s: float
    adapter_receive_monotonic_s: float


def parse_and_validate_adoption_evidence(
    raw_data: str, *, now_ros_s: Optional[float] = None, max_age_s: float = ADOPTION_EVIDENCE_MAX_AGE_S,
) -> tuple[Optional[AdoptionEvidence], str]:
    """Pure. Fail-closed on every malformed/stale/wrong-schema input --
    never returns a partially-trusted AdoptionEvidence. Returns
    (evidence, "") on success or (None, reason) on any rejection."""
    try:
        payload = json.loads(raw_data)
    except (ValueError, TypeError):
        return None, "MALFORMED_JSON"
    if not isinstance(payload, dict):
        return None, "MALFORMED_JSON"

    for field_name, expected_type in _ADOPTION_EVIDENCE_REQUIRED_FIELDS.items():
        if field_name not in payload:
            return None, f"MISSING_FIELD:{field_name}"
        if not _field_type_ok(payload[field_name], expected_type):
            return None, f"WRONG_FIELD_TYPE:{field_name}"

    if payload["schema_version"] != ADOPTION_EVIDENCE_SCHEMA_VERSION:
        return None, f"SCHEMA_VERSION_MISMATCH:{payload['schema_version']!r}"

    for coord_field in ("target_x_m", "target_y_m", "adapter_receive_time_s", "adapter_receive_monotonic_s"):
        v = float(payload[coord_field])
        if math.isnan(v) or math.isinf(v):
            return None, f"NON_FINITE_FIELD:{coord_field}"

    adapter_receive_time_s = float(payload["adapter_receive_time_s"])
    if now_ros_s is not None:
        age_s = now_ros_s - adapter_receive_time_s
        if abs(age_s) > max_age_s:
            return None, f"STALE_EVIDENCE:age_s={age_s:.3f}"

    evidence = AdoptionEvidence(
        schema_version=payload["schema_version"],
        goal_id=payload["goal_id"],
        source_robot_id=int(payload["source_robot_id"]),
        source_sequence=int(payload["source_sequence"]),
        accepted=bool(payload["accepted"]),
        duplicate=bool(payload["duplicate"]),
        target_x_m=float(payload["target_x_m"]),
        target_y_m=float(payload["target_y_m"]),
        adapter_receive_time_s=adapter_receive_time_s,
        adapter_receive_monotonic_s=float(payload["adapter_receive_monotonic_s"]),
    )
    return evidence, ""


class State(Enum):
    PREPARED = "PREPARED"
    WAITING_FOR_EVENT = "WAITING_FOR_EVENT"
    VALIDATING_RAW_COMMAND = "VALIDATING_RAW_COMMAND"
    ACTIVE = "ACTIVE"
    ZERO_BURST = "ZERO_BURST"
    DISARMED = "DISARMED"
    COMPLETE = "COMPLETE"
    ABORT_ZERO = "ABORT_ZERO"
    FAILED = "FAILED"


TERMINAL_STATES = frozenset((State.COMPLETE, State.FAILED))


@dataclass(frozen=True)
class TwistSample:
    linear_x: float
    linear_y: float
    linear_z: float
    angular_x: float
    angular_y: float
    angular_z: float

    def as_dict(self) -> dict:
        return {
            "linear_x": self.linear_x, "linear_y": self.linear_y, "linear_z": self.linear_z,
            "angular_x": self.angular_x, "angular_y": self.angular_y, "angular_z": self.angular_z,
        }


def validate_twist(t: TwistSample, *, require_min_linear: bool) -> tuple[bool, str]:
    """Pure, fail-closed. ANY violation rejects the ENTIRE message -- this
    supervisor never clamps and never forwards a partially-valid command.
    require_min_linear=True only during the VALIDATING_RAW_COMMAND ->
    ACTIVE transition (a zero/near-zero command must never open the
    motion window); False during ACTIVE itself (a later sample legally
    approaching zero, e.g. as the controller settles, is not by itself
    a fault -- but any OTHER violation during ACTIVE still aborts)."""
    for value in (t.linear_x, t.linear_y, t.linear_z, t.angular_x, t.angular_y, t.angular_z):
        if value is None or not isinstance(value, (int, float)) or math.isnan(value) or math.isinf(value):
            return False, "NON_FINITE_COMPONENT"

    if abs(t.linear_y) > ZERO_TOLERANCE:
        return False, "NONZERO_LINEAR_Y"
    if abs(t.linear_z) > ZERO_TOLERANCE:
        return False, "NONZERO_LINEAR_Z"
    if abs(t.angular_x) > ZERO_TOLERANCE:
        return False, "NONZERO_ANGULAR_X"
    if abs(t.angular_y) > ZERO_TOLERANCE:
        return False, "NONZERO_ANGULAR_Y"
    if abs(t.angular_z) > ZERO_TOLERANCE:
        return False, "NONZERO_ANGULAR_Z"

    if t.linear_x < 0.0 - ZERO_TOLERANCE:
        return False, "REVERSE_COMMAND"
    if t.linear_x > MAX_LINEAR_MPS + ZERO_TOLERANCE:
        return False, "EXCESSIVE_LINEAR_SPEED"
    if require_min_linear and t.linear_x < MIN_ACTIVE_LINEAR_MPS:
        return False, "ZERO_OR_BELOW_MIN_LINEAR_COMMAND"

    return True, ""


@dataclass
class EvidenceRecord:
    monotonic_time_s: float
    ros_time_s: Optional[float]
    state: str
    event: str
    reason: str = ""
    raw: Optional[dict] = None
    run_id: str = ""
    goal_id: str = ""

    def as_dict(self) -> dict:
        return {
            "monotonic_time_s": self.monotonic_time_s,
            "ros_time_s": self.ros_time_s,
            "state": self.state,
            "event": self.event,
            "reason": self.reason,
            "raw": self.raw,
            "run_id": self.run_id,
            "goal_id": self.goal_id,
        }


class Stage4MotionSupervisor:
    """Pure, rclpy-free one-shot state machine. See module docstring."""

    def __init__(
        self,
        *,
        goal_id: str,
        expected_target_x_m: float,
        expected_target_y_m: float,
        run_id: str = "",
        now_fn: Callable[[], float] = time.monotonic,
        ros_now_fn: Callable[[], Optional[float]] = lambda: None,
        goal_coordinate_tolerance_m: float = DEFAULT_GOAL_COORDINATE_TOLERANCE_M,
    ):
        self.goal_id = goal_id
        self.expected_target_x_m = expected_target_x_m
        self.expected_target_y_m = expected_target_y_m
        self.run_id = run_id
        self._now = now_fn
        self._ros_now = ros_now_fn
        self._goal_coordinate_tolerance_m = goal_coordinate_tolerance_m

        self.state = State.PREPARED
        self.evidence: list[EvidenceRecord] = []
        self.terminal_reason = ""

        self._approval_used = False
        self._event_wait_start_s: Optional[float] = None
        self._released_at_s: Optional[float] = None
        self._adopted_at_s: Optional[float] = None
        self._active_start_s: Optional[float] = None

    # -- internal bookkeeping -------------------------------------------------
    def _record(self, event: str, *, reason: str = "", raw: Optional[dict] = None) -> None:
        self.evidence.append(EvidenceRecord(
            monotonic_time_s=self._now(), ros_time_s=self._ros_now(),
            state=self.state.value, event=event, reason=reason, raw=raw,
            run_id=self.run_id, goal_id=self.goal_id,
        ))

    def _latch_failed(self, reason: str) -> None:
        if self.state in TERMINAL_STATES:
            return
        self.state = State.FAILED
        self.terminal_reason = reason
        self._record("LATCHED_FAILED", reason=reason)

    def _latch_complete(self) -> None:
        if self.state in TERMINAL_STATES:
            return
        self.state = State.COMPLETE
        self._record("LATCHED_COMPLETE")

    def _open_zero_burst_and_disarm(self, reason: str) -> None:
        self.state = State.ZERO_BURST
        self._record("ZERO_BURST_OPENED", reason=reason)
        self._record("ZERO_PUBLISHED")
        self.state = State.DISARMED
        self._record("DISARM_PUBLISHED")

    @property
    def is_terminal(self) -> bool:
        return self.state in TERMINAL_STATES

    # -- external inputs -------------------------------------------------------
    def approve(self, token: str) -> None:
        """The operator's ONE action. No separate /hil_guard/arm publish by
        the operator is ever required or accepted -- this call is the sole
        approval path; everything after it is automatic."""
        if self.state in TERMINAL_STATES:
            self._record("APPROVAL_REJECTED_TERMINAL", raw={"token": token})
            return
        if self.state != State.PREPARED:
            self._record("APPROVAL_REJECTED_NOT_PREPARED", reason=f"state={self.state.value}")
            return
        if self._approval_used:
            self._record("APPROVAL_REJECTED_ALREADY_USED")
            return
        self._approval_used = True
        self._event_wait_start_s = self._now()
        self.state = State.WAITING_FOR_EVENT
        self._record("APPROVAL_ACCEPTED", raw={"token": token})
        self._record("READINESS_WAITING_FOR_EVENT")

    def on_virtual_scout_released(self) -> None:
        if self.state != State.WAITING_FOR_EVENT or self._released_at_s is not None:
            self._record("VIRTUAL_SCOUT_RELEASE_IGNORED", reason=f"state={self.state.value}")
            return
        self._released_at_s = self._now()
        self._record("VIRTUAL_SCOUT_RELEASED")

    def on_unexpected_adoption_evidence_publisher_count(self, count: int) -> None:
        """Called by the orchestrator/node wrapper (ROS-level publisher
        counting is outside this pure engine's own reach) whenever the
        adoption-evidence topic does not have exactly one publisher.
        Treated as a self-check failure, not a normal rejection -- it
        indicates the topology itself is untrustworthy, not merely that
        one message was bad."""
        if self.state in (State.WAITING_FOR_EVENT, State.VALIDATING_RAW_COMMAND, State.ACTIVE):
            self.on_supervisor_self_check_failed(f"UNEXPECTED_ADOPTION_EVIDENCE_PUBLISHER_COUNT:{count}")

    def on_adoption_evidence(self, raw_json: str, *, now_ros_s: Optional[float] = None) -> None:
        """Fed from the machine-readable /hil/adoption_evidence message
        (see hil_goal_announcement_evidence.py) -- never from a scraped
        log line. Every field is independently validated (schema
        version, presence, type, finiteness, freshness) before any of
        its content is trusted; `accepted`/`duplicate` are that
        message's own already-computed facts, additionally required to
        exactly match this supervisor's goal_id/coordinates before being
        treated as the adoption that authorises arming."""
        if self.state != State.WAITING_FOR_EVENT or self._released_at_s is None:
            self._record("ADOPTION_EVIDENCE_IGNORED", reason=f"state={self.state.value}")
            return

        evidence, reason = parse_and_validate_adoption_evidence(raw_json, now_ros_s=now_ros_s)
        if evidence is None:
            self._record("ADOPTION_EVIDENCE_REJECTED", reason=reason)
            return

        if evidence.duplicate:
            self._record(
                "ADOPTION_EVIDENCE_DUPLICATE_FLAGGED",
                raw={"goal_id": evidence.goal_id, "source_sequence": evidence.source_sequence},
            )
            return
        if not evidence.accepted:
            self._record(
                "ADOPTION_EVIDENCE_NOT_ACCEPTED",
                raw={"goal_id": evidence.goal_id, "source_sequence": evidence.source_sequence},
            )
            return
        if evidence.goal_id != self.goal_id:
            self._record("ADOPTION_EVIDENCE_GOAL_ID_MISMATCH", reason=f"expected={self.goal_id} got={evidence.goal_id}")
            return
        if (abs(evidence.target_x_m - self.expected_target_x_m) > self._goal_coordinate_tolerance_m
                or abs(evidence.target_y_m - self.expected_target_y_m) > self._goal_coordinate_tolerance_m):
            self._record(
                "ADOPTION_EVIDENCE_COORDINATE_MISMATCH",
                reason=(
                    f"expected=({self.expected_target_x_m},{self.expected_target_y_m}) "
                    f"got=({evidence.target_x_m},{evidence.target_y_m})"
                ),
            )
            return

        self._adopted_at_s = self._now()
        self.state = State.VALIDATING_RAW_COMMAND
        self._record("ADOPTION_CONFIRMED", raw={
            "goal_id": evidence.goal_id, "source_sequence": evidence.source_sequence,
            "target_x_m": evidence.target_x_m, "target_y_m": evidence.target_y_m,
            "schema_version": evidence.schema_version,
        })

    def on_raw_twist(self, t: TwistSample) -> None:
        if self.state == State.VALIDATING_RAW_COMMAND:
            self._record("RAW_TWIST_RECEIVED", raw=t.as_dict())
            now = self._now()
            if not (now > self._adopted_at_s):
                # Structurally should be unreachable (the state machine
                # only reaches VALIDATING_RAW_COMMAND after ADOPTION_CONFIRMED
                # sets _adopted_at_s, and this engine's own clock only moves
                # forward) -- checked explicitly anyway so a future refactor
                # or a clock-injection bug in a test cannot silently violate
                # the ordering requirement.
                self._latch_failed("RAW_COMMAND_NOT_STRICTLY_AFTER_ADOPTION")
                return
            ok, reason = validate_twist(t, require_min_linear=True)
            if not ok:
                if reason == "ZERO_OR_BELOW_MIN_LINEAR_COMMAND":
                    # Not a safety violation -- cooperative_avoider's own
                    # real control loop can legitimately still be
                    # publishing a pre-ramp/pre-intent-update zero for one
                    # or more ticks immediately after adoption (observed
                    # live: its own acceleration-limited smoother and its
                    # own control-loop period mean the very first tick
                    # after a target switch is not guaranteed to already
                    # reflect it). Every OTHER invalid reason (non-finite,
                    # nonzero angular, reverse, excessive speed) is a
                    # genuine safety violation and still latches FAILED
                    # immediately below. RAW_COMMAND_TIMEOUT_S is what
                    # bounds how long this supervisor will wait for a
                    # genuine nonzero command before giving up.
                    return
                self._latch_failed(f"RAW_COMMAND_INVALID:{reason}")
                return
            self._active_start_s = self._now()
            self._record("ARM_PUBLISHED")
            self.state = State.ACTIVE
            self._record("ACTIVE_OPENED")
        elif self.state == State.ACTIVE:
            self._record("RAW_TWIST_RECEIVED", raw=t.as_dict())
            ok, reason = validate_twist(t, require_min_linear=False)
            if not ok:
                self._open_zero_burst_and_disarm(f"RAW_COMMAND_INVALID_DURING_ACTIVE:{reason}")
                self._latch_failed(f"RAW_COMMAND_INVALID_DURING_ACTIVE:{reason}")
        else:
            self._record("RAW_TWIST_IGNORED", reason=f"state={self.state.value}")

    def on_liveness_dropout(self, reason: str) -> None:
        """Real /epuck1/state staleness, bridge disconnect, or any other
        liveness signal the orchestrator wires in. Only meaningful during
        ACTIVE (before that, the state machine cannot have armed)."""
        if self.state == State.ACTIVE:
            self._open_zero_burst_and_disarm(f"LIVENESS_DROPOUT:{reason}")
            self._latch_failed(f"LIVENESS_DROPOUT:{reason}")

    def on_supervisor_self_check_failed(self, reason: str) -> None:
        """Hook for the orchestrator to force a FAILED classification for a
        reason it detected externally (e.g. a second supervisor process
        somehow started) without pretending the state machine chose it."""
        if self.state == State.ACTIVE:
            self._open_zero_burst_and_disarm(reason)
        self._latch_failed(reason)

    def tick_timeouts(self) -> None:
        """Call periodically (e.g. every 50ms) from the ROS timer. Pure --
        takes no arguments beyond the injected clock, so it is exercised
        directly in tests by advancing a fake clock."""
        if self.state in TERMINAL_STATES:
            return
        now = self._now()

        if self.state == State.WAITING_FOR_EVENT and self._released_at_s is None:
            if now - self._event_wait_start_s > EVENT_TIMEOUT_S:
                self._latch_failed("EVENT_TIMEOUT")

        elif self.state == State.WAITING_FOR_EVENT and self._released_at_s is not None and self._adopted_at_s is None:
            if now - self._released_at_s > ADOPTION_TIMEOUT_S:
                self._latch_failed("ADOPTION_TIMEOUT")

        elif self.state == State.VALIDATING_RAW_COMMAND:
            if now - self._adopted_at_s > RAW_COMMAND_TIMEOUT_S:
                self._latch_failed("RAW_COMMAND_TIMEOUT")

        elif self.state == State.ACTIVE:
            if now - self._active_start_s >= INTERNAL_ACTIVE_CUTOFF_S:
                self._open_zero_burst_and_disarm("INTERNAL_ACTIVE_CUTOFF_REACHED")
                self._latch_complete()

    def evidence_as_dicts(self) -> list[dict]:
        return [r.as_dict() for r in self.evidence]


# --------------------------------------------------------------------
# rclpy wrapper -- thin, no additional safety logic. Only importable
# (and only actually constructed) once rclpy/ROS message packages are
# on the path, so Stage4MotionSupervisor above stays testable without
# ROS -- same split as hil_cmd_vel_guard.py's decide_command()/
# HilCmdVelGuard.
# --------------------------------------------------------------------
def _build_node():
    import argparse

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    from epuck2_comm_interfaces.msg import EpuckState

    class HilStage4MotionSupervisorNode(Node):
        def __init__(self, args):
            super().__init__("hil_stage4_motion_supervisor")
            self.args = args
            self.last_physical_state_at_s: Optional[float] = None

            self.engine = Stage4MotionSupervisor(
                goal_id=args.goal_id,
                expected_target_x_m=args.expected_target_x_m,
                expected_target_y_m=args.expected_target_y_m,
                run_id=args.run_id,
                now_fn=time.monotonic,
                ros_now_fn=lambda: self.get_clock().now().nanoseconds / 1.0e9,
            )

            self.arm_pub = self.create_publisher(Bool, args.arm_topic, 10)
            self.cmd_pub = self.create_publisher(Twist, args.guarded_output_topic, 10)

            self.create_subscription(String, args.adoption_evidence_topic, self._adoption_evidence_cb, 10)
            # raw_cmd_vel is deliberately NOT subscribed here. Subscribing
            # at startup would let ROS's own message queue (depth 10)
            # buffer pre-adoption samples (e.g. cooperative_avoider's
            # zero output during its own startup hold), so the very
            # "first" message this node ever saw on that topic could
            # actually predate adoption -- silently violating the
            # required causal order (raw command inspected only AFTER
            # adoption). The subscription is created for the first time
            # only once adoption is confirmed (see _adoption_evidence_cb),
            # guaranteeing no message existed in this node's queue before
            # that instant.
            self._raw_cmd_vel_sub = None
            self.create_subscription(
                EpuckState, args.physical_state_topic, self._physical_state_cb, 20,
            )
            # The orchestrator publishes exactly one True message here right
            # after it actually spawns hil_virtual_peer.py -- this is the
            # online, machine-evidenced release signal (Section E/10), not a
            # log-scrape. The topic is otherwise unused by any other node.
            self.create_subscription(Bool, args.virtual_scout_released_topic, self._released_cb, 10)

            self._evidence_path = Path(args.evidence_path)
            self._evidence_path.parent.mkdir(parents=True, exist_ok=True)
            self._evidence_fh = open(self._evidence_path, "a", encoding="utf-8")

            self.create_timer(0.05, self._tick)

            self.get_logger().warn(
                "HIL_STAGE4_MOTION_SUPERVISOR_READY "
                f"goal_id={args.goal_id} run_id={args.run_id} "
                f"expected_target=({args.expected_target_x_m},{args.expected_target_y_m})"
            )

            if args.operator_approval_token:
                self.engine.approve(args.operator_approval_token)
                self._flush_new_evidence()

        # -- ROS callbacks (thin: publisher-count check, call the pure engine) --
        def _adoption_evidence_cb(self, msg) -> None:
            count = self.count_publishers(self.args.adoption_evidence_topic)
            if count != 1:
                self.engine.on_unexpected_adoption_evidence_publisher_count(count)
                self._flush_new_evidence()
                return
            self.engine.on_adoption_evidence(
                msg.data, now_ros_s=self.get_clock().now().nanoseconds / 1.0e9,
            )
            self._flush_new_evidence()
            if self.engine.state == State.VALIDATING_RAW_COMMAND and self._raw_cmd_vel_sub is None:
                self._raw_cmd_vel_sub = self.create_subscription(
                    Twist, self.args.raw_cmd_vel_topic, self._raw_twist_cb, 10,
                )

        def _raw_twist_cb(self, msg) -> None:
            was_validating = self.engine.state == State.VALIDATING_RAW_COMMAND
            self.engine.on_raw_twist(TwistSample(
                linear_x=msg.linear.x, linear_y=msg.linear.y, linear_z=msg.linear.z,
                angular_x=msg.angular.x, angular_y=msg.angular.y, angular_z=msg.angular.z,
            ))
            self._flush_new_evidence()
            if was_validating and self.engine.state == State.ACTIVE:
                self._publish_arm(True)
                self.cmd_pub.publish(msg)
            elif self.engine.state == State.ACTIVE:
                self.cmd_pub.publish(msg)
            elif self.engine.state == State.DISARMED:
                self._publish_zero()

        def _physical_state_cb(self, msg) -> None:
            self.last_physical_state_at_s = time.monotonic()
            required = self.args.required_validity_flags
            if (int(msg.validity_flags) & required) != required:
                self.engine.on_liveness_dropout("PHYSICAL_STATE_INVALID_FLAGS")
                self._flush_new_evidence()
                self._publish_zero()

        def _tick(self) -> None:
            if (self.engine.state == State.ACTIVE
                    and (self.last_physical_state_at_s is None
                         or time.monotonic() - self.last_physical_state_at_s > self.args.physical_state_timeout_s)):
                self.engine.on_liveness_dropout("PHYSICAL_STATE_STALE_OR_MISSING")
                self._publish_zero()

            was_active = self.engine.state == State.ACTIVE
            self.engine.tick_timeouts()
            if was_active and self.engine.state != State.ACTIVE:
                self._publish_zero()
                self._publish_arm(False)
            self._flush_new_evidence()

        def _publish_zero(self) -> None:
            zero = Twist()
            self.cmd_pub.publish(zero)

        def _publish_arm(self, value: bool) -> None:
            msg = Bool()
            msg.data = value
            self.arm_pub.publish(msg)

        def _released_cb(self, msg) -> None:
            if bool(msg.data):
                self.engine.on_virtual_scout_released()
                self._flush_new_evidence()

        def _flush_new_evidence(self) -> None:
            while self._logged_count() < len(self.engine.evidence):
                record = self.engine.evidence[self._logged_count_value]
                self._evidence_fh.write(json.dumps(record.as_dict()) + "\n")
                self._evidence_fh.flush()
                self._logged_count_value += 1

        _logged_count_value = 0

        def _logged_count(self) -> int:
            return self._logged_count_value

        def destroy_node(self):
            try:
                self._evidence_fh.close()
            except Exception:
                pass
            super().destroy_node()

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--goal-id", default="shared_exit")
        parser.add_argument("--run-id", required=True)
        parser.add_argument("--expected-target-x-m", type=float, required=True)
        parser.add_argument("--expected-target-y-m", type=float, required=True)
        parser.add_argument("--adoption-evidence-topic", required=True)
        parser.add_argument("--raw-cmd-vel-topic", required=True)
        parser.add_argument("--guarded-output-topic", required=True)
        parser.add_argument("--arm-topic", required=True)
        parser.add_argument("--virtual-scout-released-topic", required=True)
        parser.add_argument("--physical-state-topic", required=True)
        parser.add_argument("--physical-state-timeout-s", type=float, default=0.5)
        parser.add_argument("--required-validity-flags", type=int, default=7)
        parser.add_argument("--evidence-path", required=True)
        parser.add_argument("--operator-approval-token", default="")
        return parser.parse_args(argv)

    return HilStage4MotionSupervisorNode, parse_args


def main(argv=None):
    import sys

    import rclpy

    HilStage4MotionSupervisorNode, parse_args = _build_node()
    args = parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=[])
    node = HilStage4MotionSupervisorNode(args)
    try:
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
