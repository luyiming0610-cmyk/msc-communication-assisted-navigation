#!/usr/bin/env python3
"""Stage 3 (OFFLINE_INTEGRATION_VALIDATION) dedicated offline post-run
verifier. A separate tool from ground_diagnostic_post_run_verifier.py
because that verifier's schema (single straight-line pulse: requested/
guarded/Pi-applied speed table, stop-line clearance) does not match
Stage 3's mixed-node, no-physical-hardware evidence shape at all --
there is no Pi-applied command, no stop-line, no field geometry here.

Fails closed: any missing file, unparseable row, or missing required
evidence stream is a DATA_VALIDITY defect, never silently ignored.

DATA_VALIDITY and TASK_OUTCOME are computed independently, exactly as
in every other verdict tool in this project (matrix_verdict.py's own
convention) -- TASK_OUTCOME is only meaningful when DATA_VALIDITY is
VALID, and a genuine TASK_OUTCOME failure on VALID data is a real
result, never hidden, retried, or reclassified by this tool.

TASK_OUTCOME taxonomy is deliberately NOT a physical-safety vocabulary
(no "UNSAFE_FAILURE" here): this tool only ever evaluates a hardware-free
software chain, so every non-SUCCESS outcome is one of:
  SUCCESS | GUARD_BOUND_VIOLATION | STALE_ZERO_FAILURE |
  RECOVERY_FAILURE | ADOPTION_FAILURE | DUPLICATE_HANDLING_FAILURE |
  NOT_EVALUABLE
Every result additionally carries result_type="OFFLINE_SOFTWARE_CONTRACT_RESULT"
and an explicit physical_claim disclaimer -- this result is not evidence
of physical collision or physical unsafe motion of any kind, and a
genuine non-SUCCESS outcome must be preserved, never hidden or
automatically rerun.
"""
from __future__ import annotations

import csv
import hashlib
import json
import os
from dataclasses import dataclass, field

FORBIDDEN_ROS_DOMAIN_IDS = frozenset({0, 77, 89})
EXPECTED_STAGE3_ROS_DOMAIN_ID = 91

PRODUCTION_TOPICS = frozenset({
    "/cmd_vel",
    "/cmd_vel_unguarded",
    "/epuck1/state",
    "/epuck_bridge/status",
    "/hil_guard/arm",
})

REQUIRED_EVIDENCE_TOPICS_KEYS = (
    "own_state_topic", "virtual_peer_source_topic", "virtual_peer_guard_input_topic",
    "goal_announcement_topic", "nav_intent_topic", "requested_cmd_vel_topic",
    "guarded_cmd_vel_topic", "arm_topic", "bridge_status_topic", "phase_event_topic",
    "gate_decision_topic",
)

PHASE_EVENT_ROW_TOPIC = "PHASE_EVENT"
GATE_DECISION_EVENT_ROW_TOPIC = "GATE_DECISION_EVENT"
GATE_DECISION_VALUES = frozenset({"FORWARDED", "REJECTED_GATE_CLOSED"})

RESULT_TYPE = "OFFLINE_SOFTWARE_CONTRACT_RESULT"
PHYSICAL_CLAIM_DISCLAIMER = (
    "This is an offline, hardware-free software-contract result. It is not "
    "evidence of physical collision, physical unsafe motion, or any other "
    "physical-hardware behaviour."
)

TASK_OUTCOME_VALUES = frozenset({
    "SUCCESS", "GUARD_BOUND_VIOLATION", "STALE_ZERO_FAILURE", "RECOVERY_FAILURE",
    "ADOPTION_FAILURE", "DUPLICATE_HANDLING_FAILURE", "NOT_EVALUABLE",
    "GATE_FORWARDING_FAILURE", "BACKLOG_REPLAY_DETECTED",
})

DEFAULT_VIRTUAL_PEER_TIMEOUT_S = 1.0  # matches hil_cmd_vel_guard.py's own --virtual-peer-timeout-s default


@dataclass
class DataValidityResult:
    valid: bool
    reasons: list = field(default_factory=list)
    close_ts_ns: int | None = None
    reopen_ts_ns: int | None = None
    first_post_reopen_gate_input_ts_ns: int | None = None


@dataclass
class TaskOutcomeResult:
    outcome: str
    reasons: list = field(default_factory=list)


def _sha256_of(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def load_rows(csv_path: str) -> list:
    with open(csv_path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _as_bool(value) -> bool | None:
    if value in (None, "", "None"):
        return None
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("true", "1")


def _as_int(value):
    if value in (None, "", "None"):
        return None
    return int(float(value))


@dataclass
class StaleIntervalResult:
    ok: bool
    reasons: list = field(default_factory=list)
    close_ts_ns: int | None = None
    reopen_ts_ns: int | None = None
    first_post_reopen_gate_input_ts_ns: int | None = None
    stale_zero_ts_ns: int | None = None
    recovery_ts_ns: int | None = None


def evaluate_stale_interval(
    rows: list, *, source_topic: str, gate_input_topic: str,
) -> StaleIntervalResult:
    """Strictly determines the [PEER_GATE_CLOSED, PEER_GATE_REOPENED)
    interval from PHASE_EVENT rows' own local_time_ns (receipt-time,
    never a message-internal stamp) and proves, from the flat evidence
    rows alone:
      - both boundary events exist exactly once, close strictly precedes reopen;
      - at least one virtual_peer_source_topic row falls strictly inside the
        interval (the source itself kept publishing -- proves the interval
        is a genuine forwarding gap, not the source also going silent);
      - at least one gate-input row exists at-or-before closure (the gate
        input topic saw traffic at all before the closed interval began).

    This function deliberately does NOT attempt to prove what was or was
    not forwarded during the closed interval, nor does it attempt any
    backlog-replay detection -- those are causal claims about the
    gate's own FORWARDED/REJECTED_GATE_CLOSED decision, and this
    topic's rows and the gate_input_topic's rows are written by two
    independently-scheduled recorder subscriber callbacks with no
    guaranteed relative ordering. See evaluate_gate_forwarding_contract()
    below, which proves those specific claims from the gate's own
    synchronously-emitted GATE_DECISION_EVENT rows instead. Fails closed
    on missing, duplicated, non-finite, or misordered timestamps.
    """
    reasons: list = []
    phase_rows = [r for r in rows if r.get("topic") == PHASE_EVENT_ROW_TOPIC]

    close_rows = [r for r in phase_rows if r.get("phase") == "PEER_GATE_CLOSED"]
    reopen_rows = [r for r in phase_rows if r.get("phase") == "PEER_GATE_REOPENED"]

    if len(close_rows) != 1:
        reasons.append(f"PEER_GATE_CLOSED_EVENT_COUNT_NOT_EXACTLY_ONE({len(close_rows)})")
    if len(reopen_rows) != 1:
        reasons.append(f"PEER_GATE_REOPENED_EVENT_COUNT_NOT_EXACTLY_ONE({len(reopen_rows)})")
    if reasons:
        return StaleIntervalResult(ok=False, reasons=reasons)

    close_ts = _as_int(close_rows[0].get("local_time_ns"))
    reopen_ts = _as_int(reopen_rows[0].get("local_time_ns"))
    if close_ts is None or reopen_ts is None or close_ts <= 0 or reopen_ts <= 0:
        return StaleIntervalResult(ok=False, reasons=["MISSING_OR_NONFINITE_GATE_BOUNDARY_TIMESTAMP"])
    if close_ts >= reopen_ts:
        return StaleIntervalResult(
            ok=False,
            reasons=[f"GATE_CLOSE_DOES_NOT_STRICTLY_PRECEDE_REOPEN(close={close_ts},reopen={reopen_ts})"],
        )

    gate_input_rows = [r for r in rows if r.get("topic") == gate_input_topic]
    source_rows = [r for r in rows if r.get("topic") == source_topic]

    def _ts(r):
        return _as_int(r.get("local_time_ns"))

    source_inside = [r for r in source_rows if _ts(r) is not None and close_ts < _ts(r) < reopen_ts]
    if not source_inside:
        reasons.append("SOURCE_STATE_DID_NOT_CONTINUE_DURING_CLOSED_INTERVAL")

    before_close = [r for r in gate_input_rows if _ts(r) is not None and _ts(r) <= close_ts]
    if not before_close:
        reasons.append("NO_GATE_INPUT_ROW_BEFORE_CLOSURE")

    first_post_reopen_ts = None

    stale_zero_rows = [r for r in phase_rows if r.get("phase") == "STALE_ZERO_CONFIRMED"]
    stale_zero_ts = _ts(stale_zero_rows[0]) if len(stale_zero_rows) == 1 else None
    if len(stale_zero_rows) > 1:
        reasons.append(f"STALE_ZERO_CONFIRMED_EVENT_COUNT_NOT_AT_MOST_ONE({len(stale_zero_rows)})")

    recovery_rows = [r for r in phase_rows if r.get("phase") == "RECOVERY_CONFIRMED"]
    recovery_ts = _ts(recovery_rows[0]) if len(recovery_rows) == 1 else None
    if len(recovery_rows) > 1:
        reasons.append(f"RECOVERY_CONFIRMED_EVENT_COUNT_NOT_AT_MOST_ONE({len(recovery_rows)})")

    return StaleIntervalResult(
        ok=(len(reasons) == 0), reasons=reasons,
        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts,
        first_post_reopen_gate_input_ts_ns=first_post_reopen_ts,
        stale_zero_ts_ns=stale_zero_ts, recovery_ts_ns=recovery_ts,
    )


def _gate_epoch(r) -> int | None:
    return _as_int(r.get("gate_decision_gate_epoch"))


def _gate_decision_ts(r) -> float | None:
    value = r.get("gate_decision_decision_timestamp_s")
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if parsed > 0 else None


@dataclass
class DataValidityCheckResult:
    ok: bool
    reasons: list = field(default_factory=list)


def evaluate_gate_decision_evidence_structure(
    rows: list, *, gate_input_topic: str,
) -> DataValidityCheckResult:
    """Structural/schema validation only -- a malformed, incomplete, or
    self-contradictory gate-decision event is an evidence-quality
    (DATA_VALIDITY) defect, not a software-contract (TASK_OUTCOME)
    failure. This is deliberately separate from
    evaluate_gate_forwarding_outcome() below, which assumes well-formed
    events and evaluates whether the CONTRACT itself was honoured."""
    reasons: list = []
    gate_rows = [r for r in rows if r.get("topic") == GATE_DECISION_EVENT_ROW_TOPIC]

    if not gate_rows:
        return DataValidityCheckResult(ok=False, reasons=["NO_GATE_DECISION_EVENTS_RECORDED"])

    for r in gate_rows:
        if r.get("gate_decision_event_type") != "GATE_DECISION":
            reasons.append(f"MALFORMED_GATE_DECISION_EVENT_TYPE({r.get('gate_decision_event_type')})")
        decision = r.get("gate_decision_decision")
        if decision not in GATE_DECISION_VALUES:
            reasons.append(f"UNMATCHED_GATE_DECISION_VALUE({decision})")
        if _gate_epoch(r) is None:
            reasons.append("GATE_DECISION_EVENT_MISSING_EPOCH")
        if _gate_decision_ts(r) is None:
            reasons.append("GATE_DECISION_EVENT_MISSING_OR_NONFINITE_DECISION_TIMESTAMP")
        dest = r.get("gate_decision_forwarded_destination_topic")
        if decision == "FORWARDED":
            if dest != gate_input_topic:
                reasons.append(f"FORWARDED_EVENT_DESTINATION_TOPIC_MISMATCH(got={dest},expected={gate_input_topic})")
        elif decision == "REJECTED_GATE_CLOSED":
            if dest not in (None, ""):
                reasons.append(f"REJECTED_EVENT_HAS_NONEMPTY_FORWARDED_DESTINATION_TOPIC({dest})")

    return DataValidityCheckResult(ok=(len(reasons) == 0), reasons=reasons)


@dataclass
class GateForwardingOutcomeResult:
    """outcome is one of None (contract fully honoured), "GATE_FORWARDING_FAILURE"
    (any contract violation other than a proven backlog replay), or
    "BACKLOG_REPLAY_DETECTED" (a cached pre-reopen sequence was forwarded
    after reopening -- reported as its own, more specific outcome even
    when other GATE_FORWARDING_FAILURE reasons are also present)."""
    outcome: str | None
    reasons: list = field(default_factory=list)


def evaluate_gate_forwarding_outcome(rows: list) -> GateForwardingOutcomeResult:
    """Assumes evaluate_gate_decision_evidence_structure() has already
    passed (well-formed events) and proves the strict first-post-reopen
    forwarding contract using ONLY the gate's own GATE_DECISION_EVENT
    rows -- one event per source message, emitted synchronously at the
    gate's own decision point inside the harness. Never reconstructed by
    comparing this recorder's gate_input_topic rows against its source-
    topic rows: those are two independently-scheduled subscriber
    callbacks with no guaranteed relative ordering, so their row order
    is not causal proof of anything.

    Proves:
      - the events span at least two gate epochs (a reopen occurred);
      - every event recorded while the gate was CLOSED has
        decision=REJECTED_GATE_CLOSED (none FORWARDED);
      - at least one REJECTED_GATE_CLOSED event was recorded while
        closed (the source kept being processed, not silently ignored);
      - exactly one event in the final (post-reopen) epoch is marked
        first_source_after_reopen=true, and its decision is FORWARDED;
      - no FORWARDED event in the post-reopen epoch carries a
        source_sequence less than or equal to the highest source_sequence
        already seen while the gate was CLOSED (backlog-replay check,
        using the message's OWN sequence number, never a local receipt
        timestamp).
    A genuine violation is always returned with its specific reasons --
    never silently downgraded to a passing outcome.
    """
    reasons: list = []
    backlog_reasons: list = []
    gate_rows = [r for r in rows if r.get("topic") == GATE_DECISION_EVENT_ROW_TOPIC]

    epochs = sorted({_gate_epoch(r) for r in gate_rows if _gate_epoch(r) is not None})
    if len(epochs) < 2:
        return GateForwardingOutcomeResult(
            outcome="GATE_FORWARDING_FAILURE",
            reasons=[f"GATE_DECISION_EVENTS_DO_NOT_SPAN_A_REOPEN(epochs_seen={epochs})"],
        )
    final_epoch = epochs[-1]

    closed_rows = sorted(
        (r for r in gate_rows if r.get("gate_decision_gate_state") == "CLOSED"), key=_gate_decision_ts,
    )
    if not closed_rows:
        reasons.append("NO_GATE_DECISION_EVENTS_WHILE_CLOSED")
    else:
        forwarded_while_closed = [r for r in closed_rows if r.get("gate_decision_decision") == "FORWARDED"]
        if forwarded_while_closed:
            reasons.append(f"GATE_FORWARDED_WHILE_CLOSED({len(forwarded_while_closed)}_events)")
        rejected_while_closed = [r for r in closed_rows if r.get("gate_decision_decision") == "REJECTED_GATE_CLOSED"]
        if not rejected_while_closed:
            reasons.append("NO_REJECTED_GATE_CLOSED_EVENTS_WHILE_CLOSED")

    reopened_rows = sorted(
        (r for r in gate_rows if _gate_epoch(r) == final_epoch and r.get("gate_decision_gate_state") == "OPEN"),
        key=_gate_decision_ts,
    )
    if not reopened_rows:
        reasons.append("NO_GATE_DECISION_EVENTS_FOR_FINAL_REOPENED_EPOCH")
    else:
        first_reopened = reopened_rows[0]
        if not _as_bool(first_reopened.get("gate_decision_first_source_after_reopen")):
            reasons.append("FIRST_POST_REOPEN_SOURCE_MESSAGE_NOT_MARKED_FIRST_AFTER_REOPEN")
        if first_reopened.get("gate_decision_decision") != "FORWARDED":
            reasons.append(
                f"FIRST_POST_REOPEN_SOURCE_MESSAGE_NOT_FORWARDED(decision={first_reopened.get('gate_decision_decision')})"
            )
        others_marked_first = [
            r for r in reopened_rows[1:] if _as_bool(r.get("gate_decision_first_source_after_reopen"))
        ]
        if others_marked_first:
            reasons.append(f"MULTIPLE_EVENTS_MARKED_FIRST_SOURCE_AFTER_REOPEN({len(others_marked_first)}_extra)")

        closed_seqs = [_as_int(r.get("gate_decision_source_sequence")) for r in closed_rows]
        closed_seqs = [s for s in closed_seqs if s is not None]
        max_closed_seq = max(closed_seqs) if closed_seqs else None
        if max_closed_seq is not None:
            for r in reopened_rows:
                if r.get("gate_decision_decision") != "FORWARDED":
                    continue
                seq = _as_int(r.get("gate_decision_source_sequence"))
                if seq is not None and seq <= max_closed_seq:
                    backlog_reasons.append(
                        f"BACKLOG_REPLAY_DETECTED(forwarded_sequence={seq}<=last_closed_epoch_sequence={max_closed_seq})"
                    )

    if backlog_reasons:
        return GateForwardingOutcomeResult(outcome="BACKLOG_REPLAY_DETECTED", reasons=backlog_reasons + reasons)
    if reasons:
        return GateForwardingOutcomeResult(outcome="GATE_FORWARDING_FAILURE", reasons=reasons)
    return GateForwardingOutcomeResult(outcome=None, reasons=[])


def evaluate_data_validity(
    *,
    csv_path: str,
    summary_json_path: str,
    residual_process_detected: bool,
    expected_domain_id: int = EXPECTED_STAGE3_ROS_DOMAIN_ID,
) -> DataValidityResult:
    """expected_domain_id defaults to the one real, sanctioned Stage 3
    domain (91). It is only ever overridden by this preparation
    package's own recorder-verifier integration test, which
    legitimately runs under a different, still-non-production,
    non-forbidden domain (93) and must state that explicitly rather
    than silently relaxing the real check's default."""
    reasons = []

    if not os.path.isfile(csv_path) or os.path.getsize(csv_path) == 0:
        reasons.append(f"MISSING_OR_EMPTY_CSV({csv_path})")
    if not os.path.isfile(summary_json_path) or os.path.getsize(summary_json_path) == 0:
        reasons.append(f"MISSING_OR_EMPTY_SUMMARY_JSON({summary_json_path})")
    if reasons:
        return DataValidityResult(valid=False, reasons=reasons)

    try:
        rows = load_rows(csv_path)
    except Exception as exc:
        return DataValidityResult(valid=False, reasons=[f"CSV_PARSE_ERROR({exc})"])

    try:
        with open(summary_json_path, encoding="utf-8") as f:
            summary = json.load(f)
    except Exception as exc:
        return DataValidityResult(valid=False, reasons=[f"SUMMARY_JSON_PARSE_ERROR({exc})"])

    domain_id = summary.get("ros_domain_id")
    if domain_id in FORBIDDEN_ROS_DOMAIN_IDS:
        reasons.append(f"ROS_DOMAIN_ID_FORBIDDEN({domain_id})")
    elif domain_id != expected_domain_id:
        reasons.append(f"ROS_DOMAIN_ID_NOT_SANCTIONED(got={domain_id},expected={expected_domain_id})")

    topic_contract = summary.get("topic_contract", {})
    for key in REQUIRED_EVIDENCE_TOPICS_KEYS:
        if key not in topic_contract:
            reasons.append(f"MISSING_TOPIC_CONTRACT_KEY({key})")
            continue
        topic = topic_contract[key]
        if topic in PRODUCTION_TOPICS:
            reasons.append(f"PRODUCTION_TOPIC_USED({key}={topic})")
        if not str(topic).startswith("/hil_offline_stage3"):
            reasons.append(f"TOPIC_NOT_ISOLATED({key}={topic})")

    row_counts = summary.get("row_count_by_topic", {})
    for key in REQUIRED_EVIDENCE_TOPICS_KEYS:
        topic = topic_contract.get(key)
        if topic is not None and row_counts.get(topic, 0) <= 0:
            reasons.append(f"NO_EVIDENCE_ROWS_FOR_TOPIC({key}={topic})")

    if residual_process_detected:
        reasons.append("RESIDUAL_PROCESS_DETECTED_AFTER_SHUTDOWN")

    own_state_topic = topic_contract.get("own_state_topic")
    if own_state_topic:
        own_state_rows = [r for r in rows if r.get("topic") == own_state_topic]
        bad_flags = [r for r in own_state_rows if _as_int(r.get("validity_flags")) != 7]
        if not own_state_rows:
            reasons.append("NO_OWN_STATE_ROWS")
        elif bad_flags:
            reasons.append(f"OWN_STATE_VALIDITY_FLAGS_NOT_7({len(bad_flags)}_of_{len(own_state_rows)})")

    timestamps = [_as_int(r.get("local_time_ns")) for r in rows]
    timestamps = [t for t in timestamps if t is not None]
    if not timestamps:
        reasons.append("NO_TIMESTAMPED_ROWS")
    elif any(t <= 0 for t in timestamps):
        reasons.append("NON_FINITE_OR_NONPOSITIVE_TIMESTAMP_PRESENT")
    elif timestamps != sorted(timestamps):
        reasons.append("TIMESTAMPS_NOT_MONOTONIC_NONDECREASING")

    stale_interval = StaleIntervalResult(ok=True)
    gate_input_topic = topic_contract.get("virtual_peer_guard_input_topic")
    source_topic = topic_contract.get("virtual_peer_source_topic")
    if gate_input_topic and source_topic:
        stale_interval = evaluate_stale_interval(
            rows, source_topic=source_topic, gate_input_topic=gate_input_topic,
        )
        reasons.extend(stale_interval.reasons)

    if gate_input_topic:
        gate_structure = evaluate_gate_decision_evidence_structure(rows, gate_input_topic=gate_input_topic)
        reasons.extend(gate_structure.reasons)

    return DataValidityResult(
        valid=(len(reasons) == 0), reasons=reasons,
        close_ts_ns=stale_interval.close_ts_ns, reopen_ts_ns=stale_interval.reopen_ts_ns,
        first_post_reopen_gate_input_ts_ns=stale_interval.first_post_reopen_gate_input_ts_ns,
    )


def evaluate_task_outcome(
    *,
    rows: list,
    topic_contract: dict,
    test_only_angular_bound_rps: float,
    test_only_linear_bound_mps: float,
    close_ts_ns: int | None = None,
    reopen_ts_ns: int | None = None,
    virtual_peer_timeout_s: float = DEFAULT_VIRTUAL_PEER_TIMEOUT_S,
) -> TaskOutcomeResult:
    """Returns the single most specific applicable non-physical outcome
    category (priority: ADOPTION_FAILURE > DUPLICATE_HANDLING_FAILURE >
    BACKLOG_REPLAY_DETECTED > GATE_FORWARDING_FAILURE >
    GUARD_BOUND_VIOLATION > STALE_ZERO_FAILURE > RECOVERY_FAILURE),
    or SUCCESS with no reasons. A genuine failure is always returned
    with its specific reasons -- never silently downgraded to SUCCESS."""
    adoption_reasons: list = []
    duplicate_reasons: list = []
    guard_bound_reasons: list = []
    stale_zero_reasons: list = []
    recovery_reasons: list = []
    other_reasons: list = []

    announcement_topic = topic_contract.get("goal_announcement_topic")
    announcement_rows = [r for r in rows if r.get("topic") == announcement_topic]
    accepted_count = sum(1 for r in rows if r.get("phase") == "ANNOUNCEMENT_ADOPTED")
    if len(announcement_rows) < 1:
        adoption_reasons.append("NO_ANNOUNCEMENT_OBSERVED")
    if accepted_count != 1:
        adoption_reasons.append(f"ADOPTION_EVENT_COUNT_NOT_EXACTLY_ONE({accepted_count})")

    duplicate_sent_rows = [r for r in rows if _as_bool(r.get("duplicate_sent")) is True]
    if len(duplicate_sent_rows) != 1:
        duplicate_reasons.append(f"DUPLICATE_SENT_EVIDENCE_COUNT_NOT_EXACTLY_ONE({len(duplicate_sent_rows)})")

    guarded_topic = topic_contract.get("guarded_cmd_vel_topic")
    guarded_rows = [r for r in rows if r.get("topic") == guarded_topic]
    out_of_bounds = [
        r for r in guarded_rows
        if r.get("linear_x") not in (None, "")
        and (abs(float(r["linear_x"])) > test_only_linear_bound_mps + 1e-9
             or abs(float(r.get("angular_z") or 0.0)) > test_only_angular_bound_rps + 1e-9)
    ]
    if out_of_bounds:
        guard_bound_reasons.append(f"GUARDED_COMMAND_OUT_OF_TEST_ONLY_BOUNDS({len(out_of_bounds)}_rows)")

    phase_rows = [r for r in rows if r.get("topic") == PHASE_EVENT_ROW_TOPIC]
    stale_zero_row = next((r for r in phase_rows if r.get("phase") == "STALE_ZERO_CONFIRMED"), None)
    if stale_zero_row is None:
        stale_zero_reasons.append("STALE_PEER_ZERO_NOT_CONFIRMED")
    elif close_ts_ns is not None and reopen_ts_ns is not None:
        stale_zero_ts = _as_int(stale_zero_row.get("local_time_ns"))
        earliest_allowed = close_ts_ns + int(virtual_peer_timeout_s * 1e9)
        if stale_zero_ts is None:
            stale_zero_reasons.append("STALE_ZERO_EVIDENCE_MISSING_TIMESTAMP")
        elif stale_zero_ts < earliest_allowed:
            stale_zero_reasons.append(
                f"STALE_ZERO_CONFIRMED_BEFORE_PEER_TIMEOUT_ELAPSED(ts={stale_zero_ts},"
                f"earliest_allowed={earliest_allowed})"
            )
        elif stale_zero_ts >= reopen_ts_ns:
            stale_zero_reasons.append(
                f"STALE_ZERO_CONFIRMED_NOT_BEFORE_REOPEN(ts={stale_zero_ts},reopen={reopen_ts_ns})"
            )

    recovery_row = next((r for r in phase_rows if r.get("phase") == "RECOVERY_CONFIRMED"), None)
    if recovery_row is None:
        recovery_reasons.append("RECOVERY_NOT_CONFIRMED")
    else:
        recovery_ts = _as_int(recovery_row.get("local_time_ns"))
        # Whether a genuine fresh post-reopen forward occurred at all is
        # already fully and race-free proven by evaluate_gate_forwarding_
        # outcome() below, from the gate's own decision events alone
        # (epoch/sequence-based, never receipt-time-based). This check is
        # deliberately narrower: it only confirms that event's EXISTENCE,
        # never its recorder-receipt-time ordering relative to this
        # DIFFERENT topic's RECOVERY_CONFIRMED row. Two independently-
        # scheduled recorder subscriptions (phase_event_topic and
        # gate_decision_topic) can be delivered microseconds apart in
        # either order regardless of the harness's true internal
        # sequencing (which does correctly create the forward before
        # advancing the phase) -- comparing their receipt timestamps for
        # a strict ordering claim is exactly the cross-topic causal
        # inference this module exists to avoid, and was found to
        # produce spurious failures under real production timing.
        gate_decision_rows = [r for r in rows if r.get("topic") == GATE_DECISION_EVENT_ROW_TOPIC]
        first_after_reopen_rows = [
            r for r in gate_decision_rows
            if _as_bool(r.get("gate_decision_first_source_after_reopen"))
            and r.get("gate_decision_decision") == "FORWARDED"
        ]
        if recovery_ts is None:
            recovery_reasons.append("RECOVERY_EVIDENCE_MISSING_TIMESTAMP")
        elif not first_after_reopen_rows:
            recovery_reasons.append("NO_FIRST_POST_REOPEN_FORWARDED_GATE_DECISION_EVENT_FOUND")
        elif reopen_ts_ns is not None and recovery_ts <= reopen_ts_ns:
            # Same-topic (PHASE_EVENT_ROW_TOPIC) ordering, both rows written
            # by this one recorder subscription in true chronological
            # order -- safe to compare directly, unlike the cross-topic
            # comparison removed above.
            recovery_reasons.append(
                f"RECOVERY_CONFIRMED_NOT_AFTER_REOPEN_PHASE_EVENT(recovery_ts={recovery_ts},reopen_ts={reopen_ts_ns})"
            )

    gate_forwarding_reasons: list = []
    backlog_replay_reasons: list = []
    gate_outcome = evaluate_gate_forwarding_outcome(rows)
    if gate_outcome.outcome == "BACKLOG_REPLAY_DETECTED":
        backlog_replay_reasons = gate_outcome.reasons
    elif gate_outcome.outcome == "GATE_FORWARDING_FAILURE":
        gate_forwarding_reasons = gate_outcome.reasons

    completed = any(r.get("phase") == "COMPLETE" for r in phase_rows)
    if not completed:
        other_reasons.append("RUN_DID_NOT_REACH_COMPLETE_PHASE")

    if adoption_reasons:
        return TaskOutcomeResult(outcome="ADOPTION_FAILURE", reasons=adoption_reasons)
    if duplicate_reasons:
        return TaskOutcomeResult(outcome="DUPLICATE_HANDLING_FAILURE", reasons=duplicate_reasons)
    if backlog_replay_reasons:
        return TaskOutcomeResult(outcome="BACKLOG_REPLAY_DETECTED", reasons=backlog_replay_reasons)
    if gate_forwarding_reasons:
        return TaskOutcomeResult(outcome="GATE_FORWARDING_FAILURE", reasons=gate_forwarding_reasons)
    if guard_bound_reasons:
        return TaskOutcomeResult(outcome="GUARD_BOUND_VIOLATION", reasons=guard_bound_reasons)
    if stale_zero_reasons:
        return TaskOutcomeResult(outcome="STALE_ZERO_FAILURE", reasons=stale_zero_reasons)
    if recovery_reasons:
        return TaskOutcomeResult(outcome="RECOVERY_FAILURE", reasons=recovery_reasons)
    if other_reasons:
        return TaskOutcomeResult(
            outcome="NOT_EVALUABLE" if not guarded_rows else "GUARD_BOUND_VIOLATION",
            reasons=other_reasons,
        )
    return TaskOutcomeResult(outcome="SUCCESS", reasons=[])


def run_verifier(
    *,
    csv_path: str,
    summary_json_path: str,
    residual_process_detected: bool,
    test_only_angular_bound_rps: float,
    test_only_linear_bound_mps: float,
    virtual_peer_timeout_s: float = DEFAULT_VIRTUAL_PEER_TIMEOUT_S,
    expected_domain_id: int = EXPECTED_STAGE3_ROS_DOMAIN_ID,
) -> dict:
    data_validity = evaluate_data_validity(
        csv_path=csv_path, summary_json_path=summary_json_path,
        residual_process_detected=residual_process_detected,
        expected_domain_id=expected_domain_id,
    )
    result = {
        "DATA_VALIDITY": "VALID" if data_validity.valid else "INVALID",
        "data_validity_reasons": data_validity.reasons,
        "result_type": RESULT_TYPE,
        "physical_claim": PHYSICAL_CLAIM_DISCLAIMER,
    }
    if not data_validity.valid:
        result["TASK_OUTCOME"] = "NOT_EVALUABLE"
        result["task_outcome_reasons"] = ["DATA_VALIDITY=INVALID, task outcome cannot be trusted"]
        return result

    rows = load_rows(csv_path)
    with open(summary_json_path, encoding="utf-8") as f:
        summary = json.load(f)
    task_outcome = evaluate_task_outcome(
        rows=rows, topic_contract=summary.get("topic_contract", {}),
        test_only_angular_bound_rps=test_only_angular_bound_rps,
        test_only_linear_bound_mps=test_only_linear_bound_mps,
        close_ts_ns=data_validity.close_ts_ns, reopen_ts_ns=data_validity.reopen_ts_ns,
        virtual_peer_timeout_s=virtual_peer_timeout_s,
    )
    result["TASK_OUTCOME"] = task_outcome.outcome
    result["task_outcome_reasons"] = task_outcome.reasons
    result["evidence_sha256"] = {
        csv_path: _sha256_of(csv_path),
        summary_json_path: _sha256_of(summary_json_path),
    }
    return result


def main(argv=None):
    import argparse
    import sys

    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--summary-json", required=True)
    parser.add_argument("--residual-process-detected", action="store_true")
    parser.add_argument("--test-only-angular-bound-rps", type=float, required=True)
    parser.add_argument("--test-only-linear-bound-mps", type=float, required=True)
    parser.add_argument("--virtual-peer-timeout-s", type=float, default=DEFAULT_VIRTUAL_PEER_TIMEOUT_S)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    result = run_verifier(
        csv_path=args.csv, summary_json_path=args.summary_json,
        residual_process_detected=args.residual_process_detected,
        test_only_angular_bound_rps=args.test_only_angular_bound_rps,
        test_only_linear_bound_mps=args.test_only_linear_bound_mps,
        virtual_peer_timeout_s=args.virtual_peer_timeout_s,
    )
    print(json.dumps(result, indent=2))
    return 0 if result["DATA_VALIDITY"] == "VALID" and result["TASK_OUTCOME"] == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
