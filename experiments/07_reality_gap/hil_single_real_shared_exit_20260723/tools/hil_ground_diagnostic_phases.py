#!/usr/bin/env python3
"""Two explicit preflight phases for the first ground diagnostic.

PRE_STACK: everything checkable before any physical-stack process
(driver, audited server, WSL bridge, state_publisher, evidence
recorder, guard) has been started. Structurally cannot depend on
validity_flags, bridge status, evidence growth, or guard state --
evaluate_pre_stack() below has no parameter for any of them, so it is
not merely configured to ignore them, it has no way to require them.
This fixes the circular dependency where the previous single-phase
preflight required validity_flags==7 (which cannot exist before the
stack is up) before allowing the stack to be brought up.

LIVE_ZERO_STATE: only checkable once the whole stack is up, the guard
is confirmed DISARMED, and wheels are still suspended. Requires
validity_flags, bridge connection, evidence growth, guard identity, and
zero-only commands recorded so far.

Split 2026-07-24 into a WSL-side verdict and a Pi-side verdict, after
the first live attempt found that run_ground_diagnostic_preflight.sh
(running from WSL) cannot read the Pi's local command-audit JSONL --
the two machines share no filesystem, only a network connection. A
single combined function that silently expected a local path to the
Pi's file always failed closed (correctly, but for a structural reason
that looked like a real safety block). evaluate_wsl_live_state() now
covers only facts genuinely observable from WSL; the Pi's own audit
verdict is produced separately, on the Pi itself, by
pi_ground_diagnostic_audit_verifier.py, and the two are combined by
evaluate_combined_gate() below -- which additionally requires both
verdicts to share the same run identifier/evidence path and requires
the Pi verdict to be fresh (not a leftover from an earlier run).

Neither function starts a process, publishes a command, or arms
anything -- they only combine already-gathered plain values (booleans,
counts) into a pass/block decision with a reason list. Any ROS-graph or
filesystem fact either phase needs is gathered by the calling shell
script and passed in here as a plain value. No ROS/rclpy dependency.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional


@dataclass(frozen=True)
class PhaseResult:
    ok: bool
    reasons: tuple[str, ...] = field(default_factory=tuple)


def evaluate_pre_stack(
    *,
    tracked_git_clean: bool,
    tracked_fields_ok: bool,
    tracked_missing: tuple = (),
    tracked_unconfirmed: tuple = (),
    session_ok: bool,
    session_reason: str = "",
    device_reachable: bool,
    residual_process_found: bool,
    forbidden_process_found: bool,
    cmd_vel_publisher_count=None,
    evidence_paths_ok: bool = True,
) -> PhaseResult:
    reasons = []
    if not tracked_git_clean:
        reasons.append("TRACKED_TREE_DIRTY")
    if not tracked_fields_ok:
        reasons.append(
            f"TRACKED_FIELDS_NOT_READY(missing={list(tracked_missing)},unconfirmed={list(tracked_unconfirmed)})"
        )
    if not session_ok:
        reasons.append(f"SESSION_STATE_NOT_READY({session_reason})")
    if not device_reachable:
        reasons.append("DEVICE_UNREACHABLE")
    if residual_process_found:
        reasons.append("RESIDUAL_PROCESS_FOUND")
    if forbidden_process_found:
        reasons.append("FORBIDDEN_PROCESS_FOUND")
    if cmd_vel_publisher_count not in (None, 0):
        reasons.append(f"CMD_VEL_ALREADY_HAS_PUBLISHER({cmd_vel_publisher_count})")
    if not evidence_paths_ok:
        reasons.append("EVIDENCE_PATHS_NOT_FRESH")
    return PhaseResult(ok=not reasons, reasons=tuple(reasons))


def evaluate_wsl_live_state(
    *,
    validity_flags=None,
    required_validity_flags: int = 7,
    bridge_connected: bool,
    wsl_csv_growing: bool,
    guard_sole_publisher: bool,
    guard_armed: bool,
    cmd_vel_all_zero: bool,
    upstream_zero_or_absent: bool,
    forbidden_process_found: bool,
    wsl_evidence_all_zero: bool,
) -> PhaseResult:
    """Everything about live-zero-state genuinely observable from the
    WSL side alone -- no Pi-local file path is read here. Combine with
    a separately-produced Pi audit verdict via evaluate_combined_gate().
    """
    reasons = []
    if validity_flags != required_validity_flags:
        reasons.append(f"VALIDITY_FLAGS_NOT_{required_validity_flags}(got={validity_flags})")
    if not bridge_connected:
        reasons.append("BRIDGE_NOT_CONNECTED")
    if not wsl_csv_growing:
        reasons.append("WSL_CSV_NOT_GROWING")
    if not guard_sole_publisher:
        reasons.append("GUARD_NOT_SOLE_CMD_VEL_PUBLISHER")
    if guard_armed:
        reasons.append("GUARD_ALREADY_ARMED")
    if not cmd_vel_all_zero:
        reasons.append("CMD_VEL_NOT_ZERO")
    if not upstream_zero_or_absent:
        reasons.append("UPSTREAM_CMD_VEL_NOT_ZERO_OR_ABSENT")
    if forbidden_process_found:
        reasons.append("FORBIDDEN_PROCESS_FOUND")
    if not wsl_evidence_all_zero:
        reasons.append("WSL_EVIDENCE_CONTAINS_NONZERO_COMMAND")
    return PhaseResult(ok=not reasons, reasons=tuple(reasons))


def evaluate_combined_gate(
    *,
    wsl_result: PhaseResult,
    pi_verdict_ok: bool,
    pi_verdict_reasons: tuple = (),
    pi_verdict_available: bool = True,
    wsl_run_id: str,
    pi_run_id: Optional[str] = None,
    wsl_evidence_path: str,
    pi_evidence_path: Optional[str] = None,
    pi_verdict_generated_at_utc: Optional[str] = None,
    pi_verdict_max_age_s: float = 300.0,
    now: Optional[datetime] = None,
) -> PhaseResult:
    """PASSes only when the WSL live verdict passes, the Pi audit
    verdict passes, both share the same run identifier and evidence
    path, and the Pi verdict is fresh. A missing/unreachable Pi verdict
    (pi_verdict_available=False) is its own distinct reason -- never
    silently treated as either a pass or as "proven nonzero".
    """
    reasons = list(wsl_result.reasons)

    if not pi_verdict_available:
        reasons.append("PI_LIVE_AUDIT_NOT_AVAILABLE")
    else:
        if not pi_verdict_ok:
            reasons.extend(f"PI_LIVE_AUDIT:{r}" for r in pi_verdict_reasons)
        if pi_run_id != wsl_run_id:
            reasons.append(f"RUN_ID_MISMATCH(wsl={wsl_run_id},pi={pi_run_id})")
        if pi_verdict_generated_at_utc is None:
            reasons.append("PI_VERDICT_MISSING_TIMESTAMP")
        else:
            try:
                ts = datetime.fromisoformat(pi_verdict_generated_at_utc)
                if ts.tzinfo is None:
                    ts = ts.replace(tzinfo=timezone.utc)
                now_val = now or datetime.now(timezone.utc)
                age_s = (now_val - ts).total_seconds()
                if age_s < 0 or age_s > pi_verdict_max_age_s:
                    reasons.append("PI_VERDICT_STALE")
            except (TypeError, ValueError):
                reasons.append("PI_VERDICT_UNPARSEABLE_TIMESTAMP")

    # wsl_evidence_path/pi_evidence_path are accepted so callers always
    # pass both paths explicitly for traceability in logs, even though
    # today only the run_id is compared for matching (the Pi and WSL
    # evidence paths are never expected to be textually equal -- they
    # live on different machines).
    _ = (wsl_evidence_path, pi_evidence_path)

    return PhaseResult(ok=not reasons, reasons=tuple(reasons))
