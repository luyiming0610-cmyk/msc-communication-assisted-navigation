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

Neither function starts a process, publishes a command, or arms
anything -- they only combine already-gathered plain values (booleans,
counts) into a pass/block decision with a reason list. Any ROS-graph or
filesystem fact either phase needs is gathered by the calling shell
script and passed in here as a plain value. No ROS/rclpy dependency.
"""
from __future__ import annotations

from dataclasses import dataclass, field


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


def evaluate_live_zero_state(
    *,
    validity_flags=None,
    required_validity_flags: int = 7,
    bridge_connected: bool,
    wsl_csv_growing: bool,
    pi_jsonl_growing: bool,
    guard_sole_publisher: bool,
    guard_armed: bool,
    cmd_vel_all_zero: bool,
    upstream_zero_or_absent: bool,
    forbidden_process_found: bool,
    wsl_evidence_all_zero: bool,
    pi_evidence_all_zero: bool,
) -> PhaseResult:
    reasons = []
    if validity_flags != required_validity_flags:
        reasons.append(f"VALIDITY_FLAGS_NOT_{required_validity_flags}(got={validity_flags})")
    if not bridge_connected:
        reasons.append("BRIDGE_NOT_CONNECTED")
    if not wsl_csv_growing:
        reasons.append("WSL_CSV_NOT_GROWING")
    if not pi_jsonl_growing:
        reasons.append("PI_JSONL_NOT_GROWING")
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
    if not pi_evidence_all_zero:
        reasons.append("PI_EVIDENCE_CONTAINS_NONZERO_COMMAND")
    return PhaseResult(ok=not reasons, reasons=tuple(reasons))
