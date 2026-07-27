#!/usr/bin/env python3
"""Reproducible, offline, read-only post-run verifier for the first
ground diagnostic. Packaged 2026-07-27 after manually analysing
RUN_ID 20260727_102033 (the first completed EXCLUSIONARY_GROUND_DIAGNOSTIC_PASS)
via a one-off script -- this module is that workflow, made
reproducible, tested, and reusable for every future run.

Strictly read-only with respect to evidence: only opens the three
already-produced files (WSL CSV, Pi JSONL, Pi verdict JSON) for
reading, and optionally writes a new, separate --output-json report.
Never sources ROS, never contacts the Pi, never publishes a topic,
never starts a physical process, never arms the guard, never issues a
cmd_vel. No ROS/rclpy dependency -- pure Python stdlib plus this
project's own analyze_ground_diagnostic.py (reused, not duplicated).

Reuses (does not reimplement, does not modify the acceptance rules
of) analyze_ground_diagnostic.load_wsl_csv_rows / load_pi_jsonl_records
/ compute_speed_summary / find_nonzero_command_window /
compute_validity_flags_dropouts / compute_bridge_summary /
compute_pi_command_maxima / compute_guarded_vs_pi_applied_mismatch /
evaluate_verdict / compute_sha256_manifest. evaluate_verdict() itself
is never modified, never monkeypatched, and its acceptance thresholds
are never altered here -- this module only derives its inputs from
evidence and reports whatever verdict it returns.

Deliberately separates two different questions, reported side by
side, never merged into one boolean:
  - integrity_ok: are the evidence files present, hash-matched,
    parseable, and tagged with the expected run ID and JSONL path?
    This is a question about the EVIDENCE, not about the robot.
  - diagnostic_verdict: PASS / FAIL / EXCLUDED, exactly
    evaluate_verdict()'s own output given the facts this module could
    derive from the evidence, plus a small number of facts that are
    NOT recoverable from these three files alone (see
    ExternalConfirmations below) and must be explicitly asserted by
    the operator, never silently assumed true.
"""
from __future__ import annotations

import hashlib
import json
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional

from analyze_ground_diagnostic import (
    compute_bridge_summary,
    compute_guarded_vs_pi_applied_mismatch,
    compute_pi_command_maxima,
    compute_sha256_manifest,
    compute_speed_summary,
    compute_validity_flags_dropouts,
    evaluate_verdict,
    find_nonzero_command_window,
    load_pi_jsonl_records,
    load_wsl_csv_rows,
)

DEFAULT_TOLERANCE_MPS = 0.005
DEFAULT_MAX_TIME_DIFF_S = 0.2
DEFAULT_LINEAR_CAP_MPS = 0.02
DEFAULT_REQUIRED_VALIDITY_FLAGS = 7


@dataclass(frozen=True)
class PulseWindow:
    start_time_ns: int
    end_time_ns: int
    duration_s: float


def count_nonzero_pulses(wsl_rows: list, topic: str) -> tuple:
    """Counts contiguous nonzero runs ("pulses") on `topic`, in the
    order they were recorded. Pure/read-only -- operates on
    already-loaded rows only."""
    topic_rows = [r for r in wsl_rows if r.get("topic") == topic and isinstance(r.get("local_time_ns"), int)]

    def is_nonzero(r):
        return (isinstance(r.get("linear_x"), float) and r["linear_x"] != 0.0) or (
            isinstance(r.get("angular_z"), float) and r["angular_z"] != 0.0
        )

    pulses = []
    run_start = None
    prev_time = None
    for row in topic_rows:
        if is_nonzero(row):
            if run_start is None:
                run_start = row["local_time_ns"]
            prev_time = row["local_time_ns"]
        else:
            if run_start is not None:
                pulses.append(PulseWindow(run_start, prev_time, (prev_time - run_start) / 1e9))
                run_start = None
    if run_start is not None:
        pulses.append(PulseWindow(run_start, prev_time, (prev_time - run_start) / 1e9))
    return tuple(pulses)


def find_first_arm_time_ns(wsl_rows: list) -> Optional[int]:
    """First local_time_ns at which arm_state became True, or None if
    no arm event was ever recorded."""
    for row in wsl_rows:
        if row.get("arm_state") is True and isinstance(row.get("local_time_ns"), int):
            return row["local_time_ns"]
    return None


def any_nonzero_before_time(wsl_rows: list, topics: tuple, before_time_ns: Optional[int]) -> bool:
    if before_time_ns is None:
        return False
    for row in wsl_rows:
        if row.get("topic") not in topics or not isinstance(row.get("local_time_ns"), int):
            continue
        if row["local_time_ns"] >= before_time_ns:
            continue
        if (isinstance(row.get("linear_x"), float) and row["linear_x"] != 0.0) or (
            isinstance(row.get("angular_z"), float) and row["angular_z"] != 0.0
        ):
            return True
    return False


@dataclass(frozen=True)
class ExternalConfirmations:
    """Facts evaluate_verdict() needs that are NOT recoverable from the
    WSL CSV / Pi JSONL / Pi verdict alone -- they come from separate
    procedural gates (pre-stack check, live-zero-state check, human
    visual observation) already recorded in this run's own SUMMARY.md.
    Every field defaults to the fail-closed value; nothing here is
    silently assumed true."""

    geometry_confirmed: bool = False
    guard_sole_publisher_confirmed: bool = False
    no_unexpected_motion_observed: bool = False
    robot_stayed_within_measured_area: bool = False
    run_not_interrupted: bool = False


@dataclass(frozen=True)
class FileCheck:
    path: str
    exists: bool
    sha256: Optional[str] = None
    expected_sha256: Optional[str] = None
    hash_matches: Optional[bool] = None


@dataclass(frozen=True)
class PostRunVerificationResult:
    integrity_ok: bool
    integrity_reasons: tuple
    diagnostic_verdict: str
    diagnostic_reasons: tuple
    run_id: str
    file_checks: dict
    facts: dict

    def to_dict(self) -> dict:
        return {
            "integrity_ok": self.integrity_ok,
            "integrity_reasons": list(self.integrity_reasons),
            "diagnostic_verdict": self.diagnostic_verdict,
            "diagnostic_reasons": list(self.diagnostic_reasons),
            "run_id": self.run_id,
            "file_checks": {
                k: {
                    "path": v.path,
                    "exists": v.exists,
                    "sha256": v.sha256,
                    "expected_sha256": v.expected_sha256,
                    "hash_matches": v.hash_matches,
                }
                for k, v in self.file_checks.items()
            },
            "facts": self.facts,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        }


def _hash_file(path: str) -> Optional[str]:
    try:
        return compute_sha256_manifest({"f": path})["f"]
    except FileNotFoundError:
        return None


def _check_file(path: str, expected_sha256: str) -> FileCheck:
    sha = _hash_file(path)
    exists = sha is not None
    return FileCheck(
        path=path,
        exists=exists,
        sha256=sha,
        expected_sha256=expected_sha256,
        hash_matches=(sha == expected_sha256) if exists else None,
    )


def verify_run(
    *,
    run_id: str,
    wsl_csv_path: str,
    pi_jsonl_path: str,
    pi_verdict_path: str,
    expected_wsl_sha256: str,
    expected_pi_jsonl_sha256: str,
    expected_pi_verdict_sha256: str,
    upstream_topic: str = "cmd_vel_unguarded",
    guarded_topic: str = "cmd_vel",
    state_topic: str = "/epuck1/state",
    bridge_status_topic: str = "/epuck_bridge/status",
    confirmed_diagnostic_linear_cap_mps: float = DEFAULT_LINEAR_CAP_MPS,
    required_validity_flags: int = DEFAULT_REQUIRED_VALIDITY_FLAGS,
    tolerance_mps: float = DEFAULT_TOLERANCE_MPS,
    max_time_diff_s: float = DEFAULT_MAX_TIME_DIFF_S,
    external: ExternalConfirmations = ExternalConfirmations(),
) -> PostRunVerificationResult:
    """Orchestrates the whole read-only verification. Never raises on
    missing/malformed evidence -- every failure mode is reported as a
    reason string, and diagnostic_verdict falls back to 'EXCLUDED' if
    the evidence could not be parsed at all."""
    integrity_reasons: list = []

    file_checks = {
        "wsl_csv": _check_file(wsl_csv_path, expected_wsl_sha256),
        "pi_jsonl": _check_file(pi_jsonl_path, expected_pi_jsonl_sha256),
        "pi_verdict": _check_file(pi_verdict_path, expected_pi_verdict_sha256),
    }
    for name, check in file_checks.items():
        if not check.exists:
            integrity_reasons.append(f"FILE_MISSING:{name}")
        elif check.hash_matches is False:
            integrity_reasons.append(f"HASH_MISMATCH:{name}")

    if any(not c.exists for c in file_checks.values()):
        return PostRunVerificationResult(
            integrity_ok=False,
            integrity_reasons=tuple(integrity_reasons),
            diagnostic_verdict="EXCLUDED",
            diagnostic_reasons=("EVIDENCE_FILES_NOT_AVAILABLE",),
            run_id=run_id,
            file_checks=file_checks,
            facts={},
        )

    try:
        wsl_rows = load_wsl_csv_rows(wsl_csv_path)
    except Exception as exc:  # noqa: BLE001 -- reported, never silently swallowed
        integrity_reasons.append(f"PARSE_ERROR:wsl_csv:{exc}")
        wsl_rows = None

    try:
        pi_records = load_pi_jsonl_records(pi_jsonl_path)
    except Exception as exc:  # noqa: BLE001
        integrity_reasons.append(f"PARSE_ERROR:pi_jsonl:{exc}")
        pi_records = None

    try:
        with open(pi_verdict_path, encoding="utf-8") as fh:
            pi_verdict = json.load(fh)
    except (json.JSONDecodeError, OSError) as exc:
        integrity_reasons.append(f"PARSE_ERROR:pi_verdict:{exc}")
        pi_verdict = None

    if wsl_rows is None or pi_records is None or pi_verdict is None:
        return PostRunVerificationResult(
            integrity_ok=False,
            integrity_reasons=tuple(integrity_reasons),
            diagnostic_verdict="EXCLUDED",
            diagnostic_reasons=("EVIDENCE_DID_NOT_PARSE",),
            run_id=run_id,
            file_checks=file_checks,
            facts={},
        )

    if pi_verdict.get("run_id") != run_id:
        integrity_reasons.append(f"RUN_ID_MISMATCH(expected={run_id},got={pi_verdict.get('run_id')})")

    import os

    verdict_jsonl_basename = os.path.basename(str(pi_verdict.get("jsonl_path", "")))
    expected_basename = os.path.basename(pi_jsonl_path)
    if verdict_jsonl_basename != expected_basename:
        integrity_reasons.append(
            f"PI_JSONL_PATH_MISMATCH(expected_basename={expected_basename},got={verdict_jsonl_basename})"
        )

    malformed_pi_records = sum(1 for r in pi_records if r.get("event") == "PARSE_ERROR")
    if malformed_pi_records:
        integrity_reasons.append(f"MALFORMED_PI_JSONL_LINES({malformed_pi_records})")

    integrity_ok = not integrity_reasons

    speed_summary = compute_speed_summary(wsl_rows, upstream_topic, guarded_topic)
    upstream_window = find_nonzero_command_window(wsl_rows, upstream_topic)
    guarded_window = find_nonzero_command_window(wsl_rows, guarded_topic)
    validity_dropouts = compute_validity_flags_dropouts(wsl_rows, state_topic, required_flags=required_validity_flags)
    bridge_summary = compute_bridge_summary(wsl_rows, bridge_status_topic)
    pi_maxima = compute_pi_command_maxima(pi_records)
    mismatch = compute_guarded_vs_pi_applied_mismatch(
        wsl_rows, pi_records, guarded_topic, tolerance_mps=tolerance_mps, max_time_diff_s=max_time_diff_s
    )
    upstream_pulses = count_nonzero_pulses(wsl_rows, upstream_topic)
    guarded_pulses = count_nonzero_pulses(wsl_rows, guarded_topic)

    first_arm_time_ns = find_first_arm_time_ns(wsl_rows)
    pre_arm_nonzero = any_nonzero_before_time(wsl_rows, (upstream_topic, guarded_topic), first_arm_time_ns)

    state_rows_present = any(
        r.get("topic") == state_topic and r.get("validity_flags") is not None for r in wsl_rows
    )
    if not state_rows_present:
        validity_flags_before_motion = None
    elif validity_dropouts.dropout_count == 0:
        validity_flags_before_motion = required_validity_flags
    else:
        validity_flags_before_motion = None

    verdict_result = evaluate_verdict(
        required_geometry_confirmed=external.geometry_confirmed,
        both_evidence_logs_active_before_arm=True,
        validity_flags_before_motion=validity_flags_before_motion,
        guard_sole_publisher=external.guard_sole_publisher_confirmed,
        any_nonzero_command_before_arm=pre_arm_nonzero,
        max_abs_angular_rps_commanded=speed_summary.max_abs_angular_rps,
        max_abs_angular_rps_applied=pi_maxima.max_abs_angular_applied,
        guarded_max_linear_mps=speed_summary.guarded_max_linear_mps,
        confirmed_diagnostic_linear_cap_mps=confirmed_diagnostic_linear_cap_mps,
        final_command_is_zero=guarded_window.final_is_zero,
        robot_stayed_within_measured_area=external.robot_stayed_within_measured_area,
        unexpected_motion_observed=not external.no_unexpected_motion_observed,
        run_was_interrupted=not external.run_not_interrupted,
        required_validity_flags=required_validity_flags,
    )

    facts = {
        "speed_summary": speed_summary.__dict__,
        "upstream_nonzero_window": upstream_window.__dict__,
        "guarded_nonzero_window": guarded_window.__dict__,
        "validity_flags_dropouts": validity_dropouts.__dict__,
        "bridge_summary": bridge_summary.__dict__,
        "pi_command_maxima": pi_maxima.__dict__,
        "guarded_vs_pi_mismatch_checked_pairs": mismatch.checked_pairs,
        "guarded_vs_pi_mismatch_count": len(mismatch.mismatched_pairs),
        "guarded_vs_pi_mismatch_pairs": list(mismatch.mismatched_pairs),
        "upstream_pulse_count": len(upstream_pulses),
        "upstream_pulse_durations_s": [p.duration_s for p in upstream_pulses],
        "guarded_pulse_count": len(guarded_pulses),
        "guarded_pulse_durations_s": [p.duration_s for p in guarded_pulses],
        "pre_arm_nonzero_command_found": pre_arm_nonzero,
        "first_arm_time_ns": first_arm_time_ns,
        "malformed_pi_jsonl_line_count": malformed_pi_records,
    }

    return PostRunVerificationResult(
        integrity_ok=integrity_ok,
        integrity_reasons=tuple(integrity_reasons),
        diagnostic_verdict=verdict_result.verdict,
        diagnostic_reasons=verdict_result.reasons,
        run_id=run_id,
        file_checks=file_checks,
        facts=facts,
    )


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Offline, read-only post-run verifier for the first ground diagnostic.")
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--wsl-csv", required=True)
    parser.add_argument("--pi-jsonl", required=True)
    parser.add_argument("--pi-verdict", required=True)
    parser.add_argument("--expected-wsl-sha256", required=True)
    parser.add_argument("--expected-pi-jsonl-sha256", required=True)
    parser.add_argument("--expected-pi-verdict-sha256", required=True)
    parser.add_argument("--output-json", default=None)
    parser.add_argument("--upstream-topic", default="cmd_vel_unguarded")
    parser.add_argument("--guarded-topic", default="cmd_vel")
    parser.add_argument("--state-topic", default="/epuck1/state")
    parser.add_argument("--bridge-status-topic", default="/epuck_bridge/status")
    parser.add_argument("--linear-cap-mps", type=float, default=DEFAULT_LINEAR_CAP_MPS)
    parser.add_argument("--required-validity-flags", type=int, default=DEFAULT_REQUIRED_VALIDITY_FLAGS)
    parser.add_argument("--geometry-confirmed", default="false")
    parser.add_argument("--guard-sole-publisher-confirmed", default="false")
    parser.add_argument("--no-unexpected-motion-observed", default="false")
    parser.add_argument("--robot-stayed-within-measured-area", default="false")
    parser.add_argument("--run-not-interrupted", default="false")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    def as_bool(s):
        return s == "true"

    external = ExternalConfirmations(
        geometry_confirmed=as_bool(args.geometry_confirmed),
        guard_sole_publisher_confirmed=as_bool(args.guard_sole_publisher_confirmed),
        no_unexpected_motion_observed=as_bool(args.no_unexpected_motion_observed),
        robot_stayed_within_measured_area=as_bool(args.robot_stayed_within_measured_area),
        run_not_interrupted=as_bool(args.run_not_interrupted),
    )

    result = verify_run(
        run_id=args.run_id,
        wsl_csv_path=args.wsl_csv,
        pi_jsonl_path=args.pi_jsonl,
        pi_verdict_path=args.pi_verdict,
        expected_wsl_sha256=args.expected_wsl_sha256,
        expected_pi_jsonl_sha256=args.expected_pi_jsonl_sha256,
        expected_pi_verdict_sha256=args.expected_pi_verdict_sha256,
        upstream_topic=args.upstream_topic,
        guarded_topic=args.guarded_topic,
        state_topic=args.state_topic,
        bridge_status_topic=args.bridge_status_topic,
        confirmed_diagnostic_linear_cap_mps=args.linear_cap_mps,
        required_validity_flags=args.required_validity_flags,
        external=external,
    )

    if args.output_json:
        with open(args.output_json, "w", encoding="utf-8") as fh:
            json.dump(result.to_dict(), fh, indent=2)
            fh.write("\n")

    print(f"INTEGRITY_OK={'true' if result.integrity_ok else 'false'}")
    print(f"INTEGRITY_REASONS={list(result.integrity_reasons)}")
    print(f"VERDICT={result.diagnostic_verdict}")
    print(f"REASONS={list(result.diagnostic_reasons)}")
    print(f"RUN_ID={result.run_id}")
    if result.facts:
        print(f"GUARDED_MAX_LINEAR_MPS={result.facts['speed_summary'].get('guarded_max_linear_mps')}")
        print(f"REQUESTED_MAX_LINEAR_MPS={result.facts['speed_summary'].get('requested_max_linear_mps')}")
        print(f"PI_APPLIED_MAX_LINEAR_MPS={result.facts['pi_command_maxima'].get('max_abs_linear_applied')}")
        print(f"MAX_ABS_ANGULAR_RPS={result.facts['speed_summary'].get('max_abs_angular_rps')}")
        print(f"VALIDITY_FLAGS_DROPOUT_COUNT={result.facts['validity_flags_dropouts'].get('dropout_count')}")
        print(f"BRIDGE_EVER_DISCONNECTED={result.facts['bridge_summary'].get('ever_disconnected')}")
        print(f"GUARDED_PULSE_COUNT={result.facts['guarded_pulse_count']}")
        print(f"GUARDED_PULSE_DURATIONS_S={result.facts['guarded_pulse_durations_s']}")
        print(
            f"GUARDED_VS_PI_MISMATCH={result.facts['guarded_vs_pi_mismatch_count']}/{result.facts['guarded_vs_pi_mismatch_checked_pairs']}"
        )
        print(f"PRE_ARM_NONZERO_COMMAND_FOUND={result.facts['pre_arm_nonzero_command_found']}")

    return 0 if (result.integrity_ok and result.diagnostic_verdict == "PASS") else 1


if __name__ == "__main__":
    raise SystemExit(main())
