#!/usr/bin/env python3
"""Pure, offline, read-only batch aggregator for
SINGLE_ROBOT_GROUND_REPEATABILITY_BASELINE, added 2026-07-27.

Reads an explicit attempt manifest (never scans a directory and
silently selects files) plus each listed attempt's already-produced
`post_run_verification.json` (from ground_diagnostic_post_run_verifier.py),
validates internal consistency, applies the approved bounded retry
policy (max 3 attempts per trial slot; any EXCLUDED attempt ends the
whole batch), computes descriptive statistics (min/max/mean/sample
stddev) across VALID attempts only -- never inferential statistics --
and writes a machine-readable JSON report plus a human-readable
Markdown summary.

Never starts a process, never touches ROS, never modifies any
referenced evidence or verification file. Does not change
evaluate_verdict(), any acceptance threshold, the recorder, guard,
bridge, controller, protocol, or geometry.

Manifest schema (JSON, required, explicit -- see also
test_hil_repeatability_batch_aggregator.py for worked examples):
{
  "batch_id": "SRGRB_20260728",
  "spec_commit": "<git commit hash the batch was run against>",
  "attempts": [
    {
      "trial": 1,
      "attempt": 1,
      "run_id": "20260728_101500",
      "classification": "VALID" | "INVALID" | "EXCLUDED",
      "reason": null or "<short reason, required if not VALID>",
      "verification_json_path": "path/to/post_run_verification.json"
    },
    ...
  ]
}
"""
from __future__ import annotations

import json
import statistics
from dataclasses import dataclass, field
from typing import Optional

MAX_ATTEMPTS_PER_SLOT = 3
REQUIRED_TRIAL_SLOTS = (1, 2, 3, 4, 5)
VALID_CLASSIFICATIONS = ("VALID", "INVALID", "EXCLUDED")

# metric_name -> dotted path within a verification JSON's "facts" object.
METRIC_PATHS = {
    "requested_max_linear_mps": ("speed_summary", "requested_max_linear_mps"),
    "guarded_max_linear_mps": ("speed_summary", "guarded_max_linear_mps"),
    "max_abs_angular_rps": ("speed_summary", "max_abs_angular_rps"),
    "pi_applied_max_linear_mps": ("pi_command_maxima", "max_abs_linear_applied"),
    "pi_applied_max_angular_rps": ("pi_command_maxima", "max_abs_angular_applied"),
    "guarded_pulse_duration_s": ("guarded_pulse_durations_s", 0),
    "validity_flags_dropout_count": ("validity_flags_dropouts", "dropout_count"),
    "bridge_disconnected_sample_count": ("bridge_summary", "disconnected_sample_count"),
    "guarded_vs_pi_mismatched_pairs": ("guarded_vs_pi_mismatched_pairs",),
    "longitudinal_displacement_m": ("motion_metrics", "longitudinal_displacement_m"),
    "lateral_displacement_m": ("motion_metrics", "lateral_displacement_m"),
    "final_yaw_error_rad": ("motion_metrics", "final_yaw_error_rad"),
    "stop_line_clearance_m": ("motion_metrics", "stop_line_clearance_m"),
}


@dataclass(frozen=True)
class AttemptRecord:
    trial: int
    attempt: int
    run_id: str
    classification: str
    reason: Optional[str]
    verification_json_path: str


@dataclass(frozen=True)
class AttemptOutcome:
    attempt: AttemptRecord
    errors: tuple = field(default_factory=tuple)
    verification: Optional[dict] = None

    @property
    def counts_as_valid(self) -> bool:
        return self.attempt.classification == "VALID" and not self.errors


@dataclass(frozen=True)
class BatchAggregationResult:
    batch_status: str
    manifest_errors: tuple
    attempt_outcomes: tuple
    slot_fill: dict  # {trial: AttemptRecord or None}
    n_valid: int
    descriptive_stats: dict

    def to_dict(self) -> dict:
        return {
            "batch_status": self.batch_status,
            "manifest_errors": list(self.manifest_errors),
            "n_valid": self.n_valid,
            "slot_fill": {
                str(trial): (
                    {"attempt": rec.attempt, "run_id": rec.run_id} if rec is not None else None
                )
                for trial, rec in self.slot_fill.items()
            },
            "attempts": [
                {
                    "trial": o.attempt.trial,
                    "attempt": o.attempt.attempt,
                    "run_id": o.attempt.run_id,
                    "classification": o.attempt.classification,
                    "reason": o.attempt.reason,
                    "counts_as_valid": o.counts_as_valid,
                    "errors": list(o.errors),
                }
                for o in self.attempt_outcomes
            ],
            "descriptive_stats": self.descriptive_stats,
        }


def _parse_manifest(manifest: dict) -> tuple:
    """Returns (attempts: list[AttemptRecord], manifest_errors: list[str])."""
    errors = []
    raw_attempts = manifest.get("attempts")
    if not isinstance(raw_attempts, list) or not raw_attempts:
        return [], ["MANIFEST_HAS_NO_ATTEMPTS"]

    attempts = []
    seen_run_ids = set()
    seen_slot_attempt_pairs = set()
    for i, raw in enumerate(raw_attempts):
        trial = raw.get("trial")
        attempt_no = raw.get("attempt")
        run_id = raw.get("run_id")
        classification = raw.get("classification")
        reason = raw.get("reason")
        verification_json_path = raw.get("verification_json_path")

        if trial not in REQUIRED_TRIAL_SLOTS:
            errors.append(f"ATTEMPT[{i}]_TRIAL_OUT_OF_RANGE(got={trial})")
            continue
        if not isinstance(attempt_no, int) or attempt_no < 1:
            errors.append(f"ATTEMPT[{i}]_INVALID_ATTEMPT_NUMBER(got={attempt_no})")
            continue
        if classification not in VALID_CLASSIFICATIONS:
            errors.append(f"ATTEMPT[{i}]_INVALID_CLASSIFICATION(got={classification})")
            continue
        if classification != "VALID" and not reason:
            errors.append(f"ATTEMPT[{i}]_MISSING_REASON_FOR_{classification}")
        if not run_id:
            errors.append(f"ATTEMPT[{i}]_MISSING_RUN_ID")
            continue
        if not verification_json_path:
            errors.append(f"ATTEMPT[{i}]_MISSING_VERIFICATION_JSON_PATH")
            continue

        if run_id in seen_run_ids:
            errors.append(f"DUPLICATE_RUN_ID({run_id})")
        seen_run_ids.add(run_id)

        slot_attempt_pair = (trial, attempt_no)
        if slot_attempt_pair in seen_slot_attempt_pairs:
            errors.append(f"DUPLICATE_TRIAL_ATTEMPT_PAIR(trial={trial},attempt={attempt_no})")
        seen_slot_attempt_pairs.add(slot_attempt_pair)

        attempts.append(
            AttemptRecord(
                trial=trial,
                attempt=attempt_no,
                run_id=run_id,
                classification=classification,
                reason=reason,
                verification_json_path=verification_json_path,
            )
        )

    # Attempt numbers within a slot must be contiguous starting at 1 --
    # no gaps (an "omitted attempt"), no out-of-order numbering.
    by_slot: dict = {}
    for a in attempts:
        by_slot.setdefault(a.trial, []).append(a)
    for trial, slot_attempts in by_slot.items():
        numbers = sorted(a.attempt for a in slot_attempts)
        expected = list(range(1, len(numbers) + 1))
        if numbers != expected:
            errors.append(f"ATTEMPT_NUMBER_GAP_OR_DISORDER(trial={trial},got={numbers})")
        if len(numbers) > MAX_ATTEMPTS_PER_SLOT:
            errors.append(
                f"RETRY_BUDGET_EXCEEDED(trial={trial},attempts={len(numbers)},max={MAX_ATTEMPTS_PER_SLOT})"
            )

    return attempts, errors


def _load_verification(path: str) -> tuple:
    """Returns (parsed_dict_or_None, errors: list[str])."""
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh), []
    except FileNotFoundError:
        return None, [f"VERIFICATION_JSON_NOT_FOUND({path})"]
    except json.JSONDecodeError as exc:
        return None, [f"VERIFICATION_JSON_MALFORMED({path}:{exc})"]


def _validate_attempt_against_verification(attempt: AttemptRecord, verification: Optional[dict]) -> list:
    errors = []
    if verification is None:
        return errors  # already reported by _load_verification
    if verification.get("run_id") != attempt.run_id:
        errors.append(
            f"RUN_ID_MISMATCH_WITH_MANIFEST(manifest={attempt.run_id},verification={verification.get('run_id')})"
        )
    if not verification.get("file_checks"):
        errors.append("VERIFICATION_JSON_MISSING_FILE_CHECKS")
    if attempt.classification == "VALID":
        if verification.get("integrity_ok") is not True:
            errors.append("VALID_ATTEMPT_INTEGRITY_NOT_OK")
        if verification.get("diagnostic_verdict") != "PASS":
            errors.append(f"VALID_ATTEMPT_VERDICT_NOT_PASS(got={verification.get('diagnostic_verdict')})")
        if verification.get("motion_metrics_required") is not True:
            errors.append("VALID_ATTEMPT_MOTION_METRICS_NOT_REQUIRED")
        if verification.get("motion_metrics_ok") is not True:
            errors.append("VALID_ATTEMPT_MOTION_METRICS_NOT_OK")
    return errors


def _get_metric(verification: dict, path: tuple):
    node = verification.get("facts", {})
    for key in path:
        if isinstance(node, dict):
            node = node.get(key)
        elif isinstance(node, list):
            node = node[key] if isinstance(key, int) and key < len(node) else None
        else:
            return None
    return node


def _descriptive_stats(values: list) -> dict:
    clean = [v for v in values if isinstance(v, (int, float))]
    if not clean:
        return {"n": 0, "min": None, "max": None, "mean": None, "sample_stddev": None}
    return {
        "n": len(clean),
        "min": min(clean),
        "max": max(clean),
        "mean": statistics.mean(clean),
        "sample_stddev": statistics.stdev(clean) if len(clean) >= 2 else None,
    }


def aggregate_batch(manifest: dict) -> BatchAggregationResult:
    attempts, manifest_errors = _parse_manifest(manifest)

    outcomes = []
    verifications_by_attempt = {}
    for attempt in attempts:
        verification, load_errors = _load_verification(attempt.verification_json_path)
        consistency_errors = _validate_attempt_against_verification(attempt, verification)
        outcomes.append(
            AttemptOutcome(
                attempt=attempt,
                errors=tuple(load_errors + consistency_errors),
                verification=verification,
            )
        )
        verifications_by_attempt[(attempt.trial, attempt.attempt)] = verification

    hard_errors = list(manifest_errors)

    # Slot fill: exactly one VALID (error-free) attempt should fill each slot.
    slot_fill: dict = {trial: None for trial in REQUIRED_TRIAL_SLOTS}
    for trial in REQUIRED_TRIAL_SLOTS:
        slot_valid = [o for o in outcomes if o.attempt.trial == trial and o.counts_as_valid]
        if len(slot_valid) > 1:
            hard_errors.append(f"MULTIPLE_VALID_ATTEMPTS_IN_SLOT(trial={trial})")
        elif len(slot_valid) == 1:
            slot_fill[trial] = slot_valid[0].attempt

    # Any EXCLUDED attempt ends the whole batch -- and must be the last
    # attempt chronologically listed in the manifest (manifest order is
    # the execution order, per this module's contract).
    excluded_indices = [i for i, o in enumerate(outcomes) if o.attempt.classification == "EXCLUDED"]
    excluded_not_last = bool(excluded_indices) and excluded_indices[-1] != len(outcomes) - 1

    n_valid = sum(1 for rec in slot_fill.values() if rec is not None)

    if hard_errors or excluded_not_last:
        if excluded_not_last:
            hard_errors.append("ATTEMPTS_CONTINUED_AFTER_EXCLUDED_ATTEMPT")
        batch_status = "BATCH_INVALID_PROTOCOL"
    elif excluded_indices:
        batch_status = "BATCH_ABORTED_EXCLUDED"
    elif n_valid == len(REQUIRED_TRIAL_SLOTS):
        batch_status = "BATCH_COMPLETE"
    else:
        batch_status = "INCOMPLETE_BATCH"

    descriptive_stats = {}
    if batch_status in ("BATCH_COMPLETE", "INCOMPLETE_BATCH"):
        valid_verifications = [
            verifications_by_attempt[(trial, rec.attempt)] for trial, rec in slot_fill.items() if rec is not None
        ]
        for metric_name, path in METRIC_PATHS.items():
            values = [_get_metric(v, path) for v in valid_verifications]
            descriptive_stats[metric_name] = _descriptive_stats(values)

    return BatchAggregationResult(
        batch_status=batch_status,
        manifest_errors=tuple(hard_errors),
        attempt_outcomes=tuple(outcomes),
        slot_fill=slot_fill,
        n_valid=n_valid,
        descriptive_stats=descriptive_stats,
    )


def render_markdown(batch_id: str, spec_commit: str, result: BatchAggregationResult) -> str:
    lines = [
        f"# Batch summary -- {batch_id}",
        "",
        f"Specification/frozen commit: `{spec_commit}`",
        "",
        f"**BATCH_STATUS = {result.batch_status}**",
        f"n_valid = {result.n_valid}/{len(REQUIRED_TRIAL_SLOTS)}"
        + (
            ""
            if result.batch_status == "BATCH_COMPLETE"
            else " (INCOMPLETE -- descriptive values below are partial, not a completed batch)"
            if result.batch_status == "INCOMPLETE_BATCH"
            else ""
        ),
        "",
    ]
    if result.manifest_errors:
        lines.append("## Manifest/protocol errors")
        lines.extend(f"- {e}" for e in result.manifest_errors)
        lines.append("")

    lines.append("## Attempts")
    lines.append("| Trial | Attempt | Run ID | Classification | Reason | Counts as valid | Errors |")
    lines.append("|---|---|---|---|---|---|---|")
    for o in result.attempt_outcomes:
        a = o.attempt
        lines.append(
            f"| {a.trial} | {a.attempt} | {a.run_id} | {a.classification} | {a.reason or ''} | "
            f"{o.counts_as_valid} | {'; '.join(o.errors) if o.errors else ''} |"
        )
    lines.append("")

    if result.descriptive_stats:
        lines.append("## Descriptive statistics (valid attempts only, no inferential claim)")
        lines.append("| Metric | n | min | max | mean | sample stddev |")
        lines.append("|---|---|---|---|---|---|")
        for metric_name, stats in result.descriptive_stats.items():
            lines.append(
                f"| {metric_name} | {stats['n']} | {stats['min']} | {stats['max']} | "
                f"{stats['mean']} | {stats['sample_stddev']} |"
            )
        lines.append("")

    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="Offline, read-only batch aggregator for SINGLE_ROBOT_GROUND_REPEATABILITY_BASELINE.")
    parser.add_argument("--batch-id", required=True)
    parser.add_argument("--spec-commit", required=True)
    parser.add_argument("--attempts-manifest", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-md", required=True)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    with open(args.attempts_manifest, encoding="utf-8") as fh:
        manifest = json.load(fh)

    result = aggregate_batch(manifest)

    output = result.to_dict()
    output["batch_id"] = args.batch_id
    output["spec_commit"] = args.spec_commit

    with open(args.output_json, "w", encoding="utf-8") as fh:
        json.dump(output, fh, indent=2)
        fh.write("\n")

    with open(args.output_md, "w", encoding="utf-8") as fh:
        fh.write(render_markdown(args.batch_id, args.spec_commit, result))

    print(f"BATCH_STATUS={result.batch_status}")
    print(f"N_VALID={result.n_valid}/{len(REQUIRED_TRIAL_SLOTS)}")
    print(f"MANIFEST_ERRORS={list(result.manifest_errors)}")
    return 0 if result.batch_status == "BATCH_COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
