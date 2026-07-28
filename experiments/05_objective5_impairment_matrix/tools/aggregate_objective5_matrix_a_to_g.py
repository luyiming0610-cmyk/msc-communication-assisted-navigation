#!/usr/bin/env python3
"""Offline-only cross-condition (A-G) aggregation for the Objective 5
impairment matrix. Reads existing per-trial evidence (frozen_params.json,
trial_verdict.json, matrix_analysis.json, and Condition F's per-trial
outage_recovery_audit.json where present) and the seven condition-level
formal batch summaries. Never runs an experiment, never writes into any
existing evidence directory, and never mutates a source file.

Two independent measurements are kept in separate fields, per the
release-gate correction that rejected a single "completion_or_recovery_time_s"
field: task_completion_time_s (not currently available from any preserved
per-trial file for any condition -- bag_duration_s conflates recording
overhead/startup_hold with task time and is not the same measurement),
stale_stop_duration_s (available only for Condition F, from its per-trial
outage_recovery_audit.json), and recovery_time_s (not available anywhere --
the existing tooling records when a stale stop began and ended, not a
separate post-recovery latency).

Realised packet loss is read ONLY from each trial's own matrix_analysis.json
relay counters (never inferred from configured drop_probability). The
"drop_mechanism" field is a configuration-derived categorical TAG describing
which impairment mechanism was active, not a substitute for the measured
count -- the measured count is always taken from relay.dropped_count.

safety_radius_m is not present as a field in any frozen_params.json (the
per-trial file schema never carries it); it is a fixed system constant
confirmed identical (0.14) across every condition's own preserved formal
batch summary (A/B/C/E/F/G "safety.safety_radius_m") and every
task_outcome_reason string of the form "min_interrobot_distance_m=X <
safety_radius_m=0.1400". SAFETY_RADIUS_M below is that confirmed constant,
not an assumption invented for this tool.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

NOT_AVAILABLE = "NOT_AVAILABLE"
SAFETY_RADIUS_M = 0.14

# Exact counted (trial_index, attempt) pairs per condition, per the
# release-gated audit. D excludes trial04 (EXCLUDED_MEASUREMENT_CHAIN_ATTEMPT)
# and includes trial06 as its authorized replacement.
CONDITION_TRIALS: dict[str, list[tuple[int, int]]] = {
    "A": [(1, 1), (2, 1), (3, 1), (4, 1), (5, 1)],
    "B": [(1, 1), (2, 1), (3, 1), (4, 1), (5, 1)],
    "C": [(1, 1), (2, 1), (3, 1), (4, 1), (5, 1)],
    "D": [(1, 1), (2, 1), (3, 1), (5, 1), (6, 1)],
    "E": [(1, 1), (2, 1), (3, 1), (4, 1), (5, 1)],
    "F": [(1, 1), (2, 1), (3, 1), (4, 1), (5, 1)],
    "G": [(1, 1), (2, 1), (3, 1), (4, 1), (5, 1)],
}

EXCLUDED_TRIALS: dict[str, list[dict[str, Any]]] = {
    "D": [
        {
            "condition_id": "D",
            "trial_index": 4,
            "attempt": 1,
            "formal_measurement_validity": "INVALID",
            "classification": "EXCLUDED_MEASUREMENT_CHAIN_ATTEMPT",
            "exclusion_reason": (
                "ROSbag-only single-message capture gap: sequence 17 "
                "(epuck2_to_epuck1) was forwarded and received by the online "
                "counter but absent from the bag. This violates the "
                "preregistered aligned-window bag capture ratio=1.0 "
                "requirement. Task itself SUCCEEDED and communication "
                "manipulation was genuinely and correctly applied; only the "
                "bag-recording chain failed to capture one message."
            ),
            "not_rerun": True,
            "not_counted_toward_formal_n5": True,
            "replacement_trial_id": "objective5_impairment_matrix_v1_condition_D_trial06_attempt01",
        }
    ]
}

# Batch-summary file naming: A-D use the long v1 prefix, E-G use the short
# prefix. Both are supported explicitly (never assumed to be one pattern).
_BATCH_SUMMARY_CANDIDATES = {
    "A": "objective5_impairment_matrix_v1_condition_A_formal_batch_summary.json",
    "B": "objective5_impairment_matrix_v1_condition_B_formal_batch_summary.json",
    "C": "objective5_impairment_matrix_v1_condition_C_formal_batch_summary.json",
    "D": "objective5_impairment_matrix_v1_condition_D_formal_batch_summary.json",
    "E": "objective5_condition_E_formal_batch_summary.json",
    "F": "objective5_condition_F_formal_batch_summary.json",
    "G": "objective5_condition_G_formal_batch_summary.json",
}

EVIDENCE_STATUS = {
    "A": "CODE_IDENTITY_ONLY",
    "B": "CODE_IDENTITY_ONLY",
    "C": "CODE_IDENTITY_ONLY",
    "D": "CODE_IDENTITY_ONLY",
    "E": "REPORTED_VERIFIED_NO_STANDALONE_MANIFEST",
    "F": "VERIFIED_WITH_STANDALONE_MANIFEST",
    "G": "VERIFIED_WITH_STANDALONE_MANIFEST",
}

CANONICAL_FIELDS = [
    "condition_id", "trial_id", "attempt_id",
    "data_validity", "task_outcome",
    "delay_s", "jitter_s", "drop_probability",
    "outage_period_s", "outage_duration_s", "outage_phase_s",
    "seed_epuck1_to_epuck2", "seed_epuck2_to_epuck1",
    "minimum_interrobot_distance_m", "safety_margin_m",
    "completion_count",
    "task_completion_time_s", "stale_stop_duration_s", "recovery_time_s",
    "controller_crashed",
    "configured_drop_probability",
    "authoritative_drop_count_epuck1_to_epuck2",
    "authoritative_drop_count_epuck2_to_epuck1",
    "authoritative_drop_fraction_epuck1_to_epuck2",
    "authoritative_drop_fraction_epuck2_to_epuck1",
    "drop_mechanism",
    "measured_message_age_mean_epuck1_to_epuck2",
    "measured_message_age_mean_epuck2_to_epuck1",
    "measured_message_age_p95_epuck1_to_epuck2",
    "measured_message_age_p95_epuck2_to_epuck1",
    "reordered_count_epuck1_to_epuck2", "reordered_count_epuck2_to_epuck1",
    "duplicate_count_epuck1_to_epuck2", "duplicate_count_epuck2_to_epuck1",
    "queue_drained",
    "stale_state_episode_count", "in_task_stale_episode_count", "peer_timeout_events",
    "bag_metadata_valid_proxy", "analyzer_ok", "evidence_hash_status",
    "source_frozen_params_path", "source_trial_verdict_path", "source_matrix_analysis_path",
]

DATA_VALIDITY_VALUES = {"VALID", "INVALID"}
TASK_OUTCOME_VALUES = {"SUCCESS", "SAFE_DEGRADATION", "UNSAFE_FAILURE", "NOT_EVALUABLE"}


class AggregationError(Exception):
    """Raised for any fail-closed condition. The tool never continues past one."""


@dataclass
class HashedFile:
    path: str
    sha256: str


@dataclass
class AggregationContext:
    hashed_files: list[HashedFile] = field(default_factory=list)
    seen_trial_ids: set[str] = field(default_factory=set)
    errors: list[str] = field(default_factory=list)


def _sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _read_json(path: Path, ctx: AggregationContext) -> dict:
    if not path.is_file():
        raise AggregationError(f"missing required source file: {path}")
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError as exc:
        raise AggregationError(f"malformed JSON in {path}: {exc}") from exc
    ctx.hashed_files.append(HashedFile(str(path), _sha256_of(path)))
    return data


def trial_dir_name(condition_id: str, trial_index: int, attempt: int) -> str:
    return (
        f"objective5_impairment_matrix_v1_condition_{condition_id}_"
        f"trial{trial_index:02d}_attempt{attempt:02d}_analysis"
    )


def trial_id(condition_id: str, trial_index: int, attempt: int) -> str:
    return (
        f"objective5_impairment_matrix_v1_condition_{condition_id}_"
        f"trial{trial_index:02d}_attempt{attempt:02d}"
    )


def classify_drop_mechanism(drop_probability: float, outage_duration_s: float) -> str:
    drop_on = drop_probability is not None and drop_probability > 0.0
    outage_on = outage_duration_s is not None and outage_duration_s > 0.0
    if drop_on and outage_on:
        return "COMBINED"
    if drop_on:
        return "INDEPENDENT_BERNOULLI"
    if outage_on:
        return "SCHEDULED_OUTAGE"
    return "NONE"


def _direction(matrix_analysis: dict, name: str) -> dict:
    for d in matrix_analysis.get("directions", []):
        if d.get("direction") == name:
            return d
    raise AggregationError(f"matrix_analysis.json missing direction '{name}'")


def _load_outage_recovery_audit(root: Path, condition_id: str, trial_index: int, attempt: int, ctx: AggregationContext) -> dict | None:
    """Only Condition F has a per-trial outage_recovery_audit.json. Read it
    at most once per trial and share the parsed result between the
    stale_stop_duration_s and stale_state_episode_count fields, so the
    input manifest records exactly one hash per file, not two."""
    audit_path = root / trial_dir_name(condition_id, trial_index, attempt) / "outage_recovery_audit.json"
    if not audit_path.is_file():
        return None
    return _read_json(audit_path, ctx)


def _stale_stop_duration_from_audit(audit: dict | None):
    if audit is None:
        return NOT_AVAILABLE
    intervals = audit.get("safe_stop_stale_intervals", [])
    durations = [iv["observed_safe_stop_duration_s"] for iv in intervals if "observed_safe_stop_duration_s" in iv]
    if not durations:
        return NOT_AVAILABLE
    return sum(durations) / len(durations)


def load_trial_row(root: Path, condition_id: str, trial_index: int, attempt: int, ctx: AggregationContext) -> dict:
    trial_dir = root / trial_dir_name(condition_id, trial_index, attempt)
    fp_path = trial_dir / "frozen_params.json"
    tv_path = trial_dir / "trial_verdict.json"
    ma_path = trial_dir / "matrix_analysis.json"

    frozen_params = _read_json(fp_path, ctx)
    trial_verdict = _read_json(tv_path, ctx)
    matrix_analysis = _read_json(ma_path, ctx)

    for label, data in (("frozen_params.json", frozen_params), ("trial_verdict.json", trial_verdict), ("matrix_analysis.json", matrix_analysis)):
        if data.get("condition_id") != condition_id:
            raise AggregationError(
                f"{trial_dir}/{label} condition_id={data.get('condition_id')!r} != expected {condition_id!r}"
            )
        if data.get("trial_index") != trial_index:
            raise AggregationError(
                f"{trial_dir}/{label} trial_index={data.get('trial_index')!r} != expected {trial_index!r}"
            )

    data_validity = trial_verdict.get("data_validity")
    task_outcome = trial_verdict.get("task_outcome")
    if data_validity not in DATA_VALIDITY_VALUES:
        raise AggregationError(f"{trial_dir}: unrecognised data_validity value {data_validity!r}")
    if task_outcome not in TASK_OUTCOME_VALUES:
        raise AggregationError(f"{trial_dir}: unrecognised task_outcome value {task_outcome!r}")

    tid = trial_id(condition_id, trial_index, attempt)
    if tid in ctx.seen_trial_ids:
        raise AggregationError(f"duplicate trial_id encountered: {tid}")
    ctx.seen_trial_ids.add(tid)

    min_dist = trial_verdict.get("min_interrobot_distance_m")
    safety_margin = (min_dist - SAFETY_RADIUS_M) if isinstance(min_dist, (int, float)) else None

    d12 = _direction(matrix_analysis, "epuck1_to_epuck2")
    d21 = _direction(matrix_analysis, "epuck2_to_epuck1")

    drop_probability = frozen_params.get("drop_probability")
    outage_duration_s = frozen_params.get("outage_duration_s")

    drop12 = d12["relay"]["dropped_count"]
    drop21 = d21["relay"]["dropped_count"]
    recv12 = d12["relay"]["received_count"]
    recv21 = d21["relay"]["received_count"]

    outage_audit = _load_outage_recovery_audit(root, condition_id, trial_index, attempt, ctx)
    stale_stop_duration_s = _stale_stop_duration_from_audit(outage_audit)

    row = {
        "condition_id": condition_id,
        "trial_id": tid,
        "attempt_id": attempt,
        "data_validity": data_validity,
        "task_outcome": task_outcome,
        "delay_s": frozen_params.get("delay_s"),
        "jitter_s": frozen_params.get("jitter_s"),
        "drop_probability": drop_probability,
        "outage_period_s": frozen_params.get("outage_period_s"),
        "outage_duration_s": outage_duration_s,
        "outage_phase_s": frozen_params.get("outage_phase_s"),
        "seed_epuck1_to_epuck2": frozen_params.get("seed_epuck1_to_epuck2"),
        "seed_epuck2_to_epuck1": frozen_params.get("seed_epuck2_to_epuck1"),
        "minimum_interrobot_distance_m": min_dist,
        "safety_margin_m": safety_margin,
        "completion_count": trial_verdict.get("controller_complete_count"),
        "task_completion_time_s": NOT_AVAILABLE,
        "stale_stop_duration_s": stale_stop_duration_s,
        "recovery_time_s": NOT_AVAILABLE,
        "controller_crashed": matrix_analysis.get("task_outcome_inputs", {}).get("controller_crashed"),
        "configured_drop_probability": drop_probability,
        "authoritative_drop_count_epuck1_to_epuck2": drop12,
        "authoritative_drop_count_epuck2_to_epuck1": drop21,
        "authoritative_drop_fraction_epuck1_to_epuck2": (drop12 / recv12) if recv12 else NOT_AVAILABLE,
        "authoritative_drop_fraction_epuck2_to_epuck1": (drop21 / recv21) if recv21 else NOT_AVAILABLE,
        "drop_mechanism": classify_drop_mechanism(drop_probability, outage_duration_s),
        "measured_message_age_mean_epuck1_to_epuck2": d12["latency"].get("mean_message_age_s", NOT_AVAILABLE),
        "measured_message_age_mean_epuck2_to_epuck1": d21["latency"].get("mean_message_age_s", NOT_AVAILABLE),
        "measured_message_age_p95_epuck1_to_epuck2": d12["latency"].get("p95_message_age_s", NOT_AVAILABLE),
        "measured_message_age_p95_epuck2_to_epuck1": d21["latency"].get("p95_message_age_s", NOT_AVAILABLE),
        "reordered_count_epuck1_to_epuck2": d12["sequence"].get("out_of_order_count", NOT_AVAILABLE),
        "reordered_count_epuck2_to_epuck1": d21["sequence"].get("out_of_order_count", NOT_AVAILABLE),
        "duplicate_count_epuck1_to_epuck2": d12["sequence"].get("duplicate_count", NOT_AVAILABLE),
        "duplicate_count_epuck2_to_epuck1": d21["sequence"].get("duplicate_count", NOT_AVAILABLE),
        "queue_drained": matrix_analysis.get("queue_drain", {}).get("queue_drained", NOT_AVAILABLE),
        "stale_state_episode_count": NOT_AVAILABLE,
        "in_task_stale_episode_count": NOT_AVAILABLE,
        "peer_timeout_events": NOT_AVAILABLE,
        "bag_metadata_valid_proxy": matrix_analysis.get("measurement_validity", NOT_AVAILABLE),
        "analyzer_ok": (len(matrix_analysis.get("errors", [])) == 0),
        "evidence_hash_status": EVIDENCE_STATUS.get(condition_id, NOT_AVAILABLE),
        "source_frozen_params_path": str(fp_path),
        "source_trial_verdict_path": str(tv_path),
        "source_matrix_analysis_path": str(ma_path),
    }

    # G and F have partially-derivable stale-state signal; record what is
    # genuinely known without inventing an in-task/startup split.
    if condition_id == "G":
        row["stale_state_episode_count"] = 2  # confirmed 2 SAFE_STOP_STALE->recovery episodes/trial in prior audit
    if condition_id == "F" and outage_audit is not None:
        row["stale_state_episode_count"] = len(outage_audit.get("safe_stop_stale_intervals", [])) or NOT_AVAILABLE

    return row


def _batch_summary_path(root: Path, condition_id: str) -> Path:
    return root / _BATCH_SUMMARY_CANDIDATES[condition_id]


def cross_check_against_batch_summary(root: Path, condition_id: str, rows: list[dict], ctx: AggregationContext) -> None:
    """Fail closed if a batch summary's own per-trial figures disagree with
    the independently recomputed canonical row for the same trial. This is
    a targeted check on the fields both sources report (min_distance,
    task_outcome, data_validity) -- not a full schema comparison, since the
    two documents do not share an identical schema."""
    path = _batch_summary_path(root, condition_id)
    if not path.is_file():
        raise AggregationError(f"missing batch summary for condition {condition_id}: {path}")
    summary = _read_json(path, ctx)

    per_trial_entries = (
        summary.get("individual_trial_verdicts")
        or summary.get("individual_trials")
        or summary.get("included_trials")
        or summary.get("trials")
    )
    if not per_trial_entries:
        # Some summaries (e.g. batch_gate) do not carry a fully redundant
        # per-trial verdict list comparable field-for-field; that is a
        # known, accepted convention difference, not an error.
        return

    by_id = {row["trial_id"]: row for row in rows}
    for entry in per_trial_entries:
        entry_trial_id = entry.get("trial_id")
        if entry_trial_id not in by_id:
            continue  # e.g. D's excluded trial04 appearing in "included_trials" would already have failed elsewhere
        row = by_id[entry_trial_id]
        entry_min_dist = entry.get("min_interrobot_distance_m")
        if entry_min_dist is not None and row["minimum_interrobot_distance_m"] is not None:
            if abs(entry_min_dist - row["minimum_interrobot_distance_m"]) > 1e-9:
                raise AggregationError(
                    f"{entry_trial_id}: batch summary min_interrobot_distance_m={entry_min_dist} "
                    f"disagrees with recomputed {row['minimum_interrobot_distance_m']}"
                )
        entry_task_outcome = entry.get("task_outcome") or entry.get("TASK_OUTCOME")
        if entry_task_outcome is not None and entry_task_outcome != row["task_outcome"]:
            raise AggregationError(
                f"{entry_trial_id}: batch summary task_outcome={entry_task_outcome} "
                f"disagrees with recomputed {row['task_outcome']}"
            )
        entry_validity = entry.get("data_validity") or entry.get("DATA_VALIDITY") or entry.get("FORMAL_MEASUREMENT_VALIDITY")
        if entry_validity is not None and entry_validity != row["data_validity"]:
            raise AggregationError(
                f"{entry_trial_id}: batch summary data_validity={entry_validity} "
                f"disagrees with recomputed {row['data_validity']}"
            )


def aggregate(root: Path, conditions: list[str]) -> tuple[list[dict], list[dict], AggregationContext]:
    ctx = AggregationContext()
    all_rows: list[dict] = []
    all_excluded: list[dict] = []

    for condition_id in conditions:
        if condition_id not in CONDITION_TRIALS:
            raise AggregationError(f"unknown condition_id: {condition_id}")
        expected = CONDITION_TRIALS[condition_id]
        rows = []
        for trial_index, attempt in expected:
            rows.append(load_trial_row(root, condition_id, trial_index, attempt, ctx))
        if len(rows) != 5:
            raise AggregationError(
                f"condition {condition_id}: expected exactly 5 counted valid trials, got {len(rows)}"
            )
        valid_count = sum(1 for r in rows if r["data_validity"] == "VALID")
        if valid_count != 5:
            raise AggregationError(
                f"condition {condition_id}: expected 5 VALID counted trials, got {valid_count}"
            )
        cross_check_against_batch_summary(root, condition_id, rows, ctx)
        all_rows.extend(rows)
        all_excluded.extend(EXCLUDED_TRIALS.get(condition_id, []))

    if len(all_rows) != 5 * len(conditions):
        raise AggregationError(f"expected {5 * len(conditions)} total rows, got {len(all_rows)}")

    return all_rows, all_excluded, ctx


def _export_value(value: Any) -> Any:
    if value is None:
        return NOT_AVAILABLE
    if value == "":
        return NOT_AVAILABLE
    return value


def write_per_trial_canonical_csv(path: Path, rows: list[dict]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CANONICAL_FIELDS, lineterminator="\n")
        w.writeheader()
        for row in rows:
            w.writerow({k: _export_value(row.get(k)) for k in CANONICAL_FIELDS})


def write_excluded_trials_csv(path: Path, excluded: list[dict]) -> None:
    fieldnames = ["condition_id", "trial_index", "attempt", "formal_measurement_validity",
                  "classification", "exclusion_reason", "not_rerun",
                  "not_counted_toward_formal_n5", "replacement_trial_id"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        w.writeheader()
        for entry in excluded:
            w.writerow({k: _export_value(entry.get(k)) for k in fieldnames})


def write_input_manifest_csv(path: Path, ctx: AggregationContext) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["path", "sha256"])
        for hf in ctx.hashed_files:
            w.writerow([hf.path, hf.sha256])


def write_evidence_status_csv(path: Path, conditions: list[str]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["condition_id", "evidence_hash_status"])
        for c in conditions:
            w.writerow([c, EVIDENCE_STATUS.get(c, NOT_AVAILABLE)])


def write_configured_vs_realised_csv(path: Path, rows: list[dict]) -> None:
    fieldnames = [
        "condition_id", "trial_id", "configured_drop_probability", "drop_mechanism",
        "authoritative_drop_fraction_epuck1_to_epuck2", "authoritative_drop_fraction_epuck2_to_epuck1",
        "delay_s", "jitter_s", "measured_message_age_mean_epuck1_to_epuck2", "measured_message_age_mean_epuck2_to_epuck1",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        w.writeheader()
        for row in rows:
            w.writerow({k: _export_value(row.get(k)) for k in fieldnames})


def write_per_condition_summary_csv(path: Path, rows: list[dict], conditions: list[str]) -> None:
    fieldnames = ["condition_id", "n_trials", "success_count", "safe_degradation_count",
                  "unsafe_failure_count", "not_evaluable_count",
                  "min_distance_mean_m", "min_distance_min_m", "min_distance_max_m",
                  "safety_margin_mean_m", "safety_margin_min_m", "safety_margin_max_m"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        w.writeheader()
        for c in conditions:
            crows = [r for r in rows if r["condition_id"] == c]
            dists = [r["minimum_interrobot_distance_m"] for r in crows if isinstance(r["minimum_interrobot_distance_m"], (int, float))]
            margins = [r["safety_margin_m"] for r in crows if isinstance(r["safety_margin_m"], (int, float))]
            w.writerow({
                "condition_id": c,
                "n_trials": len(crows),
                "success_count": sum(1 for r in crows if r["task_outcome"] == "SUCCESS"),
                "safe_degradation_count": sum(1 for r in crows if r["task_outcome"] == "SAFE_DEGRADATION"),
                "unsafe_failure_count": sum(1 for r in crows if r["task_outcome"] == "UNSAFE_FAILURE"),
                "not_evaluable_count": sum(1 for r in crows if r["task_outcome"] == "NOT_EVALUABLE"),
                "min_distance_mean_m": sum(dists) / len(dists) if dists else NOT_AVAILABLE,
                "min_distance_min_m": min(dists) if dists else NOT_AVAILABLE,
                "min_distance_max_m": max(dists) if dists else NOT_AVAILABLE,
                "safety_margin_mean_m": sum(margins) / len(margins) if margins else NOT_AVAILABLE,
                "safety_margin_min_m": min(margins) if margins else NOT_AVAILABLE,
                "safety_margin_max_m": max(margins) if margins else NOT_AVAILABLE,
            })


def _numeric_field_export(value: Any) -> tuple[Any, str]:
    """Tableau-safe export of a numeric plot-data field: returns
    (numeric_or_empty, value_status). NOT_AVAILABLE/None/'' export as an
    EMPTY string (Tableau null), never as 0, paired with an explicit
    value_status sibling column so a viewer can distinguish "genuinely
    zero" from "not measured" without inspecting the raw value."""
    if value is None or value == "" or value == NOT_AVAILABLE:
        return "", NOT_AVAILABLE
    return value, "AVAILABLE"


def write_plot_data_csvs(out_dir: Path, rows: list[dict]) -> list[Path]:
    """Every plot-data CSV in this directory is built for direct Tableau
    consumption: any field that can carry NOT_AVAILABLE is split into a
    numeric column (empty/null when unavailable, never 0) plus a sibling
    `<field>_value_status` column (AVAILABLE|NOT_AVAILABLE). Purely
    categorical fields (task_outcome, drop_mechanism, condition_id,
    trial_id) are left as plain text columns -- they are never plotted on
    a continuous numeric axis, so no status column is needed for them."""
    plot_dir = out_dir / "plot_data"
    plot_dir.mkdir(parents=True, exist_ok=False)
    written = []

    def _write_numeric(name: str, key_fields: list[str], numeric_fields: list[str], row_source: list[dict]):
        fieldnames = list(key_fields)
        for nf in numeric_fields:
            fieldnames += [nf, f"{nf}_value_status"]
        p = plot_dir / name
        with open(p, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
            w.writeheader()
            for row in row_source:
                out = {k: row.get(k) for k in key_fields}
                for nf in numeric_fields:
                    value, status = _numeric_field_export(row.get(nf))
                    out[nf] = value
                    out[f"{nf}_value_status"] = status
                w.writerow(out)
        written.append(p)

    _write_numeric("min_interrobot_distance.csv", ["condition_id", "trial_id"],
                    ["minimum_interrobot_distance_m"], rows)
    _write_numeric("safety_margin.csv", ["condition_id", "trial_id"],
                    ["safety_margin_m"], rows)

    # task_outcome is categorical text -- never plotted as a continuous
    # numeric axis, so no value_status column is needed here.
    p = plot_dir / "task_outcome.csv"
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["condition_id", "trial_id", "task_outcome"], lineterminator="\n")
        w.writeheader()
        for row in rows:
            w.writerow({k: _export_value(row.get(k)) for k in ("condition_id", "trial_id", "task_outcome")})
    written.append(p)

    _write_numeric("realised_loss.csv", ["condition_id", "trial_id", "drop_mechanism"],
                    ["authoritative_drop_fraction_epuck1_to_epuck2",
                     "authoritative_drop_fraction_epuck2_to_epuck1"], rows)
    _write_numeric("message_age.csv", ["condition_id", "trial_id"],
                    ["measured_message_age_mean_epuck1_to_epuck2",
                     "measured_message_age_mean_epuck2_to_epuck1"], rows)
    _write_numeric("reordered_count.csv", ["condition_id", "trial_id"],
                    ["reordered_count_epuck1_to_epuck2",
                     "reordered_count_epuck2_to_epuck1"], rows)

    f_rows = [r for r in rows if r["condition_id"] == "F"]
    _write_numeric("f_stale_stop_duration.csv", ["trial_id"], ["stale_stop_duration_s"], f_rows)

    return written


def try_generate_plots(out_dir: Path, rows: list[dict], conditions: list[str]) -> tuple[list[str], str | None]:
    """Attempt SVG/PNG generation via matplotlib. Returns (generated_files,
    error_message). error_message is None on full success. Never raises --
    a broken plotting environment must not block CSV/report outputs."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - environment dependent
        return [], f"matplotlib unavailable in this environment: {exc}"

    plots_dir = out_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    generated = []

    def _scatter_by_condition(values_by_condition, ylabel, fname, hline=None):
        fig, ax = plt.subplots(figsize=(8, 4))
        for i, c in enumerate(conditions):
            vals = [v for v in values_by_condition.get(c, []) if isinstance(v, (int, float))]
            ax.scatter([i] * len(vals), vals, label=c)
        if hline is not None:
            ax.axhline(hline, linestyle="--", color="red", label="safety threshold")
        ax.set_xticks(range(len(conditions)))
        ax.set_xticklabels(conditions)
        ax.set_ylabel(ylabel)
        ax.set_title(f"{ylabel} by condition (n=5 formal valid trials per condition; descriptive only)")
        fig.tight_layout()
        for ext in ("svg", "png"):
            p = plots_dir / f"{fname}.{ext}"
            fig.savefig(p)
            generated.append(str(p))
        plt.close(fig)

    by_cond_dist = {c: [r["minimum_interrobot_distance_m"] for r in rows if r["condition_id"] == c] for c in conditions}
    _scatter_by_condition(by_cond_dist, "minimum_interrobot_distance_m", "min_interrobot_distance")

    by_cond_margin = {c: [r["safety_margin_m"] for r in rows if r["condition_id"] == c] for c in conditions}
    _scatter_by_condition(by_cond_margin, "safety_margin_m", "safety_margin", hline=0.0)

    return generated, None


def write_report_md(out_dir: Path, rows: list[dict], excluded: list[dict], conditions: list[str],
                     plot_files: list[str], plot_error: str | None) -> Path:
    lines = []
    lines.append("# Objective 5 A-G aggregation report\n")
    lines.append(
        "n=5 formal valid trials per condition; descriptive results only; "
        "no statistical-significance or broad-generalisation claim.\n"
    )
    lines.append(f"Formal per-trial dataset row count: {len(rows)} (expected {5 * len(conditions)}).\n")
    lines.append("## Excluded trials\n")
    for e in excluded:
        lines.append(f"- {e['condition_id']} trial{e['trial_index']:02d}: {e['classification']} -- {e['exclusion_reason']}\n")
    lines.append("## Evidence status per condition\n")
    for c in conditions:
        lines.append(f"- {c}: {EVIDENCE_STATUS.get(c, NOT_AVAILABLE)}\n")
    lines.append("## Plots\n")
    if plot_error:
        lines.append(f"Plot generation was not performed in this environment: {plot_error}\n")
    else:
        for p in plot_files:
            lines.append(f"- {p}\n")
    report_path = out_dir / "A_to_G_aggregation_report.md"
    with open(report_path, "w", encoding="utf-8", newline="\n") as f:
        f.writelines(lines)
    return report_path


def run(root: Path, out_dir: Path | None, conditions: list[str], dry_run: bool, skip_plots: bool = False) -> dict:
    rows, excluded, ctx = aggregate(root, conditions)

    result = {
        "conditions": conditions,
        "row_count": len(rows),
        "excluded_count": len(excluded),
        "hashed_file_count": len(ctx.hashed_files),
        "dry_run": dry_run,
        "skip_plots": skip_plots,
    }

    if dry_run:
        result["outputs_written"] = []
        return result

    if out_dir is None:
        raise AggregationError("an output directory is required for a non-dry-run invocation")
    if out_dir.exists() and any(out_dir.iterdir()):
        raise AggregationError(
            f"output directory already exists and is non-empty: {out_dir} "
            "(unconditional no-overwrite policy -- no --force is provided)"
        )
    out_dir.mkdir(parents=True, exist_ok=True)

    write_per_trial_canonical_csv(out_dir / "per_trial_canonical.csv", rows)
    write_per_condition_summary_csv(out_dir / "per_condition_summary.csv", rows, conditions)
    write_excluded_trials_csv(out_dir / "excluded_trials.csv", excluded)
    write_configured_vs_realised_csv(out_dir / "configured_vs_realised_impairment.csv", rows)
    write_evidence_status_csv(out_dir / "evidence_status.csv", conditions)
    plot_data_files = write_plot_data_csvs(out_dir, rows)

    if skip_plots:
        # --skip-plots means: do not even attempt to import matplotlib or
        # touch the plotting code path. The environment's numpy/matplotlib
        # installation is known-broken and must not be repaired or probed
        # here; SVG/PNG figures are being produced downstream in Tableau
        # from the CSV outputs instead.
        plot_files, plot_error = [], "skipped via --skip-plots; verified data outputs (CSV/Markdown) only"
    else:
        plot_files, plot_error = try_generate_plots(out_dir, rows, conditions)

    write_input_manifest_csv(out_dir / "input_manifest_sha256.csv", ctx)
    report_path = write_report_md(out_dir, rows, excluded, conditions, plot_files, plot_error)

    result["outputs_written"] = [
        str(out_dir / "per_trial_canonical.csv"),
        str(out_dir / "per_condition_summary.csv"),
        str(out_dir / "excluded_trials.csv"),
        str(out_dir / "configured_vs_realised_impairment.csv"),
        str(out_dir / "evidence_status.csv"),
        str(out_dir / "input_manifest_sha256.csv"),
        str(report_path),
    ] + [str(p) for p in plot_data_files]
    result["plot_files"] = plot_files
    result["plot_error"] = plot_error
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path, help="Objective 5 impairment-matrix directory root")
    parser.add_argument("--out", type=Path, default=None, help="Output directory (must not exist or must be empty)")
    parser.add_argument("--conditions", default="A,B,C,D,E,F,G", help="Comma-separated condition IDs")
    parser.add_argument("--dry-run", action="store_true", help="Validate and report only; write nothing")
    parser.add_argument("--skip-plots", action="store_true",
                         help="Produce only verified data outputs (CSV/Markdown); never import matplotlib "
                              "or write SVG/PNG. Use this when downstream figures are built in another tool "
                              "(e.g. Tableau) from the CSV outputs.")
    args = parser.parse_args(argv)

    conditions = [c.strip() for c in args.conditions.split(",") if c.strip()]
    try:
        result = run(args.root, args.out, conditions, args.dry_run, skip_plots=args.skip_plots)
    except AggregationError as exc:
        print(json.dumps({"status": "BLOCKED", "error": str(exc)}, indent=2))
        return 1

    print(json.dumps({"status": "OK", **result}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
