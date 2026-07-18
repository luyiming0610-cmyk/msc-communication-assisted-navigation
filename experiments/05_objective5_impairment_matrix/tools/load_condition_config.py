#!/usr/bin/env python3
"""Reads one condition's frozen parameters from
objective5_impairment_matrix_conditions.csv and resolves them into the
concrete relay launch arguments for one trial -- so the orchestrator
NEVER accepts a hand-typed delay/jitter/drop/seed override on the
command line (design doc's "禁止手工命令行临时改值" requirement). The
only inputs the orchestrator takes are: which frozen CSV, which
condition_id, and which trial index (1-5) within that condition.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path


class UnknownConditionError(ValueError):
    pass


class ConditionNotExecutableError(ValueError):
    """Raised for a condition whose parameters are not real floats yet
    (Condition F before its relay extension is frozen with real values --
    the CSV currently records NOT_IMPLEMENTED literals there on purpose,
    see design doc section 3, and must never be silently parsed as 0.0)."""


class TrialIndexError(ValueError):
    pass


_NOT_EXECUTABLE_MARKERS = {"NOT_IMPLEMENTED", "N/A (not implemented)"}


@dataclass(frozen=True)
class RelayLaunchParams:
    condition_id: str
    trial_index: int
    delay_s: float
    jitter_s: float
    drop_probability: float
    seed_epuck1: int
    seed_epuck2: int
    outage_period_s: float
    outage_duration_s: float
    outage_phase_s: float


def _parse_seed_list(field: str) -> list:
    if field.strip().lower() == "none":
        return []
    return [int(s.strip()) for s in field.split(",") if s.strip()]


def load_conditions(csv_path: Path) -> dict:
    """Returns {condition_id: row_dict} for every row in the frozen CSV."""
    with open(csv_path, encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        return {row["condition_id"]: row for row in reader}


def resolve_trial_params(csv_path: Path, condition_id: str, trial_index: int) -> RelayLaunchParams:
    """trial_index is 1-based (1..n_trials for that condition)."""
    conditions = load_conditions(csv_path)
    if condition_id not in conditions:
        raise UnknownConditionError(
            f"condition_id={condition_id!r} not found in {csv_path}; known conditions: {sorted(conditions)}"
        )
    row = conditions[condition_id]

    for field in ("delay_s", "jitter_s", "drop_probability"):
        if row[field].strip() in _NOT_EXECUTABLE_MARKERS:
            raise ConditionNotExecutableError(
                f"condition {condition_id!r} field {field!r} is {row[field]!r} -- "
                "this condition is not yet executable (see design doc section 3)"
            )

    delay_s = float(row["delay_s"])
    jitter_s = float(row["jitter_s"])
    drop_probability = float(row["drop_probability"])

    seeds_1 = _parse_seed_list(row["seed_epuck1_to_epuck2"])
    seeds_2 = _parse_seed_list(row["seed_epuck2_to_epuck1"])
    deterministic = not seeds_1 and not seeds_2
    if deterministic:
        seed_1 = seed_2 = 0
    else:
        if trial_index < 1 or trial_index > len(seeds_1) or trial_index > len(seeds_2):
            raise TrialIndexError(
                f"trial_index={trial_index} out of range for condition {condition_id!r} "
                f"(has {len(seeds_1)} epuck1 seeds, {len(seeds_2)} epuck2 seeds)"
            )
        seed_1 = seeds_1[trial_index - 1]
        seed_2 = seeds_2[trial_index - 1]

    # outage_* fields are present as literal 0.0 for every condition
    # except F (where they're NOT_IMPLEMENTED and already rejected above);
    # absent/blank is treated as 0.0 (disabled), never as an error, since
    # every condition except F genuinely has no outage component.
    def _outage_field(name):
        raw = row.get(name, "").strip()
        if raw in ("", *_NOT_EXECUTABLE_MARKERS):
            return 0.0
        return float(raw)

    return RelayLaunchParams(
        condition_id=condition_id,
        trial_index=trial_index,
        delay_s=delay_s,
        jitter_s=jitter_s,
        drop_probability=drop_probability,
        seed_epuck1=seed_1,
        seed_epuck2=seed_2,
        outage_period_s=_outage_field("outage_period_s"),
        outage_duration_s=_outage_field("outage_duration_s"),
        outage_phase_s=_outage_field("outage_phase_s"),
    )


def main():
    """CLI: prints `KEY=value` lines (one per RelayLaunchParams field) so
    the bash orchestrator can `source` this command's output directly --
    the only sanctioned way parameters reach the orchestrator (no
    hand-typed CLI overrides, per the design doc's explicit requirement)."""
    import argparse
    import sys

    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--condition-id", required=True)
    parser.add_argument("--trial-index", required=True, type=int)
    args = parser.parse_args()

    try:
        params = resolve_trial_params(args.csv, args.condition_id, args.trial_index)
    except (UnknownConditionError, ConditionNotExecutableError, TrialIndexError) as exc:
        print(f"CONDITION_CONFIG_ERROR={exc}", file=sys.stderr)
        sys.exit(1)

    for field_name in params.__dataclass_fields__:
        print(f"{field_name.upper()}={getattr(params, field_name)}")


if __name__ == "__main__":
    main()
