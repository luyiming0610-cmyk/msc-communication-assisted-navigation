#!/usr/bin/env python3
"""The two-dimensional per-trial verdict, as real code -- not only
design-doc prose. DATA_VALIDITY and TASK_OUTCOME are computed by two
independent pure functions so each can be unit-tested without a real
bag/controller-log/relay.

DATA_VALIDITY = VALID | INVALID -- infrastructure/measurement question.
TASK_OUTCOME = SUCCESS | SAFE_DEGRADATION | UNSAFE_FAILURE | NOT_EVALUABLE
-- scientific-result question, computed ONLY when DATA_VALIDITY=VALID
(otherwise NOT_EVALUABLE, since a task outcome cannot be trusted from
data that already failed its own validity checks).

A COLLISION, TASK_TIMEOUT, or safety stop that a frozen, correctly
applied impairment condition genuinely produced is SAFE_DEGRADATION (if
no collision) or UNSAFE_FAILURE (if a collision/crash did occur) --
either is a valid experimental result with DATA_VALIDITY=VALID, never a
reason to exclude the trial or stop the batch. Only DATA_VALIDITY=INVALID
does that (see classify_data_validity's callers in the orchestrator).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DataValidityCheck:
    seeds_match_frozen_config: bool
    realtime_factor_ok: bool
    bag_metadata_ok: bool
    analyzer_ok: bool
    queue_drained: bool
    bag_log_clean: bool = True


def classify_data_validity(check: DataValidityCheck) -> tuple:
    """Returns (verdict, reasons). verdict is "VALID" only if every
    individual check passed; otherwise "INVALID" with every failing
    check named (not just the first one found) so a single diagnosis
    pass can see everything that went wrong at once.

    Each field must be sourced from its OWN dedicated signal by the
    caller, never from a shared/overloaded "something failed somewhere"
    flag -- doing so was a real bug found during Pilot 2 (Condition F):
    the orchestrator fed one shared bash accumulator into queue_drained,
    so a bag-recorder-log warning got misreported as "queue not
    drained" even though the queue genuinely was drained. Each reason
    string here is a specific, falsifiable claim; it must only fire for
    the check it names."""
    reasons = []
    if not check.seeds_match_frozen_config:
        reasons.append("deployed relay seed(s) do not match the frozen conditions CSV")
    if not check.realtime_factor_ok:
        reasons.append("realtime_factor outside the accepted tolerance band")
    if not check.bag_metadata_ok:
        reasons.append("rosbag metadata.yaml missing, empty, or truncated")
    if not check.analyzer_ok:
        reasons.append("analyzer failed or did not run to completion")
    if not check.queue_drained:
        reasons.append("relay pending_queue_depth was not confirmed 0 before shutdown (queue-drain rule)")
    if not check.bag_log_clean:
        reasons.append("bag_record.log contained drop/warn/error line(s)")
    verdict = "INVALID" if reasons else "VALID"
    return verdict, reasons


@dataclass(frozen=True)
class TaskOutcomeSignals:
    complete_count: int
    expected_complete_count: int
    min_interrobot_distance_m: float | None
    safety_radius_m: float
    controller_crashed: bool


def classify_task_outcome(data_validity: str, signals: TaskOutcomeSignals) -> tuple:
    """Returns (outcome, reason). Collision heuristic (documented, not
    hidden): min_interrobot_distance_m falling below safety_radius_m at
    ANY point in the trial is treated as UNSAFE_FAILURE. This is
    deliberately conservative -- it does not attempt to distinguish a
    genuine collision from a very close clean pass via closing-speed
    sign (unlike collision_math.collision_risk's own gating), because a
    trial-level pass/fail classification should err toward flagging a
    close call for human review rather than silently accepting it as
    safe. If this proves too conservative in practice (i.e. clean passes
    routinely get flagged), that is itself a reportable finding, not a
    reason to quietly loosen the threshold after seeing results."""
    if data_validity != "VALID":
        return "NOT_EVALUABLE", f"data_validity={data_validity}, task outcome cannot be trusted"
    if signals.controller_crashed:
        return "UNSAFE_FAILURE", "controller process crashed or exited abnormally"
    if signals.min_interrobot_distance_m is not None and signals.min_interrobot_distance_m < signals.safety_radius_m:
        return "UNSAFE_FAILURE", (
            f"min_interrobot_distance_m={signals.min_interrobot_distance_m:.4f} "
            f"< safety_radius_m={signals.safety_radius_m:.4f}"
        )
    if signals.complete_count >= signals.expected_complete_count:
        return "SUCCESS", f"complete_count={signals.complete_count} >= expected {signals.expected_complete_count}"
    return "SAFE_DEGRADATION", (
        f"complete_count={signals.complete_count} < expected {signals.expected_complete_count}, "
        "but no collision/crash detected (e.g. max_runtime timeout or a safe stop that never recovered)"
    )


def main():
    """CLI for the orchestrator: reads the already-computed signals as
    JSON on stdin (the orchestrator gathers them from frozen_params.json,
    preliminary_runtime_manifest.json, the controller log, and
    min_interrobot_distance.py's own output -- this script does no I/O
    of its own, only classification), writes {data_validity,
    data_validity_reasons, task_outcome, task_outcome_reason} as JSON to
    stdout."""
    import json
    import sys

    payload = json.load(sys.stdin)
    data_validity, data_validity_reasons = classify_data_validity(DataValidityCheck(
        seeds_match_frozen_config=payload["seeds_match_frozen_config"],
        realtime_factor_ok=payload["realtime_factor_ok"],
        bag_metadata_ok=payload["bag_metadata_ok"],
        analyzer_ok=payload["analyzer_ok"],
        queue_drained=payload["queue_drained"],
        bag_log_clean=payload.get("bag_log_clean", True),
    ))
    task_outcome, task_outcome_reason = classify_task_outcome(data_validity, TaskOutcomeSignals(
        complete_count=payload["complete_count"],
        expected_complete_count=payload["expected_complete_count"],
        min_interrobot_distance_m=payload["min_interrobot_distance_m"],
        safety_radius_m=payload["safety_radius_m"],
        controller_crashed=payload["controller_crashed"],
    ))
    print(json.dumps({
        "data_validity": data_validity,
        "data_validity_reasons": data_validity_reasons,
        "task_outcome": task_outcome,
        "task_outcome_reason": task_outcome_reason,
    }))


if __name__ == "__main__":
    main()
