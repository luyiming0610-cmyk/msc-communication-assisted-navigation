#!/usr/bin/env python3
"""Repeatability-specific pre-pulse pose-evidence gate, added
2026-07-27 for SINGLE_ROBOT_GROUND_REPEATABILITY_BASELINE.

Purely ADDITIVE: does not weaken, replace, or reinterpret
hil_ground_diagnostic_phases.evaluate_wsl_live_state()/
evaluate_combined_gate() -- those remain the binding live-zero-state
acceptance rules for every ground-diagnostic run, unchanged. This gate
is a SEPARATE, stricter check required only for the repeatability
baseline (per its specification), run in addition to -- after --
`GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_PASS` and before ground
placement/pulse. A historical or future non-repeatability ground
diagnostic never calls this module at all, so its CSV (which may lack
the pose columns) is entirely unaffected.

Read-only in the strongest sense available: unlike the ROS-topic-based
live-zero-state checks, this gate never touches ROS at all -- it only
reads hil_command_evidence_recorder.py's own CSV file (already being
written by the running recorder) from the filesystem. It never
publishes, never subscribes, never arms anything, and never contacts
the guard/controller/bridge directly.

Two layers, matching this project's established pure-logic +
thin-wrapper split:
  - evaluate_repeatability_pose_readiness(): pure, takes only plain
    already-computed facts, returns PASS/BLOCKED + reasons.
  - compute_pose_readiness_facts(): pure, derives those facts from
    already-loaded CSV rows (reuses
    hil_motion_repeatability_metrics.extract_valid_state_samples).
  - csv_header_has_pose_columns() / is_csv_growing(): thin file-I/O
    helpers (the only two facts that cannot be derived from a single
    already-loaded row list -- header presence needs the raw header
    line, and "growing" is inherently a two-time-point observation).
"""
from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass, field
from typing import Optional

from analyze_ground_diagnostic import load_wsl_csv_rows
from hil_motion_repeatability_metrics import extract_valid_state_samples

DEFAULT_MAX_POSE_SAMPLE_STALENESS_S = 1.0
DEFAULT_RECENT_LOOKBACK_ROWS = 5
DEFAULT_REQUIRED_VALIDITY_FLAGS = 7
DEFAULT_GROWTH_WAIT_S = 1.5
REQUIRED_POSE_COLUMNS = ("state_x_m", "state_y_m", "state_yaw_rad")

# Frozen production relationship between the recorder's CSV flush
# interval and this gate's own max_pose_sample_staleness_s, added
# 2026-07-27 after SRGRB_20260727_02 Trial 1 Attempt 1 was classified
# INVALID (POSE_READINESS_FLUSH_FRESHNESS_INCOMPATIBLE, observed age
# 1.078s against a 1.0s flush interval and a 1.0s staleness
# threshold). hil_command_evidence_recorder.py's CommandEvidenceCsvWriter
# flushes only when a WRITE arrives at/after flush_interval_s has
# elapsed since the last flush (not on a background timer) -- so
# between two consecutive flushes, the freshest sample actually
# readable on disk stays fixed at that last flush's own row, and its
# age (as measured by a check happening near the end of that window)
# approaches flush_interval_s itself even under perfectly healthy
# conditions. A flush_interval_s equal to (or close to)
# max_pose_sample_staleness_s therefore leaves ZERO margin: on top of
# that intrinsic buffering lag, the ordinary, unavoidable overhead of
# actually running the check (this module's own CSV parse time, which
# grows with file size; Python interpreter/import startup; ordinary
# clock/scheduling jitter -- none of them a sensor, bridge, or
# connectivity fault) is enough to push the observed age over the
# threshold, exactly as seen live.
#
# state_publisher's frozen publish rate for every ground-diagnostic and
# repeatability run is 10.0 Hz (period 0.1 s -- GROUND_DIAGNOSTIC_RUNBOOK.md
# step 6, `-p mode:=periodic`'s own default) -- recorded here for
# traceability, even though the buffering-lag bound below depends only
# on flush_interval_s, not on the state topic's own rate (any rate at
# least as fast as one write per flush_interval_s reproduces the same
# bound). Repeatability trials must start the recorder with a flush
# interval comfortably below max_pose_sample_staleness_s --
# RECOMMENDED_REPEATABILITY_FLUSH_INTERVAL_S below leaves roughly 70%
# margin even after adding a realistic check-execution overhead
# (worst_case_on_disk_pose_sample_age_s(0.2) == 0.2s against a 1.0s
# threshold, plus overhead still well under 1.0s), verified
# live-conditions-faithfully in test_hil_repeatability_pose_readiness.py's
# FlushFreshnessIncompatibilityTest. This is a PARAMETER CHOICE for how
# the recorder is invoked for repeatability trials (--flush-interval-s),
# not a code change to state_publisher, the protocol, controller,
# bridge, or guard, and does not alter this gate's own
# max_pose_sample_staleness_s safety rule.
STATE_TOPIC_PUBLISH_PERIOD_S = 0.1
RECOMMENDED_REPEATABILITY_FLUSH_INTERVAL_S = 0.2


def worst_case_on_disk_pose_sample_age_s(flush_interval_s: float) -> float:
    """Worst-case age (seconds) of the freshest pose sample actually
    readable on disk, purely from the recorder's own flush buffering --
    not a sensor/bridge/connectivity measure, and not including any
    check-execution overhead on top (see the module docstring above and
    FlushFreshnessIncompatibilityTest for why real overhead matters
    too). The recorder's CommandEvidenceCsvWriter flushes only when a
    write arrives at/after flush_interval_s has elapsed since the last
    flush (not on a background timer); between two consecutive
    flushes, the on-disk freshest sample's age approaches
    flush_interval_s itself as the check moment approaches the next
    flush boundary. Pure arithmetic, no I/O."""
    return flush_interval_s


@dataclass(frozen=True)
class RepeatabilityPoseReadinessResult:
    ok: bool
    reasons: tuple = field(default_factory=tuple)


def evaluate_repeatability_pose_readiness(
    *,
    csv_exists: bool,
    csv_growing: bool,
    header_has_pose_columns: bool,
    valid_recent_pose_sample_count: int,
    timestamps_ordered: bool,
    latest_valid_pose_sample_age_s: Optional[float],
    validity_flags: Optional[int],
    guarded_cmd_nonzero: bool,
    upstream_cmd_nonzero: bool,
    max_pose_sample_staleness_s: float = DEFAULT_MAX_POSE_SAMPLE_STALENESS_S,
    required_validity_flags: int = DEFAULT_REQUIRED_VALIDITY_FLAGS,
    min_recent_valid_pose_samples: int = 2,
) -> RepeatabilityPoseReadinessResult:
    """Pure boolean/count facts in, PASS/BLOCKED + reasons out. Never
    reads a file, never touches ROS, never raises."""
    reasons = []
    if not csv_exists:
        reasons.append("CSV_NOT_FOUND")
        return RepeatabilityPoseReadinessResult(ok=False, reasons=tuple(reasons))

    if not csv_growing:
        reasons.append("CSV_NOT_GROWING")
    if not header_has_pose_columns:
        reasons.append("POSE_COLUMNS_MISSING_FROM_HEADER")
    if valid_recent_pose_sample_count < min_recent_valid_pose_samples:
        reasons.append(
            f"INSUFFICIENT_VALID_POSE_SAMPLES(got={valid_recent_pose_sample_count},"
            f"required={min_recent_valid_pose_samples})"
        )
    if not timestamps_ordered:
        reasons.append("POSE_SAMPLE_TIMESTAMPS_NOT_ORDERED")
    if latest_valid_pose_sample_age_s is None:
        reasons.append("NO_VALID_POSE_SAMPLE_TO_CHECK_FRESHNESS")
    elif latest_valid_pose_sample_age_s > max_pose_sample_staleness_s:
        reasons.append(
            f"LATEST_POSE_SAMPLE_STALE(age_s={latest_valid_pose_sample_age_s:.3f},"
            f"max_s={max_pose_sample_staleness_s})"
        )
    if validity_flags != required_validity_flags:
        reasons.append(f"VALIDITY_FLAGS_NOT_{required_validity_flags}(got={validity_flags})")
    if guarded_cmd_nonzero:
        reasons.append("GUARDED_CMD_NONZERO")
    if upstream_cmd_nonzero:
        reasons.append("UPSTREAM_CMD_NONZERO")

    return RepeatabilityPoseReadinessResult(ok=not reasons, reasons=tuple(reasons))


def compute_pose_readiness_facts(
    wsl_rows: list,
    state_topic: str,
    guarded_topic: str,
    upstream_topic: str,
    now_ns: int,
    recent_lookback_rows: int = DEFAULT_RECENT_LOOKBACK_ROWS,
) -> dict:
    """Pure -- derives every fact computable from an already-loaded row
    list plus a caller-supplied `now_ns` (never reads the system clock
    itself, so it stays fully deterministic and testable)."""
    all_state_rows = [r for r in wsl_rows if r.get("topic") == state_topic]
    recent_rows = all_state_rows[-recent_lookback_rows:] if all_state_rows else []

    valid_samples = extract_valid_state_samples(wsl_rows, state_topic)
    recent_valid_samples = extract_valid_state_samples(recent_rows, state_topic)

    times = [s.local_time_ns for s in valid_samples]
    timestamps_ordered = all(times[i] <= times[i + 1] for i in range(len(times) - 1))

    latest_valid_pose_sample_age_s = None
    if valid_samples:
        latest_valid_pose_sample_age_s = (now_ns - valid_samples[-1].local_time_ns) / 1e9

    latest_state_row = all_state_rows[-1] if all_state_rows else None
    validity_flags = latest_state_row.get("validity_flags") if latest_state_row else None

    def _topic_last_nonzero(topic: str) -> bool:
        rows = [r for r in wsl_rows if r.get("topic") == topic]
        if not rows:
            return False
        last = rows[-1]
        return (isinstance(last.get("linear_x"), float) and last["linear_x"] != 0.0) or (
            isinstance(last.get("angular_z"), float) and last["angular_z"] != 0.0
        )

    return {
        "valid_recent_pose_sample_count": len(recent_valid_samples),
        "timestamps_ordered": timestamps_ordered,
        "latest_valid_pose_sample_age_s": latest_valid_pose_sample_age_s,
        "validity_flags": validity_flags,
        "guarded_cmd_nonzero": _topic_last_nonzero(guarded_topic),
        "upstream_cmd_nonzero": _topic_last_nonzero(upstream_topic),
    }


def csv_header_has_pose_columns(path: str) -> bool:
    """Reads only the header line -- never loads the whole file just to
    answer this."""
    with open(path, encoding="utf-8", newline="") as fh:
        try:
            header = next(csv.reader(fh))
        except StopIteration:
            return False
    return all(col in header for col in REQUIRED_POSE_COLUMNS)


def is_csv_growing(path: str, wait_s: float = DEFAULT_GROWTH_WAIT_S) -> bool:
    """Read-only: compares file size before and after a real wait. Never
    writes to the file. `wait_s` should exceed the recorder's own
    --flush-interval-s so a genuinely-running recorder has had a chance
    to flush at least one new row."""
    size_before = os.path.getsize(path)
    time.sleep(wait_s)
    size_after = os.path.getsize(path)
    return size_after > size_before


def main(argv=None) -> int:
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description="Read-only repeatability-specific pre-pulse pose-evidence gate."
    )
    parser.add_argument("--wsl-csv", required=True)
    parser.add_argument("--state-topic", default="/epuck1/state")
    parser.add_argument("--guarded-topic", default="cmd_vel")
    parser.add_argument("--upstream-topic", default="cmd_vel_unguarded")
    parser.add_argument("--required-validity-flags", type=int, default=DEFAULT_REQUIRED_VALIDITY_FLAGS)
    parser.add_argument(
        "--max-pose-sample-staleness-s", type=float, default=DEFAULT_MAX_POSE_SAMPLE_STALENESS_S
    )
    parser.add_argument("--recent-lookback-rows", type=int, default=DEFAULT_RECENT_LOOKBACK_ROWS)
    parser.add_argument("--growth-wait-s", type=float, default=DEFAULT_GROWTH_WAIT_S)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if not os.path.exists(args.wsl_csv):
        result = RepeatabilityPoseReadinessResult(ok=False, reasons=("CSV_NOT_FOUND",))
    else:
        csv_growing = is_csv_growing(args.wsl_csv, wait_s=args.growth_wait_s)
        header_has_pose_columns = csv_header_has_pose_columns(args.wsl_csv)
        wsl_rows = load_wsl_csv_rows(args.wsl_csv)
        now_ns = time.time_ns()
        facts = compute_pose_readiness_facts(
            wsl_rows,
            args.state_topic,
            args.guarded_topic,
            args.upstream_topic,
            now_ns,
            recent_lookback_rows=args.recent_lookback_rows,
        )
        result = evaluate_repeatability_pose_readiness(
            csv_exists=True,
            csv_growing=csv_growing,
            header_has_pose_columns=header_has_pose_columns,
            max_pose_sample_staleness_s=args.max_pose_sample_staleness_s,
            required_validity_flags=args.required_validity_flags,
            **facts,
        )

    print("REPEATABILITY_POSE_READINESS_PASS" if result.ok else "REPEATABILITY_POSE_READINESS_BLOCKED")
    print(f"REASONS={list(result.reasons)}")
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
