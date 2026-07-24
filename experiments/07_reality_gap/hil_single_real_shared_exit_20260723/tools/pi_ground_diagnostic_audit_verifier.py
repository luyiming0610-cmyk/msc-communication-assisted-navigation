#!/usr/bin/env python3
"""Read-only Pi-side audit verifier for the first ground diagnostic.

Closes the structural gap found 2026-07-24 during the first live
attempt: run_ground_diagnostic_preflight.sh's live-zero-state phase,
run from WSL, tried to read the Pi's command-audit JSONL by a plain
local filesystem path -- but the Pi and WSL machine share no
filesystem, only a network connection, so the file was never actually
readable from WSL. The check correctly failed closed
(FileNotFoundError -> not proven zero -> BLOCKED), which was the safe
behavior, but the underlying design could never pass while the Pi
audit file only exists on the Pi.

This module is meant to be run ON THE PI ITSELF (or against a
verified, hash-checked copy of the JSONL), producing a small,
machine-readable verdict file that is then combined with the WSL-side
live verdict by hil_ground_diagnostic_phases.evaluate_combined_gate().
Pure logic (parsing, growth-sampling, verdict construction) is
decoupled from real time.sleep()/file I/O via injected callables, so
it is fully unit-testable without an actual Pi or actual elapsed time.

Reuses analyze_ground_diagnostic.load_pi_jsonl_records and
compute_pi_command_maxima (not reimplemented) for JSONL parsing and
nonzero-command detection -- both already parse with Python's `json`
module, never grep, and already preserve a malformed line as a visible
PARSE_ERROR marker rather than silently dropping it.

No ROS/rclpy dependency. Never publishes anything, never contacts the
robot -- reads a file (or samples its row count) only.
"""
from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable, Optional

from analyze_ground_diagnostic import compute_pi_command_maxima, load_pi_jsonl_records

DEFAULT_GROWTH_INTERVAL_S = 1.0
DEFAULT_MAX_VERDICT_AGE_S = 300.0


def count_jsonl_lines(path: str) -> int:
    """Counts non-empty lines without parsing -- used only for the
    cheap growth sample; the real parse (and malformed-line detection)
    happens separately via load_pi_jsonl_records."""
    count = 0
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if line.strip():
                count += 1
    return count


@dataclass(frozen=True)
class GrowthSample:
    before: Optional[int]
    after: Optional[int]
    growing: bool
    file_missing: bool = False


def sample_row_count_growth(
    path: str,
    interval_s: float = DEFAULT_GROWTH_INTERVAL_S,
    count_fn: Callable[[str], int] = count_jsonl_lines,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> GrowthSample:
    """Samples the row count twice with a bounded interval in between
    -- count_fn/sleep_fn are injectable so tests can exercise this
    without a real file growing in real time. A missing file at either
    sample point is reported as file_missing=True, growing=False --
    never silently treated as zero rows."""
    try:
        before = count_fn(path)
    except FileNotFoundError:
        return GrowthSample(before=None, after=None, growing=False, file_missing=True)

    sleep_fn(interval_s)

    try:
        after = count_fn(path)
    except FileNotFoundError:
        return GrowthSample(before=before, after=None, growing=False, file_missing=True)

    return GrowthSample(before=before, after=after, growing=after > before)


@dataclass(frozen=True)
class PiAuditVerdict:
    ok: bool
    reasons: tuple = field(default_factory=tuple)
    run_id: Optional[str] = None
    jsonl_path: Optional[str] = None
    total_records: Optional[int] = None
    malformed_count: Optional[int] = None
    nonzero_received_count: Optional[int] = None
    nonzero_applied_count: Optional[int] = None
    growing: Optional[bool] = None
    latest_zero_reason: Optional[str] = None
    latest_linear: Optional[float] = None
    latest_angular: Optional[float] = None
    generated_at_utc: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "reasons": list(self.reasons),
            "run_id": self.run_id,
            "jsonl_path": self.jsonl_path,
            "total_records": self.total_records,
            "malformed_count": self.malformed_count,
            "nonzero_received_count": self.nonzero_received_count,
            "nonzero_applied_count": self.nonzero_applied_count,
            "growing": self.growing,
            "latest_zero_reason": self.latest_zero_reason,
            "latest_linear": self.latest_linear,
            "latest_angular": self.latest_angular,
            "generated_at_utc": self.generated_at_utc,
        }


def _find_latest_tick(records: list) -> dict:
    for record in reversed(records):
        if record.get("event") == "tick_applied":
            return record
    return {}


def build_pi_audit_verdict(
    *,
    jsonl_path: str,
    run_id: str,
    growth: GrowthSample,
    records: Optional[list],
    now: Optional[datetime] = None,
) -> PiAuditVerdict:
    """Pure verdict construction from an already-sampled growth result
    and already-loaded records (or None if the file could not be read
    at all). Never treats a missing/inaccessible file as "proven
    nonzero" -- that case gets its own JSONL_NOT_AVAILABLE reason,
    distinct from PI_EVIDENCE_CONTAINS_NONZERO_COMMAND.
    """
    now = now or datetime.now(timezone.utc)
    generated_at = now.isoformat()

    if growth.file_missing or records is None:
        return PiAuditVerdict(
            ok=False,
            reasons=("JSONL_NOT_AVAILABLE",),
            run_id=run_id,
            jsonl_path=jsonl_path,
            generated_at_utc=generated_at,
        )

    malformed_count = sum(1 for r in records if r.get("event") == "PARSE_ERROR")
    maxima = compute_pi_command_maxima(records)
    latest_tick = _find_latest_tick(records)

    reasons = []
    if malformed_count:
        reasons.append("MALFORMED_JSON_LINES_PRESENT")
    if not growth.growing:
        reasons.append("PI_JSONL_NOT_GROWING")
    if maxima.nonzero_received_count != 0 or maxima.nonzero_applied_count != 0:
        reasons.append("PI_EVIDENCE_CONTAINS_NONZERO_COMMAND")

    return PiAuditVerdict(
        ok=not reasons,
        reasons=tuple(reasons),
        run_id=run_id,
        jsonl_path=jsonl_path,
        total_records=len(records),
        malformed_count=malformed_count,
        nonzero_received_count=maxima.nonzero_received_count,
        nonzero_applied_count=maxima.nonzero_applied_count,
        growing=growth.growing,
        latest_zero_reason=latest_tick.get("zero_reason"),
        latest_linear=latest_tick.get("linear"),
        latest_angular=latest_tick.get("angular"),
        generated_at_utc=generated_at,
    )


def verify_pi_audit(
    jsonl_path: str,
    run_id: str,
    growth_interval_s: float = DEFAULT_GROWTH_INTERVAL_S,
) -> PiAuditVerdict:
    """Thin orchestration: sample growth, load records (if the file is
    reachable), build the verdict. This is the only function that
    performs real I/O/sleep with no injected fakes -- used by the CLI.
    """
    growth = sample_row_count_growth(jsonl_path, interval_s=growth_interval_s)
    records = None
    if not growth.file_missing:
        try:
            records = load_pi_jsonl_records(jsonl_path)
        except FileNotFoundError:
            growth = GrowthSample(before=growth.before, after=None, growing=False, file_missing=True)
    return build_pi_audit_verdict(jsonl_path=jsonl_path, run_id=run_id, growth=growth, records=records)


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--path", required=True, help="exact approved Pi command-audit JSONL path")
    parser.add_argument("--run-id", required=True, help="run identifier shared with the WSL-side evidence")
    parser.add_argument("--growth-interval-s", type=float, default=DEFAULT_GROWTH_INTERVAL_S)
    parser.add_argument("--output-json", default=None, help="optional path to also write the verdict as JSON")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    verdict = verify_pi_audit(args.path, args.run_id, growth_interval_s=args.growth_interval_s)

    if args.output_json:
        with open(args.output_json, "w", encoding="utf-8") as fh:
            json.dump(verdict.to_dict(), fh, indent=2)
            fh.write("\n")

    print(f"PI_AUDIT_VERDICT={'PASS' if verdict.ok else 'BLOCKED'}")
    print(f"REASONS={list(verdict.reasons)}")
    print(f"RUN_ID={verdict.run_id}")
    print(f"JSONL_PATH={verdict.jsonl_path}")
    print(f"TOTAL_RECORDS={verdict.total_records}")
    print(f"MALFORMED_COUNT={verdict.malformed_count}")
    print(f"NONZERO_RECEIVED_COUNT={verdict.nonzero_received_count}")
    print(f"NONZERO_APPLIED_COUNT={verdict.nonzero_applied_count}")
    print(f"GROWING={verdict.growing}")
    print(f"LATEST_ZERO_REASON={verdict.latest_zero_reason}")
    print(f"LATEST_LINEAR={verdict.latest_linear}")
    print(f"LATEST_ANGULAR={verdict.latest_angular}")
    print(f"GENERATED_AT_UTC={verdict.generated_at_utc}")
    return 0 if verdict.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
