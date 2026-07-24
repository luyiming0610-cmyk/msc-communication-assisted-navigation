#!/usr/bin/env python3
"""CLI wrapper computing the live-zero-state combined verdict --
extracted 2026-07-24 from an inline heredoc in
run_ground_diagnostic_preflight.sh after that heredoc's positional
argv unpacking had an off-by-one bug (`sys.argv[1:20]` silently
dropped the last of 20 passed arguments, crashing with
"not enough values to unpack"). That heredoc was never exercised by
any test -- only the underlying evaluate_wsl_live_state() /
evaluate_combined_gate() functions were unit-tested directly in
Python, never through the shell script's actual argv plumbing.

Using named --flags here (via argparse) instead of positional
heredoc arguments removes the entire class of off-by-one/argument-
count bugs the previous design was vulnerable to, and this file is
directly subprocess-invocable by a test with a fixed, realistic
argument list -- see test_hil_live_zero_state_verdict.py.

No ROS/rclpy dependency; performs no I/O beyond reading its own CLI
arguments and printing a verdict.
"""
from __future__ import annotations

import ast
import sys

from hil_ground_diagnostic_phases import evaluate_combined_gate, evaluate_wsl_live_state


def _as_bool(s: str) -> bool:
    return s == "true"


def _as_list(s: str) -> tuple:
    try:
        return tuple(ast.literal_eval(s))
    except (ValueError, SyntaxError):
        return ()


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--validity-flags", default="")
    parser.add_argument("--bridge-connected", default="false")
    parser.add_argument("--wsl-csv-growing", default="false")
    parser.add_argument("--guard-sole-publisher", default="false")
    parser.add_argument("--guard-armed", default="true")
    parser.add_argument("--cmd-vel-all-zero", default="false")
    parser.add_argument("--upstream-zero-or-absent", default="false")
    parser.add_argument("--forbidden-process-found", default="false")
    parser.add_argument("--wsl-evidence-all-zero", default="false")
    parser.add_argument("--pi-verdict-available", default="false")
    parser.add_argument("--pi-verdict-malformed", default="false")
    parser.add_argument("--pi-verdict-ok", default="false")
    parser.add_argument("--pi-verdict-reasons", default="[]")
    parser.add_argument("--pi-run-id", default="")
    parser.add_argument("--pi-verdict-generated-at", default="")
    parser.add_argument("--pi-jsonl-path", default="")
    parser.add_argument("--wsl-run-id", default="")
    parser.add_argument("--wsl-evidence-path", default="")
    parser.add_argument("--expected-pi-jsonl-path", default="")
    parser.add_argument("--pi-verdict-max-age-s", default="300")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    wsl_result = evaluate_wsl_live_state(
        validity_flags=int(args.validity_flags) if args.validity_flags.strip().isdigit() else None,
        bridge_connected=_as_bool(args.bridge_connected),
        wsl_csv_growing=_as_bool(args.wsl_csv_growing),
        guard_sole_publisher=_as_bool(args.guard_sole_publisher),
        guard_armed=_as_bool(args.guard_armed),
        cmd_vel_all_zero=_as_bool(args.cmd_vel_all_zero),
        upstream_zero_or_absent=_as_bool(args.upstream_zero_or_absent),
        forbidden_process_found=_as_bool(args.forbidden_process_found),
        wsl_evidence_all_zero=_as_bool(args.wsl_evidence_all_zero),
    )

    result = evaluate_combined_gate(
        wsl_result=wsl_result,
        pi_verdict_ok=_as_bool(args.pi_verdict_ok),
        pi_verdict_reasons=_as_list(args.pi_verdict_reasons),
        pi_verdict_available=_as_bool(args.pi_verdict_available),
        pi_verdict_malformed=_as_bool(args.pi_verdict_malformed),
        wsl_run_id=args.wsl_run_id or None,
        pi_run_id=args.pi_run_id or None,
        wsl_evidence_path=args.wsl_evidence_path,
        pi_evidence_path=args.pi_jsonl_path or None,
        expected_pi_evidence_path=args.expected_pi_jsonl_path or None,
        pi_verdict_generated_at_utc=args.pi_verdict_generated_at or None,
        pi_verdict_max_age_s=float(args.pi_verdict_max_age_s),
    )

    if result.ok:
        print("LIVE_ZERO_STATE_VERDICT=PASS")
    else:
        print("LIVE_ZERO_STATE_VERDICT=BLOCKED")
        print(f"REASONS={list(result.reasons)}")
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
