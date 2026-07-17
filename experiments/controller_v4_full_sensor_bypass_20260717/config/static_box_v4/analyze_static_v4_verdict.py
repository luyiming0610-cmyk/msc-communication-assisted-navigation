#!/usr/bin/env python3
"""controller_v4_ros_time_consistency: consolidated, authoritative pilot
verdict -- combines analyze_static_v4_task.py's collision/clearance
findings with analyze_static_v4_controller_log.py's mode/FAILSAFE findings
and the run script's own realtime-factor/git-commit metadata into ONE JSON
that states explicitly whether the pilot passed, and why not if it did not.

Hard rule, per controller_v4_ros_time_consistency section four: a script
exit code of 0 is NOT a pass. "COMPLETE: maximum runtime reached" is NOT a
pass. Ground-truth collision, FAILSAFE, or SENSOR_INVALID unconditionally
force FAIL regardless of anything else in the summaries. This script is the
single place that combines all of that into one verdict so no other script
or human reading only one of the two summary JSONs can be misled.
"""

import argparse
import json
import subprocess
from pathlib import Path


def _git_commit(repo_dir: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(repo_dir),
            capture_output=True,
            text=True,
            timeout=10,
            check=True,
        )
        return out.stdout.strip()
    except Exception as exc:  # noqa: BLE001 -- best-effort provenance only
        return f"UNKNOWN ({exc})"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--repo-dir", type=Path, required=True)
    parser.add_argument("--preload-factor", type=float, required=True)
    parser.add_argument("--full-load-factor", type=float, required=True)
    parser.add_argument("--controller-version", type=str, default="controller_v4_ros_time_consistency")
    parser.add_argument("--min-clearance-threshold-m", type=float, default=0.005)
    args = parser.parse_args()

    analysis_dir = args.bag_path / "analysis"
    task_summary = json.loads((analysis_dir / "static_v4_task_summary.json").read_text(encoding="utf-8"))
    log_summary = json.loads((analysis_dir / "static_v4_controller_log_summary.json").read_text(encoding="utf-8"))

    collision = bool(task_summary["box_collision_detected"].get("/epuck1/state", False))
    min_clearance = task_summary["minimum_box_clearance_m"].get("/epuck1/state")
    clearance_ok = min_clearance is not None and min_clearance >= args.min_clearance_threshold_m
    passed_box = bool(task_summary["epuck1_passed_box"])
    returned_to_danger = bool(task_summary["returned_to_danger_zone_after_passing"])
    failsafe = bool(log_summary["failsafe_occurred"])
    sensor_invalid = bool(log_summary["sensor_invalid_occurred"])
    pass_confirm = bool(log_summary["pass_confirm_occurred"])
    local_recover = bool(log_summary["local_recover_occurred"])
    cruise_resumed = bool(log_summary["cruise_resumed"])
    final_mode = log_summary["final_mode"]
    complete_message = log_summary["complete_message"] or ""
    # controller_v4_timebase_fix_20260717: this used to be purely
    # informational, which let a run report verdict=PASS while this field
    # simultaneously read true -- self-contradictory, since a pilot whose
    # own internal ceiling (not genuine task completion) produced the
    # final log line has NOT demonstrated the behaviour it claims to. It
    # is now an enforced FAIL reason (see reasons_fail below), so PASS and
    # stopped_by_max_runtime_only=true can never both appear in one report.
    stopped_by_max_runtime_only = "maximum runtime reached" in complete_message and not failsafe and final_mode == "CRUISE"
    legacy_bypass = bool(log_summary["legacy_local_bypass_appeared"])

    realtime_ok = 0.8 <= args.preload_factor <= 1.2 and 0.8 <= args.full_load_factor <= 1.2

    reasons_fail = []
    if collision:
        reasons_fail.append("GROUND_TRUTH_COLLISION")
    if not clearance_ok:
        reasons_fail.append(f"MIN_CLEARANCE_BELOW_THRESHOLD({min_clearance})")
    if not passed_box:
        reasons_fail.append("NEVER_PASSED_BOX")
    if returned_to_danger:
        reasons_fail.append("RETURNED_TO_DANGER_ZONE")
    if failsafe:
        reasons_fail.append(f"FAILSAFE(causes={log_summary.get('failsafe_causes')})")
    if sensor_invalid:
        reasons_fail.append("SENSOR_INVALID")
    if not pass_confirm:
        reasons_fail.append("PASS_CONFIRM_NEVER_REACHED")
    if not local_recover:
        reasons_fail.append("LOCAL_RECOVER_NEVER_REACHED")
    if not cruise_resumed or final_mode != "CRUISE":
        reasons_fail.append(f"DID_NOT_END_IN_STABLE_CRUISE(final_mode={final_mode})")
    if legacy_bypass:
        reasons_fail.append("LEGACY_LOCAL_BYPASS_APPEARED")
    if stopped_by_max_runtime_only:
        reasons_fail.append(
            "STOPPED_BY_MAX_RUNTIME_ONLY(the controller's own internal "
            "elapsed>=max_runtime_s ceiling produced the final COMPLETE "
            "line, not genuine task completion; verdict cannot be PASS "
            "even if every ground-truth check above happens to pass)"
        )
    if not realtime_ok:
        reasons_fail.append(
            f"REALTIME_FACTOR_OUT_OF_RANGE(preload={args.preload_factor},full_load={args.full_load_factor})"
        )

    verdict = "PASS" if not reasons_fail else "FAIL"

    summary = {
        "bag_path": str(args.bag_path),
        "controller_version": args.controller_version,
        "git_commit": _git_commit(args.repo_dir),
        "verdict": verdict,
        "fail_reasons": reasons_fail,
        "stop_reason": {
            "complete_message": complete_message,
            "final_mode": final_mode,
            "final_failsafe_cause": log_summary.get("final_failsafe_cause"),
            "stopped_by_max_runtime_only": stopped_by_max_runtime_only,
        },
        "collision": collision,
        "minimum_box_clearance_m": min_clearance,
        "min_clearance_threshold_m": args.min_clearance_threshold_m,
        # controller_v4_timebase_fix_20260717: this is the ABSOLUTE maximum
        # x_m reached by epuck1 anywhere in the full recorded bag (computed
        # after the run, not a live snapshot). It is NOT the same quantity
        # as any "x_m_at_detection_instant" that may appear in an early-
        # success-watcher log line -- that is a live reading at one instant
        # during the run's settle window and is typically smaller than the
        # eventual maximum, since the robot keeps moving afterwards.
        "max_epuck1_x_m": task_summary["max_epuck1_x_m"],
        "epuck1_passed_box": passed_box,
        "returned_to_danger_zone_after_passing": returned_to_danger,
        "pass_confirm_occurred": pass_confirm,
        "local_recover_occurred": local_recover,
        "cruise_resumed": cruise_resumed,
        "failsafe_occurred": failsafe,
        "failsafe_causes": log_summary.get("failsafe_causes"),
        "sensor_invalid_occurred": sensor_invalid,
        "legacy_local_bypass_appeared": legacy_bypass,
        "preload_realtime_factor": args.preload_factor,
        "full_load_realtime_factor": args.full_load_factor,
        "realtime_factor_ok": realtime_ok,
        "excluded_reset_artifact_samples": task_summary["excluded_reset_artifact_samples"],
        "excluded_out_of_window_samples": task_summary["excluded_out_of_window_samples"],
        "note": (
            "This is the ONLY authoritative verdict field for this pilot. "
            "A script exit code of 0, or 'COMPLETE: maximum runtime reached' "
            "appearing in the log, must NEVER be read as PASS on their own -- "
            "see fail_reasons above for what was actually checked."
        ),
    }
    output = analysis_dir / "static_v4_verdict.json"
    output.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"VERDICT: {verdict}")
    print(f"static v4 verdict written to: {output}")


if __name__ == "__main__":
    main()
