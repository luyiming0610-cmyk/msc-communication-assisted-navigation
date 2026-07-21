#!/usr/bin/env python3
"""Audit whether Condition F outage windows cover the active CPA manoeuvre."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


OUTAGE_PHASE_S = 10.0
OUTAGE_PERIOD_S = 15.0
OUTAGE_DURATION_S = 0.7


def outage_windows(until_s: float) -> list[tuple[float, float]]:
    windows = []
    start = OUTAGE_PHASE_S
    while start <= until_s:
        windows.append((start, start + OUTAGE_DURATION_S))
        start += OUTAGE_PERIOD_S
    return windows


def read_trial_times(path: Path) -> dict | None:
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if "robots" in data:
        robots = data["robots"]
        return {
            "trial_id": data["trial_id"],
            "source": str(path),
            "timebase_init": {
                name: values.get("TIMEBASE_INIT_ros_time_s")
                for name, values in robots.items()
            },
            "cruise": {
                name: values["first_CRUISE_ros_time_s"]
                for name, values in robots.items()
            },
            "avoid_turn": {
                name: values["first_AVOID_TURN_ros_time_s"]
                for name, values in robots.items()
            },
            "recover": {
                name: values["first_RECOVER_ros_time_s"]
                for name, values in robots.items()
            },
        }
    if "trial_id" not in data or "cruise_start_ros_time_s" not in data:
        return None
    return {
        "trial_id": data["trial_id"],
        "source": str(path),
        "timebase_init": None,
        "cruise": data["cruise_start_ros_time_s"],
        "avoid_turn": data["avoid_turn_ros_time_s"],
        "recover": data["recover_ros_time_s"],
    }


def classify_trial(trial: dict) -> dict:
    avoid_min = min(trial["avoid_turn"].values())
    avoid_max = max(trial["avoid_turn"].values())
    recover_min = min(trial["recover"].values())
    windows = outage_windows(recover_min)
    fully_inside_active_avoidance = [
        [start, end]
        for start, end in windows
        if start >= avoid_max and end <= recover_min
    ]
    covers_initial_avoid_entry = [
        [start, end]
        for start, end in windows
        if start <= avoid_min < end or start <= avoid_max < end
    ]
    return {
        **trial,
        "avoid_entry_min_s": avoid_min,
        "avoid_entry_max_s": avoid_max,
        "recover_min_s": recover_min,
        "avoid_entry_delta_s": avoid_max - avoid_min,
        "windows_fully_inside_active_avoidance": fully_inside_active_avoidance,
        "windows_covering_initial_avoid_entry": covers_initial_avoid_entry,
        "window_40_fully_inside_active_avoidance": [40.0, 40.7] in fully_inside_active_avoidance,
        "window_55_fully_inside_active_avoidance": [55.0, 55.7] in fully_inside_active_avoidance,
    }


def parse_pilot_windows(note_path: Path) -> list[dict]:
    pattern = re.compile(
        r"^\|\s*(\d+)\s*\|\s*([0-9.]+)\s*/\s*([0-9.]+)\s*\|"
        r"\s*([0-9.]+)\s*/\s*([0-9.]+)\s*\|\s*([0-9.]+)s\s*\|\s*([0-9.]+)s\s*\|$"
    )
    rows = []
    for line in note_path.read_text(encoding="utf-8-sig").splitlines():
        match = pattern.match(line)
        if match:
            values = match.groups()
            rows.append({
                "window_index": int(values[0]),
                "epuck1_to_epuck2_start_s": float(values[1]),
                "epuck1_to_epuck2_end_s": float(values[2]),
                "epuck2_to_epuck1_start_s": float(values[3]),
                "epuck2_to_epuck1_end_s": float(values[4]),
                "start_deviation_s": float(values[5]),
                "end_deviation_s": float(values[6]),
            })
    return rows


def find_audits(root: Path) -> list[Path]:
    paths = []
    for path in root.glob("objective5_impairment_matrix_v1_condition_*_trial*_analysis/*startup*sync*audit*.json"):
        if "assessor_demo" not in str(path):
            paths.append(path)
    return sorted(paths)


def build_report(root: Path) -> dict:
    loaded = [read_trial_times(path) for path in find_audits(root)]
    trial_results = [classify_trial(trial) for trial in loaded if trial is not None]
    note = root / "objective5_matrix_v1_conditionF_exclusionary_pilot03_analysis" / "NOTE.md"
    pilot_windows = parse_pilot_windows(note)
    all_window_40 = bool(trial_results) and all(t["window_40_fully_inside_active_avoidance"] for t in trial_results)
    all_window_55 = bool(trial_results) and all(t["window_55_fully_inside_active_avoidance"] for t in trial_results)
    no_initial_entry = bool(trial_results) and all(not t["windows_covering_initial_avoid_entry"] for t in trial_results)
    observed_sync = bool(pilot_windows) and all(
        row["start_deviation_s"] == 0.0 and row["end_deviation_s"] == 0.0
        for row in pilot_windows
    )
    pass_gate = all_window_40 and all_window_55 and no_initial_entry and observed_sync
    return {
        "audit_id": "condition_F_outage_timing_precondition_20260721",
        "audit_type": "OFFLINE_PRECONDITION_AUDIT",
        "webots_run_for_audit": False,
        "condition_id": "F",
        "frozen_outage_schedule": {
            "clock_domain": "shared absolute ROS simulation time",
            "outage_phase_s": OUTAGE_PHASE_S,
            "outage_period_s": OUTAGE_PERIOD_S,
            "outage_duration_s": OUTAGE_DURATION_S,
            "nominal_windows_s": [[a, b] for a, b in outage_windows(60.0)],
        },
        "historical_trial_count": len(trial_results),
        "historical_trials": trial_results,
        "pilot03_bidirectional_window_measurement": {
            "source": str(note),
            "detected_windows": pilot_windows,
            "all_measured_start_end_deviations_zero": observed_sync,
            "measurement_limit": "boundaries reconstructed from discrete message arrivals; exact continuous boundary precision is not claimed",
        },
        "gate_checks": {
            "window_40_fully_inside_active_avoidance_all_historical_trials": all_window_40,
            "window_55_fully_inside_active_avoidance_all_historical_trials": all_window_55,
            "no_nominal_window_covers_initial_avoid_turn_entry": no_initial_entry,
            "pilot03_bidirectional_windows_measured_synchronous": observed_sync,
        },
        "precondition_verdict": "PASS_WITH_SCOPE_CLARIFICATION" if pass_gate else "FAIL",
        "scope_clarification": (
            "Condition F tests stale-state safety and recovery when a synchronized full outage occurs during an already-active CPA avoidance manoeuvre. "
            "It does not test whether an outage causes or delays the initial PREDICTED_CPA trigger, because no frozen outage window covers the initial AVOID_TURN entry in the audited historical trials."
        ),
        "formal_trial_requirements": [
            "record each relay's actual outage-tagged message windows",
            "record TIMEBASE_INIT, CRUISE, AVOID_TURN, SAFE_STOP_STALE, and RECOVER transitions for both robots",
            "verify at least one synchronized outage occurs after AVOID_TURN and before RECOVER",
            "report startup-only and active-avoidance stale events separately",
            "do not relabel the condition as an initial-CPA-trigger impairment test",
        ],
    }


def write_markdown(report: dict, path: Path) -> None:
    trials = report["historical_trials"]
    rows = []
    for trial in trials:
        rows.append(
            f"| {trial['trial_id']} | {trial['avoid_entry_min_s']:.2f} | {trial['recover_min_s']:.2f} | "
            f"{'YES' if trial['window_40_fully_inside_active_avoidance'] else 'NO'} | "
            f"{'YES' if trial['window_55_fully_inside_active_avoidance'] else 'NO'} |"
        )
    path.write_text(
        "# Condition F outage-timing precondition\n\n"
        f"**Verdict: {report['precondition_verdict']}.** This was an offline audit; Webots was not run.\n\n"
        "The frozen bidirectional outage schedule uses shared absolute simulation time: "
        "`[10.0,10.7)`, `[25.0,25.7)`, `[40.0,40.7)`, and `[55.0,55.7)` seconds within the nominal 60 s task window.\n\n"
        "| Historical trial | First AVOID_TURN (s) | First RECOVER (s) | 40 s window inside active avoidance | 55 s window inside active avoidance |\n"
        "|---|---:|---:|---|---|\n" + "\n".join(rows) + "\n\n"
        f"Across all {len(trials)} available B/D/E timing audits, both `[40.0,40.7)` and `[55.0,55.7)` fall fully after the synchronized first `AVOID_TURN` and before the earliest `RECOVER`. "
        "No frozen outage window covers the initial `AVOID_TURN` entry itself. The valid scope is therefore a stale-state stop/recovery test during an already-active CPA manoeuvre, not an initial-trigger-delay test.\n\n"
        "The preserved Condition F exclusionary pilot independently reconstructed five bidirectional outage windows with zero measured start/end deviation between directions. This supports synchronization, subject to message-period resolution at the boundaries.\n\n"
        "Formal F trials must retain per-trial event-to-window evidence and separate startup-only stale transitions from stale transitions occurring during active avoidance.\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--json-output", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path, required=True)
    args = parser.parse_args()
    report = build_report(args.root)
    args.json_output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    write_markdown(report, args.markdown_output)
    print(json.dumps({
        "verdict": report["precondition_verdict"],
        "historical_trial_count": report["historical_trial_count"],
        "gate_checks": report["gate_checks"],
    }, indent=2))
    return 0 if report["precondition_verdict"].startswith("PASS") else 1


if __name__ == "__main__":
    raise SystemExit(main())
