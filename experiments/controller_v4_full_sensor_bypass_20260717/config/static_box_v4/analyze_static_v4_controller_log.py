#!/usr/bin/env python3
"""controller_v4_full_sensor_bypass_20260717 pilot controller-log analysis.

Builds the self.mode/phase timeline, distinguishing LOCAL_ENCOUNTER_FAILSAFE
from LOCAL_SENSOR_INVALID via the raw_local_mode suffix (both collapse
self.mode to "SAFE_STOP_LOCAL_SENSORS"), and reports whether PASS_CONFIRM/
LOCAL_RECOVER/CRUISE were ever reached, whether the legacy "LOCAL_BYPASS"
mode name ever appeared (it must not -- that branch no longer exists in v4).

controller_v4_ros_time_consistency: also parses the fine-grained
"TRANSITION ..." log lines cooperative_avoider.py now emits on every mode
change, extracting the explicit, never-generic failsafe_cause enum
(TURN_LEDGER_CEILING / BYPASS_EXTENSION_CEILING / DURATION_CEILING /
PERSISTENT_DRIFT) instead of a bare "FAILSAFE" string.

Does NOT decide the pilot's pass/fail verdict on its own -- see
analyze_static_v4_verdict.py, which combines this summary with
analyze_static_v4_task.py's collision/clearance verdict and the run
script's realtime-factor/git-commit metadata into one authoritative
report. "COMPLETE: maximum runtime reached" must never be read as success.
"""

import argparse
import json
import re
from pathlib import Path


LOG_RE = re.compile(
    r"\[(?P<t>\d+\.\d+)\].*?mode=(?P<mode>\S+) "
    r"(?:distance=(?P<distance>[-\d.]+)m tcpa=(?P<tcpa>[-\d.]+)s "
    r"dcpa=(?P<dcpa>[-\d.]+)m closing=(?P<closing>[-\d.]+)m/s |peer=disabled )"
    r"local=\((?P<front>[-\w.]+),(?P<left>[-\w.]+),(?P<right>[-\w.]+)\)m"
    r" cmd=\((?P<lin>[-\d.]+),(?P<ang>[-\d.]+)\)"
    r"(?: raw_local_mode=(?P<raw>\S+))?"
)
COMPLETE_RE = re.compile(r"COMPLETE: (.+)")
TRANSITION_RE = re.compile(
    r"TRANSITION wall_time=(?P<wall>[-\d.]+) ros_time=(?P<ros>[-\d.]+) "
    r"mode=(?P<from>\S+)->(?P<to>\S+) phase=(?P<phase>\S+) "
    r"encounter_elapsed=(?P<encounter_elapsed>\S+) "
    r"pass_confirm_elapsed=(?P<pass_confirm_elapsed>\S+) "
    r"duration_remaining=(?P<duration_remaining>\S+) "
    r"turn_ledger=(?P<turn_ledger>[-\d.]+)rad "
    r"longitudinal=(?P<longitudinal>\S+) lateral=(?P<lateral>\S+) "
    r"zones=\((?P<zones>[^)]*)\) "
    r"applied_cmd=\((?P<applied_lin>[-\d.]+),(?P<applied_ang>[-\d.]+)\) "
    r"failsafe_cause=(?P<failsafe_cause>\S+) drift_events=(?P<drift_events>\d+)"
)


def _maybe_float(text):
    if text in (None, "None"):
        return None
    try:
        return float(text)
    except ValueError:
        return None


def parse_log(log_path: Path):
    rows = []
    transitions_fine = []
    complete_message = None
    with log_path.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            m = COMPLETE_RE.search(line)
            if m and complete_message is None:
                complete_message = m.group(1).strip()
            m = TRANSITION_RE.search(line)
            if m:
                transitions_fine.append(
                    {
                        "wall_time": float(m.group("wall")),
                        "ros_time": float(m.group("ros")),
                        "mode_from": m.group("from"),
                        "mode_to": m.group("to"),
                        "phase": m.group("phase"),
                        "encounter_elapsed_s": _maybe_float(m.group("encounter_elapsed").rstrip("s")),
                        "pass_confirm_elapsed_s": _maybe_float(m.group("pass_confirm_elapsed").rstrip("s")),
                        "duration_remaining_s": _maybe_float(m.group("duration_remaining").rstrip("s")),
                        "turn_ledger_rad": float(m.group("turn_ledger")),
                        "longitudinal_m": _maybe_float(m.group("longitudinal").rstrip("m")),
                        "lateral_m": _maybe_float(m.group("lateral").rstrip("m")),
                        "applied_cmd": [float(m.group("applied_lin")), float(m.group("applied_ang"))],
                        "failsafe_cause": m.group("failsafe_cause"),
                        "drift_events": int(m.group("drift_events")),
                    }
                )
                continue
            m = LOG_RE.search(line)
            if not m:
                continue
            rows.append(
                {
                    "t": float(m.group("t")),
                    "mode": m.group("mode"),
                    "raw_local_mode": m.group("raw"),
                }
            )
    return rows, transitions_fine, complete_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("controller_log", type=Path)
    args = parser.parse_args()

    rows, transitions_fine, complete_message = parse_log(args.controller_log)
    transitions = []
    last_mode = None
    for row in rows:
        if row["mode"] != last_mode:
            transitions.append((row["t"], row["mode"], row["raw_local_mode"]))
            last_mode = row["mode"]
    modes_seen = sorted(set(row["mode"] for row in rows))

    failsafe_causes = sorted(
        {t["failsafe_cause"] for t in transitions_fine if t["failsafe_cause"] != "NONE"}
    )
    final_transition = transitions_fine[-1] if transitions_fine else None

    summary = {
        "bag_path": str(args.bag_path),
        "controller_log": str(args.controller_log),
        "complete_message": complete_message,
        "mode_transitions": transitions,
        "fine_grained_transitions": transitions_fine,
        "modes_seen_set": modes_seen,
        "legacy_local_bypass_appeared": "LOCAL_BYPASS" in modes_seen,
        "detect_turn_occurred": "LOCAL_DETECT_TURN" in modes_seen
        or "LOCAL_FRONT_DANGER" in modes_seen
        or "LOCAL_FRONT_WARN" in modes_seen,
        "side_track_occurred": "LOCAL_SIDE_TRACK" in modes_seen
        or "LOCAL_SIDE_TRACK_HOLD" in modes_seen
        or "LOCAL_SIDE_TRACK_CREEP" in modes_seen,
        "pass_confirm_occurred": "LOCAL_PASS_CONFIRM" in modes_seen,
        "local_recover_occurred": "LOCAL_RECOVER" in modes_seen,
        "cruise_resumed": "CRUISE" in modes_seen,
        "failsafe_occurred": any(row["raw_local_mode"] == "LOCAL_ENCOUNTER_FAILSAFE" for row in rows),
        "failsafe_causes": failsafe_causes,
        "final_failsafe_cause": final_transition["failsafe_cause"] if final_transition and final_transition["failsafe_cause"] != "NONE" else None,
        "sensor_invalid_occurred": any(row["raw_local_mode"] == "LOCAL_SENSOR_INVALID" for row in rows),
        "final_mode": rows[-1]["mode"] if rows else None,
        "final_drift_events": final_transition["drift_events"] if final_transition else 0,
        "log_row_count": len(rows),
        "fine_grained_transition_count": len(transitions_fine),
    }
    output = args.bag_path / "analysis" / "static_v4_controller_log_summary.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"static v4 controller-log analysis written to: {output}")


if __name__ == "__main__":
    main()
