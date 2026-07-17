#!/usr/bin/env python3
"""controller_v4_full_sensor_bypass_20260717 pilot controller-log analysis.

Builds the self.mode/phase timeline, distinguishing LOCAL_ENCOUNTER_FAILSAFE
from LOCAL_SENSOR_INVALID via the raw_local_mode suffix (both collapse
self.mode to "SAFE_STOP_LOCAL_SENSORS"), and reports whether PASS_CONFIRM/
LOCAL_RECOVER/CRUISE were ever reached, whether the legacy "LOCAL_BYPASS"
mode name ever appeared (it must not -- that branch no longer exists in v4),
and simulation realtime-factor bookkeeping is left to the run script's own
verify_realtime_factor (PRELOAD/FULL_LOAD), unchanged.

Does NOT decide the pilot's pass/fail verdict -- see run_static_box_v4_pilot.sh
and analyze_static_v4_task.py's own module docstrings for why "COMPLETE:
maximum runtime reached" must never be read as success.
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


def parse_log(log_path: Path):
    rows = []
    complete_message = None
    with log_path.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            m = COMPLETE_RE.search(line)
            if m and complete_message is None:
                complete_message = m.group(1).strip()
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
    return rows, complete_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("controller_log", type=Path)
    args = parser.parse_args()

    rows, complete_message = parse_log(args.controller_log)
    transitions = []
    last_mode = None
    for row in rows:
        if row["mode"] != last_mode:
            transitions.append((row["t"], row["mode"], row["raw_local_mode"]))
            last_mode = row["mode"]
    modes_seen = sorted(set(row["mode"] for row in rows))

    summary = {
        "bag_path": str(args.bag_path),
        "controller_log": str(args.controller_log),
        "complete_message": complete_message,
        "mode_transitions": transitions,
        "modes_seen_set": modes_seen,
        "legacy_local_bypass_appeared": "LOCAL_BYPASS" in modes_seen,
        "detect_turn_occurred": "LOCAL_DETECT_TURN" in modes_seen
        or "LOCAL_FRONT_DANGER" in modes_seen
        or "LOCAL_FRONT_WARN" in modes_seen,
        "side_track_occurred": "LOCAL_SIDE_TRACK" in modes_seen
        or "LOCAL_SIDE_TRACK_HOLD" in modes_seen,
        "pass_confirm_occurred": "LOCAL_PASS_CONFIRM" in modes_seen,
        "local_recover_occurred": "LOCAL_RECOVER" in modes_seen,
        "cruise_resumed": "CRUISE" in modes_seen,
        "failsafe_occurred": any(row["raw_local_mode"] == "LOCAL_ENCOUNTER_FAILSAFE" for row in rows),
        "sensor_invalid_occurred": any(row["raw_local_mode"] == "LOCAL_SENSOR_INVALID" for row in rows),
        "final_mode": rows[-1]["mode"] if rows else None,
        "log_row_count": len(rows),
    }
    output = args.bag_path / "analysis" / "static_v4_controller_log_summary.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"static v4 controller-log analysis written to: {output}")


if __name__ == "__main__":
    main()
