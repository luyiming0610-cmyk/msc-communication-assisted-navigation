#!/usr/bin/env python3
"""controller_v3_unified_encounter_20260717 pilot_a3 analysis.

Builds the self.mode timeline from the controller log, using the same
raw_local_mode suffix cooperative_avoider.py's _log() emits to distinguish
LOCAL_ENCOUNTER_FAILSAFE from LOCAL_SENSOR_INVALID -- both collapse
self.mode to the shared "SAFE_STOP_LOCAL_SENSORS" label, so raw_local_mode
is the only way to tell them apart in the log (see cooperative_avoider.py
_control()/_log()).

Also cross-checks the CONSTRAINED phase's HOLD-vs-CREEP choice against the
raw local=(front,left,right) reading logged on the same line: a HOLD tick
must have a raw side/narrow distance still inside decide_local_obstacle()'s
own release band, and a CREEP tick must not. And computes a post-hoc
cumulative-yaw turn ledger directly from /epuck1/state (independent of the
node's own internal turn_ledger_used_rad, which is not logged) to verify the
shared ledger cap was respected and never re-issued mid-encounter.

This script does NOT decide the pilot_a3 pass/fail verdict -- "COMPLETE:
maximum runtime reached" is emitted whether or not the encounter actually
resolved safely (see run_static_box_v3_pilot.sh's own docstring), so
`complete_message` must be read, not assumed. The verdict is decided by
hand against the pilot_a3 acceptance checklist using this summary plus
combined_task_summary.json (collision/clearance/max_x).
"""

import argparse
import json
import math
import re
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


LOG_RE = re.compile(
    r"\[(?P<t>\d+\.\d+)\].*?mode=(?P<mode>\S+) "
    r".*?local=\((?P<front>[-\w.]+),(?P<left>[-\w.]+),(?P<right>[-\w.]+)\)m"
    r".*?cmd=\((?P<lin>[-\d.]+),(?P<ang>[-\d.]+)\)"
    r"(?: raw_local_mode=(?P<raw>\S+))?"
)
COMPLETE_RE = re.compile(r"COMPLETE: (.+)")

# Same thresholds as decide_local_obstacle()'s own defaults
# (local_obstacle_logic.py); side_release_m is the relevant hysteresis band
# for judging whether a HOLD tick's raw reading genuinely still justifies
# holding.
SIDE_RELEASE_M = 0.058
FRONT_RELEASE_M = 0.220

ENCOUNTER_PHASE_OF_MODE = {
    "LOCAL_FRONT_DANGER": "ACTIVE",
    "LOCAL_FRONT_WARN": "ACTIVE",
    "LOCAL_LEFT_SIDE": "ACTIVE",
    "LOCAL_RIGHT_SIDE": "ACTIVE",
    "LOCAL_NARROW": "ACTIVE",
    "LOCAL_CLEARANCE": "ACTIVE",
    "LOCAL_ENCOUNTER_HOLD": "CONSTRAINED",
    "LOCAL_ENCOUNTER_CREEP": "CONSTRAINED",
    "LOCAL_RECOVER": "RECOVERY",
}


def _to_float(text):
    try:
        return float(text)
    except ValueError:
        return math.inf if text == "inf" else math.nan


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
                    "front_m": _to_float(m.group("front")),
                    "left_m": _to_float(m.group("left")),
                    "right_m": _to_float(m.group("right")),
                    "linear_mps": float(m.group("lin")),
                    "angular_rps": float(m.group("ang")),
                    "raw_local_mode": m.group("raw"),
                }
            )
    return rows, complete_message


def mode_transitions(rows):
    transitions = []
    last = None
    for row in rows:
        if row["mode"] != last:
            transitions.append((row["t"], row["mode"], row["raw_local_mode"]))
            last = row["mode"]
    return transitions


def hold_creep_audit(rows):
    """Independently verify HOLD ticks still see a raw side/narrow trigger
    and CREEP ticks do not, using the same release-band logic
    decide_local_obstacle() itself uses."""
    violations = []
    for row in rows:
        side_active = row["left_m"] <= SIDE_RELEASE_M or row["right_m"] <= SIDE_RELEASE_M
        if row["mode"] == "LOCAL_ENCOUNTER_HOLD" and not side_active:
            violations.append(
                {"t": row["t"], "issue": "HOLD tick with no raw side trigger", "row": row}
            )
        if row["mode"] == "LOCAL_ENCOUNTER_CREEP" and side_active:
            violations.append(
                {"t": row["t"], "issue": "CREEP tick despite raw side trigger", "row": row}
            )
    return violations


def encounter_window(rows):
    """[start_t, end_t] spanning the first raw local trigger through the
    last CONSTRAINED/RECOVERY-phase tick (inclusive), or None if the
    encounter never opened."""
    start = None
    end = None
    for row in rows:
        phase = ENCOUNTER_PHASE_OF_MODE.get(row["mode"])
        if row["raw_local_mode"] == "LOCAL_ENCOUNTER_FAILSAFE":
            phase = "CONSTRAINED"
        if phase is None:
            continue
        if start is None:
            start = row["t"]
        end = row["t"]
    if start is None:
        return None
    return start, end


def post_hoc_turn_ledger(bag_path: Path, window):
    """Cumulative |normalize_angle(delta_yaw)| over /epuck1/state samples
    restricted to the encounter window, independent of the node's own
    internal turn_ledger_used_rad (not logged) -- a second, independently
    computed check that the shared ledger cap was respected and never
    re-issued mid-encounter."""
    if window is None:
        return {"window_s": None, "cumulative_turn_rad": 0.0, "sample_count": 0}
    start_t, end_t = window
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if "/epuck1/state" not in types:
        return {"window_s": [start_t, end_t], "cumulative_turn_rad": 0.0, "sample_count": 0}
    state_type = get_message(types["/epuck1/state"])
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/epuck1/state"]))
    prev_yaw = None
    total = 0.0
    samples = 0
    while reader.has_next():
        _, raw, t_ns = reader.read_next()
        # t_ns is the bag's absolute recording epoch, matching the
        # controller log's own "[<epoch>.<frac>]" timestamps directly --
        # do NOT rebase to a bag-relative t_s=0 origin here, or every
        # window comparison against start_t/end_t (which are absolute
        # log-epoch timestamps) silently filters out all samples.
        t_s = t_ns / 1.0e9
        if t_s < start_t or t_s > end_t:
            continue
        msg = deserialize_message(raw, state_type)
        yaw = float(msg.yaw_rad)
        if prev_yaw is not None:
            delta = math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw))
            total += abs(delta)
            samples += 1
        prev_yaw = yaw
    return {
        "window_s": [start_t, end_t],
        "cumulative_turn_rad": total,
        "sample_count": samples,
    }


def realtime_clock_active_streak(rows):
    """Longest unbroken streak of consecutive ticks reporting a raw local
    trigger (front/side/narrow/FAILSAFE), used to confirm sensors did not
    silently drop out mid-encounter (would show as gaps then SAFE_STOP)."""
    streak = 0
    longest = 0
    for row in rows:
        active = (
            row["mode"]
            in (
                "LOCAL_FRONT_DANGER",
                "LOCAL_FRONT_WARN",
                "LOCAL_LEFT_SIDE",
                "LOCAL_RIGHT_SIDE",
                "LOCAL_NARROW",
                "LOCAL_ENCOUNTER_HOLD",
            )
            or row["raw_local_mode"] == "LOCAL_ENCOUNTER_FAILSAFE"
        )
        if active:
            streak += 1
            longest = max(longest, streak)
        else:
            streak = 0
    return longest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("controller_log", type=Path)
    args = parser.parse_args()

    rows, complete_message = parse_log(args.controller_log)
    transitions = mode_transitions(rows)
    modes_seen = [mode for _, mode, _ in transitions]

    failsafe_occurred = any(r["raw_local_mode"] == "LOCAL_ENCOUNTER_FAILSAFE" for r in rows)
    sensor_invalid_occurred = any(
        r["raw_local_mode"] == "LOCAL_SENSOR_INVALID" for r in rows
    )
    local_recover_occurred = "LOCAL_RECOVER" in modes_seen
    cruise_resumed = "CRUISE" in modes_seen
    hold_occurred = "LOCAL_ENCOUNTER_HOLD" in modes_seen
    creep_occurred = "LOCAL_ENCOUNTER_CREEP" in modes_seen

    window = encounter_window(rows)
    ledger = post_hoc_turn_ledger(args.bag_path, window)
    hold_creep_violations = hold_creep_audit(rows)

    # Phase-labelled timeline (collapses consecutive same-phase modes so the
    # report can show ACTIVE -> CONSTRAINED -> RECOVERY -> CLOSED/CRUISE at a
    # glance instead of every individual raw-mode flicker).
    phase_timeline = []
    last_phase = None
    for row in rows:
        phase = ENCOUNTER_PHASE_OF_MODE.get(row["mode"])
        if row["raw_local_mode"] == "LOCAL_ENCOUNTER_FAILSAFE":
            phase = "FAILSAFE"
        elif row["raw_local_mode"] == "LOCAL_SENSOR_INVALID":
            phase = "SENSOR_INVALID"
        elif phase is None and row["mode"] == "CRUISE":
            phase = "CRUISE"
        elif phase is None:
            phase = row["mode"]
        if phase != last_phase:
            phase_timeline.append((row["t"], phase))
            last_phase = phase

    summary = {
        "bag_path": str(args.bag_path),
        "controller_log": str(args.controller_log),
        "complete_message": complete_message,
        "mode_transitions": transitions,
        "phase_timeline": phase_timeline,
        "modes_seen_set": sorted(set(modes_seen)),
        "failsafe_occurred": failsafe_occurred,
        "sensor_invalid_occurred": sensor_invalid_occurred,
        "hold_occurred": hold_occurred,
        "creep_occurred": creep_occurred,
        "local_recover_occurred": local_recover_occurred,
        "cruise_resumed_after_encounter": cruise_resumed,
        "hold_creep_cross_check_violations": hold_creep_violations,
        "post_hoc_turn_ledger": ledger,
        "longest_consecutive_raw_active_streak_ticks": realtime_clock_active_streak(rows),
        "final_mode": modes_seen[-1] if modes_seen else None,
        "log_row_count": len(rows),
    }
    output = args.bag_path / "analysis" / "static_v3_controller_log_summary.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"static v3 controller-log analysis written to: {output}")


if __name__ == "__main__":
    main()
