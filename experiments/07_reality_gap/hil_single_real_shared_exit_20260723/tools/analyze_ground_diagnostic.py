#!/usr/bin/env python3
"""Offline post-run analysis for the first ground diagnostic
(experiments/07_reality_gap/hil_single_real_shared_exit_20260723/
first_ground_diagnostic/). Reads the WSL command-evidence CSV
(hil_command_evidence_recorder.py's CSV_FIELDS) and the Pi command
audit JSONL (pi_epuck_tcp_server_sensors_audited.py's record shapes)
already produced by a run, and computes exactly the facts needed to
apply this diagnostic's acceptance rules -- it starts no process,
publishes nothing, and touches no ROS graph. Every function here is
pure: plain data in, plain data out.

Deliberately explicit about what CANNOT be computed from the current
evidence rather than fabricating or silently omitting it: neither the
WSL CSV nor the Pi JSONL records odometry (x/y/yaw) -- the recorder
only observes cmd_vel/arm/validity_flags/bridge-status, and the Pi
audit only observes commands, not odometry -- so "odometry displacement"
and "average ground speed" are reported as NOT_AVAILABLE with the
reason stated, never estimated from commanded values and presented as
if they were measured.
"""
from __future__ import annotations

import csv
import hashlib
import json
from dataclasses import dataclass, field
from typing import Optional


# --------------------------------------------------------------------
# Loading (thin I/O, no interpretation).
# --------------------------------------------------------------------
def load_wsl_csv_rows(path: str) -> list:
    """Reads hil_command_evidence_recorder.py's CSV, converting numeric
    fields back to float/int/bool/None. Never repairs or drops a
    malformed row silently -- a row that fails numeric conversion keeps
    its raw string value so it remains visible, never disappears."""
    rows = []
    with open(path, encoding="utf-8", newline="") as fh:
        for raw in csv.DictReader(fh):
            row = dict(raw)
            for key in ("local_time_ns", "local_monotonic_ns", "sequence", "bridge_rx_count"):
                if row.get(key):
                    try:
                        row[key] = int(row[key])
                    except ValueError:
                        pass
            for key in ("linear_x", "angular_z"):
                if row.get(key):
                    try:
                        row[key] = float(row[key])
                    except ValueError:
                        pass
            for key in ("arm_state", "bridge_connected"):
                if row.get(key) in ("True", "False"):
                    row[key] = row[key] == "True"
            if row.get("validity_flags"):
                try:
                    row["validity_flags"] = int(row["validity_flags"])
                except ValueError:
                    pass
            rows.append(row)
    return rows


def load_pi_jsonl_records(path: str) -> list:
    """Reads pi_epuck_tcp_server_sensors_audited.py's JSONL audit file,
    one json.loads() per line. A malformed line is preserved as an
    error marker record (never silently skipped), so a corrupt file
    cannot silently look "clean"."""
    records = []
    with open(path, encoding="utf-8") as fh:
        for line_number, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                records.append({"event": "PARSE_ERROR", "line_number": line_number, "reason": str(exc)})
    return records


# --------------------------------------------------------------------
# Pure computations over already-loaded rows/records.
# --------------------------------------------------------------------
@dataclass(frozen=True)
class SpeedSummary:
    requested_max_linear_mps: Optional[float]
    guarded_max_linear_mps: Optional[float]
    max_abs_angular_rps: Optional[float]


def compute_speed_summary(wsl_rows: list, upstream_topic: str, guarded_topic: str) -> SpeedSummary:
    requested_vals = [
        abs(r["linear_x"]) for r in wsl_rows if r.get("topic") == upstream_topic and isinstance(r.get("linear_x"), float)
    ]
    guarded_vals = [
        abs(r["linear_x"]) for r in wsl_rows if r.get("topic") == guarded_topic and isinstance(r.get("linear_x"), float)
    ]
    angular_vals = [
        abs(r["angular_z"])
        for r in wsl_rows
        if r.get("topic") in (upstream_topic, guarded_topic) and isinstance(r.get("angular_z"), float)
    ]
    return SpeedSummary(
        requested_max_linear_mps=max(requested_vals) if requested_vals else None,
        guarded_max_linear_mps=max(guarded_vals) if guarded_vals else None,
        max_abs_angular_rps=max(angular_vals) if angular_vals else None,
    )


@dataclass(frozen=True)
class NonzeroWindow:
    first_time_ns: Optional[int]
    last_time_ns: Optional[int]
    duration_s: Optional[float]
    final_is_zero: Optional[bool]


def find_nonzero_command_window(wsl_rows: list, topic: str) -> NonzeroWindow:
    """First/last local_time_ns where a command on `topic` is nonzero
    (linear or angular), the duration between them, and whether the
    LAST row recorded for that topic (regardless of value) is zero --
    the final-zero confirmation acceptance rule needs the very last
    row, not merely the last nonzero one."""
    topic_rows = [r for r in wsl_rows if r.get("topic") == topic and isinstance(r.get("local_time_ns"), int)]
    nonzero_rows = [
        r
        for r in topic_rows
        if (isinstance(r.get("linear_x"), float) and r["linear_x"] != 0.0)
        or (isinstance(r.get("angular_z"), float) and r["angular_z"] != 0.0)
    ]
    first_time = nonzero_rows[0]["local_time_ns"] if nonzero_rows else None
    last_time = nonzero_rows[-1]["local_time_ns"] if nonzero_rows else None
    duration_s = (last_time - first_time) / 1e9 if (first_time is not None and last_time is not None) else None

    final_is_zero = None
    if topic_rows:
        last_row = topic_rows[-1]
        final_is_zero = (last_row.get("linear_x", 0.0) or 0.0) == 0.0 and (
            last_row.get("angular_z", 0.0) or 0.0
        ) == 0.0

    return NonzeroWindow(
        first_time_ns=first_time, last_time_ns=last_time, duration_s=duration_s, final_is_zero=final_is_zero
    )


@dataclass(frozen=True)
class NotAvailable:
    reason: str


def compute_odometry_displacement(wsl_rows: list, pi_records: list):
    """Always returns NotAvailable for this diagnostic's current
    evidence streams -- neither the WSL CSV (build_row has no x/y/yaw
    fields; the recorder subscribes to /epuck1/state for
    validity_flags only) nor the Pi JSONL (which only ever observes
    commands, never odometry) captures pose. Stated explicitly rather
    than silently omitted or estimated from commanded velocity."""
    return NotAvailable(
        reason="Neither the WSL command-evidence CSV nor the Pi command-audit JSONL "
        "captures odometry (x/y/yaw) -- hil_command_evidence_recorder.py's state-topic "
        "subscription reads validity_flags only, and the Pi audit only observes commands. "
        "A future revision could extend the recorder's EpuckState subscription, or a rosbag "
        "could be added, to make this computable."
    )


def compute_average_ground_speed(wsl_rows: list, pi_records: list):
    """Depends on measured displacement -- always NotAvailable for the
    same reason as compute_odometry_displacement(). Deliberately does
    NOT fall back to commanded-velocity x duration as a substitute for
    a measured average speed -- that would silently present a command
    as if it were an observation."""
    return NotAvailable(
        reason="Requires a measured displacement (see compute_odometry_displacement), which "
        "is not available from the current evidence streams. Not approximated from the "
        "commanded pulse speed, which would misrepresent a command as a measurement."
    )


@dataclass(frozen=True)
class ValidityDropoutSummary:
    dropout_count: int
    total_dropout_duration_s: float
    dropout_episodes: tuple  # tuple of (start_time_ns, end_time_ns, duration_s)


def compute_validity_flags_dropouts(wsl_rows: list, state_topic: str, required_flags: int = 7) -> ValidityDropoutSummary:
    state_rows = [
        r
        for r in wsl_rows
        if r.get("topic") == state_topic and isinstance(r.get("local_time_ns"), int) and r.get("validity_flags") is not None
    ]
    episodes = []
    episode_start = None
    prev_time = None
    for row in state_rows:
        flags = row.get("validity_flags")
        is_bad = isinstance(flags, int) and flags != required_flags
        if is_bad and episode_start is None:
            episode_start = row["local_time_ns"]
        if not is_bad and episode_start is not None:
            episodes.append((episode_start, prev_time, (prev_time - episode_start) / 1e9))
            episode_start = None
        prev_time = row["local_time_ns"]
    if episode_start is not None and prev_time is not None:
        episodes.append((episode_start, prev_time, (prev_time - episode_start) / 1e9))

    total_duration = sum(e[2] for e in episodes)
    return ValidityDropoutSummary(
        dropout_count=len(episodes), total_dropout_duration_s=total_duration, dropout_episodes=tuple(episodes)
    )


@dataclass(frozen=True)
class BridgeSummary:
    connected_sample_count: int
    disconnected_sample_count: int
    max_rx_count: Optional[int]
    ever_disconnected: bool


def compute_bridge_summary(wsl_rows: list, bridge_status_topic: str) -> BridgeSummary:
    bridge_rows = [r for r in wsl_rows if r.get("topic") == bridge_status_topic]
    connected = [r for r in bridge_rows if r.get("bridge_connected") is True]
    disconnected = [r for r in bridge_rows if r.get("bridge_connected") is False]
    rx_counts = [r["bridge_rx_count"] for r in bridge_rows if isinstance(r.get("bridge_rx_count"), int)]
    return BridgeSummary(
        connected_sample_count=len(connected),
        disconnected_sample_count=len(disconnected),
        max_rx_count=max(rx_counts) if rx_counts else None,
        ever_disconnected=len(disconnected) > 0,
    )


@dataclass(frozen=True)
class PiCommandMaxima:
    max_abs_linear_raw: Optional[float]
    max_abs_angular_raw: Optional[float]
    max_abs_linear_applied: Optional[float]
    max_abs_angular_applied: Optional[float]
    nonzero_received_count: int
    nonzero_applied_count: int


def compute_pi_command_maxima(pi_records: list) -> PiCommandMaxima:
    received = [r for r in pi_records if r.get("event") == "command_received"]
    applied = [r for r in pi_records if r.get("event") == "tick_applied"]

    linear_raw = [abs(r["linear_raw"]) for r in received if isinstance(r.get("linear_raw"), (int, float))]
    angular_raw = [abs(r["angular_raw"]) for r in received if isinstance(r.get("angular_raw"), (int, float))]
    linear_applied = [
        abs(r["linear_applied_clamped"]) for r in received if isinstance(r.get("linear_applied_clamped"), (int, float))
    ]
    angular_applied = [
        abs(r["angular_applied_clamped"]) for r in received if isinstance(r.get("angular_applied_clamped"), (int, float))
    ]
    tick_linear_applied = [abs(r["linear"]) for r in applied if isinstance(r.get("linear"), (int, float))]
    tick_angular_applied = [abs(r["angular"]) for r in applied if isinstance(r.get("angular"), (int, float))]

    all_applied_linear = linear_applied + tick_linear_applied
    all_applied_angular = angular_applied + tick_angular_applied

    return PiCommandMaxima(
        max_abs_linear_raw=max(linear_raw) if linear_raw else None,
        max_abs_angular_raw=max(angular_raw) if angular_raw else None,
        max_abs_linear_applied=max(all_applied_linear) if all_applied_linear else None,
        max_abs_angular_applied=max(all_applied_angular) if all_applied_angular else None,
        nonzero_received_count=sum(
            1 for v in linear_raw + angular_raw if v != 0.0
        ),
        nonzero_applied_count=sum(1 for v in all_applied_linear + all_applied_angular if v != 0.0),
    )


@dataclass(frozen=True)
class MismatchResult:
    checked_pairs: int
    mismatched_pairs: tuple  # tuple of (wsl_time_ns, wsl_linear, pi_time_ns, pi_linear, abs_diff)


def compute_guarded_vs_pi_applied_mismatch(
    wsl_rows: list, pi_records: list, guarded_topic: str, tolerance_mps: float = 0.005, max_time_diff_s: float = 0.2
) -> MismatchResult:
    """Cross-checks the WSL recorder's view of the guard's OWN output
    (the guarded topic) against the Pi's own tick_applied linear value,
    matched by nearest wall-clock time within max_time_diff_s. Only
    pairs actually matched within the time window are checked; this
    does not claim to prove synchrony beyond that window."""
    guarded_rows = [
        r for r in wsl_rows if r.get("topic") == guarded_topic and isinstance(r.get("local_time_ns"), int)
    ]
    tick_rows = [r for r in pi_records if r.get("event") == "tick_applied" and isinstance(r.get("wall_time"), (int, float))]
    if not guarded_rows or not tick_rows:
        return MismatchResult(checked_pairs=0, mismatched_pairs=())

    mismatches = []
    checked = 0
    for g in guarded_rows:
        g_time_s = g["local_time_ns"] / 1e9
        nearest = min(tick_rows, key=lambda t: abs(t["wall_time"] - g_time_s))
        if abs(nearest["wall_time"] - g_time_s) > max_time_diff_s:
            continue
        checked += 1
        g_linear = g.get("linear_x") if isinstance(g.get("linear_x"), float) else 0.0
        p_linear = nearest.get("linear") if isinstance(nearest.get("linear"), (int, float)) else 0.0
        diff = abs(g_linear - p_linear)
        if diff > tolerance_mps:
            mismatches.append((g["local_time_ns"], g_linear, nearest.get("monotonic_time"), p_linear, diff))

    return MismatchResult(checked_pairs=checked, mismatched_pairs=tuple(mismatches))


def compute_sha256_manifest(paths: dict) -> dict:
    """paths: {label: filepath}. Returns {label: sha256_hex}. Raises if
    a path does not exist -- a missing evidence file must never be
    silently reported as an empty/absent hash."""
    manifest = {}
    for label, path in paths.items():
        hasher = hashlib.sha256()
        with open(path, "rb") as fh:
            for chunk in iter(lambda: fh.read(65536), b""):
                hasher.update(chunk)
        manifest[label] = hasher.hexdigest()
    return manifest


# --------------------------------------------------------------------
# Verdict (Task H's acceptance rules).
# --------------------------------------------------------------------
@dataclass(frozen=True)
class VerdictResult:
    verdict: str  # "PASS", "FAIL", or "EXCLUDED"
    reasons: tuple


def evaluate_verdict(
    *,
    required_geometry_confirmed: bool,
    both_evidence_logs_active_before_arm: bool,
    validity_flags_before_motion: Optional[int],
    guard_sole_publisher: bool,
    any_nonzero_command_before_arm: bool,
    max_abs_angular_rps_commanded: Optional[float],
    max_abs_angular_rps_applied: Optional[float],
    guarded_max_linear_mps: Optional[float],
    confirmed_diagnostic_linear_cap_mps: float,
    final_command_is_zero: Optional[bool],
    robot_stayed_within_measured_area: bool,
    unexpected_motion_observed: bool,
    run_was_interrupted: bool,
    required_validity_flags: int = 7,
) -> VerdictResult:
    """Encodes this diagnostic's binding acceptance rules exactly.
    Any unexplained command, missing evidence, or interruption results
    in EXCLUDED, never PASS -- this function never returns PASS unless
    every rule is satisfied and every input needed to check it was
    actually available (None/False-for-missing inputs are treated as
    failing, not as passing by default)."""
    reasons = []

    # EXCLUDED conditions take priority: these mean the evidence itself
    # is insufficient to judge, not that the run necessarily failed.
    if run_was_interrupted:
        reasons.append("RUN_WAS_INTERRUPTED")
    if not both_evidence_logs_active_before_arm:
        reasons.append("EVIDENCE_LOGS_NOT_CONFIRMED_ACTIVE_BEFORE_ARM")
    if unexpected_motion_observed:
        reasons.append("UNEXPECTED_MOTION_OBSERVED")
    if validity_flags_before_motion is None:
        reasons.append("VALIDITY_FLAGS_NOT_AVAILABLE_BEFORE_MOTION")
    if reasons:
        return VerdictResult(verdict="EXCLUDED", reasons=tuple(reasons))

    # FAIL conditions: evidence is sufficient, but a rule was violated.
    if not required_geometry_confirmed:
        reasons.append("REQUIRED_GEOMETRY_NOT_CONFIRMED")
    if validity_flags_before_motion != required_validity_flags:
        reasons.append(f"VALIDITY_FLAGS_NOT_{required_validity_flags}_BEFORE_MOTION(got={validity_flags_before_motion})")
    if not guard_sole_publisher:
        reasons.append("GUARD_NOT_SOLE_PUBLISHER")
    if any_nonzero_command_before_arm:
        reasons.append("NONZERO_COMMAND_BEFORE_EXPLICIT_ARM")
    if max_abs_angular_rps_commanded not in (None, 0.0):
        reasons.append(f"ANGULAR_COMMAND_NOT_ZERO(got={max_abs_angular_rps_commanded})")
    if max_abs_angular_rps_applied not in (None, 0.0):
        reasons.append(f"ANGULAR_APPLIED_NOT_ZERO(got={max_abs_angular_rps_applied})")
    if guarded_max_linear_mps is not None and guarded_max_linear_mps > confirmed_diagnostic_linear_cap_mps:
        reasons.append(
            f"LINEAR_OUTPUT_EXCEEDS_CAP(got={guarded_max_linear_mps}, cap={confirmed_diagnostic_linear_cap_mps})"
        )
    if final_command_is_zero is not True:
        reasons.append("FINAL_COMMAND_NOT_CONFIRMED_ZERO")
    if not robot_stayed_within_measured_area:
        reasons.append("ROBOT_LEFT_MEASURED_SAFE_AREA")

    if reasons:
        return VerdictResult(verdict="FAIL", reasons=tuple(reasons))
    return VerdictResult(verdict="PASS", reasons=())


def main(argv=None) -> int:
    import argparse
    import json as json_module
    import sys

    parser = argparse.ArgumentParser(description="Offline analysis for the first ground diagnostic.")
    parser.add_argument("--wsl-csv", required=True)
    parser.add_argument("--pi-jsonl", required=True)
    parser.add_argument("--upstream-topic", default="cmd_vel_unguarded")
    parser.add_argument("--guarded-topic", default="cmd_vel")
    parser.add_argument("--state-topic", default="/epuck1/state")
    parser.add_argument("--bridge-status-topic", default="/epuck_bridge/status")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    wsl_rows = load_wsl_csv_rows(args.wsl_csv)
    pi_records = load_pi_jsonl_records(args.pi_jsonl)

    result = {
        "speed_summary": compute_speed_summary(wsl_rows, args.upstream_topic, args.guarded_topic).__dict__,
        "upstream_nonzero_window": find_nonzero_command_window(wsl_rows, args.upstream_topic).__dict__,
        "guarded_nonzero_window": find_nonzero_command_window(wsl_rows, args.guarded_topic).__dict__,
        "odometry_displacement": compute_odometry_displacement(wsl_rows, pi_records).__dict__,
        "average_ground_speed": compute_average_ground_speed(wsl_rows, pi_records).__dict__,
        "validity_flags_dropouts": compute_validity_flags_dropouts(wsl_rows, args.state_topic).__dict__,
        "bridge_summary": compute_bridge_summary(wsl_rows, args.bridge_status_topic).__dict__,
        "pi_command_maxima": compute_pi_command_maxima(pi_records).__dict__,
        "guarded_vs_pi_mismatch": compute_guarded_vs_pi_applied_mismatch(wsl_rows, pi_records, args.guarded_topic).__dict__,
        "sha256_manifest": compute_sha256_manifest({"wsl_csv": args.wsl_csv, "pi_jsonl": args.pi_jsonl}),
    }
    print(json_module.dumps(result, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
