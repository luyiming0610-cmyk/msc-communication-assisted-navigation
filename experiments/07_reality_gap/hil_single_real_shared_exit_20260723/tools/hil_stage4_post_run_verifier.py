#!/usr/bin/env python3
"""Stage 4 binding post-run verifier. Pure, offline, consumes only
EXPLICIT file paths passed as arguments -- never scans a directory for
"whatever is there." Fails closed: any missing/unparseable required
input is INVALID_EVIDENCE, never treated as zero/false/success.

Two modes:
  --mode rehearsal  Software-only claim (matches the hardware-free live
                     ROS-graph rehearsal). Never usable as physical PASS.
  --mode physical    Requires explicit operator-measurement fields in
                     addition to every rehearsal-mode check. This is the
                     ONLY mode that may ever be cited as a physical
                     result, and only when every physical threshold below
                     is met.

Classification is always exactly one of:
  PASS | FAIL_VALID_EVIDENCE | INVALID_EVIDENCE

FAIL_VALID_EVIDENCE means the evidence itself is trustworthy but the
run did not meet the PASS bar (a real behavioural failure, never hidden
or silently reclassified). INVALID_EVIDENCE means the evidence cannot
be trusted at all (missing file, hash mismatch, unparseable record,
inconsistent ordering) -- distinct from FAIL_VALID_EVIDENCE, and never
conflated with it.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# Frozen thresholds -- must stay identical to hil_stage4_motion_supervisor.py's
# own frozen constants; duplicated here deliberately (this verifier must be
# able to check the supervisor's OWN committed values independently, not
# merely trust whatever the supervisor claims about itself).
INTERNAL_ACTIVE_CUTOFF_S = 6.50
HARD_MAX_NONZERO_DURATION_S = 6.67
ZERO_TOLERANCE = 1e-6
MAX_LINEAR_COMMAND_MPS = 0.015
COMMAND_LIMIT_TOLERANCE = 1e-6

# Physical-mode-only frozen thresholds (design review revision 2/3).
NOMINAL_FORWARD_DISPLACEMENT_M = 0.10
MIN_MANUAL_FORWARD_DISPLACEMENT_M = 0.05
HARD_MAX_FORWARD_DISPLACEMENT_M = 0.15
MIN_BOUNDARY_CLEARANCE_M = 0.10

CLASSIFICATIONS = ("PASS", "FAIL_VALID_EVIDENCE", "INVALID_EVIDENCE")


@dataclass
class VerifierResult:
    classification: str
    reasons: list = field(default_factory=list)
    checks: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {"classification": self.classification, "reasons": self.reasons, "checks": self.checks}


def _sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def load_supervisor_evidence(path: Path) -> Optional[list]:
    """Returns a list of parsed JSON records, or None if the file is
    missing/unreadable/contains any unparseable line -- never a partial
    list silently missing the bad lines."""
    if not path.is_file():
        return None
    records = []
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            records.append(json.loads(line))
    except (OSError, ValueError):
        return None
    return records


def verify_evidence_schema_and_ordering(records: list) -> tuple[bool, list]:
    """Structural/ordering checks independent of behavioural outcome."""
    reasons = []
    required_fields = {"monotonic_time_s", "ros_time_s", "state", "event", "reason", "run_id", "goal_id"}
    for i, r in enumerate(records):
        missing = required_fields - set(r.keys())
        if missing:
            reasons.append(f"RECORD_{i}_MISSING_FIELDS:{sorted(missing)}")
    times = [r["monotonic_time_s"] for r in records if "monotonic_time_s" in r]
    if times != sorted(times):
        reasons.append("RECORDS_NOT_MONOTONICALLY_ORDERED")
    return (len(reasons) == 0, reasons)


def _events(records: list) -> list:
    return [r.get("event") for r in records]


def _first_index(records: list, event: str) -> Optional[int]:
    for i, r in enumerate(records):
        if r.get("event") == event:
            return i
    return None


def _count(records: list, event: str) -> int:
    return sum(1 for r in records if r.get("event") == event)


def verify_causal_chain(records: list) -> tuple[bool, list]:
    reasons = []

    if _count(records, "VIRTUAL_SCOUT_RELEASED") != 1:
        reasons.append(f"EXPECTED_EXACTLY_ONE_VIRTUAL_SCOUT_RELEASED_GOT_{_count(records, 'VIRTUAL_SCOUT_RELEASED')}")

    adoption_count = _count(records, "ADOPTION_CONFIRMED")
    if adoption_count != 1:
        reasons.append(f"EXPECTED_EXACTLY_ONE_ADOPTION_CONFIRMED_GOT_{adoption_count}")

    arm_count = _count(records, "ARM_PUBLISHED")
    if arm_count != 1:
        reasons.append(f"EXPECTED_EXACTLY_ONE_ARM_PUBLISHED_GOT_{arm_count}")

    active_count = _count(records, "ACTIVE_OPENED")
    if active_count != 1:
        reasons.append(f"EXPECTED_EXACTLY_ONE_ACTIVE_WINDOW_GOT_{active_count}")

    released_idx = _first_index(records, "VIRTUAL_SCOUT_RELEASED")
    adopted_idx = _first_index(records, "ADOPTION_CONFIRMED")
    arm_idx = _first_index(records, "ARM_PUBLISHED")
    active_idx = _first_index(records, "ACTIVE_OPENED")

    if released_idx is not None and adopted_idx is not None and not (released_idx < adopted_idx):
        reasons.append("ADOPTION_NOT_AFTER_RELEASE")
    if adopted_idx is not None and arm_idx is not None and not (adopted_idx < arm_idx):
        reasons.append("ARM_NOT_AFTER_ADOPTION")
    if arm_idx is not None and active_idx is not None and not (arm_idx <= active_idx):
        reasons.append("ACTIVE_NOT_AFTER_ARM")

    # No pre-adoption motion: no RAW_TWIST_RECEIVED with a nonzero linear
    # component may appear at or before the adoption index.
    if adopted_idx is not None:
        for i, r in enumerate(records[:adopted_idx]):
            if r.get("event") == "RAW_TWIST_RECEIVED":
                raw = r.get("raw") or {}
                if abs(raw.get("linear_x", 0.0)) > ZERO_TOLERANCE:
                    reasons.append(f"PRE_ADOPTION_MOTION_AT_RECORD_{i}")

    # Prohibited raw components at any point where the supervisor treated
    # the sample as the arming command or during ACTIVE.
    for i, r in enumerate(records):
        if r.get("event") == "RAW_TWIST_RECEIVED":
            raw = r.get("raw") or {}
            for comp in ("angular_x", "angular_y", "angular_z", "linear_y", "linear_z"):
                if abs(raw.get(comp, 0.0)) > ZERO_TOLERANCE and r.get("state") == "ACTIVE":
                    reasons.append(f"PROHIBITED_RAW_COMPONENT_DURING_ACTIVE_AT_RECORD_{i}:{comp}")

    return (len(reasons) == 0, reasons)


def verify_timing(records: list) -> tuple[bool, list, dict]:
    reasons = []
    checks = {}

    active_idx = _first_index(records, "ACTIVE_OPENED")
    zero_burst_idx = _first_index(records, "ZERO_BURST_OPENED")

    if active_idx is not None and zero_burst_idx is not None:
        active_t = records[active_idx]["monotonic_time_s"]
        zero_t = records[zero_burst_idx]["monotonic_time_s"]
        duration_s = zero_t - active_t
        checks["active_duration_s"] = duration_s
        if duration_s > INTERNAL_ACTIVE_CUTOFF_S + 0.5:
            # 0.5s scheduling/transport margin, matching the design
            # review's own bounded-jitter allowance -- never silently
            # widened beyond that.
            reasons.append(f"ACTIVE_DURATION_EXCEEDS_INTERNAL_CUTOFF_MARGIN:{duration_s:.3f}s")
        if duration_s > HARD_MAX_NONZERO_DURATION_S:
            reasons.append(f"ACTIVE_DURATION_EXCEEDS_HARD_VERIFIER_MAXIMUM:{duration_s:.3f}s")
    else:
        checks["active_duration_s"] = None

    return (len(reasons) == 0, reasons, checks)


def verify_disarm_and_terminal_state(records: list) -> tuple[bool, list, str]:
    reasons = []
    if not records:
        return False, ["NO_RECORDS"], ""

    terminal_state = records[-1].get("state")
    if terminal_state not in ("COMPLETE", "FAILED"):
        reasons.append(f"NON_TERMINAL_FINAL_STATE:{terminal_state}")

    if terminal_state == "COMPLETE":
        if _count(records, "DISARM_PUBLISHED") != 1:
            reasons.append("COMPLETE_RUN_MUST_HAVE_EXACTLY_ONE_DISARM_PUBLISHED")
        if _count(records, "ZERO_PUBLISHED") < 1:
            reasons.append("COMPLETE_RUN_MUST_HAVE_AT_LEAST_ONE_ZERO_PUBLISHED")
        complete_indices = [i for i, r in enumerate(records) if r.get("event") == "LATCHED_COMPLETE"]
        if len(complete_indices) != 1:
            reasons.append(f"EXPECTED_EXACTLY_ONE_LATCHED_COMPLETE_GOT_{len(complete_indices)}")
        else:
            complete_idx = complete_indices[0]
            terminal_sequence = _events(records[max(0, complete_idx - 3):complete_idx + 1])
            expected = ["ZERO_BURST_OPENED", "ZERO_PUBLISHED", "DISARM_PUBLISHED", "LATCHED_COMPLETE"]
            if terminal_sequence != expected:
                reasons.append(f"UNEXPECTED_TERMINAL_SEQUENCE:{terminal_sequence}")

            # The supervisor intentionally remains alive until explicit cleanup.
            # Controller traffic received after it has latched COMPLETE is recorded
            # as a safe ignored tail.  It is not part of the terminal transition,
            # but no other post-terminal record is allowed.
            for i, record in enumerate(records[complete_idx + 1:], start=complete_idx + 1):
                if not (
                    record.get("state") == "COMPLETE"
                    and record.get("event") == "RAW_TWIST_IGNORED"
                    and record.get("reason") == "state=COMPLETE"
                    and record.get("raw") is None
                ):
                    reasons.append(f"UNSAFE_OR_UNEXPECTED_POST_TERMINAL_RECORD:{i}:{record.get('event')}")

    return (len(reasons) == 0, reasons, terminal_state or "")


def verify_hashes(hash_manifest_path: Path, evidence_dir: Path) -> tuple[bool, list]:
    """hash_manifest_path: a SHA256SUMS.txt-style file, two whitespace-
    separated columns per line: sha256  filename (filename relative to
    evidence_dir). Fails closed on any missing file or mismatch."""
    if not hash_manifest_path.is_file():
        return False, [f"HASH_MANIFEST_MISSING:{hash_manifest_path}"]
    reasons = []
    for line in hash_manifest_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            reasons.append(f"MALFORMED_HASH_LINE:{line!r}")
            continue
        expected_hash, filename = parts
        target = evidence_dir / filename
        if not target.is_file():
            reasons.append(f"HASHED_FILE_MISSING:{filename}")
            continue
        actual_hash = _sha256_of(target)
        if actual_hash != expected_hash:
            reasons.append(f"HASH_MISMATCH:{filename}")
    return (len(reasons) == 0, reasons)


def verify_pid_manifest_and_cleanup(pid_manifest_path: Path) -> tuple[bool, list]:
    if not pid_manifest_path.is_file():
        return False, [f"PID_MANIFEST_MISSING:{pid_manifest_path}"]
    try:
        manifest = json.loads(pid_manifest_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False, ["PID_MANIFEST_UNPARSEABLE"]
    reasons = []
    processes = manifest.get("processes", {})
    if "recorder" not in processes:
        reasons.append("PID_MANIFEST_MISSING_RECORDER_ENTRY")
    for name, info in processes.items():
        if not isinstance(info, dict) or "pid" not in info:
            reasons.append(f"PID_MANIFEST_ENTRY_MISSING_PID:{name}")
    return (len(reasons) == 0, reasons)


def verify_residual_process_result(residual_check_path: Path) -> tuple[bool, list]:
    """residual_check_path: a small JSON file the orchestrator writes
    after cleanup, e.g. {"residual_process_check": "CLEAN"}. Missing or
    any value other than exactly "CLEAN" is a failure -- never assumed
    clean by absence."""
    if not residual_check_path.is_file():
        return False, [f"RESIDUAL_PROCESS_CHECK_FILE_MISSING:{residual_check_path}"]
    try:
        payload = json.loads(residual_check_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False, ["RESIDUAL_PROCESS_CHECK_FILE_UNPARSEABLE"]
    value = payload.get("residual_process_check")
    if value != "CLEAN":
        return False, [f"RESIDUAL_PROCESS_CHECK_NOT_CLEAN:{value!r}"]
    return True, []


def _require_nonempty_file(path: Optional[Path], label: str) -> tuple[bool, list]:
    if path is None:
        return False, [f"{label}_PATH_NOT_PROVIDED"]
    if not path.is_file():
        return False, [f"{label}_MISSING:{path}"]
    if path.stat().st_size == 0:
        return False, [f"{label}_EMPTY:{path}"]
    return True, []


def _require_parseable_json(path: Optional[Path], label: str, required_keys: tuple = ()) -> tuple[bool, list, Optional[dict]]:
    ok, reasons = _require_nonempty_file(path, label)
    if not ok:
        return False, reasons, None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False, [f"{label}_UNPARSEABLE:{path}"], None
    missing = [k for k in required_keys if k not in payload]
    if missing:
        return False, [f"{label}_MISSING_KEYS:{missing}"], payload
    return True, [], payload


def _require_parseable_jsonl(path: Optional[Path], label: str) -> tuple[bool, list]:
    ok, reasons = _require_nonempty_file(path, label)
    if not ok:
        return False, reasons
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                json.loads(line)
    except (OSError, ValueError):
        return False, [f"{label}_UNPARSEABLE:{path}"]
    return True, []


def _finite_float(value, *, label: str, row_number: int, reasons: list) -> Optional[float]:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        reasons.append(f"{label}_ROW_{row_number}_NON_NUMERIC:{value!r}")
        return None
    if not math.isfinite(parsed):
        reasons.append(f"{label}_ROW_{row_number}_NON_FINITE:{value!r}")
        return None
    return parsed


def verify_wsl_command_evidence(path: Path) -> tuple[bool, list, list, dict]:
    """Validate the recorder CSV directly.

    Returns (structurally_valid, invalid_reasons, behavioural_reasons,
    metrics).  Structural failures make evidence unusable; bounded-command,
    connectivity, or safe-tail failures are real behavioural failures.
    """
    ok, invalid_reasons = _require_nonempty_file(path, "WSL_COMMAND_EVIDENCE")
    if not ok:
        return False, invalid_reasons, [], {}

    required_columns = {
        "local_time_ns", "local_monotonic_ns", "topic", "linear_x", "angular_z",
        "arm_state", "bridge_connected", "bridge_rx_count",
    }
    try:
        with path.open(encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            fieldnames = set(reader.fieldnames or [])
            missing = sorted(required_columns - fieldnames)
            if missing:
                return False, [f"WSL_COMMAND_EVIDENCE_MISSING_COLUMNS:{missing}"], [], {}
            rows = list(reader)
    except (OSError, csv.Error, UnicodeError) as exc:
        return False, [f"WSL_COMMAND_EVIDENCE_UNPARSEABLE:{type(exc).__name__}"], [], {}
    if not rows:
        return False, ["WSL_COMMAND_EVIDENCE_NO_DATA_ROWS"], [], {}

    monotonic_values = []
    command_rows = {"cmd_vel": [], "cmd_vel_unguarded": []}
    arm_states = []
    bridge_rows = []
    for row_number, row in enumerate(rows, start=2):
        monotonic = _finite_float(
            row.get("local_monotonic_ns"), label="WSL_LOCAL_MONOTONIC_NS",
            row_number=row_number, reasons=invalid_reasons,
        )
        if monotonic is not None:
            monotonic_values.append(monotonic)
        topic = row.get("topic")
        if topic in command_rows:
            linear = _finite_float(
                row.get("linear_x"), label=f"WSL_{topic}_LINEAR_X",
                row_number=row_number, reasons=invalid_reasons,
            )
            angular = _finite_float(
                row.get("angular_z"), label=f"WSL_{topic}_ANGULAR_Z",
                row_number=row_number, reasons=invalid_reasons,
            )
            if linear is not None and angular is not None:
                command_rows[topic].append((linear, angular))
        elif topic == "/hil_guard/arm":
            arm_states.append(row.get("arm_state"))
        elif topic == "/epuck_bridge/status":
            rx_count = _finite_float(
                row.get("bridge_rx_count"), label="WSL_BRIDGE_RX_COUNT",
                row_number=row_number, reasons=invalid_reasons,
            )
            if rx_count is not None:
                bridge_rows.append((row.get("bridge_connected"), rx_count))

    if monotonic_values != sorted(monotonic_values):
        invalid_reasons.append("WSL_COMMAND_EVIDENCE_NOT_MONOTONICALLY_ORDERED")
    if invalid_reasons:
        return False, invalid_reasons, [], {}

    behavioural_reasons = []
    metrics = {}
    for topic, values in command_rows.items():
        metric_prefix = "guarded" if topic == "cmd_vel" else "unguarded"
        if not values:
            behavioural_reasons.append(f"WSL_MISSING_{topic.upper()}_ROWS")
            continue
        nonzero_count = sum(abs(linear) > ZERO_TOLERANCE or abs(angular) > ZERO_TOLERANCE for linear, angular in values)
        max_linear = max(abs(linear) for linear, _ in values)
        max_angular = max(abs(angular) for _, angular in values)
        metrics[f"wsl_{metric_prefix}_nonzero_count"] = nonzero_count
        metrics[f"wsl_{metric_prefix}_max_abs_linear_mps"] = max_linear
        metrics[f"wsl_{metric_prefix}_max_abs_angular_rps"] = max_angular
        if nonzero_count == 0:
            behavioural_reasons.append(f"WSL_{topic.upper()}_HAS_NO_NONZERO_MOTION")
        if max_linear > MAX_LINEAR_COMMAND_MPS + COMMAND_LIMIT_TOLERANCE:
            behavioural_reasons.append(f"WSL_{topic.upper()}_LINEAR_LIMIT_EXCEEDED:{max_linear}")
        if max_angular > ZERO_TOLERANCE:
            behavioural_reasons.append(f"WSL_{topic.upper()}_NONZERO_ANGULAR:{max_angular}")
        if abs(values[-1][0]) > ZERO_TOLERANCE or abs(values[-1][1]) > ZERO_TOLERANCE:
            behavioural_reasons.append(f"WSL_{topic.upper()}_FINAL_COMMAND_NOT_ZERO:{values[-1]}")

    metrics["wsl_arm_states"] = arm_states
    if arm_states != ["True", "False"]:
        behavioural_reasons.append(f"WSL_UNEXPECTED_ARM_SEQUENCE:{arm_states}")
    if not bridge_rows:
        behavioural_reasons.append("WSL_BRIDGE_STATUS_ROWS_MISSING")
    else:
        metrics["wsl_bridge_status_count"] = len(bridge_rows)
        metrics["wsl_bridge_rx_first"] = bridge_rows[0][1]
        metrics["wsl_bridge_rx_last"] = bridge_rows[-1][1]
        if any(connected != "True" for connected, _ in bridge_rows):
            behavioural_reasons.append("WSL_BRIDGE_REPORTED_DISCONNECTED")
        rx_counts = [count for _, count in bridge_rows]
        if rx_counts != sorted(rx_counts):
            behavioural_reasons.append("WSL_BRIDGE_RX_COUNT_DECREASED")

    return True, [], behavioural_reasons, metrics


def verify_pi_command_audit(path: Path) -> tuple[bool, list, list, dict]:
    """Validate Pi command-audit JSONL without a legacy zero-only verdict."""
    ok, invalid_reasons = _require_nonempty_file(path, "PI_COMMAND_AUDIT")
    if not ok:
        return False, invalid_reasons, [], {}

    received = []
    applied = []
    events = []
    try:
        with path.open(encoding="utf-8") as handle:
            for row_number, line in enumerate(handle, start=1):
                if not line.strip():
                    continue
                record = json.loads(line)
                if not isinstance(record, dict) or not isinstance(record.get("event"), str):
                    invalid_reasons.append(f"PI_COMMAND_AUDIT_ROW_{row_number}_MISSING_EVENT")
                    continue
                event = record["event"]
                events.append(event)
                if event == "command_received":
                    linear = _finite_float(
                        record.get("linear_applied_clamped"), label="PI_RECEIVED_LINEAR",
                        row_number=row_number, reasons=invalid_reasons,
                    )
                    angular = _finite_float(
                        record.get("angular_applied_clamped"), label="PI_RECEIVED_ANGULAR",
                        row_number=row_number, reasons=invalid_reasons,
                    )
                    seq = record.get("seq")
                    if not isinstance(seq, int):
                        invalid_reasons.append(f"PI_COMMAND_AUDIT_ROW_{row_number}_INVALID_SEQUENCE:{seq!r}")
                    if linear is not None and angular is not None and isinstance(seq, int):
                        received.append((linear, angular, seq, bool(record.get("clamped"))))
                elif event == "tick_applied":
                    linear = _finite_float(
                        record.get("linear"), label="PI_APPLIED_LINEAR",
                        row_number=row_number, reasons=invalid_reasons,
                    )
                    angular = _finite_float(
                        record.get("angular"), label="PI_APPLIED_ANGULAR",
                        row_number=row_number, reasons=invalid_reasons,
                    )
                    if linear is not None and angular is not None:
                        applied.append((linear, angular, record.get("zero_reason")))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        invalid_reasons.append(f"PI_COMMAND_AUDIT_UNPARSEABLE:{type(exc).__name__}")
    if invalid_reasons:
        return False, invalid_reasons, [], {}

    behavioural_reasons = []
    metrics = {
        "pi_audit_record_count": len(events),
        "pi_command_received_count": len(received),
        "pi_tick_applied_count": len(applied),
    }
    if not received:
        behavioural_reasons.append("PI_COMMAND_RECEIVED_ROWS_MISSING")
    if not applied:
        behavioural_reasons.append("PI_TICK_APPLIED_ROWS_MISSING")

    for label, values in (("RECEIVED", received), ("APPLIED", applied)):
        if not values:
            continue
        nonzero_count = sum(abs(row[0]) > ZERO_TOLERANCE or abs(row[1]) > ZERO_TOLERANCE for row in values)
        max_linear = max(abs(row[0]) for row in values)
        max_angular = max(abs(row[1]) for row in values)
        metrics[f"pi_{label.lower()}_nonzero_count"] = nonzero_count
        metrics[f"pi_{label.lower()}_max_abs_linear_mps"] = max_linear
        metrics[f"pi_{label.lower()}_max_abs_angular_rps"] = max_angular
        if nonzero_count == 0:
            behavioural_reasons.append(f"PI_{label}_HAS_NO_NONZERO_MOTION")
        if max_linear > MAX_LINEAR_COMMAND_MPS + COMMAND_LIMIT_TOLERANCE:
            behavioural_reasons.append(f"PI_{label}_LINEAR_LIMIT_EXCEEDED:{max_linear}")
        if max_angular > ZERO_TOLERANCE:
            behavioural_reasons.append(f"PI_{label}_NONZERO_ANGULAR:{max_angular}")
        if abs(values[-1][0]) > ZERO_TOLERANCE or abs(values[-1][1]) > ZERO_TOLERANCE:
            behavioural_reasons.append(f"PI_{label}_FINAL_COMMAND_NOT_ZERO:{values[-1][:2]}")

    if received:
        sequences = [row[2] for row in received]
        if any(current <= previous for previous, current in zip(sequences, sequences[1:])):
            behavioural_reasons.append("PI_COMMAND_SEQUENCE_NOT_STRICTLY_INCREASING")
        if any(row[3] for row in received):
            behavioural_reasons.append("PI_COMMAND_WAS_CLAMPED")
    if events.count("socket_connected") < 1:
        behavioural_reasons.append("PI_SOCKET_CONNECTED_EVENT_MISSING")
    if events.count("socket_disconnected") < 1:
        behavioural_reasons.append("PI_SOCKET_DISCONNECTED_EVENT_MISSING")
    if applied and applied[-1][2] != "DISCONNECTED":
        behavioural_reasons.append(f"PI_FINAL_ZERO_REASON_NOT_DISCONNECTED:{applied[-1][2]!r}")

    return True, [], behavioural_reasons, metrics


PHYSICAL_MEASUREMENT_FIELDS = (
    "manual_forward_displacement_m", "corridor_crossed", "stop_line_crossed",
    "min_boundary_clearance_m", "unexpected_rotation", "unexpected_direction",
    "unexpected_sound", "unexpected_acceleration", "run_interrupted",
)


def verify_physical_measurement_schema(measurements) -> tuple[bool, list]:
    if not isinstance(measurements, dict):
        return False, ["PHYSICAL_MEASUREMENTS_ROOT_MUST_BE_OBJECT"]
    missing = [key for key in PHYSICAL_MEASUREMENT_FIELDS if key not in measurements]
    if missing:
        return False, [f"MISSING_PHYSICAL_MEASUREMENT_FIELDS:{missing}"]
    reasons = []
    for key in ("manual_forward_displacement_m", "min_boundary_clearance_m"):
        value = measurements[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            reasons.append(f"PHYSICAL_MEASUREMENT_MUST_BE_FINITE_NUMBER:{key}:{value!r}")
    for key in PHYSICAL_MEASUREMENT_FIELDS:
        if key in ("manual_forward_displacement_m", "min_boundary_clearance_m"):
            continue
        if not isinstance(measurements[key], bool):
            reasons.append(f"PHYSICAL_MEASUREMENT_MUST_BE_BOOLEAN:{key}:{measurements[key]!r}")
    return len(reasons) == 0, reasons


def verify_physical_measurements(measurements: dict) -> tuple[bool, list]:
    """measurements: explicit operator-supplied dict (never inferred).
    Required keys: manual_forward_displacement_m, corridor_crossed,
    stop_line_crossed, min_boundary_clearance_m, unexpected_rotation,
    unexpected_direction, unexpected_sound, unexpected_acceleration,
    run_interrupted. Missing key => INVALID_EVIDENCE-worthy failure,
    never defaulted to a passing value."""
    reasons = []
    displacement = measurements["manual_forward_displacement_m"]
    if not (MIN_MANUAL_FORWARD_DISPLACEMENT_M <= displacement <= HARD_MAX_FORWARD_DISPLACEMENT_M):
        reasons.append(
            f"MANUAL_FORWARD_DISPLACEMENT_OUT_OF_RANGE:{displacement} "
            f"(expected [{MIN_MANUAL_FORWARD_DISPLACEMENT_M},{HARD_MAX_FORWARD_DISPLACEMENT_M}])"
        )
    if measurements["corridor_crossed"] is not False:
        reasons.append("CORRIDOR_CROSSED_MUST_BE_FALSE")
    if measurements["stop_line_crossed"] is not False:
        reasons.append("STOP_LINE_CROSSED_MUST_BE_FALSE")
    if measurements["min_boundary_clearance_m"] <= MIN_BOUNDARY_CLEARANCE_M:
        reasons.append(f"BOUNDARY_CLEARANCE_TOO_SMALL:{measurements['min_boundary_clearance_m']}")
    for flag_key in ("unexpected_rotation", "unexpected_direction", "unexpected_sound", "unexpected_acceleration", "run_interrupted"):
        if measurements[flag_key] is not False:
            reasons.append(f"{flag_key.upper()}_MUST_BE_FALSE")

    return (len(reasons) == 0, reasons)


#: Physical-mode-only explicit inputs. Every one of these is REQUIRED
#: (not merely "checked if present") once mode="physical" -- missing any
#: of them, or a hash manifest that fails to verify them, is
#: INVALID_EVIDENCE. Rehearsal mode never requires or reads any of
#: these -- physical mode must never silently fall back to a rehearsal
#: default for any of them.
PHYSICAL_MODE_REQUIRED_PATH_ARGS = (
    "adoption_evidence_path",
    "wsl_command_evidence_path",
    "pi_command_audit_path",
    "source_identity_manifest_path",
    "launcher_status_path",
)


def run_verifier(
    *,
    supervisor_evidence_path: Path,
    mode: str,
    hash_manifest_path: Optional[Path] = None,
    evidence_dir: Optional[Path] = None,
    pid_manifest_path: Optional[Path] = None,
    residual_check_path: Optional[Path] = None,
    physical_measurements_path: Optional[Path] = None,
    adoption_evidence_path: Optional[Path] = None,
    wsl_command_evidence_path: Optional[Path] = None,
    pi_command_audit_path: Optional[Path] = None,
    source_identity_manifest_path: Optional[Path] = None,
    launcher_status_path: Optional[Path] = None,
) -> VerifierResult:
    if mode not in ("rehearsal", "physical"):
        return VerifierResult("INVALID_EVIDENCE", [f"UNKNOWN_MODE:{mode}"])

    records = load_supervisor_evidence(supervisor_evidence_path)
    if records is None:
        return VerifierResult("INVALID_EVIDENCE", [f"SUPERVISOR_EVIDENCE_MISSING_OR_UNPARSEABLE:{supervisor_evidence_path}"])
    if not records:
        return VerifierResult("INVALID_EVIDENCE", ["SUPERVISOR_EVIDENCE_EMPTY"])

    all_reasons = []
    checks = {}

    ok, reasons = verify_evidence_schema_and_ordering(records)
    checks["schema_and_ordering"] = ok
    if not ok:
        return VerifierResult("INVALID_EVIDENCE", reasons, checks)

    if hash_manifest_path is not None and evidence_dir is not None:
        ok, reasons = verify_hashes(hash_manifest_path, evidence_dir)
        checks["hashes"] = ok
        if not ok:
            return VerifierResult("INVALID_EVIDENCE", reasons, checks)

    if pid_manifest_path is not None:
        ok, reasons = verify_pid_manifest_and_cleanup(pid_manifest_path)
        checks["pid_manifest"] = ok
        if not ok:
            return VerifierResult("INVALID_EVIDENCE", reasons, checks)

    if residual_check_path is not None:
        ok, reasons = verify_residual_process_result(residual_check_path)
        checks["residual_process"] = ok
        if not ok:
            return VerifierResult("INVALID_EVIDENCE", reasons, checks)

    # From here on, evidence is trustworthy -- any failure below is a
    # genuine behavioural result (FAIL_VALID_EVIDENCE), not an evidence
    # problem.
    ok, reasons = verify_causal_chain(records)
    checks["causal_chain"] = ok
    all_reasons += reasons

    ok, reasons, timing_checks = verify_timing(records)
    checks["timing"] = ok
    checks.update(timing_checks)
    all_reasons += reasons

    ok, reasons, terminal_state = verify_disarm_and_terminal_state(records)
    checks["disarm_and_terminal_state"] = ok
    checks["terminal_state"] = terminal_state
    all_reasons += reasons

    if terminal_state != "COMPLETE":
        all_reasons.append(f"TERMINAL_STATE_NOT_COMPLETE:{terminal_state}")

    if mode == "physical":
        # Every physical-only input is REQUIRED -- collect ALL missing
        # ones before returning, so a physical run with several gaps
        # gets a complete report, not just the first missing path.
        provided = {
            "adoption_evidence_path": adoption_evidence_path,
            "wsl_command_evidence_path": wsl_command_evidence_path,
            "pi_command_audit_path": pi_command_audit_path,
            "source_identity_manifest_path": source_identity_manifest_path,
            "launcher_status_path": launcher_status_path,
        }
        invalid_reasons = []

        if pid_manifest_path is None:
            invalid_reasons.append("PHYSICAL_MODE_REQUIRES_PID_MANIFEST_PATH")
        if hash_manifest_path is None or evidence_dir is None:
            invalid_reasons.append("PHYSICAL_MODE_REQUIRES_HASH_MANIFEST_AND_EVIDENCE_DIR")
        if physical_measurements_path is None:
            invalid_reasons.append("PHYSICAL_MODE_REQUIRES_MEASUREMENTS_PATH")

        for arg_name in PHYSICAL_MODE_REQUIRED_PATH_ARGS:
            if provided[arg_name] is None:
                invalid_reasons.append(f"PHYSICAL_MODE_REQUIRES_{arg_name.upper()}")

        if invalid_reasons:
            return VerifierResult("INVALID_EVIDENCE", invalid_reasons, checks)

        ok, reasons = _require_nonempty_file(adoption_evidence_path, "ADOPTION_EVIDENCE")
        checks["adoption_evidence_present"] = ok
        invalid_reasons += reasons

        ok, reasons, command_reasons, command_metrics = verify_wsl_command_evidence(wsl_command_evidence_path)
        checks["wsl_command_evidence_valid"] = ok
        checks.update(command_metrics)
        invalid_reasons += reasons
        all_reasons += command_reasons

        ok, reasons, pi_reasons, pi_metrics = verify_pi_command_audit(pi_command_audit_path)
        checks["pi_command_audit_valid"] = ok
        checks.update(pi_metrics)
        invalid_reasons += reasons
        all_reasons += pi_reasons

        ok, reasons, source_identity = _require_parseable_json(
            source_identity_manifest_path, "SOURCE_IDENTITY_MANIFEST", required_keys=("overall_result",),
        )
        checks["source_identity_manifest_present"] = ok
        invalid_reasons += reasons

        ok, reasons, _launcher_status = _require_parseable_json(launcher_status_path, "LAUNCHER_STATUS")
        checks["launcher_status_present"] = ok
        invalid_reasons += reasons

        if invalid_reasons:
            return VerifierResult("INVALID_EVIDENCE", invalid_reasons, checks)

        if source_identity is not None and source_identity.get("overall_result") != "PASS":
            return VerifierResult(
                "INVALID_EVIDENCE",
                [f"SOURCE_IDENTITY_NOT_PASS:{source_identity.get('overall_result')!r}"],
                checks,
            )

        if not physical_measurements_path.is_file():
            return VerifierResult("INVALID_EVIDENCE", [f"PHYSICAL_MEASUREMENTS_FILE_MISSING:{physical_measurements_path}"], checks)
        try:
            measurements = json.loads(physical_measurements_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return VerifierResult("INVALID_EVIDENCE", ["PHYSICAL_MEASUREMENTS_FILE_UNPARSEABLE"], checks)
        ok, reasons = verify_physical_measurement_schema(measurements)
        checks["physical_measurements_schema"] = ok
        if not ok:
            return VerifierResult("INVALID_EVIDENCE", reasons, checks)
        ok, reasons = verify_physical_measurements(measurements)
        checks["physical_measurements"] = ok
        all_reasons += reasons

    if all_reasons:
        return VerifierResult("FAIL_VALID_EVIDENCE", all_reasons, checks)
    return VerifierResult("PASS", [], checks)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--supervisor-evidence-path", required=True, type=Path)
    parser.add_argument("--mode", required=True, choices=("rehearsal", "physical"))
    parser.add_argument("--hash-manifest-path", type=Path, default=None)
    parser.add_argument("--evidence-dir", type=Path, default=None)
    parser.add_argument("--pid-manifest-path", type=Path, default=None)
    parser.add_argument("--residual-check-path", type=Path, default=None)
    parser.add_argument("--physical-measurements-path", type=Path, default=None)
    parser.add_argument("--adoption-evidence-path", type=Path, default=None)
    parser.add_argument("--wsl-command-evidence-path", type=Path, default=None)
    parser.add_argument("--pi-command-audit-path", type=Path, default=None)
    parser.add_argument("--source-identity-manifest-path", type=Path, default=None)
    parser.add_argument("--launcher-status-path", type=Path, default=None)
    parser.add_argument("--report-path", type=Path, default=None)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    result = run_verifier(
        supervisor_evidence_path=args.supervisor_evidence_path,
        mode=args.mode,
        hash_manifest_path=args.hash_manifest_path,
        evidence_dir=args.evidence_dir,
        pid_manifest_path=args.pid_manifest_path,
        residual_check_path=args.residual_check_path,
        physical_measurements_path=args.physical_measurements_path,
        adoption_evidence_path=args.adoption_evidence_path,
        wsl_command_evidence_path=args.wsl_command_evidence_path,
        pi_command_audit_path=args.pi_command_audit_path,
        source_identity_manifest_path=args.source_identity_manifest_path,
        launcher_status_path=args.launcher_status_path,
    )

    report = result.as_dict()
    output = json.dumps(report, indent=2)
    print(output)
    if args.report_path is not None:
        args.report_path.write_text(output, encoding="utf-8")

    return 0 if result.classification == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
