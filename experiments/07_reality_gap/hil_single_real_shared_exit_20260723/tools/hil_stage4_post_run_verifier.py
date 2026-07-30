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
import hashlib
import json
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
        tail = _events(records)[-4:]
        if tail != ["ZERO_BURST_OPENED", "ZERO_PUBLISHED", "DISARM_PUBLISHED", "LATCHED_COMPLETE"]:
            reasons.append(f"UNEXPECTED_TERMINAL_TAIL_SEQUENCE:{tail}")

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


def verify_physical_measurements(measurements: dict) -> tuple[bool, list]:
    """measurements: explicit operator-supplied dict (never inferred).
    Required keys: manual_forward_displacement_m, corridor_crossed,
    stop_line_crossed, min_boundary_clearance_m, unexpected_rotation,
    unexpected_direction, unexpected_sound, unexpected_acceleration,
    run_interrupted. Missing key => INVALID_EVIDENCE-worthy failure,
    never defaulted to a passing value."""
    required_keys = (
        "manual_forward_displacement_m", "corridor_crossed", "stop_line_crossed",
        "min_boundary_clearance_m", "unexpected_rotation", "unexpected_direction",
        "unexpected_sound", "unexpected_acceleration", "run_interrupted",
    )
    reasons = []
    missing = [k for k in required_keys if k not in measurements]
    if missing:
        return False, [f"MISSING_PHYSICAL_MEASUREMENT_FIELDS:{missing}"]

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
    "pi_verifier_verdict_path",
    "source_identity_manifest_path",
    "launcher_status_path",
    "bridge_status_path",
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
    pi_verifier_verdict_path: Optional[Path] = None,
    source_identity_manifest_path: Optional[Path] = None,
    launcher_status_path: Optional[Path] = None,
    bridge_status_path: Optional[Path] = None,
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
            "pi_verifier_verdict_path": pi_verifier_verdict_path,
            "source_identity_manifest_path": source_identity_manifest_path,
            "launcher_status_path": launcher_status_path,
            "bridge_status_path": bridge_status_path,
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

        ok, reasons = _require_nonempty_file(wsl_command_evidence_path, "WSL_COMMAND_EVIDENCE")
        checks["wsl_command_evidence_present"] = ok
        invalid_reasons += reasons

        ok, reasons = _require_parseable_jsonl(pi_command_audit_path, "PI_COMMAND_AUDIT")
        checks["pi_command_audit_present"] = ok
        invalid_reasons += reasons

        ok, reasons, pi_verdict = _require_parseable_json(pi_verifier_verdict_path, "PI_VERIFIER_VERDICT", required_keys=("verdict",))
        checks["pi_verifier_verdict_present"] = ok
        invalid_reasons += reasons

        ok, reasons = _require_nonempty_file(source_identity_manifest_path, "SOURCE_IDENTITY_MANIFEST")
        checks["source_identity_manifest_present"] = ok
        invalid_reasons += reasons

        ok, reasons, _launcher_status = _require_parseable_json(launcher_status_path, "LAUNCHER_STATUS")
        checks["launcher_status_present"] = ok
        invalid_reasons += reasons

        ok, reasons = _require_nonempty_file(bridge_status_path, "BRIDGE_STATUS")
        checks["bridge_status_present"] = ok
        invalid_reasons += reasons

        if invalid_reasons:
            return VerifierResult("INVALID_EVIDENCE", invalid_reasons, checks)

        if pi_verdict is not None and pi_verdict.get("verdict") != "PASS":
            all_reasons.append(f"PI_VERIFIER_VERDICT_NOT_PASS:{pi_verdict.get('verdict')!r}")

        if not physical_measurements_path.is_file():
            return VerifierResult("INVALID_EVIDENCE", [f"PHYSICAL_MEASUREMENTS_FILE_MISSING:{physical_measurements_path}"], checks)
        try:
            measurements = json.loads(physical_measurements_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return VerifierResult("INVALID_EVIDENCE", ["PHYSICAL_MEASUREMENTS_FILE_UNPARSEABLE"], checks)
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
    parser.add_argument("--pi-verifier-verdict-path", type=Path, default=None)
    parser.add_argument("--source-identity-manifest-path", type=Path, default=None)
    parser.add_argument("--launcher-status-path", type=Path, default=None)
    parser.add_argument("--bridge-status-path", type=Path, default=None)
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
        pi_verifier_verdict_path=args.pi_verifier_verdict_path,
        source_identity_manifest_path=args.source_identity_manifest_path,
        launcher_status_path=args.launcher_status_path,
        bridge_status_path=args.bridge_status_path,
    )

    report = result.as_dict()
    output = json.dumps(report, indent=2)
    print(output)
    if args.report_path is not None:
        args.report_path.write_text(output, encoding="utf-8")

    return 0 if result.classification == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
