#!/usr/bin/env python3
"""Offline, hardware-free tests for hil_stage4_post_run_verifier.py.
Covers: a successful rehearsal PASS, a valid behavioural failure
(FAIL_VALID_EVIDENCE), and several invalid-evidence cases (missing
file, malformed JSON, non-monotonic ordering, hash mismatch, missing
PID manifest entry, residual process not clean, physical-mode
measurement violations). No ROS, no Pi, no Webots, no process of any
kind is started by this file."""
from __future__ import annotations

import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from hil_stage4_post_run_verifier import (  # noqa: E402
    HARD_MAX_NONZERO_DURATION_S,
    INTERNAL_ACTIVE_CUTOFF_S,
    run_verifier,
)

GOAL_ID = "shared_exit"
RUN_ID = "verifier_test_run"


def _record(monotonic_time_s, state, event, reason="", raw=None):
    return {
        "monotonic_time_s": monotonic_time_s, "ros_time_s": monotonic_time_s,
        "state": state, "event": event, "reason": reason, "raw": raw,
        "run_id": RUN_ID, "goal_id": GOAL_ID,
    }


def _successful_records(active_duration_s=6.50):
    t = 1000.0
    records = [
        _record(t, "WAITING_FOR_EVENT", "APPROVAL_ACCEPTED"),
        _record(t + 0.01, "WAITING_FOR_EVENT", "READINESS_WAITING_FOR_EVENT"),
        _record(t + 1.0, "WAITING_FOR_EVENT", "VIRTUAL_SCOUT_RELEASED"),
        _record(t + 2.0, "VALIDATING_RAW_COMMAND", "ADOPTION_CONFIRMED"),
        _record(t + 2.05, "VALIDATING_RAW_COMMAND", "RAW_TWIST_RECEIVED", raw={
            "linear_x": 0.015, "linear_y": 0.0, "linear_z": 0.0,
            "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0,
        }),
        _record(t + 2.05, "VALIDATING_RAW_COMMAND", "ARM_PUBLISHED"),
        _record(t + 2.05, "ACTIVE", "ACTIVE_OPENED"),
        _record(t + 2.05 + active_duration_s, "ZERO_BURST", "ZERO_BURST_OPENED", reason="INTERNAL_ACTIVE_CUTOFF_REACHED"),
        _record(t + 2.05 + active_duration_s, "ZERO_BURST", "ZERO_PUBLISHED"),
        _record(t + 2.05 + active_duration_s, "DISARMED", "DISARM_PUBLISHED"),
        _record(t + 2.05 + active_duration_s, "COMPLETE", "LATCHED_COMPLETE"),
    ]
    return records


def _write_jsonl(path: Path, records: list) -> None:
    path.write_text("\n".join(json.dumps(r) for r in records) + "\n", encoding="utf-8")


def _default_physical_measurements() -> dict:
    return {
        "manual_forward_displacement_m": 0.09,
        "corridor_crossed": False,
        "stop_line_crossed": False,
        "min_boundary_clearance_m": 0.20,
        "unexpected_rotation": False,
        "unexpected_direction": False,
        "unexpected_sound": False,
        "unexpected_acceleration": False,
        "run_interrupted": False,
    }


def _full_physical_fixture(d: Path, measurements: dict = None) -> dict:
    """Builds a complete, valid set of physical-mode inputs in directory
    `d` and returns the exact kwargs run_verifier(mode="physical")
    expects. Individual pieces can be deleted/corrupted by the caller
    after this returns to test a specific gap."""
    pid_manifest = d / "pid_manifest.json"
    pid_manifest.write_text(json.dumps({"processes": {"recorder": {"pid": 1}, "guard": {"pid": 2}}}), encoding="utf-8")

    wsl_evidence = d / "command_evidence.csv"
    wsl_evidence.write_text("topic,linear_x,angular_z\ncmd_vel,0.015,0.0\n", encoding="utf-8")

    hash_manifest = d / "SHA256SUMS.txt"
    wsl_hash = hashlib.sha256(wsl_evidence.read_bytes()).hexdigest()
    hash_manifest.write_text(f"{wsl_hash}  command_evidence.csv\n", encoding="utf-8")

    adoption_evidence = d / "adoption_evidence.jsonl"
    adoption_evidence.write_text(json.dumps({"goal_id": GOAL_ID, "accepted": True}) + "\n", encoding="utf-8")

    pi_command_audit = d / "pi_command_audit.jsonl"
    pi_command_audit.write_text(json.dumps({"linear_x": 0.015, "angular_z": 0.0}) + "\n", encoding="utf-8")

    pi_verifier_verdict = d / "pi_verifier_verdict.json"
    pi_verifier_verdict.write_text(json.dumps({"verdict": "PASS"}), encoding="utf-8")

    source_identity_manifest = d / "source_identity_manifest.json"
    source_identity_manifest.write_text(json.dumps({"overall_result": "PASS"}), encoding="utf-8")

    launcher_status = d / "launcher_status.json"
    launcher_status.write_text(json.dumps({"status": "COMPLETE"}), encoding="utf-8")

    bridge_status = d / "bridge_status.jsonl"
    bridge_status.write_text(json.dumps({"connected": True}) + "\n", encoding="utf-8")

    measurements_path = d / "measurements.json"
    measurements_path.write_text(json.dumps(measurements or _default_physical_measurements()), encoding="utf-8")

    return dict(
        pid_manifest_path=pid_manifest,
        hash_manifest_path=hash_manifest,
        evidence_dir=d,
        adoption_evidence_path=adoption_evidence,
        wsl_command_evidence_path=wsl_evidence,
        pi_command_audit_path=pi_command_audit,
        pi_verifier_verdict_path=pi_verifier_verdict,
        source_identity_manifest_path=source_identity_manifest,
        launcher_status_path=launcher_status,
        bridge_status_path=bridge_status,
        physical_measurements_path=measurements_path,
    )


class SuccessfulRehearsalPassTest(unittest.TestCase):
    def test_pass(self):
        with tempfile.TemporaryDirectory() as d:
            evidence_path = Path(d) / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="rehearsal")
            self.assertEqual(result.classification, "PASS", result.reasons)
            self.assertEqual(result.reasons, [])
            self.assertEqual(result.checks["terminal_state"], "COMPLETE")


class ValidBehaviouralFailureTest(unittest.TestCase):
    def test_terminal_state_failed_is_fail_valid_evidence(self):
        with tempfile.TemporaryDirectory() as d:
            evidence_path = Path(d) / "evidence.jsonl"
            records = _successful_records()[:4] + [
                _record(1010.0, "FAILED", "LATCHED_FAILED", reason="ADOPTION_TIMEOUT"),
            ]
            _write_jsonl(evidence_path, records)
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="rehearsal")
            self.assertEqual(result.classification, "FAIL_VALID_EVIDENCE")
            self.assertTrue(any("TERMINAL_STATE_NOT_COMPLETE" in r for r in result.reasons))

    def test_active_duration_exceeding_hard_maximum_is_fail_valid_evidence(self):
        with tempfile.TemporaryDirectory() as d:
            evidence_path = Path(d) / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records(active_duration_s=HARD_MAX_NONZERO_DURATION_S + 1.0))
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="rehearsal")
            self.assertEqual(result.classification, "FAIL_VALID_EVIDENCE")
            self.assertTrue(any("EXCEEDS_HARD_VERIFIER_MAXIMUM" in r for r in result.reasons))

    def test_second_arm_published_is_fail_valid_evidence(self):
        with tempfile.TemporaryDirectory() as d:
            evidence_path = Path(d) / "evidence.jsonl"
            records = _successful_records()
            records.insert(7, _record(1002.06, "ACTIVE", "ARM_PUBLISHED"))
            _write_jsonl(evidence_path, records)
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="rehearsal")
            self.assertEqual(result.classification, "FAIL_VALID_EVIDENCE")
            self.assertTrue(any("EXACTLY_ONE_ARM_PUBLISHED" in r for r in result.reasons))

    def test_pre_adoption_motion_is_fail_valid_evidence(self):
        with tempfile.TemporaryDirectory() as d:
            evidence_path = Path(d) / "evidence.jsonl"
            records = _successful_records()
            records.insert(3, _record(1001.5, "WAITING_FOR_EVENT", "RAW_TWIST_RECEIVED", raw={"linear_x": 0.01}))
            _write_jsonl(evidence_path, records)
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="rehearsal")
            self.assertEqual(result.classification, "FAIL_VALID_EVIDENCE")
            self.assertTrue(any("PRE_ADOPTION_MOTION" in r for r in result.reasons))


class InvalidEvidenceTest(unittest.TestCase):
    def test_missing_evidence_file(self):
        result = run_verifier(supervisor_evidence_path=Path("/nonexistent/evidence.jsonl"), mode="rehearsal")
        self.assertEqual(result.classification, "INVALID_EVIDENCE")

    def test_empty_evidence_file(self):
        with tempfile.TemporaryDirectory() as d:
            evidence_path = Path(d) / "evidence.jsonl"
            evidence_path.write_text("", encoding="utf-8")
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="rehearsal")
            self.assertEqual(result.classification, "INVALID_EVIDENCE")

    def test_malformed_json_line(self):
        with tempfile.TemporaryDirectory() as d:
            evidence_path = Path(d) / "evidence.jsonl"
            evidence_path.write_text('{"not": "closed"\n', encoding="utf-8")
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="rehearsal")
            self.assertEqual(result.classification, "INVALID_EVIDENCE")

    def test_non_monotonic_ordering(self):
        with tempfile.TemporaryDirectory() as d:
            evidence_path = Path(d) / "evidence.jsonl"
            records = _successful_records()
            records[3], records[4] = records[4], records[3]
            _write_jsonl(evidence_path, records)
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="rehearsal")
            self.assertEqual(result.classification, "INVALID_EVIDENCE")
            self.assertIn("RECORDS_NOT_MONOTONICALLY_ORDERED", result.reasons)

    def test_hash_mismatch(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            other_file = d / "some_evidence.csv"
            other_file.write_text("a,b,c\n1,2,3\n", encoding="utf-8")
            wrong_hash = hashlib.sha256(b"not the real content").hexdigest()
            hash_manifest = d / "SHA256SUMS.txt"
            hash_manifest.write_text(f"{wrong_hash}  some_evidence.csv\n", encoding="utf-8")
            result = run_verifier(
                supervisor_evidence_path=evidence_path, mode="rehearsal",
                hash_manifest_path=hash_manifest, evidence_dir=d,
            )
            self.assertEqual(result.classification, "INVALID_EVIDENCE")
            self.assertTrue(any("HASH_MISMATCH" in r for r in result.reasons))

    def test_hash_manifest_missing_file_entry(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            hash_manifest = d / "SHA256SUMS.txt"
            hash_manifest.write_text("deadbeef  never_created.csv\n", encoding="utf-8")
            result = run_verifier(
                supervisor_evidence_path=evidence_path, mode="rehearsal",
                hash_manifest_path=hash_manifest, evidence_dir=d,
            )
            self.assertEqual(result.classification, "INVALID_EVIDENCE")
            self.assertTrue(any("HASHED_FILE_MISSING" in r for r in result.reasons))

    def test_pid_manifest_missing_recorder_entry(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            pid_manifest = d / "pid_manifest.json"
            pid_manifest.write_text(json.dumps({"processes": {"guard": {"pid": 123}}}), encoding="utf-8")
            result = run_verifier(
                supervisor_evidence_path=evidence_path, mode="rehearsal", pid_manifest_path=pid_manifest,
            )
            self.assertEqual(result.classification, "INVALID_EVIDENCE")
            self.assertTrue(any("MISSING_RECORDER_ENTRY" in r for r in result.reasons))

    def test_residual_process_not_clean(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            residual_path = d / "residual_check.json"
            residual_path.write_text(json.dumps({"residual_process_check": "PROCESS_STILL_ALIVE"}), encoding="utf-8")
            result = run_verifier(
                supervisor_evidence_path=evidence_path, mode="rehearsal", residual_check_path=residual_path,
            )
            self.assertEqual(result.classification, "INVALID_EVIDENCE")
            self.assertTrue(any("RESIDUAL_PROCESS_CHECK_NOT_CLEAN" in r for r in result.reasons))

    def test_physical_mode_requires_measurements_path(self):
        with tempfile.TemporaryDirectory() as d:
            evidence_path = Path(d) / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="physical")
            self.assertEqual(result.classification, "INVALID_EVIDENCE")
            self.assertTrue(any("PHYSICAL_MODE_REQUIRES_MEASUREMENTS_PATH" in r for r in result.reasons))

    def test_physical_mode_missing_pi_evidence_is_invalid(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            kwargs = _full_physical_fixture(d)
            del kwargs["pi_command_audit_path"]
            del kwargs["pi_verifier_verdict_path"]
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="physical", **kwargs)
            self.assertEqual(result.classification, "INVALID_EVIDENCE")
            self.assertTrue(any("PI_COMMAND_AUDIT_PATH" in r for r in result.reasons))
            self.assertTrue(any("PI_VERIFIER_VERDICT_PATH" in r for r in result.reasons))

    def test_physical_measurements_missing_field_is_invalid(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            kwargs = _full_physical_fixture(d)
            kwargs["physical_measurements_path"].write_text(
                json.dumps({"manual_forward_displacement_m": 0.09}), encoding="utf-8",
            )
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="physical", **kwargs)
            self.assertEqual(result.classification, "FAIL_VALID_EVIDENCE")
            self.assertTrue(any("MISSING_PHYSICAL_MEASUREMENT_FIELDS" in r for r in result.reasons))


class PhysicalModePassTest(unittest.TestCase):
    def test_physical_pass(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            kwargs = _full_physical_fixture(d)
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="physical", **kwargs)
            self.assertEqual(result.classification, "PASS", result.reasons)

    def test_physical_displacement_out_of_range_fails(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            kwargs = _full_physical_fixture(d, measurements=_default_physical_measurements() | {"manual_forward_displacement_m": 0.20})
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="physical", **kwargs)
            self.assertEqual(result.classification, "FAIL_VALID_EVIDENCE")
            self.assertTrue(any("DISPLACEMENT_OUT_OF_RANGE" in r for r in result.reasons))

    def test_physical_corridor_crossed_fails(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            kwargs = _full_physical_fixture(d, measurements=_default_physical_measurements() | {"corridor_crossed": True})
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="physical", **kwargs)
            self.assertEqual(result.classification, "FAIL_VALID_EVIDENCE")
            self.assertIn("CORRIDOR_CROSSED_MUST_BE_FALSE", result.reasons)

    def test_physical_pi_verdict_not_pass_fails(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            kwargs = _full_physical_fixture(d)
            kwargs["pi_verifier_verdict_path"].write_text(json.dumps({"verdict": "FAIL"}), encoding="utf-8")
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="physical", **kwargs)
            self.assertEqual(result.classification, "FAIL_VALID_EVIDENCE")
            self.assertTrue(any("PI_VERIFIER_VERDICT_NOT_PASS" in r for r in result.reasons))

    def test_physical_hash_mismatch_is_invalid(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            evidence_path = d / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            kwargs = _full_physical_fixture(d)
            kwargs["hash_manifest_path"].write_text(
                hashlib.sha256(b"corrupted").hexdigest() + "  command_evidence.csv\n", encoding="utf-8",
            )
            result = run_verifier(supervisor_evidence_path=evidence_path, mode="physical", **kwargs)
            self.assertEqual(result.classification, "INVALID_EVIDENCE")
            self.assertTrue(any("HASH_MISMATCH" in r for r in result.reasons))

    def test_rehearsal_pass_is_not_physical_pass(self):
        """Explicit guard against the exact mislabeling risk called out
        in the design review: a rehearsal PASS and a physical PASS are
        produced by different modes and must never be conflated."""
        with tempfile.TemporaryDirectory() as d:
            evidence_path = Path(d) / "evidence.jsonl"
            _write_jsonl(evidence_path, _successful_records())
            rehearsal_result = run_verifier(supervisor_evidence_path=evidence_path, mode="rehearsal")
            self.assertEqual(rehearsal_result.classification, "PASS")
            physical_result = run_verifier(supervisor_evidence_path=evidence_path, mode="physical")
            self.assertEqual(physical_result.classification, "INVALID_EVIDENCE")
            self.assertNotEqual(rehearsal_result.classification, "PASS" if physical_result.classification == "PASS" else None)


if __name__ == "__main__":
    unittest.main()
