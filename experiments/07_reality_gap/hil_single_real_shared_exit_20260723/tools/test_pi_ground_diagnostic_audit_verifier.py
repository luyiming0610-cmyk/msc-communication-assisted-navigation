#!/usr/bin/env python3
"""Tests for pi_ground_diagnostic_audit_verifier.py -- the Pi-side
read-only audit verifier that closed the structural gap found
2026-07-24 (WSL cannot read a Pi-local file path). No ROS/rclpy
dependency. sample_row_count_growth and build_pi_audit_verdict are
pure/injectable and tested without real sleeping or real Pi files;
verify_pi_audit (the only function that does real I/O + real
time.sleep) gets one small real-file integration test with a short
interval.
"""
import json
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from pi_ground_diagnostic_audit_verifier import (
    GrowthSample,
    build_pi_audit_verdict,
    count_jsonl_lines,
    sample_row_count_growth,
    verify_pi_audit,
)

MODULE_PATH = Path(__file__).parent / "pi_ground_diagnostic_audit_verifier.py"

ZERO_TICK = {"event": "tick_applied", "linear": 0.0, "angular": 0.0, "zero_reason": "DISCONNECTED", "wall_time": 1.0}
NONZERO_TICK = {"event": "tick_applied", "linear": 0.015, "angular": 0.0, "zero_reason": None, "wall_time": 2.0}
ZERO_RECEIVED = {
    "event": "command_received",
    "linear_raw": 0.0,
    "angular_raw": 0.0,
    "linear_applied_clamped": 0.0,
    "angular_applied_clamped": 0.0,
}
NONZERO_RECEIVED = {
    "event": "command_received",
    "linear_raw": 0.02,
    "angular_raw": 0.0,
    "linear_applied_clamped": 0.02,
    "angular_applied_clamped": 0.0,
}
PARSE_ERROR_RECORD = {"event": "PARSE_ERROR", "line_number": 3, "reason": "bad json"}


class CountJsonlLinesTest(unittest.TestCase):
    def test_counts_non_empty_lines(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "audit.jsonl")
            Path(path).write_text('{"a": 1}\n{"a": 2}\n\n{"a": 3}\n', encoding="utf-8")
            self.assertEqual(count_jsonl_lines(path), 3)

    def test_missing_file_raises(self):
        with self.assertRaises(FileNotFoundError):
            count_jsonl_lines("/nonexistent/path/does_not_exist.jsonl")


class SampleRowCountGrowthTest(unittest.TestCase):
    def test_growing_when_after_greater_than_before(self):
        calls = iter([3, 7])
        sample = sample_row_count_growth(
            "unused_path", interval_s=0.0, count_fn=lambda p: next(calls), sleep_fn=lambda s: None
        )
        self.assertTrue(sample.growing)
        self.assertEqual(sample.before, 3)
        self.assertEqual(sample.after, 7)
        self.assertFalse(sample.file_missing)

    def test_not_growing_when_after_equals_before(self):
        calls = iter([5, 5])
        sample = sample_row_count_growth(
            "unused_path", interval_s=0.0, count_fn=lambda p: next(calls), sleep_fn=lambda s: None
        )
        self.assertFalse(sample.growing)
        self.assertFalse(sample.file_missing)

    def test_missing_file_before_first_sample(self):
        def count_fn(_path):
            raise FileNotFoundError()

        sample = sample_row_count_growth("unused_path", interval_s=0.0, count_fn=count_fn, sleep_fn=lambda s: None)
        self.assertTrue(sample.file_missing)
        self.assertFalse(sample.growing)
        self.assertIsNone(sample.before)
        self.assertIsNone(sample.after)

    def test_missing_file_between_samples(self):
        call_count = {"n": 0}

        def count_fn(_path):
            call_count["n"] += 1
            if call_count["n"] == 1:
                return 4
            raise FileNotFoundError()

        sample = sample_row_count_growth("unused_path", interval_s=0.0, count_fn=count_fn, sleep_fn=lambda s: None)
        self.assertTrue(sample.file_missing)
        self.assertEqual(sample.before, 4)
        self.assertIsNone(sample.after)
        self.assertFalse(sample.growing)

    def test_uses_the_given_interval(self):
        recorded = []
        sample_row_count_growth(
            "unused_path", interval_s=1.5, count_fn=lambda p: 1, sleep_fn=lambda s: recorded.append(s)
        )
        self.assertEqual(recorded, [1.5])


class BuildPiAuditVerdictTest(unittest.TestCase):
    def _growth(self, before=1, after=5, growing=True, file_missing=False):
        return GrowthSample(before=before, after=after, growing=growing, file_missing=file_missing)

    def test_file_missing_reports_not_available_never_proven_nonzero(self):
        verdict = build_pi_audit_verdict(
            jsonl_path="/home/pi/x.jsonl", run_id="run1", growth=self._growth(file_missing=True), records=None
        )
        self.assertFalse(verdict.ok)
        self.assertEqual(verdict.reasons, ("JSONL_NOT_AVAILABLE",))
        self.assertNotIn("PI_EVIDENCE_CONTAINS_NONZERO_COMMAND", verdict.reasons)

    def test_all_zero_growing_no_malformed_passes(self):
        records = [ZERO_TICK, ZERO_RECEIVED, ZERO_TICK]
        verdict = build_pi_audit_verdict(
            jsonl_path="/home/pi/x.jsonl", run_id="run1", growth=self._growth(), records=records
        )
        self.assertTrue(verdict.ok, verdict.reasons)
        self.assertEqual(verdict.reasons, ())
        self.assertEqual(verdict.total_records, 3)
        self.assertEqual(verdict.malformed_count, 0)
        self.assertEqual(verdict.nonzero_received_count, 0)
        self.assertEqual(verdict.nonzero_applied_count, 0)

    def test_non_growing_file_blocks(self):
        records = [ZERO_TICK, ZERO_RECEIVED]
        verdict = build_pi_audit_verdict(
            jsonl_path="/home/pi/x.jsonl", run_id="run1", growth=self._growth(growing=False), records=records
        )
        self.assertFalse(verdict.ok)
        self.assertIn("PI_JSONL_NOT_GROWING", verdict.reasons)

    def test_malformed_json_line_blocks(self):
        records = [ZERO_TICK, PARSE_ERROR_RECORD, ZERO_TICK]
        verdict = build_pi_audit_verdict(
            jsonl_path="/home/pi/x.jsonl", run_id="run1", growth=self._growth(), records=records
        )
        self.assertFalse(verdict.ok)
        self.assertIn("MALFORMED_JSON_LINES_PRESENT", verdict.reasons)
        self.assertEqual(verdict.malformed_count, 1)

    def test_actual_nonzero_received_command_blocks(self):
        records = [ZERO_TICK, NONZERO_RECEIVED]
        verdict = build_pi_audit_verdict(
            jsonl_path="/home/pi/x.jsonl", run_id="run1", growth=self._growth(), records=records
        )
        self.assertFalse(verdict.ok)
        self.assertIn("PI_EVIDENCE_CONTAINS_NONZERO_COMMAND", verdict.reasons)
        self.assertGreater(verdict.nonzero_received_count, 0)

    def test_actual_nonzero_applied_tick_blocks(self):
        records = [ZERO_TICK, NONZERO_TICK]
        verdict = build_pi_audit_verdict(
            jsonl_path="/home/pi/x.jsonl", run_id="run1", growth=self._growth(), records=records
        )
        self.assertFalse(verdict.ok)
        self.assertIn("PI_EVIDENCE_CONTAINS_NONZERO_COMMAND", verdict.reasons)

    def test_latest_zero_reason_and_values_from_last_tick(self):
        records = [ZERO_TICK, {**ZERO_TICK, "zero_reason": "DISCONNECTED", "linear": 0.0, "angular": 0.0}]
        verdict = build_pi_audit_verdict(
            jsonl_path="/home/pi/x.jsonl", run_id="run1", growth=self._growth(), records=records
        )
        self.assertEqual(verdict.latest_zero_reason, "DISCONNECTED")
        self.assertEqual(verdict.latest_linear, 0.0)
        self.assertEqual(verdict.latest_angular, 0.0)

    def test_run_id_and_path_and_timestamp_are_tagged(self):
        now = datetime(2026, 7, 24, 15, 40, 0, tzinfo=timezone.utc)
        verdict = build_pi_audit_verdict(
            jsonl_path="/home/pi/x.jsonl", run_id="run_20260724_153950", growth=self._growth(), records=[ZERO_TICK], now=now
        )
        self.assertEqual(verdict.run_id, "run_20260724_153950")
        self.assertEqual(verdict.jsonl_path, "/home/pi/x.jsonl")
        self.assertEqual(verdict.generated_at_utc, now.isoformat())

    def test_multiple_failures_all_reported(self):
        records = [PARSE_ERROR_RECORD, NONZERO_RECEIVED]
        verdict = build_pi_audit_verdict(
            jsonl_path="/home/pi/x.jsonl", run_id="run1", growth=self._growth(growing=False), records=records
        )
        self.assertFalse(verdict.ok)
        self.assertIn("PI_JSONL_NOT_GROWING", verdict.reasons)
        self.assertIn("MALFORMED_JSON_LINES_PRESENT", verdict.reasons)
        self.assertIn("PI_EVIDENCE_CONTAINS_NONZERO_COMMAND", verdict.reasons)


class VerifyPiAuditIntegrationTest(unittest.TestCase):
    """One small real-file, real-time.sleep integration test proving
    the pieces work together end to end -- everything else above tests
    the pure/injectable functions without real sleeping."""

    def test_growing_all_zero_file_passes(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "audit.jsonl")
            Path(path).write_text(json.dumps(ZERO_TICK) + "\n", encoding="utf-8")

            import threading

            def append_more():
                with open(path, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(ZERO_TICK) + "\n")

            timer = threading.Timer(0.1, append_more)
            timer.start()
            verdict = verify_pi_audit(path, run_id="run1", growth_interval_s=2.0)
            timer.join()

            self.assertTrue(verdict.ok, verdict.reasons)
            self.assertTrue(verdict.growing)

    def test_missing_file_reports_not_available(self):
        verdict = verify_pi_audit("/nonexistent/path/audit.jsonl", run_id="run1", growth_interval_s=0.01)
        self.assertFalse(verdict.ok)
        self.assertEqual(verdict.reasons, ("JSONL_NOT_AVAILABLE",))


class CliTest(unittest.TestCase):
    def _run(self, *args):
        return subprocess.run(
            [sys.executable, str(MODULE_PATH), *args],
            capture_output=True,
            text=True,
        )

    def test_cli_reports_pass_and_writes_json_verdict(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "audit.jsonl")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(json.dumps(ZERO_TICK) + "\n")

            import threading

            def append_more():
                with open(path, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(ZERO_TICK) + "\n")

            timer = threading.Timer(0.2, append_more)
            timer.start()
            out_json = str(Path(tmp) / "verdict.json")
            result = self._run("--path", path, "--run-id", "run1", "--growth-interval-s", "5.0", "--output-json", out_json)
            timer.join()

            self.assertEqual(result.returncode, 0)
            self.assertIn("PI_AUDIT_VERDICT=PASS", result.stdout)
            with open(out_json, encoding="utf-8") as fh:
                verdict = json.load(fh)
            self.assertTrue(verdict["ok"])
            self.assertEqual(verdict["run_id"], "run1")

    def test_cli_reports_blocked_for_missing_file(self):
        result = self._run("--path", "/nonexistent/path/audit.jsonl", "--run-id", "run1", "--growth-interval-s", "0.01")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("PI_AUDIT_VERDICT=BLOCKED", result.stdout)
        self.assertIn("JSONL_NOT_AVAILABLE", result.stdout)


if __name__ == "__main__":
    unittest.main()
