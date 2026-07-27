#!/usr/bin/env python3
"""Regression fixtures for ground_diagnostic_post_run_verifier.py.

Every fixture below writes small, synthetic evidence files to a
temporary directory and calls verify_run() directly -- no ROS, no
network, no subprocess, no physical process of any kind. Column names
match hil_command_evidence_recorder.py's CSV_FIELDS and
pi_epuck_tcp_server_sensors_audited.py's JSONL record shapes exactly,
as already used by test_analyze_ground_diagnostic.py.
"""
import hashlib
import json
import os
import tempfile
import unittest

from ground_diagnostic_post_run_verifier import ExternalConfirmations, main, verify_run

CSV_HEADER = "topic,local_time_ns,local_monotonic_ns,sequence,linear_x,angular_z,arm_state,validity_flags,bridge_connected,bridge_rx_count\n"

ALL_CONFIRMED = ExternalConfirmations(
    geometry_confirmed=True,
    guard_sole_publisher_confirmed=True,
    no_unexpected_motion_observed=True,
    robot_stayed_within_measured_area=True,
    run_not_interrupted=True,
)


def _csv_row(topic="", t_ns="", linear_x="", angular_z="", arm_state="", validity_flags="", bridge_connected="", rx="", seq=""):
    return f"{topic},{t_ns},{t_ns},{seq},{linear_x},{angular_z},{arm_state},{validity_flags},{bridge_connected},{rx}\n"


def _sha256(path):
    hasher = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


class VerifierFixtureBase(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmpdir.cleanup)
        self.run_id = "20260101_000000"
        self.wsl_csv_path = os.path.join(self.tmpdir.name, "command_evidence.csv")
        self.pi_jsonl_path = os.path.join(self.tmpdir.name, "command_audit.jsonl")
        self.pi_verdict_path = os.path.join(self.tmpdir.name, "pi_audit_verdict.json")

    def _write_csv(self, rows):
        with open(self.wsl_csv_path, "w", encoding="utf-8") as fh:
            fh.write(CSV_HEADER)
            for row in rows:
                fh.write(row)

    def _write_jsonl(self, records):
        with open(self.pi_jsonl_path, "w", encoding="utf-8") as fh:
            for r in records:
                fh.write(json.dumps(r) + "\n")

    def _write_verdict(self, extra=None):
        payload = {"run_id": self.run_id, "jsonl_path": self.pi_jsonl_path, "verdict": "PASS"}
        if extra:
            payload.update(extra)
        with open(self.pi_verdict_path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh)

    def _expected_hashes(self):
        return {
            "wsl": _sha256(self.wsl_csv_path),
            "jsonl": _sha256(self.pi_jsonl_path),
            "verdict": _sha256(self.pi_verdict_path),
        }

    def _verify(self, external=ALL_CONFIRMED, **overrides):
        hashes = self._expected_hashes()
        kwargs = dict(
            run_id=self.run_id,
            wsl_csv_path=self.wsl_csv_path,
            pi_jsonl_path=self.pi_jsonl_path,
            pi_verdict_path=self.pi_verdict_path,
            expected_wsl_sha256=hashes["wsl"],
            expected_pi_jsonl_sha256=hashes["jsonl"],
            expected_pi_verdict_sha256=hashes["verdict"],
            external=external,
        )
        kwargs.update(overrides)
        return verify_run(**kwargs)


def _clean_pulse_evidence(self):
    """A minimal, self-consistent PASS-shaped fixture: arm, a short
    clean pulse on both upstream and guarded topics, final zero,
    validity_flags always 7, bridge always connected."""
    self._write_csv(
        [
            _csv_row("/hil_guard/arm", 1_000_000_000, arm_state="True"),
            _csv_row("/epuck1/state", 1_000_000_000, validity_flags=7),
            _csv_row("cmd_vel_unguarded", 2_000_000_000, linear_x=0.015, angular_z=0.0),
            _csv_row("cmd_vel", 2_000_000_000, linear_x=0.015, angular_z=0.0),
            _csv_row("/epuck1/state", 2_000_000_000, validity_flags=7),
            _csv_row("cmd_vel_unguarded", 3_000_000_000, linear_x=0.0, angular_z=0.0),
            _csv_row("cmd_vel", 3_000_000_000, linear_x=0.0, angular_z=0.0),
            _csv_row("/epuck1/state", 3_000_000_000, validity_flags=7),
            _csv_row("/epuck_bridge/status", 1_000_000_000, bridge_connected="True", rx=1),
            _csv_row("/epuck_bridge/status", 3_000_000_000, bridge_connected="True", rx=2),
        ]
    )
    self._write_jsonl(
        [
            {"event": "tick_applied", "wall_time": 2.0, "monotonic_time": 2.0, "linear": 0.015, "angular": 0.0},
            {"event": "tick_applied", "wall_time": 3.0, "monotonic_time": 3.0, "linear": 0.0, "angular": 0.0},
        ]
    )
    self._write_verdict()


class ExactMatchPassTest(VerifierFixtureBase):
    def test_clean_evidence_reproduces_pass(self):
        _clean_pulse_evidence(self)
        result = self._verify()
        self.assertTrue(result.integrity_ok)
        self.assertEqual(result.integrity_reasons, ())
        self.assertEqual(result.diagnostic_verdict, "PASS")
        self.assertEqual(result.diagnostic_reasons, ())


class PulseEdgeTimingMismatchTest(VerifierFixtureBase):
    def test_rise_and_fall_edge_mismatches_are_reported_but_do_not_block_pass(self):
        self._write_csv(
            [
                _csv_row("/hil_guard/arm", 1_000_000_000, arm_state="True"),
                _csv_row("/epuck1/state", 1_000_000_000, validity_flags=7),
                _csv_row("cmd_vel", 10_000_000_000, linear_x=0.0, angular_z=0.0),
                _csv_row("cmd_vel", 10_100_000_000, linear_x=0.015, angular_z=0.0),
                _csv_row("cmd_vel", 11_000_000_000, linear_x=0.0, angular_z=0.0),
                _csv_row("/epuck1/state", 11_000_000_000, validity_flags=7),
                _csv_row("/epuck_bridge/status", 1_000_000_000, bridge_connected="True", rx=1),
            ]
        )
        self._write_jsonl(
            [
                {"event": "tick_applied", "wall_time": 10.05, "monotonic_time": 500.0, "linear": 0.015, "angular": 0.0},
                {"event": "tick_applied", "wall_time": 11.0, "monotonic_time": 501.0, "linear": 0.0, "angular": 0.0},
            ]
        )
        self._write_verdict()
        result = self._verify()
        self.assertTrue(result.integrity_ok)
        self.assertGreater(result.facts["guarded_vs_pi_mismatch_count"], 0)
        # An edge-timing mismatch alone (both sides otherwise proven
        # zero at rest) must not force the binding verdict to FAIL --
        # it is diagnostic-only, exactly as documented in SUMMARY.md.
        self.assertEqual(result.diagnostic_verdict, "PASS")


class TrueUnexpectedMismatchTest(VerifierFixtureBase):
    def test_genuine_command_discrepancy_is_flagged_in_facts(self):
        _clean_pulse_evidence(self)
        # Overwrite the JSONL with a genuinely wrong applied value that
        # cannot be explained by rise/fall timing.
        self._write_jsonl(
            [
                {"event": "tick_applied", "wall_time": 2.0, "monotonic_time": 2.0, "linear": 0.05, "angular": 0.0},
                {"event": "tick_applied", "wall_time": 3.0, "monotonic_time": 3.0, "linear": 0.0, "angular": 0.0},
            ]
        )
        self._write_verdict()
        result = self._verify()
        self.assertEqual(result.facts["guarded_vs_pi_mismatch_count"], 1)
        mismatch = result.facts["guarded_vs_pi_mismatch_pairs"][0]
        self.assertAlmostEqual(mismatch[4], 0.035, places=3)


class UnsortedTimestampTest(VerifierFixtureBase):
    def test_out_of_order_jsonl_gives_identical_result_to_sorted(self):
        _clean_pulse_evidence(self)
        result_sorted = self._verify()

        shuffled_records = [
            {"event": "tick_applied", "wall_time": 3.0, "monotonic_time": 3.0, "linear": 0.0, "angular": 0.0},
            {"event": "tick_applied", "wall_time": 2.0, "monotonic_time": 2.0, "linear": 0.015, "angular": 0.0},
        ]
        self._write_jsonl(shuffled_records)
        self._write_verdict()
        result_shuffled = self._verify()

        self.assertEqual(result_sorted.diagnostic_verdict, result_shuffled.diagnostic_verdict)
        self.assertEqual(
            result_sorted.facts["guarded_vs_pi_mismatch_count"], result_shuffled.facts["guarded_vs_pi_mismatch_count"]
        )


class EmptyOrMalformedEvidenceTest(VerifierFixtureBase):
    def test_empty_files_are_excluded_not_crashed_on(self):
        open(self.wsl_csv_path, "w", encoding="utf-8").close()
        open(self.pi_jsonl_path, "w", encoding="utf-8").close()
        self._write_verdict()
        result = self._verify()
        self.assertEqual(result.diagnostic_verdict, "EXCLUDED")

    def test_malformed_jsonl_line_is_reported_not_silently_dropped(self):
        _clean_pulse_evidence(self)
        with open(self.pi_jsonl_path, "a", encoding="utf-8") as fh:
            fh.write("{not valid json\n")
        result = self._verify()
        self.assertGreaterEqual(result.facts.get("malformed_pi_jsonl_line_count", 0), 1)
        self.assertIn("MALFORMED_PI_JSONL_LINES(1)", result.integrity_reasons)
        self.assertFalse(result.integrity_ok)

    def test_malformed_verdict_json_yields_excluded(self):
        _clean_pulse_evidence(self)
        with open(self.pi_verdict_path, "w", encoding="utf-8") as fh:
            fh.write("{not valid json")
        result = self._verify()
        self.assertEqual(result.diagnostic_verdict, "EXCLUDED")
        self.assertFalse(result.integrity_ok)


class HashMismatchTest(VerifierFixtureBase):
    def test_tampered_csv_after_hashing_is_detected(self):
        _clean_pulse_evidence(self)
        hashes = self._expected_hashes()
        with open(self.wsl_csv_path, "a", encoding="utf-8") as fh:
            fh.write(_csv_row("cmd_vel", 4_000_000_000, linear_x=0.05, angular_z=0.0))
        result = verify_run(
            run_id=self.run_id,
            wsl_csv_path=self.wsl_csv_path,
            pi_jsonl_path=self.pi_jsonl_path,
            pi_verdict_path=self.pi_verdict_path,
            expected_wsl_sha256=hashes["wsl"],
            expected_pi_jsonl_sha256=hashes["jsonl"],
            expected_pi_verdict_sha256=hashes["verdict"],
            external=ALL_CONFIRMED,
        )
        self.assertFalse(result.integrity_ok)
        self.assertIn("HASH_MISMATCH:wsl_csv", result.integrity_reasons)


class WrongRunIdOrPathTest(VerifierFixtureBase):
    def test_wrong_run_id_in_verdict_is_detected(self):
        _clean_pulse_evidence(self)
        self._write_verdict(extra={"run_id": "20261231_235959"})
        result = self._verify()
        self.assertFalse(result.integrity_ok)
        self.assertTrue(any(r.startswith("RUN_ID_MISMATCH") for r in result.integrity_reasons))

    def test_wrong_jsonl_path_in_verdict_is_detected(self):
        _clean_pulse_evidence(self)
        self._write_verdict(extra={"jsonl_path": "/some/other/place/command_audit_different.jsonl"})
        result = self._verify()
        self.assertFalse(result.integrity_ok)
        self.assertTrue(any(r.startswith("PI_JSONL_PATH_MISMATCH") for r in result.integrity_reasons))


class FinalNonzeroCommandTest(VerifierFixtureBase):
    def test_final_nonzero_command_fails_the_diagnostic_verdict(self):
        self._write_csv(
            [
                _csv_row("/hil_guard/arm", 1_000_000_000, arm_state="True"),
                _csv_row("/epuck1/state", 1_000_000_000, validity_flags=7),
                _csv_row("cmd_vel", 2_000_000_000, linear_x=0.015, angular_z=0.0),
                # No zero row after -- final recorded command stays nonzero.
                _csv_row("/epuck_bridge/status", 1_000_000_000, bridge_connected="True", rx=1),
            ]
        )
        self._write_jsonl(
            [
                {"event": "tick_applied", "wall_time": 2.0, "monotonic_time": 2.0, "linear": 0.015, "angular": 0.0},
            ]
        )
        self._write_verdict()
        result = self._verify()
        self.assertTrue(result.integrity_ok)
        self.assertEqual(result.diagnostic_verdict, "FAIL")
        self.assertIn("FINAL_COMMAND_NOT_CONFIRMED_ZERO", result.diagnostic_reasons)


class ValidityDropoutTest(VerifierFixtureBase):
    def test_validity_flags_dropout_excludes_the_diagnostic_verdict(self):
        _clean_pulse_evidence(self)
        # Append a dropout episode (validity_flags != 7) into the state rows.
        with open(self.wsl_csv_path, "a", encoding="utf-8") as fh:
            fh.write(_csv_row("/epuck1/state", 2_500_000_000, validity_flags=3))
        result = self._verify()
        self.assertTrue(result.integrity_ok)
        self.assertEqual(result.facts["validity_flags_dropouts"]["dropout_count"], 1)
        self.assertEqual(result.diagnostic_verdict, "EXCLUDED")
        self.assertIn("VALIDITY_FLAGS_NOT_AVAILABLE_BEFORE_MOTION", result.diagnostic_reasons)


class BridgeDisconnectTest(VerifierFixtureBase):
    def test_bridge_disconnection_is_reported_in_facts(self):
        _clean_pulse_evidence(self)
        with open(self.wsl_csv_path, "a", encoding="utf-8") as fh:
            fh.write(_csv_row("/epuck_bridge/status", 2_500_000_000, bridge_connected="False", rx=1))
        result = self._verify()
        self.assertTrue(result.facts["bridge_summary"]["ever_disconnected"])
        self.assertEqual(result.facts["bridge_summary"]["disconnected_sample_count"], 1)


class MissingFileTest(VerifierFixtureBase):
    def test_missing_wsl_csv_is_excluded_not_crashed_on(self):
        # Never write the CSV at all.
        self._write_jsonl([{"event": "tick_applied", "wall_time": 1.0, "monotonic_time": 1.0, "linear": 0.0, "angular": 0.0}])
        self._write_verdict()
        result = verify_run(
            run_id=self.run_id,
            wsl_csv_path=self.wsl_csv_path,
            pi_jsonl_path=self.pi_jsonl_path,
            pi_verdict_path=self.pi_verdict_path,
            expected_wsl_sha256="0" * 64,
            expected_pi_jsonl_sha256=_sha256(self.pi_jsonl_path),
            expected_pi_verdict_sha256=_sha256(self.pi_verdict_path),
            external=ALL_CONFIRMED,
        )
        self.assertFalse(result.integrity_ok)
        self.assertIn("FILE_MISSING:wsl_csv", result.integrity_reasons)
        self.assertEqual(result.diagnostic_verdict, "EXCLUDED")


class PreArmNonzeroTest(VerifierFixtureBase):
    def test_nonzero_command_before_arm_fails_the_diagnostic_verdict(self):
        self._write_csv(
            [
                _csv_row("cmd_vel", 500_000_000, linear_x=0.01, angular_z=0.0),  # before arm
                _csv_row("/hil_guard/arm", 1_000_000_000, arm_state="True"),
                _csv_row("/epuck1/state", 1_000_000_000, validity_flags=7),
                _csv_row("cmd_vel", 2_000_000_000, linear_x=0.015, angular_z=0.0),
                _csv_row("cmd_vel", 3_000_000_000, linear_x=0.0, angular_z=0.0),
                _csv_row("/epuck_bridge/status", 1_000_000_000, bridge_connected="True", rx=1),
            ]
        )
        self._write_jsonl(
            [
                {"event": "tick_applied", "wall_time": 2.0, "monotonic_time": 2.0, "linear": 0.015, "angular": 0.0},
                {"event": "tick_applied", "wall_time": 3.0, "monotonic_time": 3.0, "linear": 0.0, "angular": 0.0},
            ]
        )
        self._write_verdict()
        result = self._verify()
        self.assertTrue(result.facts["pre_arm_nonzero_command_found"])
        self.assertEqual(result.diagnostic_verdict, "FAIL")
        self.assertIn("NONZERO_COMMAND_BEFORE_EXPLICIT_ARM", result.diagnostic_reasons)


class CliContractTest(VerifierFixtureBase):
    """Exercises the real argparse-wired CLI (main()), not verify_run()
    directly -- named-argument off-by-one/omission bugs only show up
    through actual argv plumbing (the exact bug class that motivated
    hil_live_zero_state_verdict.py's own CLI test)."""

    def _cli_args(self, output_json_path):
        _clean_pulse_evidence(self)
        hashes = self._expected_hashes()
        return [
            "--run-id", self.run_id,
            "--wsl-csv", self.wsl_csv_path,
            "--pi-jsonl", self.pi_jsonl_path,
            "--pi-verdict", self.pi_verdict_path,
            "--expected-wsl-sha256", hashes["wsl"],
            "--expected-pi-jsonl-sha256", hashes["jsonl"],
            "--expected-pi-verdict-sha256", hashes["verdict"],
            "--geometry-confirmed", "true",
            "--guard-sole-publisher-confirmed", "true",
            "--no-unexpected-motion-observed", "true",
            "--robot-stayed-within-measured-area", "true",
            "--run-not-interrupted", "true",
            "--output-json", output_json_path,
        ]

    def test_output_json_is_required_by_the_cli(self):
        _clean_pulse_evidence(self)
        hashes = self._expected_hashes()
        args_without_output_json = [
            "--run-id", self.run_id,
            "--wsl-csv", self.wsl_csv_path,
            "--pi-jsonl", self.pi_jsonl_path,
            "--pi-verdict", self.pi_verdict_path,
            "--expected-wsl-sha256", hashes["wsl"],
            "--expected-pi-jsonl-sha256", hashes["jsonl"],
            "--expected-pi-verdict-sha256", hashes["verdict"],
        ]
        with self.assertRaises(SystemExit):
            main(args_without_output_json)

    def test_cli_writes_a_valid_json_report_with_the_reconciled_mismatch_count(self):
        output_json_path = os.path.join(self.tmpdir.name, "post_run_verification.json")
        exit_code = main(self._cli_args(output_json_path))
        self.assertEqual(exit_code, 0)

        with open(output_json_path, encoding="utf-8") as fh:
            report = json.load(fh)

        self.assertTrue(report["integrity_ok"])
        self.assertEqual(report["diagnostic_verdict"], "PASS")
        self.assertEqual(report["diagnostic_reasons"], [])
        self.assertIn("guarded_vs_pi_matched_pairs", report["facts"])
        self.assertIn("guarded_vs_pi_mismatched_pairs", report["facts"])


if __name__ == "__main__":
    unittest.main()
