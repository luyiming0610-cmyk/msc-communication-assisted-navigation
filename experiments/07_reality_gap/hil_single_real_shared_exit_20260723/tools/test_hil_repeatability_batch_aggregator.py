#!/usr/bin/env python3
"""Regression fixtures for hil_repeatability_batch_aggregator.py.
Synthetic manifests + synthetic verification JSON files in a temp
directory -- no ROS, no real evidence, no physical process."""
import json
import os
import tempfile
import unittest

from hil_repeatability_batch_aggregator import aggregate_batch

FACTS_TEMPLATE = {
    "speed_summary": {"requested_max_linear_mps": 0.015, "guarded_max_linear_mps": 0.015, "max_abs_angular_rps": 0.0},
    "pi_command_maxima": {"max_abs_linear_applied": 0.015, "max_abs_angular_applied": 0.0},
    "guarded_pulse_durations_s": [1.95],
    "validity_flags_dropouts": {"dropout_count": 0},
    "bridge_summary": {"disconnected_sample_count": 0},
    "guarded_vs_pi_mismatched_pairs": 3,
    "motion_metrics": {
        "available": True,
        "longitudinal_displacement_m": 0.03,
        "lateral_displacement_m": 0.001,
        "final_yaw_error_rad": 0.0,
        "stop_line_clearance_m": 0.07,
    },
}


class AggregatorFixtureBase(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmpdir.cleanup)

    def _write_verification(
        self,
        name,
        run_id,
        *,
        integrity_ok=True,
        diagnostic_verdict="PASS",
        motion_metrics_required=True,
        motion_metrics_ok=True,
        file_checks=None,
        facts=None,
    ):
        path = os.path.join(self.tmpdir.name, name)
        payload = {
            "integrity_ok": integrity_ok,
            "integrity_reasons": [],
            "diagnostic_verdict": diagnostic_verdict,
            "diagnostic_reasons": [],
            "run_id": run_id,
            "motion_metrics_required": motion_metrics_required,
            "motion_metrics_ok": motion_metrics_ok,
            "file_checks": file_checks if file_checks is not None else {"wsl_csv": {"hash_matches": True}},
            "facts": facts if facts is not None else FACTS_TEMPLATE,
        }
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh)
        return path

    def _valid_attempt(self, trial, attempt, run_id):
        path = self._write_verification(f"t{trial}a{attempt}.json", run_id)
        return {
            "trial": trial,
            "attempt": attempt,
            "run_id": run_id,
            "classification": "VALID",
            "reason": None,
            "verification_json_path": path,
        }

    def _invalid_attempt(self, trial, attempt, run_id, reason="PROCEDURAL_ISSUE"):
        path = self._write_verification(f"t{trial}a{attempt}.json", run_id, diagnostic_verdict="FAIL")
        return {
            "trial": trial,
            "attempt": attempt,
            "run_id": run_id,
            "classification": "INVALID",
            "reason": reason,
            "verification_json_path": path,
        }

    def _excluded_attempt(self, trial, attempt, run_id, reason="UNEXPECTED_MOTION"):
        path = self._write_verification(f"t{trial}a{attempt}.json", run_id, diagnostic_verdict="EXCLUDED")
        return {
            "trial": trial,
            "attempt": attempt,
            "run_id": run_id,
            "classification": "EXCLUDED",
            "reason": reason,
            "verification_json_path": path,
        }


class CompleteBatchTest(AggregatorFixtureBase):
    def test_five_of_five_valid_is_batch_complete(self):
        attempts = [self._valid_attempt(t, 1, f"2026072{t}_100000") for t in range(1, 6)]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertEqual(result.batch_status, "BATCH_COMPLETE")
        self.assertEqual(result.n_valid, 5)
        self.assertEqual(result.manifest_errors, ())
        self.assertIn("longitudinal_displacement_m", result.descriptive_stats)
        self.assertEqual(result.descriptive_stats["longitudinal_displacement_m"]["n"], 5)


class IncompleteBatchTest(AggregatorFixtureBase):
    def test_three_of_five_valid_is_incomplete(self):
        attempts = [self._valid_attempt(t, 1, f"2026072{t}_100000") for t in range(1, 4)]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertEqual(result.batch_status, "INCOMPLETE_BATCH")
        self.assertEqual(result.n_valid, 3)


class RetryTest(AggregatorFixtureBase):
    def test_invalid_then_valid_retry_fills_slot(self):
        attempts = [
            self._invalid_attempt(1, 1, "20260721_100000"),
            self._valid_attempt(1, 2, "20260721_101000"),
        ] + [self._valid_attempt(t, 1, f"2026072{t}_100000") for t in range(2, 6)]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertEqual(result.batch_status, "BATCH_COMPLETE")
        self.assertEqual(result.n_valid, 5)
        self.assertEqual(result.slot_fill[1].attempt, 2)


class ExhaustedRetryBudgetTest(AggregatorFixtureBase):
    def test_three_invalid_attempts_leave_slot_unfilled_but_is_not_a_protocol_violation(self):
        attempts = [
            self._invalid_attempt(1, 1, "20260721_100000"),
            self._invalid_attempt(1, 2, "20260721_101000"),
            self._invalid_attempt(1, 3, "20260721_102000"),
        ] + [self._valid_attempt(t, 1, f"2026072{t}_100000") for t in range(2, 6)]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertEqual(result.batch_status, "INCOMPLETE_BATCH")
        self.assertIsNone(result.slot_fill[1])
        self.assertEqual(result.n_valid, 4)
        self.assertEqual(result.manifest_errors, ())

    def test_a_fourth_attempt_at_one_slot_is_a_protocol_violation(self):
        attempts = [
            self._invalid_attempt(1, 1, "20260721_100000"),
            self._invalid_attempt(1, 2, "20260721_101000"),
            self._invalid_attempt(1, 3, "20260721_102000"),
            self._invalid_attempt(1, 4, "20260721_103000"),
        ]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertEqual(result.batch_status, "BATCH_INVALID_PROTOCOL")
        self.assertTrue(any("RETRY_BUDGET_EXCEEDED" in e for e in result.manifest_errors))


class DuplicateRunIdTest(AggregatorFixtureBase):
    def test_duplicate_run_id_across_attempts_is_a_protocol_violation(self):
        attempts = [
            self._valid_attempt(1, 1, "20260721_100000"),
            self._valid_attempt(2, 1, "20260721_100000"),  # same run_id reused
        ]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertEqual(result.batch_status, "BATCH_INVALID_PROTOCOL")
        self.assertTrue(any("DUPLICATE_RUN_ID" in e for e in result.manifest_errors))


class DuplicateSlotAttemptTest(AggregatorFixtureBase):
    def test_duplicate_trial_attempt_pair_is_a_protocol_violation(self):
        attempts = [
            self._valid_attempt(1, 1, "20260721_100000"),
            self._valid_attempt(1, 1, "20260721_100001"),  # same (trial, attempt) pair
        ]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertEqual(result.batch_status, "BATCH_INVALID_PROTOCOL")
        self.assertTrue(any("DUPLICATE_TRIAL_ATTEMPT_PAIR" in e for e in result.manifest_errors))


class OmittedAttemptTest(AggregatorFixtureBase):
    def test_gap_in_attempt_numbering_is_a_protocol_violation(self):
        attempts = [
            self._invalid_attempt(1, 1, "20260721_100000"),
            self._valid_attempt(1, 3, "20260721_101000"),  # attempt 2 omitted
        ]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertEqual(result.batch_status, "BATCH_INVALID_PROTOCOL")
        self.assertTrue(any("ATTEMPT_NUMBER_GAP_OR_DISORDER" in e for e in result.manifest_errors))


class ExcludedAttemptTest(AggregatorFixtureBase):
    def test_excluded_attempt_as_final_entry_aborts_the_batch(self):
        attempts = [
            self._valid_attempt(1, 1, "20260721_100000"),
            self._excluded_attempt(2, 1, "20260721_101000"),
        ]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertEqual(result.batch_status, "BATCH_ABORTED_EXCLUDED")
        self.assertEqual(result.n_valid, 1)

    def test_attempts_continuing_after_excluded_is_a_protocol_violation(self):
        attempts = [
            self._excluded_attempt(1, 1, "20260721_100000"),
            self._valid_attempt(2, 1, "20260721_101000"),  # should never have happened
        ]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertEqual(result.batch_status, "BATCH_INVALID_PROTOCOL")
        self.assertTrue(any("ATTEMPTS_CONTINUED_AFTER_EXCLUDED_ATTEMPT" in e for e in result.manifest_errors))


class MissingMotionMetricsTest(AggregatorFixtureBase):
    def test_valid_classification_without_motion_metrics_ok_is_flagged_and_does_not_fill_the_slot(self):
        path = self._write_verification("bad.json", "20260721_100000", motion_metrics_ok=False)
        attempts = [
            {
                "trial": 1,
                "attempt": 1,
                "run_id": "20260721_100000",
                "classification": "VALID",
                "reason": None,
                "verification_json_path": path,
            }
        ]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertIsNone(result.slot_fill[1])
        outcome = result.attempt_outcomes[0]
        self.assertFalse(outcome.counts_as_valid)
        self.assertIn("VALID_ATTEMPT_MOTION_METRICS_NOT_OK", outcome.errors)


class MalformedJsonTest(AggregatorFixtureBase):
    def test_malformed_verification_json_does_not_crash_the_aggregator(self):
        path = os.path.join(self.tmpdir.name, "malformed.json")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("{not valid json")
        attempts = [
            {
                "trial": 1,
                "attempt": 1,
                "run_id": "20260721_100000",
                "classification": "VALID",
                "reason": None,
                "verification_json_path": path,
            }
        ]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        self.assertIsNone(result.slot_fill[1])
        outcome = result.attempt_outcomes[0]
        self.assertTrue(any("VERIFICATION_JSON_MALFORMED" in e for e in outcome.errors))


class RelativeVerificationPathTest(AggregatorFixtureBase):
    """verification_json_path must resolve against the manifest's own
    directory (base_dir), never the process's current working
    directory -- see hil_repeatability_batch_aggregator.py's
    _resolve_verification_path()."""

    def test_relative_path_resolves_against_base_dir(self):
        subdir = os.path.join(self.tmpdir.name, "trial1_attempt1_20260727_131437")
        os.makedirs(subdir)
        payload_path = os.path.join(subdir, "post_run_verification.json")
        with open(payload_path, "w", encoding="utf-8") as fh:
            json.dump(
                {
                    "integrity_ok": True,
                    "diagnostic_verdict": "PASS",
                    "run_id": "20260727_131437",
                    "motion_metrics_required": True,
                    "motion_metrics_ok": True,
                    "file_checks": {"a": {"hash_matches": True}},
                    "facts": FACTS_TEMPLATE,
                },
                fh,
            )
        attempts = [
            {
                "trial": 1,
                "attempt": 1,
                "run_id": "20260727_131437",
                "classification": "VALID",
                "reason": None,
                "verification_json_path": "trial1_attempt1_20260727_131437/post_run_verification.json",
            }
        ]
        result = aggregate_batch({"attempts": attempts}, base_dir=self.tmpdir.name)
        self.assertTrue(result.attempt_outcomes[0].counts_as_valid)
        self.assertEqual(result.attempt_outcomes[0].errors, ())

    def test_relative_path_resolution_is_independent_of_current_working_directory(self):
        subdir = os.path.join(self.tmpdir.name, "trial1_attempt1_20260727_131437")
        os.makedirs(subdir)
        payload_path = os.path.join(subdir, "post_run_verification.json")
        with open(payload_path, "w", encoding="utf-8") as fh:
            json.dump(
                {
                    "integrity_ok": True,
                    "diagnostic_verdict": "PASS",
                    "run_id": "20260727_131437",
                    "motion_metrics_required": True,
                    "motion_metrics_ok": True,
                    "file_checks": {"a": {"hash_matches": True}},
                    "facts": FACTS_TEMPLATE,
                },
                fh,
            )
        attempts = [
            {
                "trial": 1,
                "attempt": 1,
                "run_id": "20260727_131437",
                "classification": "VALID",
                "reason": None,
                "verification_json_path": "trial1_attempt1_20260727_131437/post_run_verification.json",
            }
        ]
        original_cwd = os.getcwd()
        elsewhere = tempfile.mkdtemp()
        try:
            os.chdir(elsewhere)
            result = aggregate_batch({"attempts": attempts}, base_dir=self.tmpdir.name)
        finally:
            os.chdir(original_cwd)
        self.assertTrue(result.attempt_outcomes[0].counts_as_valid)

    def test_missing_tracked_verification_json_is_reported_not_crashed_on(self):
        attempts = [
            {
                "trial": 1,
                "attempt": 1,
                "run_id": "20260727_131437",
                "classification": "VALID",
                "reason": None,
                "verification_json_path": "trial1_attempt1_20260727_131437/post_run_verification.json",
            }
        ]
        result = aggregate_batch({"attempts": attempts}, base_dir=self.tmpdir.name)
        outcome = result.attempt_outcomes[0]
        self.assertFalse(outcome.counts_as_valid)
        self.assertTrue(any("VERIFICATION_JSON_NOT_FOUND" in e for e in outcome.errors))
        self.assertIsNone(result.slot_fill[1])

    def test_absolute_path_is_used_as_is_regardless_of_base_dir(self):
        path = self._write_verification("abs.json", "20260727_131437")
        attempts = [
            {
                "trial": 1,
                "attempt": 1,
                "run_id": "20260727_131437",
                "classification": "VALID",
                "reason": None,
                "verification_json_path": path,
            }
        ]
        result = aggregate_batch({"attempts": attempts}, base_dir="/some/unrelated/directory")
        self.assertTrue(result.attempt_outcomes[0].counts_as_valid)


class DescriptiveStatsOnlyTest(AggregatorFixtureBase):
    def test_stats_are_descriptive_not_inferential(self):
        attempts = [self._valid_attempt(t, 1, f"2026072{t}_100000") for t in range(1, 6)]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        stats = result.descriptive_stats["longitudinal_displacement_m"]
        self.assertEqual(set(stats.keys()), {"n", "min", "max", "mean", "sample_stddev"})
        # No confidence interval, p-value, or similar inferential field exists anywhere.
        for metric_stats in result.descriptive_stats.values():
            self.assertEqual(set(metric_stats.keys()), {"n", "min", "max", "mean", "sample_stddev"})


class PerAttemptIdentityFieldsTest(AggregatorFixtureBase):
    """spec_commit/execution_code_commit are optional, per-attempt,
    purely informational identity fields -- added after a real
    identity mismatch (SRGRB_20260727_02: a field named
    "execution_head" was set to a documentation/registration commit
    instead of the actual code commit an attempt would run against).
    Must be preserved end-to-end into to_dict(), never validated for
    format, and never affect batch_status/slot-filling/stats."""

    def test_spec_commit_and_execution_code_commit_pass_through_to_dict(self):
        path = self._write_verification("t1a1.json", "20260721_100000")
        attempts = [
            {
                "trial": 1,
                "attempt": 1,
                "run_id": "20260721_100000",
                "classification": "VALID",
                "reason": None,
                "verification_json_path": path,
                "spec_commit": "4d777590d7189fedaffb105eddfd5003ea1cb40e",
                "execution_code_commit": "8515dbb0cc6dba30dfc342bb215d453ce3b6286c",
            }
        ]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        as_dict = result.to_dict()
        attempt_dict = as_dict["attempts"][0]
        self.assertEqual(attempt_dict["spec_commit"], "4d777590d7189fedaffb105eddfd5003ea1cb40e")
        self.assertEqual(attempt_dict["execution_code_commit"], "8515dbb0cc6dba30dfc342bb215d453ce3b6286c")
        # Purely informational -- a VALID attempt with these fields
        # still fills its slot exactly as without them.
        self.assertTrue(result.attempt_outcomes[0].counts_as_valid)
        self.assertEqual(result.slot_fill[1].attempt, 1)

    def test_missing_identity_fields_default_to_none_not_an_error(self):
        # Backward compatibility: an older manifest (like
        # SRGRB_20260727's, written before these fields existed) must
        # still aggregate cleanly.
        path = self._write_verification("t1a1.json", "20260721_100000")
        attempts = [
            {
                "trial": 1,
                "attempt": 1,
                "run_id": "20260721_100000",
                "classification": "VALID",
                "reason": None,
                "verification_json_path": path,
            }
        ]
        result = aggregate_batch({"batch_id": "SRGRB_TEST", "attempts": attempts})
        attempt_dict = result.to_dict()["attempts"][0]
        self.assertIsNone(attempt_dict["spec_commit"])
        self.assertIsNone(attempt_dict["execution_code_commit"])
        self.assertTrue(result.attempt_outcomes[0].counts_as_valid)


if __name__ == "__main__":
    unittest.main()
