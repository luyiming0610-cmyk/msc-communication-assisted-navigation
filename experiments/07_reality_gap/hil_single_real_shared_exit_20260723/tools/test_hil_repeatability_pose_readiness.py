#!/usr/bin/env python3
"""Regression fixtures for hil_repeatability_pose_readiness.py. Pure
in-memory row lists for the logic; a couple of small real temp files
for the two file-I/O helpers. No ROS, no physical process."""
import csv
import os
import tempfile
import unittest

from hil_repeatability_pose_readiness import (
    compute_pose_readiness_facts,
    csv_header_has_pose_columns,
    evaluate_repeatability_pose_readiness,
    is_csv_growing,
)

STATE_TOPIC = "/epuck1/state"
GUARDED_TOPIC = "cmd_vel"
UPSTREAM_TOPIC = "cmd_vel_unguarded"


def _state_row(t_ns, x="", y="", yaw="", validity_flags=7):
    row = {"topic": STATE_TOPIC, "local_time_ns": t_ns, "validity_flags": validity_flags}
    if x != "":
        row["state_x_m"] = x
    if y != "":
        row["state_y_m"] = y
    if yaw != "":
        row["state_yaw_rad"] = yaw
    return row


def _cmd_row(topic, t_ns, linear_x=0.0, angular_z=0.0):
    return {"topic": topic, "local_time_ns": t_ns, "linear_x": linear_x, "angular_z": angular_z}


GOOD_ARGS = dict(
    csv_exists=True,
    csv_growing=True,
    header_has_pose_columns=True,
    valid_recent_pose_sample_count=2,
    timestamps_ordered=True,
    latest_valid_pose_sample_age_s=0.2,
    validity_flags=7,
    guarded_cmd_nonzero=False,
    upstream_cmd_nonzero=False,
)


class EvaluateRepeatabilityPoseReadinessTest(unittest.TestCase):
    def test_all_good_facts_pass(self):
        result = evaluate_repeatability_pose_readiness(**GOOD_ARGS)
        self.assertTrue(result.ok)
        self.assertEqual(result.reasons, ())

    def test_csv_not_found_is_the_only_reason(self):
        result = evaluate_repeatability_pose_readiness(**{**GOOD_ARGS, "csv_exists": False})
        self.assertFalse(result.ok)
        self.assertEqual(result.reasons, ("CSV_NOT_FOUND",))

    def test_missing_pose_columns_blocks(self):
        result = evaluate_repeatability_pose_readiness(**{**GOOD_ARGS, "header_has_pose_columns": False})
        self.assertFalse(result.ok)
        self.assertIn("POSE_COLUMNS_MISSING_FROM_HEADER", result.reasons)

    def test_insufficient_valid_samples_blocks(self):
        result = evaluate_repeatability_pose_readiness(**{**GOOD_ARGS, "valid_recent_pose_sample_count": 1})
        self.assertFalse(result.ok)
        self.assertTrue(any(r.startswith("INSUFFICIENT_VALID_POSE_SAMPLES") for r in result.reasons))

    def test_unordered_timestamps_blocks(self):
        result = evaluate_repeatability_pose_readiness(**{**GOOD_ARGS, "timestamps_ordered": False})
        self.assertFalse(result.ok)
        self.assertIn("POSE_SAMPLE_TIMESTAMPS_NOT_ORDERED", result.reasons)

    def test_stale_latest_sample_blocks(self):
        result = evaluate_repeatability_pose_readiness(**{**GOOD_ARGS, "latest_valid_pose_sample_age_s": 5.0})
        self.assertFalse(result.ok)
        self.assertTrue(any(r.startswith("LATEST_POSE_SAMPLE_STALE") for r in result.reasons))

    def test_no_valid_sample_at_all_blocks_with_distinct_reason(self):
        result = evaluate_repeatability_pose_readiness(**{**GOOD_ARGS, "latest_valid_pose_sample_age_s": None})
        self.assertFalse(result.ok)
        self.assertIn("NO_VALID_POSE_SAMPLE_TO_CHECK_FRESHNESS", result.reasons)

    def test_wrong_validity_flags_blocks(self):
        result = evaluate_repeatability_pose_readiness(**{**GOOD_ARGS, "validity_flags": 3})
        self.assertFalse(result.ok)
        self.assertIn("VALIDITY_FLAGS_NOT_7(got=3)", result.reasons)

    def test_guarded_nonzero_command_blocks(self):
        result = evaluate_repeatability_pose_readiness(**{**GOOD_ARGS, "guarded_cmd_nonzero": True})
        self.assertFalse(result.ok)
        self.assertIn("GUARDED_CMD_NONZERO", result.reasons)

    def test_upstream_nonzero_command_blocks(self):
        result = evaluate_repeatability_pose_readiness(**{**GOOD_ARGS, "upstream_cmd_nonzero": True})
        self.assertFalse(result.ok)
        self.assertIn("UPSTREAM_CMD_NONZERO", result.reasons)

    def test_csv_not_growing_blocks(self):
        result = evaluate_repeatability_pose_readiness(**{**GOOD_ARGS, "csv_growing": False})
        self.assertFalse(result.ok)
        self.assertIn("CSV_NOT_GROWING", result.reasons)


class ComputePoseReadinessFactsTest(unittest.TestCase):
    def test_current_valid_samples_yields_expected_facts(self):
        rows = [
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _state_row(1_200_000_000, 0.25, 0.125, 0.0),
            _cmd_row(GUARDED_TOPIC, 1_200_000_000, 0.0, 0.0),
            _cmd_row(UPSTREAM_TOPIC, 1_200_000_000, 0.0, 0.0),
        ]
        facts = compute_pose_readiness_facts(rows, STATE_TOPIC, GUARDED_TOPIC, UPSTREAM_TOPIC, now_ns=1_500_000_000)
        self.assertEqual(facts["valid_recent_pose_sample_count"], 2)
        self.assertTrue(facts["timestamps_ordered"])
        self.assertAlmostEqual(facts["latest_valid_pose_sample_age_s"], 0.3, places=6)
        self.assertEqual(facts["validity_flags"], 7)
        self.assertFalse(facts["guarded_cmd_nonzero"])
        self.assertFalse(facts["upstream_cmd_nonzero"])

    def test_blank_pose_fields_are_excluded(self):
        rows = [
            _state_row(1_000_000_000, "", "", ""),  # blank -- missing keys entirely
            _state_row(1_200_000_000, 0.25, 0.125, 0.0),
        ]
        facts = compute_pose_readiness_facts(rows, STATE_TOPIC, GUARDED_TOPIC, UPSTREAM_TOPIC, now_ns=1_300_000_000)
        self.assertEqual(facts["valid_recent_pose_sample_count"], 1)

    def test_nan_inf_pose_fields_are_excluded(self):
        rows = [
            _state_row(1_000_000_000, float("nan"), 0.125, 0.0),
            _state_row(1_100_000_000, float("inf"), 0.125, 0.0),
            _state_row(1_200_000_000, 0.25, 0.125, 0.0),
        ]
        facts = compute_pose_readiness_facts(rows, STATE_TOPIC, GUARDED_TOPIC, UPSTREAM_TOPIC, now_ns=1_300_000_000)
        self.assertEqual(facts["valid_recent_pose_sample_count"], 1)

    def test_unordered_timestamps_detected(self):
        rows = [
            _state_row(2_000_000_000, 0.25, 0.125, 0.0),
            _state_row(1_000_000_000, 0.26, 0.125, 0.0),  # out of order
        ]
        facts = compute_pose_readiness_facts(rows, STATE_TOPIC, GUARDED_TOPIC, UPSTREAM_TOPIC, now_ns=2_100_000_000)
        self.assertFalse(facts["timestamps_ordered"])

    def test_nonzero_guarded_command_detected(self):
        rows = [
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _cmd_row(GUARDED_TOPIC, 1_000_000_000, 0.015, 0.0),
        ]
        facts = compute_pose_readiness_facts(rows, STATE_TOPIC, GUARDED_TOPIC, UPSTREAM_TOPIC, now_ns=1_100_000_000)
        self.assertTrue(facts["guarded_cmd_nonzero"])

    def test_nonzero_upstream_command_detected(self):
        rows = [
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _cmd_row(UPSTREAM_TOPIC, 1_000_000_000, 0.0, 0.05),
        ]
        facts = compute_pose_readiness_facts(rows, STATE_TOPIC, GUARDED_TOPIC, UPSTREAM_TOPIC, now_ns=1_100_000_000)
        self.assertTrue(facts["upstream_cmd_nonzero"])

    def test_no_pose_columns_at_all_yields_zero_valid_samples_and_no_freshness(self):
        rows = [{"topic": STATE_TOPIC, "local_time_ns": 1_000_000_000, "validity_flags": 7}]
        facts = compute_pose_readiness_facts(rows, STATE_TOPIC, GUARDED_TOPIC, UPSTREAM_TOPIC, now_ns=2_000_000_000)
        self.assertEqual(facts["valid_recent_pose_sample_count"], 0)
        self.assertIsNone(facts["latest_valid_pose_sample_age_s"])


class CsvHeaderHasPoseColumnsTest(unittest.TestCase):
    def test_header_with_pose_columns_is_detected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "evidence.csv")
            with open(path, "w", encoding="utf-8", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(["topic", "local_time_ns", "state_x_m", "state_y_m", "state_yaw_rad"])
            self.assertTrue(csv_header_has_pose_columns(path))

    def test_header_missing_pose_columns_is_detected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "evidence.csv")
            with open(path, "w", encoding="utf-8", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(["topic", "local_time_ns", "linear_x", "angular_z"])
            self.assertFalse(csv_header_has_pose_columns(path))

    def test_empty_file_is_not_a_crash(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "evidence.csv")
            open(path, "w", encoding="utf-8").close()
            self.assertFalse(csv_header_has_pose_columns(path))


class IsCsvGrowingTest(unittest.TestCase):
    def test_unchanged_file_is_reported_not_growing(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "evidence.csv")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write("topic,local_time_ns\n")
            self.assertFalse(is_csv_growing(path, wait_s=0.05))


if __name__ == "__main__":
    unittest.main()
