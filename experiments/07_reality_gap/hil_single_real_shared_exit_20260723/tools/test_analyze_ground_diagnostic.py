#!/usr/bin/env python3
"""Pure-logic tests for analyze_ground_diagnostic.py. No rclpy
dependency, no real evidence files -- everything constructed from
synthetic rows/records.
"""
import csv
import json
import tempfile
import unittest
from pathlib import Path

from analyze_ground_diagnostic import (
    compute_average_ground_speed,
    compute_bridge_summary,
    compute_guarded_vs_pi_applied_mismatch,
    compute_odometry_displacement,
    compute_pi_command_maxima,
    compute_sha256_manifest,
    compute_speed_summary,
    compute_validity_flags_dropouts,
    evaluate_verdict,
    find_nonzero_command_window,
    load_pi_jsonl_records,
    load_wsl_csv_rows,
)

CSV_FIELDS = [
    "local_time_ns",
    "local_monotonic_ns",
    "topic",
    "linear_x",
    "angular_z",
    "arm_state",
    "validity_flags",
    "sequence",
    "bridge_connected",
    "bridge_rx_count",
]


def _write_csv(rows):
    return _write_csv_with_fields(CSV_FIELDS, rows)


def _write_csv_with_fields(fields, rows):
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False, newline="", encoding="utf-8")
    writer = csv.DictWriter(tmp, fieldnames=fields)
    writer.writeheader()
    for row in rows:
        full = {k: "" for k in fields}
        full.update(row)
        writer.writerow(full)
    tmp.close()
    return tmp.name


def _write_jsonl(records):
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".jsonl", delete=False, encoding="utf-8")
    for rec in records:
        tmp.write(json.dumps(rec) + "\n")
    tmp.close()
    return tmp.name


class LoadWslCsvRowsTest(unittest.TestCase):
    def test_numeric_fields_are_converted(self):
        path = _write_csv(
            [{"local_time_ns": 1000, "topic": "/cmd_vel", "linear_x": 0.02, "angular_z": 0.0}]
        )
        rows = load_wsl_csv_rows(path)
        self.assertEqual(rows[0]["local_time_ns"], 1000)
        self.assertEqual(rows[0]["linear_x"], 0.02)

    def test_bool_fields_are_converted(self):
        path = _write_csv([{"topic": "/hil_guard/arm", "arm_state": "True"}])
        rows = load_wsl_csv_rows(path)
        self.assertIs(rows[0]["arm_state"], True)

    def test_state_pose_fields_are_converted_when_present(self):
        # Additive columns (2026-07-27) -- a CSV that DOES carry them
        # (a new-format recorder output) must parse them as floats.
        path = _write_csv_with_fields(
            CSV_FIELDS + ["state_x_m", "state_y_m", "state_yaw_rad"],
            [
                {
                    "topic": "/epuck1/state",
                    "local_time_ns": 1000,
                    "validity_flags": 7,
                    "state_x_m": 0.28,
                    "state_y_m": 0.125,
                    "state_yaw_rad": 0.0,
                }
            ],
        )
        rows = load_wsl_csv_rows(path)
        self.assertEqual(rows[0]["state_x_m"], 0.28)
        self.assertEqual(rows[0]["state_y_m"], 0.125)
        self.assertEqual(rows[0]["state_yaw_rad"], 0.0)

    def test_old_format_csv_without_pose_columns_still_parses(self):
        # Backward compatibility: a CSV written before this change (no
        # state_x_m/state_y_m/state_yaw_rad columns at all) must still
        # load without error -- the keys simply do not appear in the
        # resulting row dicts.
        path = _write_csv([{"topic": "/epuck1/state", "local_time_ns": 1000, "validity_flags": 7}])
        rows = load_wsl_csv_rows(path)
        self.assertNotIn("state_x_m", rows[0])


class LoadPiJsonlRecordsTest(unittest.TestCase):
    def test_valid_lines_parsed(self):
        path = _write_jsonl([{"event": "tick_applied", "linear": 0.0}])
        records = load_pi_jsonl_records(path)
        self.assertEqual(records[0]["event"], "tick_applied")

    def test_malformed_line_preserved_not_silently_dropped(self):
        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".jsonl", delete=False, encoding="utf-8")
        tmp.write("not json at all\n")
        tmp.close()
        records = load_pi_jsonl_records(tmp.name)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["event"], "PARSE_ERROR")


class ComputeSpeedSummaryTest(unittest.TestCase):
    def test_reports_requested_guarded_and_angular_maxima(self):
        rows = [
            {"topic": "cmd_vel_unguarded", "linear_x": 0.015, "angular_z": 0.0},
            {"topic": "cmd_vel", "linear_x": 0.015, "angular_z": 0.0},
            {"topic": "cmd_vel", "linear_x": -0.02, "angular_z": 0.0},
        ]
        summary = compute_speed_summary(rows, "cmd_vel_unguarded", "cmd_vel")
        self.assertEqual(summary.requested_max_linear_mps, 0.015)
        self.assertEqual(summary.guarded_max_linear_mps, 0.02)
        self.assertEqual(summary.max_abs_angular_rps, 0.0)

    def test_empty_input_yields_none(self):
        summary = compute_speed_summary([], "cmd_vel_unguarded", "cmd_vel")
        self.assertIsNone(summary.requested_max_linear_mps)
        self.assertIsNone(summary.guarded_max_linear_mps)
        self.assertIsNone(summary.max_abs_angular_rps)


class FindNonzeroCommandWindowTest(unittest.TestCase):
    def test_first_last_and_duration(self):
        rows = [
            {"topic": "cmd_vel", "local_time_ns": 0, "linear_x": 0.0, "angular_z": 0.0},
            {"topic": "cmd_vel", "local_time_ns": 1_000_000_000, "linear_x": 0.015, "angular_z": 0.0},
            {"topic": "cmd_vel", "local_time_ns": 2_000_000_000, "linear_x": 0.015, "angular_z": 0.0},
            {"topic": "cmd_vel", "local_time_ns": 3_000_000_000, "linear_x": 0.0, "angular_z": 0.0},
        ]
        window = find_nonzero_command_window(rows, "cmd_vel")
        self.assertEqual(window.first_time_ns, 1_000_000_000)
        self.assertEqual(window.last_time_ns, 2_000_000_000)
        self.assertAlmostEqual(window.duration_s, 1.0)
        self.assertTrue(window.final_is_zero)

    def test_final_not_zero_detected(self):
        rows = [
            {"topic": "cmd_vel", "local_time_ns": 0, "linear_x": 0.02, "angular_z": 0.0},
        ]
        window = find_nonzero_command_window(rows, "cmd_vel")
        self.assertFalse(window.final_is_zero)

    def test_no_rows_for_topic_yields_none_fields(self):
        window = find_nonzero_command_window([], "cmd_vel")
        self.assertIsNone(window.first_time_ns)
        self.assertIsNone(window.final_is_zero)


class NotAvailableMetricsTest(unittest.TestCase):
    def test_odometry_displacement_is_not_available_with_a_stated_reason(self):
        result = compute_odometry_displacement([], [])
        self.assertTrue(result.reason)
        self.assertIn("odometry", result.reason.lower())

    def test_average_ground_speed_is_not_available_and_not_derived_from_commands(self):
        result = compute_average_ground_speed([], [])
        self.assertTrue(result.reason)
        self.assertNotIn("estimate", result.reason.lower().replace("not approximated", ""))


class ComputeValidityFlagsDropoutsTest(unittest.TestCase):
    def test_single_dropout_episode_detected(self):
        rows = [
            {"topic": "/epuck1/state", "local_time_ns": 0, "validity_flags": 7},
            {"topic": "/epuck1/state", "local_time_ns": 1_000_000_000, "validity_flags": 0},
            {"topic": "/epuck1/state", "local_time_ns": 2_000_000_000, "validity_flags": 0},
            {"topic": "/epuck1/state", "local_time_ns": 3_000_000_000, "validity_flags": 7},
        ]
        summary = compute_validity_flags_dropouts(rows, "/epuck1/state")
        self.assertEqual(summary.dropout_count, 1)
        self.assertAlmostEqual(summary.total_dropout_duration_s, 1.0)

    def test_no_dropouts_when_always_valid(self):
        rows = [
            {"topic": "/epuck1/state", "local_time_ns": i * 1_000_000_000, "validity_flags": 7} for i in range(5)
        ]
        summary = compute_validity_flags_dropouts(rows, "/epuck1/state")
        self.assertEqual(summary.dropout_count, 0)
        self.assertEqual(summary.total_dropout_duration_s, 0.0)

    def test_dropout_still_open_at_end_of_data_is_counted(self):
        rows = [
            {"topic": "/epuck1/state", "local_time_ns": 0, "validity_flags": 7},
            {"topic": "/epuck1/state", "local_time_ns": 1_000_000_000, "validity_flags": 0},
        ]
        summary = compute_validity_flags_dropouts(rows, "/epuck1/state")
        self.assertEqual(summary.dropout_count, 1)


class ComputeBridgeSummaryTest(unittest.TestCase):
    def test_connected_and_disconnected_counted(self):
        rows = [
            {"topic": "/epuck_bridge/status", "bridge_connected": True, "bridge_rx_count": 10},
            {"topic": "/epuck_bridge/status", "bridge_connected": True, "bridge_rx_count": 20},
            {"topic": "/epuck_bridge/status", "bridge_connected": False, "bridge_rx_count": 20},
        ]
        summary = compute_bridge_summary(rows, "/epuck_bridge/status")
        self.assertEqual(summary.connected_sample_count, 2)
        self.assertEqual(summary.disconnected_sample_count, 1)
        self.assertEqual(summary.max_rx_count, 20)
        self.assertTrue(summary.ever_disconnected)


class ComputePiCommandMaximaTest(unittest.TestCase):
    def test_maxima_and_nonzero_counts(self):
        records = [
            {"event": "command_received", "linear_raw": 0.015, "angular_raw": 0.0, "linear_applied_clamped": 0.015, "angular_applied_clamped": 0.0},
            {"event": "command_received", "linear_raw": 0.0, "angular_raw": 0.0, "linear_applied_clamped": 0.0, "angular_applied_clamped": 0.0},
            {"event": "tick_applied", "linear": 0.015, "angular": 0.0},
        ]
        maxima = compute_pi_command_maxima(records)
        self.assertEqual(maxima.max_abs_linear_raw, 0.015)
        self.assertEqual(maxima.max_abs_linear_applied, 0.015)
        self.assertEqual(maxima.nonzero_received_count, 1)
        # nonzero_applied_count pools TWO distinct audit record types:
        # command_received's linear_applied_clamped (logged once, at
        # parse-time) and tick_applied's linear (logged again,
        # separately, at each periodic republish) -- these are not the
        # same sample, so one nonzero command_received plus one
        # nonzero tick_applied correctly yields 2, not 1. Verified
        # live against the actual function rather than assumed.
        self.assertEqual(maxima.nonzero_applied_count, 2)

    def test_empty_records_yield_none_and_zero(self):
        maxima = compute_pi_command_maxima([])
        self.assertIsNone(maxima.max_abs_linear_raw)
        self.assertEqual(maxima.nonzero_received_count, 0)


class ComputeGuardedVsPiMismatchTest(unittest.TestCase):
    def test_matching_values_within_tolerance_report_no_mismatch(self):
        wsl_rows = [{"topic": "cmd_vel", "local_time_ns": 1_000_000_000, "linear_x": 0.015}]
        pi_records = [{"event": "tick_applied", "wall_time": 1.0, "monotonic_time": 1.0, "linear": 0.015}]
        result = compute_guarded_vs_pi_applied_mismatch(wsl_rows, pi_records, "cmd_vel")
        self.assertEqual(result.checked_pairs, 1)
        self.assertEqual(result.mismatched_pairs, ())

    def test_differing_values_are_flagged(self):
        wsl_rows = [{"topic": "cmd_vel", "local_time_ns": 1_000_000_000, "linear_x": 0.015}]
        pi_records = [{"event": "tick_applied", "wall_time": 1.0, "monotonic_time": 1.0, "linear": 0.0}]
        result = compute_guarded_vs_pi_applied_mismatch(wsl_rows, pi_records, "cmd_vel")
        self.assertEqual(len(result.mismatched_pairs), 1)

    def test_no_data_yields_zero_checked_pairs(self):
        result = compute_guarded_vs_pi_applied_mismatch([], [], "cmd_vel")
        self.assertEqual(result.checked_pairs, 0)

    def test_out_of_order_input_gives_identical_result_to_sorted_input(self):
        # The optimized (sort + bisect) implementation must not depend
        # on either input already being time-ordered.
        wsl_rows = [
            {"topic": "cmd_vel", "local_time_ns": 3_000_000_000, "linear_x": 0.015},
            {"topic": "cmd_vel", "local_time_ns": 1_000_000_000, "linear_x": 0.0},
            {"topic": "cmd_vel", "local_time_ns": 2_000_000_000, "linear_x": 0.015},
        ]
        pi_records_sorted = [
            {"event": "tick_applied", "wall_time": 1.0, "monotonic_time": 1.0, "linear": 0.0},
            {"event": "tick_applied", "wall_time": 2.0, "monotonic_time": 2.0, "linear": 0.015},
            {"event": "tick_applied", "wall_time": 3.0, "monotonic_time": 3.0, "linear": 0.015},
        ]
        pi_records_shuffled = [pi_records_sorted[2], pi_records_sorted[0], pi_records_sorted[1]]

        result_sorted = compute_guarded_vs_pi_applied_mismatch(wsl_rows, pi_records_sorted, "cmd_vel")
        result_shuffled = compute_guarded_vs_pi_applied_mismatch(wsl_rows, pi_records_shuffled, "cmd_vel")
        self.assertEqual(result_sorted.checked_pairs, result_shuffled.checked_pairs)
        self.assertEqual(set(result_sorted.mismatched_pairs), set(result_shuffled.mismatched_pairs))
        self.assertEqual(result_sorted.mismatched_pairs, ())

    def test_pulse_edge_timing_produces_expected_mismatch_shape(self):
        # Reproduces the exact shape found in RUN_ID 20260727_102033's
        # real evidence: samples right at a rising/falling edge where
        # one side has already transitioned and the other has not,
        # both values otherwise valid (0.0 or the pulse speed), never
        # anything else.
        wsl_rows = [
            {"topic": "cmd_vel", "local_time_ns": 10_000_000_000, "linear_x": 0.0},  # just before rise
            {"topic": "cmd_vel", "local_time_ns": 10_100_000_000, "linear_x": 0.015},  # after rise
        ]
        pi_records = [
            {"event": "tick_applied", "wall_time": 10.05, "monotonic_time": 500.0, "linear": 0.015},
        ]
        result = compute_guarded_vs_pi_applied_mismatch(wsl_rows, pi_records, "cmd_vel", max_time_diff_s=0.2)
        self.assertEqual(result.checked_pairs, 2)
        self.assertEqual(len(result.mismatched_pairs), 1)
        mismatched_linear_values = {m[1] for m in result.mismatched_pairs} | {m[3] for m in result.mismatched_pairs}
        self.assertEqual(mismatched_linear_values, {0.0, 0.015})

    def test_true_unexpected_mismatch_outside_pulse_is_flagged(self):
        # Unlike the edge-timing case, a mismatch far from any known
        # transition (large abs_diff, no adjacent same-value sample)
        # must still be flagged -- the optimized algorithm must not
        # suppress genuine discrepancies.
        wsl_rows = [{"topic": "cmd_vel", "local_time_ns": 50_000_000_000, "linear_x": 0.02}]
        pi_records = [{"event": "tick_applied", "wall_time": 50.0, "monotonic_time": 900.0, "linear": 0.0}]
        result = compute_guarded_vs_pi_applied_mismatch(wsl_rows, pi_records, "cmd_vel")
        self.assertEqual(result.checked_pairs, 1)
        self.assertEqual(len(result.mismatched_pairs), 1)
        self.assertAlmostEqual(result.mismatched_pairs[0][4], 0.02)


class ComputeSha256ManifestTest(unittest.TestCase):
    def test_hashes_are_deterministic_and_correct(self):
        import hashlib

        tmp = tempfile.NamedTemporaryFile(mode="w", delete=False, encoding="utf-8")
        tmp.write("hello world")
        tmp.close()
        expected = hashlib.sha256(b"hello world").hexdigest()
        manifest = compute_sha256_manifest({"f": tmp.name})
        self.assertEqual(manifest["f"], expected)

    def test_missing_file_raises_rather_than_silently_reporting_no_hash(self):
        with self.assertRaises(OSError):
            compute_sha256_manifest({"f": "/no/such/path/for/this/test.csv"})


class EvaluateVerdictTest(unittest.TestCase):
    def _all_pass_kwargs(self):
        return dict(
            required_geometry_confirmed=True,
            both_evidence_logs_active_before_arm=True,
            validity_flags_before_motion=7,
            guard_sole_publisher=True,
            any_nonzero_command_before_arm=False,
            max_abs_angular_rps_commanded=0.0,
            max_abs_angular_rps_applied=0.0,
            guarded_max_linear_mps=0.015,
            confirmed_diagnostic_linear_cap_mps=0.02,
            final_command_is_zero=True,
            robot_stayed_within_measured_area=True,
            unexpected_motion_observed=False,
            run_was_interrupted=False,
        )

    def test_all_conditions_satisfied_yields_pass(self):
        result = evaluate_verdict(**self._all_pass_kwargs())
        self.assertEqual(result.verdict, "PASS")
        self.assertEqual(result.reasons, ())

    def test_unexpected_motion_yields_excluded_not_fail(self):
        kwargs = self._all_pass_kwargs()
        kwargs["unexpected_motion_observed"] = True
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "EXCLUDED")
        self.assertIn("UNEXPECTED_MOTION_OBSERVED", result.reasons)

    def test_interrupted_run_yields_excluded(self):
        kwargs = self._all_pass_kwargs()
        kwargs["run_was_interrupted"] = True
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "EXCLUDED")

    def test_missing_evidence_yields_excluded(self):
        kwargs = self._all_pass_kwargs()
        kwargs["both_evidence_logs_active_before_arm"] = False
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "EXCLUDED")

    def test_missing_validity_flags_yields_excluded_not_fail(self):
        kwargs = self._all_pass_kwargs()
        kwargs["validity_flags_before_motion"] = None
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "EXCLUDED")

    def test_nonzero_angular_commanded_yields_fail(self):
        kwargs = self._all_pass_kwargs()
        kwargs["max_abs_angular_rps_commanded"] = 0.05
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "FAIL")
        self.assertTrue(any("ANGULAR_COMMAND_NOT_ZERO" in r for r in result.reasons))

    def test_nonzero_angular_applied_yields_fail(self):
        kwargs = self._all_pass_kwargs()
        kwargs["max_abs_angular_rps_applied"] = 0.05
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "FAIL")

    def test_linear_exceeding_cap_yields_fail(self):
        kwargs = self._all_pass_kwargs()
        kwargs["guarded_max_linear_mps"] = 0.03
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "FAIL")
        self.assertTrue(any("LINEAR_OUTPUT_EXCEEDS_CAP" in r for r in result.reasons))

    def test_final_command_not_zero_yields_fail(self):
        kwargs = self._all_pass_kwargs()
        kwargs["final_command_is_zero"] = False
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "FAIL")

    def test_nonzero_command_before_arm_yields_fail(self):
        kwargs = self._all_pass_kwargs()
        kwargs["any_nonzero_command_before_arm"] = True
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "FAIL")
        self.assertIn("NONZERO_COMMAND_BEFORE_EXPLICIT_ARM", result.reasons)

    def test_guard_not_sole_publisher_yields_fail(self):
        kwargs = self._all_pass_kwargs()
        kwargs["guard_sole_publisher"] = False
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "FAIL")

    def test_robot_left_measured_area_yields_fail(self):
        kwargs = self._all_pass_kwargs()
        kwargs["robot_stayed_within_measured_area"] = False
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "FAIL")

    def test_unconfirmed_geometry_yields_fail(self):
        kwargs = self._all_pass_kwargs()
        kwargs["required_geometry_confirmed"] = False
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "FAIL")

    def test_excluded_takes_priority_over_simultaneous_fail_conditions(self):
        kwargs = self._all_pass_kwargs()
        kwargs["unexpected_motion_observed"] = True
        kwargs["guard_sole_publisher"] = False
        result = evaluate_verdict(**kwargs)
        self.assertEqual(result.verdict, "EXCLUDED")


if __name__ == "__main__":
    unittest.main()
