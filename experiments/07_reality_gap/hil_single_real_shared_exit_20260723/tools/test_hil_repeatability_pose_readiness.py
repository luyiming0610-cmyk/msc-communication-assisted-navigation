#!/usr/bin/env python3
"""Regression fixtures for hil_repeatability_pose_readiness.py. Pure
in-memory row lists for the logic; a couple of small real temp files
for the two file-I/O helpers. No ROS, no physical process."""
import csv
import os
import tempfile
import time
import unittest

from analyze_ground_diagnostic import load_wsl_csv_rows
from hil_command_evidence_recorder import CommandEvidenceCsvWriter, build_row
from hil_repeatability_pose_readiness import (
    DEFAULT_MAX_POSE_SAMPLE_STALENESS_S,
    RECOMMENDED_REPEATABILITY_FLUSH_INTERVAL_S,
    STATE_TOPIC_PUBLISH_PERIOD_S,
    compute_pose_readiness_facts,
    csv_header_has_pose_columns,
    evaluate_repeatability_pose_readiness,
    is_csv_growing,
    worst_case_on_disk_pose_sample_age_s,
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


class FlushFreshnessIncompatibilityTest(unittest.TestCase):
    """Deterministic reproduction of the timing incompatibility found
    live in SRGRB_20260727_02 Trial 1 Attempt 1 (classified INVALID,
    reason POSE_READINESS_FLUSH_FRESHNESS_INCOMPATIBLE, observed age
    1.078s against a 1.0s flush interval and a 1.0s threshold).

    Uses the REAL CommandEvidenceCsvWriter (with an injected monotonic
    clock -- no real sleep, fully deterministic) and the REAL
    load_wsl_csv_rows/compute_pose_readiness_facts/
    evaluate_repeatability_pose_readiness -- not a synthetic shortcut --
    so this exercises the actual write/flush/read path.

    Mechanism: the writer flushes only when a WRITE arrives at/after
    flush_interval_s has elapsed since the last flush (not a
    background timer). Between two consecutive flushes, the freshest
    sample actually readable on disk stays fixed at the last flush's
    own row; a check happening near the end of that window sees an
    age approaching flush_interval_s itself, even under perfectly
    healthy conditions. A small, explicitly-modeled, realistic
    check-execution overhead (CSV parse time, interpreter startup,
    ordinary clock/scheduling jitter -- all present in the live
    failure, none of them a sensor/bridge fault) is added on top,
    matching the observed ~0.078s excess over the flush interval.
    """

    REALISTIC_CHECK_OVERHEAD_S = 0.08

    def _simulate_and_check(self, flush_interval_s):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "command_evidence.csv")
            writer = CommandEvidenceCsvWriter(path, flush_interval_s=flush_interval_s)
            t0 = time.monotonic()

            # Write clearly more than one flush_interval_s worth of
            # 10Hz-spaced rows (state_publisher's frozen production
            # rate) -- extra margin (not landing exactly on the
            # trigger boundary) avoids floating-point cancellation
            # error when adding a small delta to t0's real, large
            # monotonic value. Read back which row actually got
            # flushed rather than assuming it analytically, so the
            # test is robust to exactly where the real trigger lands.
            # Row timestamps (local_time_ns) are anchored to the SAME
            # base (t0) as now_monotonic below and as the freshness
            # check's own now_ns -- all three must share one time
            # base, or the computed "age" is meaningless.
            n_writes = max(2, round(flush_interval_s / STATE_TOPIC_PUBLISH_PERIOD_S) + 2)
            for i in range(1, n_writes + 1):
                logical_t = i * STATE_TOPIC_PUBLISH_PERIOD_S
                row = build_row(
                    local_time_ns=int((t0 + logical_t) * 1e9),
                    local_monotonic_ns=int((t0 + logical_t) * 1e9),
                    topic=STATE_TOPIC,
                    validity_flags=7,
                    state_x_m=0.25,
                    state_y_m=0.125,
                    state_yaw_rad=0.0,
                )
                writer.write_row(row, now_monotonic=t0 + logical_t)

            wsl_rows_before_close = load_wsl_csv_rows(path)
            writer.close()

        self.assertTrue(wsl_rows_before_close, "no row was flushed before close() -- test construction is broken")
        flushed_row_time_ns = max(r["local_time_ns"] for r in wsl_rows_before_close)

        # The worst case within this flush cycle: just before the
        # NEXT flush would be due, plus realistic check overhead.
        now_ns = flushed_row_time_ns + int((flush_interval_s - 0.001 + self.REALISTIC_CHECK_OVERHEAD_S) * 1e9)
        facts = compute_pose_readiness_facts(wsl_rows_before_close, STATE_TOPIC, GUARDED_TOPIC, UPSTREAM_TOPIC, now_ns)
        result = evaluate_repeatability_pose_readiness(
            csv_exists=True, csv_growing=True, header_has_pose_columns=True, **facts
        )
        return result, facts

    def test_flush_interval_equal_to_staleness_threshold_can_spuriously_block(self):
        result, facts = self._simulate_and_check(flush_interval_s=1.0)
        self.assertFalse(result.ok)
        self.assertTrue(any(r.startswith("LATEST_POSE_SAMPLE_STALE") for r in result.reasons))
        self.assertGreater(facts["latest_valid_pose_sample_age_s"], DEFAULT_MAX_POSE_SAMPLE_STALENESS_S)
        # Matches the magnitude actually observed live (1.078s).
        self.assertAlmostEqual(facts["latest_valid_pose_sample_age_s"], 1.079, places=2)

    def test_recommended_flush_interval_leaves_comfortable_margin(self):
        result, facts = self._simulate_and_check(flush_interval_s=RECOMMENDED_REPEATABILITY_FLUSH_INTERVAL_S)
        self.assertTrue(result.ok)
        self.assertEqual(result.reasons, ())
        self.assertLess(facts["latest_valid_pose_sample_age_s"], DEFAULT_MAX_POSE_SAMPLE_STALENESS_S / 2)

    def test_worst_case_formula_matches_the_simulation(self):
        # The documented pure-arithmetic bound (buffering only, no
        # overhead) must agree with what the real writer/reader path
        # actually produces once the added overhead is subtracted back
        # out.
        _, facts = self._simulate_and_check(flush_interval_s=1.0)
        buffering_only_age = facts["latest_valid_pose_sample_age_s"] - self.REALISTIC_CHECK_OVERHEAD_S
        self.assertAlmostEqual(buffering_only_age, worst_case_on_disk_pose_sample_age_s(1.0), places=2)


class ProductionRepeatabilitySettingsTest(unittest.TestCase):
    """Proves the selected production recorder settings
    (RECOMMENDED_REPEATABILITY_FLUSH_INTERVAL_S) repeatedly pass
    freshness across multiple check timings within a flush cycle, and
    that genuinely stale pose data (a dead sensor/bridge, not a
    buffering artifact) still correctly blocks even with the new,
    smaller flush interval."""

    def _facts_at(self, flush_interval_s, extra_wait_s):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "command_evidence.csv")
            writer = CommandEvidenceCsvWriter(path, flush_interval_s=flush_interval_s)
            t0 = time.monotonic()
            n_writes = max(2, round(flush_interval_s / STATE_TOPIC_PUBLISH_PERIOD_S) + 2)
            for i in range(1, n_writes + 1):
                logical_t = i * STATE_TOPIC_PUBLISH_PERIOD_S
                row = build_row(
                    local_time_ns=int((t0 + logical_t) * 1e9),
                    local_monotonic_ns=int((t0 + logical_t) * 1e9),
                    topic=STATE_TOPIC,
                    validity_flags=7,
                    state_x_m=0.25,
                    state_y_m=0.125,
                    state_yaw_rad=0.0,
                )
                writer.write_row(row, now_monotonic=t0 + logical_t)
            wsl_rows = load_wsl_csv_rows(path)
            writer.close()
        self.assertTrue(wsl_rows, "no row was flushed before close() -- test construction is broken")
        flushed_row_time_ns = max(r["local_time_ns"] for r in wsl_rows)
        now_ns = flushed_row_time_ns + int(extra_wait_s * 1e9)
        facts = compute_pose_readiness_facts(wsl_rows, STATE_TOPIC, GUARDED_TOPIC, UPSTREAM_TOPIC, now_ns)
        return facts

    def test_recommended_settings_pass_at_many_check_timings_within_a_cycle(self):
        # 0%, 25%, 50%, 75%, and 99% through the flush cycle, each plus
        # the same realistic check overhead used above -- all must pass.
        overhead_s = 0.08
        for fraction in (0.0, 0.25, 0.5, 0.75, 0.99):
            with self.subTest(fraction=fraction):
                extra_wait_s = fraction * RECOMMENDED_REPEATABILITY_FLUSH_INTERVAL_S + overhead_s
                facts = self._facts_at(RECOMMENDED_REPEATABILITY_FLUSH_INTERVAL_S, extra_wait_s)
                result = evaluate_repeatability_pose_readiness(
                    csv_exists=True, csv_growing=True, header_has_pose_columns=True, **facts
                )
                self.assertTrue(result.ok, f"unexpectedly blocked at fraction={fraction}: {result.reasons}")

    def test_genuinely_stale_pose_data_still_blocks_with_recommended_settings(self):
        # A dead sensor/bridge (no new samples for well beyond even a
        # generous multiple of the flush interval) must still be
        # caught -- the smaller flush interval must not silently widen
        # what counts as "fresh enough".
        facts = self._facts_at(RECOMMENDED_REPEATABILITY_FLUSH_INTERVAL_S, extra_wait_s=5.0)
        result = evaluate_repeatability_pose_readiness(
            csv_exists=True, csv_growing=True, header_has_pose_columns=True, **facts
        )
        self.assertFalse(result.ok)
        self.assertTrue(any(r.startswith("LATEST_POSE_SAMPLE_STALE") for r in result.reasons))


if __name__ == "__main__":
    unittest.main()
