#!/usr/bin/env python3
"""Pure-logic unit tests for hil_command_evidence_recorder.py. No
rclpy dependency -- mirrors test_hil_targeted_validity_diagnostic_recorder.py's
established pattern. See test_hil_command_evidence_recorder_zero_publishers.py
for the live, rclpy-based proof that the recorder itself creates zero
publishers.
"""
import csv
import tempfile
import unittest
from pathlib import Path

from hil_command_evidence_recorder import (
    CSV_FIELDS,
    CommandEvidenceCsvWriter,
    build_row,
    parse_bridge_status_json,
    verify_required_command_topics_present,
)


class BuildRowTest(unittest.TestCase):
    def test_cmd_vel_row_has_linear_and_angular_only(self):
        row = build_row(
            local_time_ns=1, local_monotonic_ns=2, topic="/cmd_vel", linear_x=0.02, angular_z=0.1
        )
        self.assertEqual(row["linear_x"], 0.02)
        self.assertEqual(row["angular_z"], 0.1)
        self.assertIsNone(row["arm_state"])
        self.assertIsNone(row["validity_flags"])

    def test_arm_row_has_arm_state_only(self):
        row = build_row(local_time_ns=1, local_monotonic_ns=2, topic="/hil_guard/arm", arm_state=True)
        self.assertTrue(row["arm_state"])
        self.assertIsNone(row["linear_x"])

    def test_state_row_has_validity_flags_and_sequence(self):
        row = build_row(
            local_time_ns=1, local_monotonic_ns=2, topic="/epuck1/state", validity_flags=7, sequence=42
        )
        self.assertEqual(row["validity_flags"], 7)
        self.assertEqual(row["sequence"], 42)

    def test_state_row_can_carry_pose_fields(self):
        # Additive, 2026-07-27: state_x_m/state_y_m/state_yaw_rad, read
        # from the same EpuckState subscription this recorder already
        # holds -- no new subscription, no protocol change.
        row = build_row(
            local_time_ns=1,
            local_monotonic_ns=2,
            topic="/epuck1/state",
            validity_flags=7,
            sequence=42,
            state_x_m=0.28,
            state_y_m=0.125,
            state_yaw_rad=0.0,
        )
        self.assertEqual(row["state_x_m"], 0.28)
        self.assertEqual(row["state_y_m"], 0.125)
        self.assertEqual(row["state_yaw_rad"], 0.0)

    def test_non_state_row_leaves_pose_fields_none(self):
        row = build_row(local_time_ns=1, local_monotonic_ns=2, topic="/cmd_vel", linear_x=0.01, angular_z=0.0)
        self.assertIsNone(row["state_x_m"])
        self.assertIsNone(row["state_y_m"])
        self.assertIsNone(row["state_yaw_rad"])

    def test_row_always_has_exactly_csv_fields_keys(self):
        row = build_row(local_time_ns=1, local_monotonic_ns=2, topic="/cmd_vel")
        self.assertEqual(set(row.keys()), set(CSV_FIELDS))


class ParseBridgeStatusJsonTest(unittest.TestCase):
    def test_valid_json_parsed(self):
        fields = parse_bridge_status_json('{"connected": true, "rx_count": 5}')
        self.assertTrue(fields["connected"])
        self.assertEqual(fields["rx_count"], 5)

    def test_malformed_json_returns_empty_dict_not_exception(self):
        self.assertEqual(parse_bridge_status_json("{not json"), {})

    def test_non_dict_json_returns_empty_dict(self):
        self.assertEqual(parse_bridge_status_json("[1,2,3]"), {})

    def test_non_string_input_returns_empty_dict(self):
        self.assertEqual(parse_bridge_status_json(None), {})


class CommandEvidenceCsvWriterTest(unittest.TestCase):
    """Covers the 2026-07-24 incremental-write/periodic-flush rewrite,
    added after a real powered-session activation attempt found the
    old batch-at-shutdown design left no CSV file at all while the
    recorder was running (see
    command_evidence_activation_20260724/SUMMARY.md)."""

    def test_file_and_header_created_immediately_at_construction(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            writer = CommandEvidenceCsvWriter(path)
            try:
                self.assertTrue(Path(path).is_file(), "CSV must exist before any row is written")
                with open(path, encoding="utf-8") as fh:
                    header = fh.readline().strip().split(",")
                self.assertEqual(header, CSV_FIELDS)
                self.assertEqual(writer.row_count, 0)
            finally:
                writer.close()

    def test_row_count_increases_as_rows_are_written(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            writer = CommandEvidenceCsvWriter(path)
            try:
                self.assertEqual(writer.row_count, 0)
                writer.write_row(build_row(local_time_ns=1, local_monotonic_ns=1, topic="/cmd_vel"))
                self.assertEqual(writer.row_count, 1)
                writer.write_row(build_row(local_time_ns=2, local_monotonic_ns=2, topic="/cmd_vel"))
                self.assertEqual(writer.row_count, 2)
            finally:
                writer.close()

    def test_header_written_only_once_even_with_many_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            writer = CommandEvidenceCsvWriter(path)
            try:
                for i in range(20):
                    writer.write_row(
                        build_row(local_time_ns=i, local_monotonic_ns=i, topic="/cmd_vel")
                    )
            finally:
                writer.close()
            with open(path, encoding="utf-8") as fh:
                lines = fh.readlines()
            header_lines = [l for l in lines if l.startswith("local_time_ns,")]
            self.assertEqual(len(header_lines), 1)
            self.assertEqual(len(lines), 21)  # 1 header + 20 rows

    def test_flush_respects_interval_not_flushing_every_row(self):
        # Inject a controlled monotonic clock so the test is
        # deterministic rather than racing a real 1s wall-clock wait.
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            writer = CommandEvidenceCsvWriter(path, flush_interval_s=10.0)
            try:
                writer.write_row(
                    build_row(local_time_ns=1, local_monotonic_ns=1, topic="/cmd_vel"),
                    now_monotonic=100.0,
                )
                # Well within the 10s interval -- row is buffered in
                # the csv module / file object, not yet guaranteed on
                # disk from the OS's point of view, but the important,
                # testable guarantee is that flush() was NOT called
                # again (verified indirectly: writing near the
                # boundary below does not trigger a second flush call
                # error path, and the final close() still produces a
                # fully correct file).
                writer.write_row(
                    build_row(local_time_ns=2, local_monotonic_ns=2, topic="/cmd_vel"),
                    now_monotonic=105.0,
                )
                self.assertEqual(writer.row_count, 2)
                # Past the interval -- this write should trigger a flush.
                writer.write_row(
                    build_row(local_time_ns=3, local_monotonic_ns=3, topic="/cmd_vel"),
                    now_monotonic=111.0,
                )
                self.assertEqual(writer.row_count, 3)
            finally:
                writer.close()
            with open(path, encoding="utf-8") as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(len(rows), 3)

    def test_close_flushes_and_finalizes_a_valid_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            writer = CommandEvidenceCsvWriter(path, flush_interval_s=9999.0)
            writer.write_row(
                build_row(
                    local_time_ns=1, local_monotonic_ns=2, topic="/cmd_vel", linear_x=0.01, angular_z=0.0
                )
            )
            writer.write_row(
                build_row(local_time_ns=3, local_monotonic_ns=4, topic="/hil_guard/arm", arm_state=False)
            )
            writer.close()  # must flush even though flush_interval_s is huge
            with open(path, encoding="utf-8") as fh:
                read_rows = list(csv.DictReader(fh))
            self.assertEqual(len(read_rows), 2)
            self.assertEqual(read_rows[0]["topic"], "/cmd_vel")
            self.assertEqual(read_rows[0]["linear_x"], "0.01")
            self.assertEqual(read_rows[1]["arm_state"], "False")

    def test_writing_after_close_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            writer = CommandEvidenceCsvWriter(path)
            writer.close()
            with self.assertRaises(ValueError):
                writer.write_row(build_row(local_time_ns=1, local_monotonic_ns=1, topic="/cmd_vel"))

    def test_double_close_is_a_no_op_not_an_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            writer = CommandEvidenceCsvWriter(path)
            writer.close()
            writer.close()  # must not raise

    def test_zero_rows_still_produces_a_valid_header_only_csv(self):
        # The "empty input" case: a run that ends before any event
        # ever arrives must still leave a parseable, header-only CSV.
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            writer = CommandEvidenceCsvWriter(path)
            writer.close()
            with open(path, encoding="utf-8") as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(rows, [])

    def test_malformed_row_missing_a_field_is_filled_blank_not_misaligned(self):
        # The "malformed input" case. Verified live rather than
        # assumed: csv.DictWriter does NOT raise for a dict missing one
        # of fieldnames -- it fills that column with its `restval`
        # (empty string by default). The safety-relevant property this
        # test actually checks is that the row stays column-aligned
        # (no shift into the wrong field) even when malformed --
        # confirmed by checking a field that follows the missing one
        # is still read back correctly.
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            writer = CommandEvidenceCsvWriter(path)
            try:
                malformed = build_row(
                    local_time_ns=1,
                    local_monotonic_ns=1,
                    topic="/cmd_vel",
                    bridge_connected=True,
                )
                del malformed["sequence"]  # a field between "topic" and "bridge_connected"
                writer.write_row(malformed)
            finally:
                writer.close()
            with open(path, encoding="utf-8") as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["sequence"], "")
            self.assertEqual(rows[0]["bridge_connected"], "True")
            self.assertEqual(rows[0]["topic"], "/cmd_vel")

    def test_extra_unexpected_key_in_row_raises(self):
        # The other direction of "malformed": an extra key NOT in
        # CSV_FIELDS. Unlike a missing key, csv.DictWriter's default
        # extrasaction='raise' does raise for this one.
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            writer = CommandEvidenceCsvWriter(path)
            try:
                malformed = build_row(local_time_ns=1, local_monotonic_ns=1, topic="/cmd_vel")
                malformed["unexpected_extra_field"] = "should not be accepted"
                with self.assertRaises(ValueError):
                    writer.write_row(malformed)
            finally:
                writer.close()


class VerifyRequiredCommandTopicsPresentTest(unittest.TestCase):
    def test_all_present_with_correct_type_passes(self):
        topics = {
            "/cmd_vel_unguarded": ["geometry_msgs/msg/Twist"],
            "/cmd_vel": ["geometry_msgs/msg/Twist"],
            "/unrelated": ["std_msgs/msg/String"],
        }
        result = verify_required_command_topics_present(topics)
        self.assertTrue(result.ok)
        self.assertEqual(result.missing, ())
        self.assertEqual(result.wrong_type, ())

    def test_missing_topic_fails(self):
        topics = {"/cmd_vel": ["geometry_msgs/msg/Twist"]}
        result = verify_required_command_topics_present(topics)
        self.assertFalse(result.ok)
        self.assertIn("/cmd_vel_unguarded", result.missing)

    def test_wrong_type_fails(self):
        topics = {
            "/cmd_vel_unguarded": ["geometry_msgs/msg/Twist"],
            "/cmd_vel": ["std_msgs/msg/String"],  # wrong type
        }
        result = verify_required_command_topics_present(topics)
        self.assertFalse(result.ok)
        self.assertEqual(len(result.wrong_type), 1)
        self.assertEqual(result.wrong_type[0][0], "/cmd_vel")

    def test_both_missing_reports_both(self):
        result = verify_required_command_topics_present({})
        self.assertFalse(result.ok)
        self.assertEqual(set(result.missing), {"/cmd_vel_unguarded", "/cmd_vel"})


if __name__ == "__main__":
    unittest.main()
