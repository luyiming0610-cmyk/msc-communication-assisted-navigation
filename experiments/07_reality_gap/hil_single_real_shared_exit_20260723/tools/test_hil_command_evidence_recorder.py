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
    build_row,
    parse_bridge_status_json,
    verify_required_command_topics_present,
    write_rows_csv,
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


class WriteRowsCsvTest(unittest.TestCase):
    def test_round_trip(self):
        rows = [
            build_row(local_time_ns=1, local_monotonic_ns=2, topic="/cmd_vel", linear_x=0.01, angular_z=0.0),
            build_row(local_time_ns=3, local_monotonic_ns=4, topic="/hil_guard/arm", arm_state=False),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            write_rows_csv(path, rows)
            with open(path, encoding="utf-8") as fh:
                read_rows = list(csv.DictReader(fh))
            self.assertEqual(len(read_rows), 2)
            self.assertEqual(read_rows[0]["topic"], "/cmd_vel")
            self.assertEqual(read_rows[0]["linear_x"], "0.01")
            self.assertEqual(read_rows[1]["arm_state"], "False")

    def test_empty_rows_still_writes_header(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "evidence.csv")
            write_rows_csv(path, [])
            with open(path, encoding="utf-8") as fh:
                header = fh.readline().strip().split(",")
            self.assertEqual(header, CSV_FIELDS)


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
