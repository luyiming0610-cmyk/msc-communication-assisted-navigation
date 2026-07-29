#!/usr/bin/env python3
"""Tests for hil_offline_stage3_evidence_recorder.py -- pure schema/writer
tests (no ROS needed) plus a structural zero-publisher proof."""
from __future__ import annotations

import csv
import json
import unittest
from pathlib import Path

from hil_offline_stage3_evidence_recorder import (
    CSV_FIELDS,
    GATE_DECISION_EVENT_ROW_TOPIC,
    PHASE_EVENT_ROW_TOPIC,
    REQUIRED_TOPIC_ARGS,
    Stage3EvidenceCsvWriter,
    Stage3RunSummary,
    build_row,
    write_summary_json,
)


class BuildRowTest(unittest.TestCase):
    def test_row_always_has_exactly_csv_fields_keys(self):
        row = build_row(local_time_ns=1, local_monotonic_ns=2, topic="t")
        self.assertEqual(set(row.keys()), set(CSV_FIELDS))

    def test_unset_fields_are_none_not_zero_or_empty_string(self):
        row = build_row(local_time_ns=1, local_monotonic_ns=2, topic="t")
        for key in CSV_FIELDS:
            if key in ("local_time_ns", "local_monotonic_ns", "topic"):
                continue
            self.assertIsNone(row[key], key)

    def test_own_state_fields_round_trip(self):
        row = build_row(
            local_time_ns=1, local_monotonic_ns=2, topic="/hil_offline_stage3/epuck1/state",
            validity_flags=7, sequence=5, robot_id=1, source=0, x_m=1.0, y_m=2.0, yaw_rad=0.3,
        )
        self.assertEqual(row["validity_flags"], 7)
        self.assertEqual(row["sequence"], 5)
        self.assertEqual(row["x_m"], 1.0)

    def test_extra_unexpected_key_raises(self):
        with self.assertRaises(TypeError):
            build_row(local_time_ns=1, local_monotonic_ns=2, topic="t", not_a_real_field=1)

    def test_phase_event_fields_round_trip_under_fixed_topic_name(self):
        row = build_row(
            local_time_ns=1, local_monotonic_ns=2, topic=PHASE_EVENT_ROW_TOPIC,
            phase="PEER_GATE_CLOSED", gate_open=False, adoption_confirmed=True,
            duplicate_sent=True, duplicate_rejected=False, guard_blocked_reasons="none",
        )
        self.assertEqual(row["topic"], "PHASE_EVENT")
        self.assertEqual(row["phase"], "PEER_GATE_CLOSED")
        self.assertFalse(row["gate_open"])
        self.assertTrue(row["duplicate_sent"])
        self.assertFalse(row["duplicate_rejected"])


    def test_gate_decision_event_fields_round_trip_under_fixed_topic_name(self):
        row = build_row(
            local_time_ns=1, local_monotonic_ns=2, topic=GATE_DECISION_EVENT_ROW_TOPIC,
            gate_decision_event_type="GATE_DECISION", gate_decision_gate_epoch=2,
            gate_decision_gate_state="OPEN", gate_decision_source_protocol_version=1,
            gate_decision_source_robot_id=3, gate_decision_source_sequence=42,
            gate_decision_source_production_stamp_s=1.5, gate_decision_decision="FORWARDED",
            gate_decision_decision_timestamp_s=1.6, gate_decision_first_source_after_reopen=True,
            gate_decision_forwarded_destination_topic="/hil_offline_stage3/x",
        )
        self.assertEqual(row["topic"], "GATE_DECISION_EVENT")
        self.assertEqual(row["gate_decision_gate_epoch"], 2)
        self.assertEqual(row["gate_decision_decision"], "FORWARDED")
        self.assertTrue(row["gate_decision_first_source_after_reopen"])
        self.assertEqual(row["gate_decision_forwarded_destination_topic"], "/hil_offline_stage3/x")

    def test_gate_decision_topic_is_a_required_topic_arg(self):
        self.assertIn("gate_decision_topic", REQUIRED_TOPIC_ARGS)


class Stage3EvidenceCsvWriterTest(unittest.TestCase):
    def test_file_and_header_created_immediately_at_construction(self, tmp_path=None):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            path = str(Path(d) / "evidence.csv")
            writer = Stage3EvidenceCsvWriter(path)
            with open(path, encoding="utf-8") as f:
                header = f.readline().strip().split(",")
            self.assertEqual(header, CSV_FIELDS)
            writer.close()

    def test_row_count_increases_and_close_finalizes_valid_csv(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            path = str(Path(d) / "evidence.csv")
            writer = Stage3EvidenceCsvWriter(path, flush_interval_s=1000.0)
            writer.write_row(build_row(local_time_ns=1, local_monotonic_ns=1, topic="a"))
            writer.write_row(build_row(local_time_ns=2, local_monotonic_ns=2, topic="b"))
            self.assertEqual(writer.row_count, 2)
            writer.close()
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 2)

    def test_double_close_is_a_no_op(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            writer = Stage3EvidenceCsvWriter(str(Path(d) / "evidence.csv"))
            writer.close()
            writer.close()  # must not raise

    def test_write_after_close_raises(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            writer = Stage3EvidenceCsvWriter(str(Path(d) / "evidence.csv"))
            writer.close()
            with self.assertRaises(ValueError):
                writer.write_row(build_row(local_time_ns=1, local_monotonic_ns=1, topic="a"))


class SummaryJsonTest(unittest.TestCase):
    def test_summary_round_trips_through_json(self):
        import tempfile
        summary = Stage3RunSummary(
            start_wall_time_ns=1, end_wall_time_ns=2, ros_domain_id=91,
            topic_contract={"own_state_topic": "/hil_offline_stage3/epuck1/state"},
            row_count_by_topic={"/hil_offline_stage3/epuck1/state": 10},
            recorder_health_ok=True,
        )
        with tempfile.TemporaryDirectory() as d:
            path = str(Path(d) / "summary.json")
            write_summary_json(path, summary)
            with open(path, encoding="utf-8") as f:
                loaded = json.load(f)
        self.assertEqual(loaded["ros_domain_id"], 91)
        self.assertEqual(loaded["row_count_by_topic"]["/hil_offline_stage3/epuck1/state"], 10)


class NoPublisherStructuralTest(unittest.TestCase):
    def test_module_source_contains_no_create_publisher_call(self):
        source = Path(__file__).with_name("hil_offline_stage3_evidence_recorder.py").read_text(encoding="utf-8")
        self.assertNotIn("create_publisher", source)

    def test_module_never_references_a_production_topic_string_literal(self):
        source = Path(__file__).with_name("hil_offline_stage3_evidence_recorder.py").read_text(encoding="utf-8")
        for topic in ("\"/cmd_vel\"", "\"/cmd_vel_unguarded\"", "\"/epuck1/state\"",
                      "\"/epuck_bridge/status\"", "\"/hil_guard/arm\""):
            self.assertNotIn(topic, source)


if __name__ == "__main__":
    unittest.main()
