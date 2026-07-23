#!/usr/bin/env python3
import csv
import os
import tempfile
import unittest

from hil_targeted_validity_diagnostic_recorder import (
    CSV_FIELDS,
    build_row,
    parse_bridge_status_json,
    write_rows_csv,
)


class BuildRowTest(unittest.TestCase):
    def test_state_row_has_all_fields_present_as_keys(self):
        row = build_row(
            local_time_ns=1, local_monotonic_ns=2, topic="/epuck1/state",
            stamp_sec=3, stamp_nanosec=4, validity_flags=7, sequence=99,
        )
        self.assertEqual(set(row.keys()), set(CSV_FIELDS))
        self.assertEqual(row["validity_flags"], 7)
        self.assertEqual(row["sequence"], 99)
        self.assertIsNone(row["connected"])

    def test_bridge_status_row_leaves_state_fields_none(self):
        row = build_row(
            local_time_ns=1, local_monotonic_ns=2, topic="/epuck_bridge/status",
            connected=True, rx_count=100, crc_errors=0, last_rtt_ms=8.5,
            last_state_age_s=0.02, state_missing=0, state_out_of_order=0,
        )
        self.assertIsNone(row["validity_flags"])
        self.assertIsNone(row["sequence"])
        self.assertEqual(row["connected"], True)
        self.assertEqual(row["rx_count"], 100)

    def test_plain_sensor_row_only_has_stamp(self):
        row = build_row(local_time_ns=1, local_monotonic_ns=2, topic="/tof", stamp_sec=5, stamp_nanosec=6)
        self.assertEqual(row["stamp_sec"], 5)
        self.assertIsNone(row["validity_flags"])
        self.assertIsNone(row["connected"])


class ParseBridgeStatusJsonTest(unittest.TestCase):
    def test_valid_json_parsed(self):
        data = '{"connected": true, "rx_count": 42, "crc_errors": 0, "last_rtt_ms": 7.5, "last_state_age_s": 0.01, "state_missing": 0, "state_out_of_order": 0}'
        parsed = parse_bridge_status_json(data)
        self.assertEqual(parsed["connected"], True)
        self.assertEqual(parsed["rx_count"], 42)

    def test_malformed_json_returns_empty_dict_not_exception(self):
        parsed = parse_bridge_status_json("not json{{{")
        self.assertEqual(parsed, {})

    def test_non_dict_json_returns_empty_dict(self):
        parsed = parse_bridge_status_json("[1, 2, 3]")
        self.assertEqual(parsed, {})

    def test_non_string_input_returns_empty_dict(self):
        parsed = parse_bridge_status_json(None)
        self.assertEqual(parsed, {})


class WriteRowsCsvTest(unittest.TestCase):
    def test_round_trip(self):
        rows = [
            build_row(local_time_ns=1, local_monotonic_ns=2, topic="/epuck1/state", validity_flags=7, sequence=1),
            build_row(local_time_ns=3, local_monotonic_ns=4, topic="/epuck1/state", validity_flags=0, sequence=2),
            build_row(local_time_ns=5, local_monotonic_ns=6, topic="/epuck_bridge/status", connected=True, rx_count=10),
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "out.csv")
            write_rows_csv(path, rows)
            with open(path, encoding="utf-8", newline="") as fh:
                reader = csv.DictReader(fh)
                read_rows = list(reader)
        self.assertEqual(len(read_rows), 3)
        self.assertEqual(read_rows[0]["topic"], "/epuck1/state")
        self.assertEqual(read_rows[1]["validity_flags"], "0")
        self.assertEqual(read_rows[2]["connected"], "True")

    def test_empty_rows_still_writes_header(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "empty.csv")
            write_rows_csv(path, [])
            with open(path, encoding="utf-8", newline="") as fh:
                reader = csv.DictReader(fh)
                self.assertEqual(reader.fieldnames, CSV_FIELDS)
                self.assertEqual(list(reader), [])


if __name__ == "__main__":
    unittest.main()
