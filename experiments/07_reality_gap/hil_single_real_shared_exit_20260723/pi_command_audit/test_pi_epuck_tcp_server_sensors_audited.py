#!/usr/bin/env python3
"""Unit tests for pi_epuck_tcp_server_sensors_audited.py's pure
command-audit logic (parse_and_clamp_command, compute_zero_reason, the
build_*_record functions, and CommandAuditSink).

None of these tests import rclpy or open a socket -- the audit-record
contract is deliberately pure/testable without a ROS graph, mirroring
hil_cmd_vel_guard.py's decide_command() pattern elsewhere in this
project. Covers: normal command, clamped command, force-zero
(disconnect), watchdog-zero (stale timeout), reconnect, and malformed
command handling, per the 2026-07-23 UNEXPECTED_PHYSICAL_MOTION
incident audits' requirement for a Pi-side command-evidence chain.
"""
import json
import math
import os
import tempfile
import unittest

from pi_epuck_tcp_server_sensors_audited import (
    CommandAuditSink,
    ParsedCommand,
    build_command_received_record,
    build_command_rejected_record,
    build_socket_connected_record,
    build_socket_disconnected_record,
    build_tick_applied_record,
    compute_zero_reason,
    parse_and_clamp_command,
)


class ParseAndClampCommandTest(unittest.TestCase):
    def test_normal_command_within_limits_is_unclamped(self):
        parsed = parse_and_clamp_command(
            {"type": "cmd_vel", "linear_x": 0.02, "angular_z": 0.5, "seq": 7},
            max_linear_mps=0.04,
            max_angular_rps=2.0,
        )
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.linear_raw, 0.02)
        self.assertEqual(parsed.linear_applied, 0.02)
        self.assertEqual(parsed.angular_raw, 0.5)
        self.assertEqual(parsed.angular_applied, 0.5)
        self.assertEqual(parsed.seq, 7)
        self.assertFalse(parsed.clamped)

    def test_command_exceeding_linear_limit_is_clamped(self):
        parsed = parse_and_clamp_command(
            {"type": "cmd_vel", "linear_x": 5.0, "angular_z": 0.0, "seq": 1},
            max_linear_mps=0.04,
            max_angular_rps=2.0,
        )
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.linear_raw, 5.0)
        self.assertEqual(parsed.linear_applied, 0.04)
        self.assertTrue(parsed.clamped)

    def test_command_exceeding_negative_angular_limit_is_clamped(self):
        parsed = parse_and_clamp_command(
            {"type": "cmd_vel", "linear_x": 0.0, "angular_z": -9.0, "seq": 1},
            max_linear_mps=0.04,
            max_angular_rps=2.0,
        )
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.angular_applied, -2.0)
        self.assertTrue(parsed.clamped)

    def test_missing_seq_defaults_to_negative_one(self):
        parsed = parse_and_clamp_command(
            {"type": "cmd_vel", "linear_x": 0.0, "angular_z": 0.0},
            max_linear_mps=0.04,
            max_angular_rps=2.0,
        )
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.seq, -1)

    def test_wrong_message_type_is_rejected(self):
        parsed = parse_and_clamp_command(
            {"type": "state", "linear_x": 0.0, "angular_z": 0.0},
            max_linear_mps=0.04,
            max_angular_rps=2.0,
        )
        self.assertIsNone(parsed)

    def test_non_numeric_linear_is_rejected(self):
        parsed = parse_and_clamp_command(
            {"type": "cmd_vel", "linear_x": "not_a_number", "angular_z": 0.0},
            max_linear_mps=0.04,
            max_angular_rps=2.0,
        )
        self.assertIsNone(parsed)

    def test_non_finite_linear_is_rejected(self):
        parsed = parse_and_clamp_command(
            {"type": "cmd_vel", "linear_x": math.inf, "angular_z": 0.0},
            max_linear_mps=0.04,
            max_angular_rps=2.0,
        )
        self.assertIsNone(parsed)

    def test_nan_angular_is_rejected(self):
        parsed = parse_and_clamp_command(
            {"type": "cmd_vel", "linear_x": 0.0, "angular_z": math.nan},
            max_linear_mps=0.04,
            max_angular_rps=2.0,
        )
        self.assertIsNone(parsed)

    def test_non_dict_payload_is_rejected(self):
        parsed = parse_and_clamp_command(
            "not a dict", max_linear_mps=0.04, max_angular_rps=2.0
        )
        self.assertIsNone(parsed)


class ComputeZeroReasonTest(unittest.TestCase):
    def test_disconnected_client_is_reported(self):
        reason = compute_zero_reason(
            client_connected=False,
            last_command_monotonic=100.0,
            now_monotonic=100.1,
            command_timeout_s=0.5,
            linear=0.0,
            angular=0.0,
        )
        self.assertEqual(reason, "DISCONNECTED")

    def test_never_received_any_command_is_reported(self):
        # Connected, but last_command_monotonic still at its initial 0.0.
        reason = compute_zero_reason(
            client_connected=True,
            last_command_monotonic=0.0,
            now_monotonic=100.0,
            command_timeout_s=0.5,
            linear=0.0,
            angular=0.0,
        )
        self.assertEqual(reason, "NEVER_RECEIVED")

    def test_watchdog_stale_timeout_is_reported(self):
        reason = compute_zero_reason(
            client_connected=True,
            last_command_monotonic=100.0,
            now_monotonic=100.6,  # 0.6s > 0.5s timeout
            command_timeout_s=0.5,
            linear=0.0,
            angular=0.0,
        )
        self.assertEqual(reason, "WATCHDOG_STALE_TIMEOUT")

    def test_fresh_zero_command_is_a_genuine_commanded_zero_not_a_safety_zero(self):
        reason = compute_zero_reason(
            client_connected=True,
            last_command_monotonic=100.0,
            now_monotonic=100.1,
            command_timeout_s=0.5,
            linear=0.0,
            angular=0.0,
        )
        self.assertEqual(reason, "COMMANDED_ZERO")

    def test_fresh_nonzero_command_has_no_zero_reason(self):
        reason = compute_zero_reason(
            client_connected=True,
            last_command_monotonic=100.0,
            now_monotonic=100.1,
            command_timeout_s=0.5,
            linear=0.02,
            angular=0.0,
        )
        self.assertIsNone(reason)

    def test_reconnect_sequence_transitions_disconnected_then_never_received(self):
        """Simulates the exact state sequence _set_client_connected(False)
        then a fresh accept() produces: DISCONNECTED while down, then
        NEVER_RECEIVED immediately after reconnect until the first new
        command arrives -- proving a reconnect can never silently
        replay the previous connection's last command."""
        disconnected_reason = compute_zero_reason(
            client_connected=False,
            last_command_monotonic=0.0,  # zeroed by _set_client_connected(False)
            now_monotonic=200.0,
            command_timeout_s=0.5,
            linear=0.0,
            angular=0.0,
        )
        self.assertEqual(disconnected_reason, "DISCONNECTED")

        just_reconnected_reason = compute_zero_reason(
            client_connected=True,
            last_command_monotonic=0.0,  # no command received on the new connection yet
            now_monotonic=200.05,
            command_timeout_s=0.5,
            linear=0.0,
            angular=0.0,
        )
        self.assertEqual(just_reconnected_reason, "NEVER_RECEIVED")


class BuildRecordFunctionsTest(unittest.TestCase):
    def test_command_received_record_contains_raw_and_applied_values(self):
        parsed = ParsedCommand(
            linear_raw=5.0,
            angular_raw=0.0,
            linear_applied=0.04,
            angular_applied=0.0,
            seq=3,
            clamped=True,
        )
        record = build_command_received_record(
            wall_time=1.0, monotonic_time=2.0, connection_id=1, parsed=parsed
        )
        self.assertEqual(record["event"], "command_received")
        self.assertEqual(record["linear_raw"], 5.0)
        self.assertEqual(record["linear_applied_clamped"], 0.04)
        self.assertTrue(record["clamped"])
        self.assertEqual(record["connection_id"], 1)

    def test_tick_applied_record_shape(self):
        record = build_tick_applied_record(
            wall_time=1.0,
            monotonic_time=2.0,
            connection_id=1,
            linear=0.01,
            angular=0.0,
            zero_reason=None,
        )
        self.assertEqual(record["event"], "tick_applied")
        self.assertIsNone(record["zero_reason"])

    def test_rejected_record_shape(self):
        record = build_command_rejected_record(
            wall_time=1.0, monotonic_time=2.0, connection_id=1, reason="unparseable_or_non_finite"
        )
        self.assertEqual(record["event"], "command_rejected_malformed")
        self.assertEqual(record["reason"], "unparseable_or_non_finite")

    def test_socket_connected_and_disconnected_record_shapes(self):
        connected = build_socket_connected_record(
            wall_time=1.0, monotonic_time=2.0, connection_id=1, peer="10.0.0.5:12345"
        )
        self.assertEqual(connected["event"], "socket_connected")
        self.assertEqual(connected["peer"], "10.0.0.5:12345")

        disconnected = build_socket_disconnected_record(
            wall_time=3.0, monotonic_time=4.0, connection_id=1, reason="peer_closed"
        )
        self.assertEqual(disconnected["event"], "socket_disconnected")
        self.assertEqual(disconnected["reason"], "peer_closed")

    def test_all_records_are_json_serializable(self):
        parsed = ParsedCommand(0.0, 0.0, 0.0, 0.0, 1, False)
        for record in (
            build_command_received_record(
                wall_time=1.0, monotonic_time=2.0, connection_id=1, parsed=parsed
            ),
            build_command_rejected_record(
                wall_time=1.0, monotonic_time=2.0, connection_id=1, reason="x"
            ),
            build_tick_applied_record(
                wall_time=1.0,
                monotonic_time=2.0,
                connection_id=1,
                linear=0.0,
                angular=0.0,
                zero_reason="COMMANDED_ZERO",
            ),
            build_socket_connected_record(
                wall_time=1.0, monotonic_time=2.0, connection_id=1, peer="x"
            ),
            build_socket_disconnected_record(
                wall_time=1.0, monotonic_time=2.0, connection_id=1, reason="x"
            ),
        ):
            json.dumps(record)  # must not raise


class CommandAuditSinkTest(unittest.TestCase):
    def test_disabled_sink_never_creates_a_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "audit.jsonl")
            sink = CommandAuditSink(path, enabled=False)
            self.assertFalse(sink.enabled)
            sink.write({"event": "should_not_be_written"})
            sink.close()
            self.assertFalse(os.path.exists(path))

    def test_enabled_sink_appends_one_json_line_per_record(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "audit.jsonl")
            sink = CommandAuditSink(path, enabled=True)
            self.assertTrue(sink.enabled)
            sink.write({"event": "a", "n": 1})
            sink.write({"event": "b", "n": 2})
            sink.close()

            with open(path, encoding="utf-8") as fh:
                lines = [json.loads(line) for line in fh]
            self.assertEqual(len(lines), 2)
            self.assertEqual(lines[0]["event"], "a")
            self.assertEqual(lines[1]["n"], 2)

    def test_enabled_sink_with_no_path_is_treated_as_disabled(self):
        sink = CommandAuditSink(None, enabled=True)
        self.assertFalse(sink.enabled)
        sink.write({"event": "should_not_raise"})
        sink.close()

    def test_sink_never_mutates_or_returns_the_record_it_was_given(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "audit.jsonl")
            sink = CommandAuditSink(path, enabled=True)
            record = {"event": "a", "n": 1}
            original = dict(record)
            result = sink.write(record)
            sink.close()
            self.assertIsNone(result, "write() must not generate/return a command")
            self.assertEqual(record, original, "write() must never mutate its input")


class CommandAuditSinkCannotGenerateCommandsTest(unittest.TestCase):
    def test_command_audit_sink_class_has_no_rclpy_or_publisher_capability(self):
        # Static, belt-and-braces check alongside
        # test_sink_never_mutates_or_returns_the_record_it_was_given:
        # CommandAuditSink is a plain file-writer with no way to reach
        # rclpy, a socket, or _cmd_pub at all.
        import inspect

        source = inspect.getsource(CommandAuditSink)
        self.assertNotIn("create_publisher", source)
        self.assertNotIn("rclpy", source)
        self.assertNotIn("socket", source)
        self.assertNotIn("_cmd_pub", source)


if __name__ == "__main__":
    unittest.main()
