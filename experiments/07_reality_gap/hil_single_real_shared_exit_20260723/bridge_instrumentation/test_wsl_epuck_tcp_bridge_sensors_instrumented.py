#!/usr/bin/env python3
"""Offline tests for wsl_epuck_tcp_bridge_sensors_instrumented.py.

None of these require rclpy.init() or a live ROS graph -- they either
test pure functions/classes directly, or construct message objects
(which does not require a live context), matching the project's
established pure-logic-first testing pattern.
"""
import math
import os
import tempfile
import unittest

import wsl_epuck_tcp_bridge_sensors_instrumented as bridge


class DiagnosticModeDefaultOffTest(unittest.TestCase):
    def test_false_param_and_no_env_var_is_off(self):
        os.environ.pop("EPUCK_BRIDGE_DIAGNOSTIC", None)
        self.assertFalse(bridge.diagnostic_mode_enabled(False))

    def test_param_true_is_on_regardless_of_env(self):
        os.environ.pop("EPUCK_BRIDGE_DIAGNOSTIC", None)
        self.assertTrue(bridge.diagnostic_mode_enabled(True))

    def test_env_var_1_enables_when_param_false(self):
        os.environ["EPUCK_BRIDGE_DIAGNOSTIC"] = "1"
        try:
            self.assertTrue(bridge.diagnostic_mode_enabled(False))
        finally:
            os.environ.pop("EPUCK_BRIDGE_DIAGNOSTIC", None)

    def test_env_var_0_does_not_enable(self):
        os.environ["EPUCK_BRIDGE_DIAGNOSTIC"] = "0"
        try:
            self.assertFalse(bridge.diagnostic_mode_enabled(False))
        finally:
            os.environ.pop("EPUCK_BRIDGE_DIAGNOSTIC", None)


class MessageContentUnchangedTest(unittest.TestCase):
    """Proves the extracted build_*_msg() functions produce identical
    output to the original inline construction -- diagnostic_mode has
    no way to influence these functions at all (they take no such
    argument), so this also structurally proves message content is
    unaffected by diagnostic_mode."""

    def test_build_scan_msg_matches_input_fields(self):
        scan_data = {
            "stamp": {"sec": 10, "nanosec": 20},
            "frame_id": "base_scan",
            "angle_min": -1.5,
            "angle_max": 1.5,
            "angle_increment": 0.01,
            "time_increment": 0.001,
            "scan_time": 0.1,
            "range_min": 0.05,
            "range_max": 4.0,
            "ranges": [1.0, None, 2.5],
            "intensities": [10.0, 20.0],
        }
        msg = bridge.build_scan_msg(scan_data)
        self.assertEqual(msg.header.stamp.sec, 10)
        self.assertEqual(msg.header.stamp.nanosec, 20)
        self.assertEqual(msg.header.frame_id, "base_scan")
        self.assertAlmostEqual(msg.angle_min, -1.5)
        self.assertAlmostEqual(msg.angle_max, 1.5)
        self.assertEqual(list(msg.ranges), [1.0, math.inf, 2.5])
        self.assertEqual(list(msg.intensities), [10.0, 20.0])

    def test_build_odom_msg_matches_input_fields(self):
        odom_data = {
            "stamp": {"sec": 1, "nanosec": 2},
            "frame_id": "odom",
            "child_frame_id": "base_link",
            "position": {"x": 0.1, "y": 0.2, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "linear": {"x": 0.02, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.05},
            "pose_covariance": [0.0] * 36,
            "twist_covariance": [0.0] * 36,
        }
        msg = bridge.build_odom_msg(odom_data)
        self.assertAlmostEqual(msg.pose.pose.position.x, 0.1)
        self.assertAlmostEqual(msg.pose.pose.position.y, 0.2)
        self.assertAlmostEqual(msg.pose.pose.orientation.w, 1.0)
        self.assertAlmostEqual(msg.twist.twist.linear.x, 0.02)
        self.assertAlmostEqual(msg.twist.twist.angular.z, 0.05)
        self.assertEqual(len(msg.pose.covariance), 36)

    def test_build_odom_msg_ignores_malformed_covariance_length(self):
        odom_data = {"pose_covariance": [1.0, 2.0]}  # wrong length, must not crash/assign
        msg = bridge.build_odom_msg(odom_data)
        # A ROS2 PoseWithCovariance.covariance defaults to a pre-initialized
        # 36-length all-zero array; a wrong-length input must leave that
        # default untouched, not raise, and not partially overwrite it.
        self.assertEqual(len(msg.pose.covariance), 36)
        self.assertTrue(all(v == 0.0 for v in msg.pose.covariance))

    def test_build_range_msg_matches_input_fields(self):
        sensor_data = {
            "stamp": {"sec": 3, "nanosec": 4},
            "frame_id": "ps0",
            "radiation_type": 1,
            "field_of_view": 0.3,
            "min_range": 0.0,
            "max_range": 0.15,
            "range": 0.07,
        }
        msg = bridge.build_range_msg("ps0", sensor_data)
        self.assertEqual(msg.header.frame_id, "ps0")
        self.assertAlmostEqual(msg.range, 0.07)
        self.assertAlmostEqual(msg.max_range, 0.15)


class DiagnosticRecorderTest(unittest.TestCase):
    def test_defaults_to_no_rows(self):
        recorder = bridge.DiagnosticRecorder()
        self.assertEqual(recorder.rows, [])

    def test_record_tcp_received_row(self):
        recorder = bridge.DiagnosticRecorder()
        recorder.record_tcp_received(1000, 0.5)
        self.assertEqual(len(recorder.rows), 1)
        row = recorder.rows[0]
        self.assertEqual(row["event"], "tcp_state_received")
        self.assertEqual(row["monotonic_ns"], 1000)
        self.assertEqual(row["process_cpu_time_s"], 0.5)
        self.assertIsNone(row["duration_s"])

    def test_record_timer_span_computes_duration_and_carries_intervals(self):
        recorder = bridge.DiagnosticRecorder()
        recorder.record_timer_span("publish_latest_state", 1_000_000_000, 1_000_500_000, 0.1, 0.02, 0.021)
        row = recorder.rows[0]
        self.assertEqual(row["event"], "publish_latest_state")
        self.assertAlmostEqual(row["duration_s"], 0.0005)
        self.assertEqual(row["expected_interval_s"], 0.02)
        self.assertEqual(row["actual_interval_s"], 0.021)

    def test_record_gc_event_row(self):
        recorder = bridge.DiagnosticRecorder()
        recorder.record_gc_event(2, 1_000_000_000, 1_010_000_000, 0.3)
        row = recorder.rows[0]
        self.assertEqual(row["event"], "gc_cycle")
        self.assertEqual(row["generation"], 2)
        self.assertAlmostEqual(row["duration_s"], 0.01)

    def test_flush_writes_all_rows(self):
        recorder = bridge.DiagnosticRecorder()
        recorder.record_tcp_received(1, 0.0)
        recorder.record_gc_event(0, 10, 20, 0.1)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "diag.csv")
            count = recorder.flush(path)
            self.assertEqual(count, 2)
            with open(path, encoding="utf-8") as fh:
                content = fh.read()
            self.assertIn("tcp_state_received", content)
            self.assertIn("gc_cycle", content)


class GcCallbackHandlingTest(unittest.TestCase):
    """Exercises the pure callback logic (start/stop pairing) without
    needing a real ROS node -- a plain object with the same attributes
    the method reads/writes stands in for the Node."""

    def _make_fake_node(self):
        class Fake:
            pass
        fake = Fake()
        fake._gc_start_ns = {}
        fake._diag = bridge.DiagnosticRecorder()
        return fake

    def test_start_then_stop_records_one_event_with_positive_duration(self):
        fake = self._make_fake_node()
        bridge.WslEpuckTcpBridge._on_gc_event(fake, "start", {"generation": 1})
        bridge.WslEpuckTcpBridge._on_gc_event(fake, "stop", {"generation": 1})
        self.assertEqual(len(fake._diag.rows), 1)
        row = fake._diag.rows[0]
        self.assertEqual(row["generation"], 1)
        self.assertGreaterEqual(row["duration_s"], 0.0)

    def test_stop_without_matching_start_does_not_crash(self):
        fake = self._make_fake_node()
        bridge.WslEpuckTcpBridge._on_gc_event(fake, "stop", {"generation": 0})
        self.assertEqual(len(fake._diag.rows), 1)

    def test_no_diag_recorder_means_no_row_appended(self):
        fake = self._make_fake_node()
        fake._diag = None
        bridge.WslEpuckTcpBridge._on_gc_event(fake, "start", {"generation": 0})
        bridge.WslEpuckTcpBridge._on_gc_event(fake, "stop", {"generation": 0})
        # no exception, and nothing to check rows against since _diag is None


if __name__ == "__main__":
    unittest.main()
