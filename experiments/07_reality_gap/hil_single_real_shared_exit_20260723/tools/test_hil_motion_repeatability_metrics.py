#!/usr/bin/env python3
"""Regression fixtures for hil_motion_repeatability_metrics.py. Pure
in-memory row lists -- no ROS, no files, no physical process."""
import math
import unittest

from hil_motion_repeatability_metrics import compute_motion_metrics, extract_valid_state_samples

STATE_TOPIC = "/epuck1/state"
GUARDED_TOPIC = "cmd_vel"


def _state_row(t_ns, x, y, yaw):
    return {"topic": STATE_TOPIC, "local_time_ns": t_ns, "state_x_m": x, "state_y_m": y, "state_yaw_rad": yaw}


def _cmd_row(t_ns, linear_x, angular_z=0.0):
    return {"topic": GUARDED_TOPIC, "local_time_ns": t_ns, "linear_x": linear_x, "angular_z": angular_z}


def _one_pulse_rows(pulse_start_ns=2_000_000_000, pulse_end_ns=4_000_000_000):
    return [
        _cmd_row(1_000_000_000, 0.0),
        _cmd_row(pulse_start_ns, 0.015),
        _cmd_row(pulse_end_ns, 0.015),
        _cmd_row(pulse_end_ns + 100_000_000, 0.0),
    ]


class ExactForwardMotionTest(unittest.TestCase):
    def test_exact_0p03m_forward_motion(self):
        rows = _one_pulse_rows() + [
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _state_row(4_500_000_000, 0.28, 0.125, 0.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC, stop_line_distance_m=0.10)
        self.assertTrue(result.available)
        self.assertAlmostEqual(result.longitudinal_displacement_m, 0.03, places=6)
        self.assertAlmostEqual(result.lateral_displacement_m, 0.0, places=6)
        self.assertAlmostEqual(result.final_yaw_error_rad, 0.0, places=6)
        self.assertAlmostEqual(result.stop_line_clearance_m, 0.07, places=6)


class LateralDriftTest(unittest.TestCase):
    def test_positive_lateral_drift(self):
        rows = _one_pulse_rows() + [
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _state_row(4_500_000_000, 0.28, 0.145, 0.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertAlmostEqual(result.lateral_displacement_m, 0.02, places=6)

    def test_negative_lateral_drift(self):
        rows = _one_pulse_rows() + [
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _state_row(4_500_000_000, 0.28, 0.105, 0.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertAlmostEqual(result.lateral_displacement_m, -0.02, places=6)


class YawWrapTest(unittest.TestCase):
    def test_yaw_wraps_correctly_across_pi_boundary(self):
        # Start just below +pi, end just above -pi -- the true change
        # is small (crossing the wrap boundary), not close to 2*pi.
        rows = _one_pulse_rows() + [
            _state_row(1_000_000_000, 0.25, 0.125, 3.0),
            _state_row(4_500_000_000, 0.28, 0.125, -3.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertTrue(result.available)
        expected = (-3.0 - 3.0 + math.pi) % (2 * math.pi) - math.pi
        self.assertAlmostEqual(result.final_yaw_error_rad, expected, places=6)
        self.assertLess(abs(result.final_yaw_error_rad), math.pi)


class MissingStateSamplesTest(unittest.TestCase):
    def test_no_pose_columns_at_all_is_not_available(self):
        rows = _one_pulse_rows() + [
            {"topic": STATE_TOPIC, "local_time_ns": 1_000_000_000, "validity_flags": 7},
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertFalse(result.available)
        self.assertEqual(result.reason, "NO_POSE_SAMPLES_AVAILABLE")

    def test_extract_valid_state_samples_excludes_incomplete_rows(self):
        rows = [
            {"topic": STATE_TOPIC, "local_time_ns": 1, "state_x_m": 0.1, "state_y_m": 0.1},  # missing yaw
            {"topic": STATE_TOPIC, "local_time_ns": 2, "state_x_m": 0.1, "state_y_m": 0.1, "state_yaw_rad": 0.0},
        ]
        samples = extract_valid_state_samples(rows, STATE_TOPIC)
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].local_time_ns, 2)


class DuplicateTimestampTest(unittest.TestCase):
    def test_duplicate_timestamps_at_pulse_start_pick_the_later_listed_sample_deterministically(self):
        pulse_start_ns = 2_000_000_000
        rows = _one_pulse_rows(pulse_start_ns=pulse_start_ns) + [
            _state_row(pulse_start_ns, 0.25, 0.125, 0.0),
            _state_row(pulse_start_ns, 0.26, 0.125, 0.0),  # duplicate local_time_ns, different pose
            _state_row(4_500_000_000, 0.28, 0.125, 0.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertTrue(result.available)
        # Deterministic: the last-listed sample at that exact timestamp
        # is the one used as the start reference (0.26, not 0.25).
        self.assertAlmostEqual(result.longitudinal_displacement_m, 0.02, places=6)

    def test_duplicate_timestamps_is_deterministic_across_repeated_calls(self):
        pulse_start_ns = 2_000_000_000
        rows = _one_pulse_rows(pulse_start_ns=pulse_start_ns) + [
            _state_row(pulse_start_ns, 0.25, 0.125, 0.0),
            _state_row(pulse_start_ns, 0.26, 0.125, 0.0),
            _state_row(4_500_000_000, 0.28, 0.125, 0.0),
        ]
        first = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        second = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertEqual(first, second)


class NoPulseTest(unittest.TestCase):
    def test_no_pulse_is_not_available(self):
        rows = [
            _cmd_row(1_000_000_000, 0.0),
            _cmd_row(2_000_000_000, 0.0),
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _state_row(2_000_000_000, 0.25, 0.125, 0.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertFalse(result.available)
        self.assertEqual(result.reason, "NO_PULSE_DETECTED")


class MultiplePulsesTest(unittest.TestCase):
    def test_multiple_pulses_is_not_available(self):
        rows = [
            _cmd_row(1_000_000_000, 0.0),
            _cmd_row(2_000_000_000, 0.015),
            _cmd_row(3_000_000_000, 0.0),
            _cmd_row(4_000_000_000, 0.015),
            _cmd_row(5_000_000_000, 0.0),
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _state_row(5_500_000_000, 0.28, 0.125, 0.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertFalse(result.available)
        self.assertEqual(result.reason, "MULTIPLE_PULSES_DETECTED")


class FinalStateBeforePulseEndTest(unittest.TestCase):
    def test_final_state_sample_before_pulse_end_is_not_available(self):
        rows = _one_pulse_rows(pulse_start_ns=2_000_000_000, pulse_end_ns=6_000_000_000) + [
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            # Last pose sample arrives BEFORE the pulse's final nonzero
            # command -- the evidence does not actually cover the full
            # motion.
            _state_row(3_000_000_000, 0.27, 0.125, 0.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertFalse(result.available)
        self.assertEqual(result.reason, "FINAL_STATE_SAMPLE_BEFORE_PULSE_END")


class BackwardDisplacementTest(unittest.TestCase):
    def test_backward_displacement_is_reported_not_rejected(self):
        rows = _one_pulse_rows() + [
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _state_row(4_500_000_000, 0.22, 0.125, 0.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertTrue(result.available)
        self.assertAlmostEqual(result.longitudinal_displacement_m, -0.03, places=6)


class NonFiniteValuesTest(unittest.TestCase):
    def test_nan_pose_sample_is_excluded_not_crashed_on(self):
        rows = _one_pulse_rows() + [
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _state_row(3_000_000_000, float("nan"), 0.125, 0.0),
            _state_row(4_500_000_000, 0.28, 0.125, 0.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertTrue(result.available)
        self.assertAlmostEqual(result.longitudinal_displacement_m, 0.03, places=6)

    def test_inf_pose_sample_is_excluded_not_crashed_on(self):
        rows = [
            _state_row(1_000_000_000, 0.25, 0.125, 0.0),
            _state_row(2_000_000_000, float("inf"), 0.125, 0.0),
        ]
        samples = extract_valid_state_samples(rows, STATE_TOPIC)
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].local_time_ns, 1_000_000_000)


class StaleStartSampleTest(unittest.TestCase):
    def test_start_sample_far_before_pulse_start_is_rejected(self):
        rows = _one_pulse_rows(pulse_start_ns=5_000_000_000, pulse_end_ns=7_000_000_000) + [
            # Only pose sample is 3s before the pulse starts -- beyond
            # the default 1.0s staleness bound.
            _state_row(2_000_000_000, 0.25, 0.125, 0.0),
            _state_row(7_500_000_000, 0.28, 0.125, 0.0),
        ]
        result = compute_motion_metrics(rows, STATE_TOPIC, GUARDED_TOPIC)
        self.assertFalse(result.available)
        self.assertEqual(result.reason, "START_SAMPLE_TOO_STALE")


if __name__ == "__main__":
    unittest.main()
