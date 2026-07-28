#!/usr/bin/env python3
import unittest

import hil_ground_single_pulse_test
import hil_wheel_suspension_test
from hil_ground_single_pulse_test import compute_phase


class ReusesNotDuplicatesTest(unittest.TestCase):
    def test_compute_phase_is_the_exact_same_function_object_as_the_suspension_tool(self):
        # Proves true reuse (import), not a copy-pasted reimplementation
        # that could silently drift out of sync.
        self.assertIs(hil_ground_single_pulse_test.compute_phase, hil_wheel_suspension_test.compute_phase)


class ComputePhaseTest(unittest.TestCase):
    """Mirrors test_hil_wheel_suspension_test.py's exact phase-boundary
    coverage, using this new field's frozen pulse duration (0.10m /
    0.015m/s = 6.67s) instead of the old field's 2.0s."""

    def test_zero_hold_phase(self):
        phase = compute_phase(0.5, zero_hold_s=1.0, pulse_s=6.67, pulse_linear_mps=0.015, post_hold_s=1.0)
        self.assertEqual(phase.name, "ZERO_HOLD")
        self.assertEqual(phase.linear_mps, 0.0)
        self.assertFalse(phase.done)

    def test_pulse_phase(self):
        phase = compute_phase(4.0, zero_hold_s=1.0, pulse_s=6.67, pulse_linear_mps=0.015, post_hold_s=1.0)
        self.assertEqual(phase.name, "PULSE_FORWARD")
        self.assertEqual(phase.linear_mps, 0.015)
        self.assertFalse(phase.done)

    def test_post_hold_phase(self):
        phase = compute_phase(8.0, zero_hold_s=1.0, pulse_s=6.67, pulse_linear_mps=0.015, post_hold_s=1.0)
        self.assertEqual(phase.name, "POST_HOLD")
        self.assertEqual(phase.linear_mps, 0.0)
        self.assertFalse(phase.done)

    def test_done_phase(self):
        phase = compute_phase(9.0, zero_hold_s=1.0, pulse_s=6.67, pulse_linear_mps=0.015, post_hold_s=1.0)
        self.assertEqual(phase.name, "DONE")
        self.assertEqual(phase.linear_mps, 0.0)
        self.assertTrue(phase.done)

    def test_negative_elapsed_clamped_to_zero_hold(self):
        phase = compute_phase(-1.0, zero_hold_s=1.0, pulse_s=6.67, pulse_linear_mps=0.015, post_hold_s=1.0)
        self.assertEqual(phase.name, "ZERO_HOLD")

    def test_boundary_exactly_at_zero_hold_enters_pulse(self):
        phase = compute_phase(1.0, zero_hold_s=1.0, pulse_s=6.67, pulse_linear_mps=0.015, post_hold_s=1.0)
        self.assertEqual(phase.name, "PULSE_FORWARD")

    def test_zero_pulse_linear_mps_never_moves(self):
        for t in (0.0, 0.5, 4.0, 8.0, 100.0):
            phase = compute_phase(t, zero_hold_s=1.0, pulse_s=6.67, pulse_linear_mps=0.0, post_hold_s=1.0)
            self.assertEqual(phase.linear_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
