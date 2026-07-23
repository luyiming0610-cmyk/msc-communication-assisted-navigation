#!/usr/bin/env python3
import unittest

from hil_wheel_suspension_test import compute_phase


class ComputePhaseTest(unittest.TestCase):
    def test_zero_hold_phase(self):
        phase = compute_phase(1.0, zero_hold_s=3.0, pulse_s=2.0, pulse_linear_mps=0.015, post_hold_s=3.0)
        self.assertEqual(phase.name, "ZERO_HOLD")
        self.assertEqual(phase.linear_mps, 0.0)
        self.assertFalse(phase.done)

    def test_pulse_phase(self):
        phase = compute_phase(4.0, zero_hold_s=3.0, pulse_s=2.0, pulse_linear_mps=0.015, post_hold_s=3.0)
        self.assertEqual(phase.name, "PULSE_FORWARD")
        self.assertEqual(phase.linear_mps, 0.015)
        self.assertFalse(phase.done)

    def test_post_hold_phase(self):
        phase = compute_phase(5.5, zero_hold_s=3.0, pulse_s=2.0, pulse_linear_mps=0.015, post_hold_s=3.0)
        self.assertEqual(phase.name, "POST_HOLD")
        self.assertEqual(phase.linear_mps, 0.0)
        self.assertFalse(phase.done)

    def test_done_phase(self):
        phase = compute_phase(9.0, zero_hold_s=3.0, pulse_s=2.0, pulse_linear_mps=0.015, post_hold_s=3.0)
        self.assertEqual(phase.name, "DONE")
        self.assertEqual(phase.linear_mps, 0.0)
        self.assertTrue(phase.done)

    def test_negative_elapsed_clamped_to_zero_hold(self):
        phase = compute_phase(-1.0, zero_hold_s=3.0, pulse_s=2.0, pulse_linear_mps=0.015, post_hold_s=3.0)
        self.assertEqual(phase.name, "ZERO_HOLD")

    def test_boundary_exactly_at_zero_hold_enters_pulse(self):
        phase = compute_phase(3.0, zero_hold_s=3.0, pulse_s=2.0, pulse_linear_mps=0.015, post_hold_s=3.0)
        self.assertEqual(phase.name, "PULSE_FORWARD")

    def test_zero_pulse_linear_mps_never_moves(self):
        for t in (0.0, 1.5, 3.5, 6.0, 100.0):
            phase = compute_phase(t, zero_hold_s=3.0, pulse_s=2.0, pulse_linear_mps=0.0, post_hold_s=3.0)
            self.assertEqual(phase.linear_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
