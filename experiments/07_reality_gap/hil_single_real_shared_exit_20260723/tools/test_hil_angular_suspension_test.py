#!/usr/bin/env python3
import unittest

from hil_angular_suspension_test import compute_angular_phase


class ComputeAngularPhaseTest(unittest.TestCase):
    def test_zero_hold_phase(self):
        phase = compute_angular_phase(1.0, zero_hold_s=3.0, pulse_s=2.0, pulse_angular_rps=0.1, post_hold_s=3.0)
        self.assertEqual(phase.name, "ZERO_HOLD")
        self.assertEqual(phase.angular_rps, 0.0)
        self.assertFalse(phase.done)

    def test_positive_pulse_phase(self):
        phase = compute_angular_phase(4.0, zero_hold_s=3.0, pulse_s=2.0, pulse_angular_rps=0.1, post_hold_s=3.0)
        self.assertEqual(phase.name, "PULSE_ROTATE")
        self.assertEqual(phase.angular_rps, 0.1)
        self.assertFalse(phase.done)

    def test_negative_pulse_phase(self):
        phase = compute_angular_phase(4.0, zero_hold_s=3.0, pulse_s=2.0, pulse_angular_rps=-0.1, post_hold_s=3.0)
        self.assertEqual(phase.name, "PULSE_ROTATE")
        self.assertEqual(phase.angular_rps, -0.1)
        self.assertFalse(phase.done)

    def test_post_hold_phase(self):
        phase = compute_angular_phase(5.5, zero_hold_s=3.0, pulse_s=2.0, pulse_angular_rps=0.1, post_hold_s=3.0)
        self.assertEqual(phase.name, "POST_HOLD")
        self.assertEqual(phase.angular_rps, 0.0)
        self.assertFalse(phase.done)

    def test_done_phase(self):
        phase = compute_angular_phase(9.0, zero_hold_s=3.0, pulse_s=2.0, pulse_angular_rps=0.1, post_hold_s=3.0)
        self.assertEqual(phase.name, "DONE")
        self.assertEqual(phase.angular_rps, 0.0)
        self.assertTrue(phase.done)

    def test_negative_elapsed_clamped_to_zero_hold(self):
        phase = compute_angular_phase(-1.0, zero_hold_s=3.0, pulse_s=2.0, pulse_angular_rps=0.1, post_hold_s=3.0)
        self.assertEqual(phase.name, "ZERO_HOLD")

    def test_boundary_exactly_at_zero_hold_enters_pulse(self):
        phase = compute_angular_phase(3.0, zero_hold_s=3.0, pulse_s=2.0, pulse_angular_rps=0.1, post_hold_s=3.0)
        self.assertEqual(phase.name, "PULSE_ROTATE")

    def test_zero_pulse_angular_rps_never_rotates(self):
        for t in (0.0, 1.5, 3.5, 6.0, 100.0):
            phase = compute_angular_phase(t, zero_hold_s=3.0, pulse_s=2.0, pulse_angular_rps=0.0, post_hold_s=3.0)
            self.assertEqual(phase.angular_rps, 0.0)


if __name__ == "__main__":
    unittest.main()
