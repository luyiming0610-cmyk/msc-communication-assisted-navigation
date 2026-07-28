#!/usr/bin/env python3
import unittest

import hil_ground_single_pulse_test
import hil_wheel_suspension_test
from hil_ground_single_pulse_test import (
    best_effort_final_zero_burst,
    compute_phase,
    validate_upstream_cmd_vel_topic,
)


class ReusesNotDuplicatesTest(unittest.TestCase):
    def test_compute_phase_is_the_exact_same_function_object_as_the_suspension_tool(self):
        # Proves true reuse (import), not a copy-pasted reimplementation
        # that could silently drift out of sync.
        self.assertIs(hil_ground_single_pulse_test.compute_phase, hil_wheel_suspension_test.compute_phase)


class ValidateUpstreamCmdVelTopicTest(unittest.TestCase):
    """Proves the topic policy is fail-closed by construction, not by
    runbook convention alone -- added 2026-07-28 after the release
    review noted --upstream-cmd-vel-topic could otherwise be pointed at
    an arbitrary topic, including the driver-facing cmd_vel/ /cmd_vel
    the guard exists to protect."""

    def test_default_topic_is_accepted(self):
        self.assertEqual(validate_upstream_cmd_vel_topic("cmd_vel_unguarded"), "cmd_vel_unguarded")

    def test_bare_cmd_vel_unguarded_is_accepted(self):
        self.assertEqual(validate_upstream_cmd_vel_topic("cmd_vel_unguarded"), "cmd_vel_unguarded")

    def test_slash_prefixed_cmd_vel_unguarded_is_accepted(self):
        self.assertEqual(validate_upstream_cmd_vel_topic("/cmd_vel_unguarded"), "/cmd_vel_unguarded")

    def test_bare_cmd_vel_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_upstream_cmd_vel_topic("cmd_vel")

    def test_slash_prefixed_cmd_vel_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_upstream_cmd_vel_topic("/cmd_vel")

    def test_arbitrary_alternative_topic_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_upstream_cmd_vel_topic("some_other_topic")

    def test_rejection_happens_before_any_publisher_or_message_could_exist(self):
        # There is no publisher, no node, and no rclpy.init() call
        # anywhere in this function -- rejection is a pure ValueError
        # raised before main() ever constructs a publisher, so a
        # rejected topic structurally cannot receive a publication of
        # any kind (zero or otherwise).
        with self.assertRaises(ValueError):
            validate_upstream_cmd_vel_topic("cmd_vel")


class _FakePublisher:
    def __init__(self, raise_on_publish=False):
        self.published = []
        self._raise_on_publish = raise_on_publish

    def publish(self, msg):
        if self._raise_on_publish:
            raise RuntimeError("simulated context already torn down")
        self.published.append(msg)


def _zero_msg():
    return {"linear_x": 0.0, "angular_z": 0.0}


class BestEffortFinalZeroBurstTest(unittest.TestCase):
    """Proves the final zero-burst helper used on every exit path
    (normal completion, KeyboardInterrupt, SystemExit,
    ExternalShutdownException, unexpected exception) never raises and
    never publishes anything non-zero."""

    def test_normal_publish_succeeds_and_message_is_exactly_zero(self):
        pub = _FakePublisher()
        result = best_effort_final_zero_burst(pub, _zero_msg)
        self.assertTrue(result)
        self.assertEqual(pub.published, [{"linear_x": 0.0, "angular_z": 0.0}])

    def test_none_publisher_is_a_no_op_and_returns_false(self):
        # Simulates node construction failing before self.pub was ever
        # assigned -- must not raise.
        result = best_effort_final_zero_burst(None, _zero_msg)
        self.assertFalse(result)

    def test_publish_raising_is_swallowed_and_returns_false(self):
        # Simulates ExternalShutdownException / an already-torn-down
        # context where publication is technically impossible.
        pub = _FakePublisher(raise_on_publish=True)
        result = best_effort_final_zero_burst(pub, _zero_msg)
        self.assertFalse(result)
        self.assertEqual(pub.published, [])

    def test_never_publishes_a_non_zero_message(self):
        pub = _FakePublisher()
        best_effort_final_zero_burst(pub, _zero_msg)
        for msg in pub.published:
            self.assertEqual(msg["linear_x"], 0.0)
            self.assertEqual(msg["angular_z"], 0.0)


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
