#!/usr/bin/env python3
"""Tests for hil_ground_diagnostic_phases.py -- the pure pass/block
decision functions behind run_ground_diagnostic_preflight.sh's two
explicit phases. No ROS/rclpy dependency, no filesystem, no network.
"""
import unittest

from hil_ground_diagnostic_phases import evaluate_live_zero_state, evaluate_pre_stack

ALL_PRE_STACK_GOOD = dict(
    tracked_git_clean=True,
    tracked_fields_ok=True,
    session_ok=True,
    device_reachable=True,
    residual_process_found=False,
    forbidden_process_found=False,
    cmd_vel_publisher_count=None,
    evidence_paths_ok=True,
)

ALL_LIVE_ZERO_STATE_GOOD = dict(
    validity_flags=7,
    bridge_connected=True,
    wsl_csv_growing=True,
    pi_jsonl_growing=True,
    guard_sole_publisher=True,
    guard_armed=False,
    cmd_vel_all_zero=True,
    upstream_zero_or_absent=True,
    forbidden_process_found=False,
    wsl_evidence_all_zero=True,
    pi_evidence_all_zero=True,
)


class EvaluatePreStackTest(unittest.TestCase):
    def test_all_good_passes(self):
        result = evaluate_pre_stack(**ALL_PRE_STACK_GOOD)
        self.assertTrue(result.ok)
        self.assertEqual(result.reasons, ())

    def test_can_pass_with_no_validity_flags_bridge_or_guard_information_at_all(self):
        # The whole point of the split: this function has no parameter
        # for validity_flags, bridge status, evidence growth, or guard
        # state -- it is structurally impossible for it to require any
        # of them, so PRE_STACK can pass before the physical stack (and
        # therefore any ROS topic that stack would publish) exists.
        result = evaluate_pre_stack(**ALL_PRE_STACK_GOOD)
        self.assertTrue(result.ok)
        self.assertNotIn("validity_flags", evaluate_pre_stack.__code__.co_varnames)
        self.assertNotIn("bridge_connected", evaluate_pre_stack.__code__.co_varnames)
        self.assertNotIn("guard_armed", evaluate_pre_stack.__code__.co_varnames)

    def test_dirty_tree_blocks(self):
        kwargs = dict(ALL_PRE_STACK_GOOD, tracked_git_clean=False)
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("TRACKED_TREE_DIRTY", result.reasons)

    def test_tracked_fields_not_ok_blocks_with_details(self):
        kwargs = dict(
            ALL_PRE_STACK_GOOD,
            tracked_fields_ok=False,
            tracked_missing=("measured_geometry.start_x_m",),
            tracked_unconfirmed=("environment.floor_condition_confirmed",),
        )
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertTrue(any("TRACKED_FIELDS_NOT_READY" in r for r in result.reasons))
        self.assertTrue(any("measured_geometry.start_x_m" in r for r in result.reasons))
        self.assertTrue(any("environment.floor_condition_confirmed" in r for r in result.reasons))

    def test_session_not_ok_blocks(self):
        kwargs = dict(ALL_PRE_STACK_GOOD, session_ok=False, session_reason="SESSION_STALE")
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertTrue(any("SESSION_STATE_NOT_READY" in r and "SESSION_STALE" in r for r in result.reasons))

    def test_device_unreachable_blocks(self):
        kwargs = dict(ALL_PRE_STACK_GOOD, device_reachable=False)
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("DEVICE_UNREACHABLE", result.reasons)

    def test_residual_process_blocks(self):
        kwargs = dict(ALL_PRE_STACK_GOOD, residual_process_found=True)
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("RESIDUAL_PROCESS_FOUND", result.reasons)

    def test_forbidden_process_blocks(self):
        kwargs = dict(ALL_PRE_STACK_GOOD, forbidden_process_found=True)
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("FORBIDDEN_PROCESS_FOUND", result.reasons)

    def test_cmd_vel_publisher_present_blocks(self):
        kwargs = dict(ALL_PRE_STACK_GOOD, cmd_vel_publisher_count=1)
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertTrue(any("CMD_VEL_ALREADY_HAS_PUBLISHER" in r for r in result.reasons))

    def test_cmd_vel_publisher_count_zero_or_none_does_not_block(self):
        for count in (None, 0):
            kwargs = dict(ALL_PRE_STACK_GOOD, cmd_vel_publisher_count=count)
            result = evaluate_pre_stack(**kwargs)
            self.assertTrue(result.ok, f"count={count} should not block")

    def test_evidence_paths_not_ok_blocks(self):
        kwargs = dict(ALL_PRE_STACK_GOOD, evidence_paths_ok=False)
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("EVIDENCE_PATHS_NOT_FRESH", result.reasons)

    def test_multiple_failures_all_reported(self):
        kwargs = dict(ALL_PRE_STACK_GOOD, tracked_git_clean=False, device_reachable=False)
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("TRACKED_TREE_DIRTY", result.reasons)
        self.assertIn("DEVICE_UNREACHABLE", result.reasons)


class EvaluateLiveZeroStateTest(unittest.TestCase):
    def test_all_good_passes(self):
        result = evaluate_live_zero_state(**ALL_LIVE_ZERO_STATE_GOOD)
        self.assertTrue(result.ok)
        self.assertEqual(result.reasons, ())

    def test_blocks_until_validity_flags_is_7(self):
        for bad_flags in (None, 0, 5, 3):
            kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, validity_flags=bad_flags)
            result = evaluate_live_zero_state(**kwargs)
            self.assertFalse(result.ok, f"flags={bad_flags} should block")
            self.assertTrue(any("VALIDITY_FLAGS_NOT_7" in r for r in result.reasons))

    def test_blocks_until_bridge_connected(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, bridge_connected=False)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("BRIDGE_NOT_CONNECTED", result.reasons)

    def test_blocks_until_wsl_csv_growing(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, wsl_csv_growing=False)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("WSL_CSV_NOT_GROWING", result.reasons)

    def test_blocks_until_pi_jsonl_growing(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, pi_jsonl_growing=False)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("PI_JSONL_NOT_GROWING", result.reasons)

    def test_blocks_unless_guard_is_sole_cmd_vel_publisher(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, guard_sole_publisher=False)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("GUARD_NOT_SOLE_CMD_VEL_PUBLISHER", result.reasons)

    def test_blocks_if_guard_already_armed(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, guard_armed=True)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("GUARD_ALREADY_ARMED", result.reasons)

    def test_blocks_unless_cmd_vel_all_zero(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, cmd_vel_all_zero=False)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("CMD_VEL_NOT_ZERO", result.reasons)

    def test_blocks_unless_upstream_zero_or_absent(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, upstream_zero_or_absent=False)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("UPSTREAM_CMD_VEL_NOT_ZERO_OR_ABSENT", result.reasons)

    def test_blocks_on_forbidden_process(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, forbidden_process_found=True)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("FORBIDDEN_PROCESS_FOUND", result.reasons)

    def test_blocks_unless_wsl_evidence_all_zero(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, wsl_evidence_all_zero=False)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("WSL_EVIDENCE_CONTAINS_NONZERO_COMMAND", result.reasons)

    def test_blocks_unless_pi_evidence_all_zero(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, pi_evidence_all_zero=False)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("PI_EVIDENCE_CONTAINS_NONZERO_COMMAND", result.reasons)

    def test_multiple_failures_all_reported(self):
        kwargs = dict(ALL_LIVE_ZERO_STATE_GOOD, guard_armed=True, cmd_vel_all_zero=False)
        result = evaluate_live_zero_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("GUARD_ALREADY_ARMED", result.reasons)
        self.assertIn("CMD_VEL_NOT_ZERO", result.reasons)


if __name__ == "__main__":
    unittest.main()
