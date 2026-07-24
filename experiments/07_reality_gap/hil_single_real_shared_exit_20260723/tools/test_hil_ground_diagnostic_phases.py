#!/usr/bin/env python3
"""Tests for hil_ground_diagnostic_phases.py -- the pure pass/block
decision functions behind run_ground_diagnostic_preflight.sh's two
explicit phases. No ROS/rclpy dependency, no filesystem, no network.
"""
import unittest
from datetime import datetime, timedelta, timezone

from hil_ground_diagnostic_phases import (
    evaluate_combined_gate,
    evaluate_pre_stack,
    evaluate_wsl_live_state,
)

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

ALL_WSL_LIVE_STATE_GOOD = dict(
    validity_flags=7,
    bridge_connected=True,
    wsl_csv_growing=True,
    guard_sole_publisher=True,
    guard_armed=False,
    cmd_vel_all_zero=True,
    upstream_zero_or_absent=True,
    forbidden_process_found=False,
    wsl_evidence_all_zero=True,
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
            tracked_unconfirmed=("safety.emergency_stop_position_confirmed",),
        )
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertTrue(any("TRACKED_FIELDS_NOT_READY" in r for r in result.reasons))
        self.assertTrue(any("measured_geometry.start_x_m" in r for r in result.reasons))
        self.assertTrue(any("safety.emergency_stop_position_confirmed" in r for r in result.reasons))

    def test_tracked_fields_ok_does_not_substitute_for_session_ok(self):
        # A tracked-file value (even fully confirmed) can never bypass
        # the separate per-session gate -- the two are independent
        # parameters, and both must be true for pre-stack to pass.
        kwargs = dict(ALL_PRE_STACK_GOOD, tracked_fields_ok=True, session_ok=False, session_reason="CONFIRMATIONS_NOT_TRUE")
        result = evaluate_pre_stack(**kwargs)
        self.assertFalse(result.ok)
        self.assertTrue(any("SESSION_STATE_NOT_READY" in r for r in result.reasons))

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


class EvaluateWslLiveStateTest(unittest.TestCase):
    def test_all_good_passes(self):
        result = evaluate_wsl_live_state(**ALL_WSL_LIVE_STATE_GOOD)
        self.assertTrue(result.ok)
        self.assertEqual(result.reasons, ())

    def test_has_no_pi_jsonl_parameter_at_all(self):
        # The whole point of the split: this function cannot depend on
        # a Pi-local file path -- it structurally has no such parameter.
        self.assertNotIn("pi_jsonl_growing", evaluate_wsl_live_state.__code__.co_varnames)
        self.assertNotIn("pi_evidence_all_zero", evaluate_wsl_live_state.__code__.co_varnames)

    def test_blocks_until_validity_flags_is_7(self):
        for bad_flags in (None, 0, 5, 3):
            kwargs = dict(ALL_WSL_LIVE_STATE_GOOD, validity_flags=bad_flags)
            result = evaluate_wsl_live_state(**kwargs)
            self.assertFalse(result.ok, f"flags={bad_flags} should block")
            self.assertTrue(any("VALIDITY_FLAGS_NOT_7" in r for r in result.reasons))

    def test_blocks_until_bridge_connected(self):
        kwargs = dict(ALL_WSL_LIVE_STATE_GOOD, bridge_connected=False)
        result = evaluate_wsl_live_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("BRIDGE_NOT_CONNECTED", result.reasons)

    def test_blocks_until_wsl_csv_growing(self):
        kwargs = dict(ALL_WSL_LIVE_STATE_GOOD, wsl_csv_growing=False)
        result = evaluate_wsl_live_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("WSL_CSV_NOT_GROWING", result.reasons)

    def test_blocks_unless_guard_is_sole_cmd_vel_publisher(self):
        kwargs = dict(ALL_WSL_LIVE_STATE_GOOD, guard_sole_publisher=False)
        result = evaluate_wsl_live_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("GUARD_NOT_SOLE_CMD_VEL_PUBLISHER", result.reasons)

    def test_blocks_if_guard_already_armed(self):
        kwargs = dict(ALL_WSL_LIVE_STATE_GOOD, guard_armed=True)
        result = evaluate_wsl_live_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("GUARD_ALREADY_ARMED", result.reasons)

    def test_blocks_unless_cmd_vel_all_zero(self):
        kwargs = dict(ALL_WSL_LIVE_STATE_GOOD, cmd_vel_all_zero=False)
        result = evaluate_wsl_live_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("CMD_VEL_NOT_ZERO", result.reasons)

    def test_blocks_unless_upstream_zero_or_absent(self):
        kwargs = dict(ALL_WSL_LIVE_STATE_GOOD, upstream_zero_or_absent=False)
        result = evaluate_wsl_live_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("UPSTREAM_CMD_VEL_NOT_ZERO_OR_ABSENT", result.reasons)

    def test_blocks_on_forbidden_process(self):
        kwargs = dict(ALL_WSL_LIVE_STATE_GOOD, forbidden_process_found=True)
        result = evaluate_wsl_live_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("FORBIDDEN_PROCESS_FOUND", result.reasons)

    def test_blocks_unless_wsl_evidence_all_zero(self):
        kwargs = dict(ALL_WSL_LIVE_STATE_GOOD, wsl_evidence_all_zero=False)
        result = evaluate_wsl_live_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("WSL_EVIDENCE_CONTAINS_NONZERO_COMMAND", result.reasons)

    def test_multiple_failures_all_reported(self):
        kwargs = dict(ALL_WSL_LIVE_STATE_GOOD, guard_armed=True, cmd_vel_all_zero=False)
        result = evaluate_wsl_live_state(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("GUARD_ALREADY_ARMED", result.reasons)
        self.assertIn("CMD_VEL_NOT_ZERO", result.reasons)


def _good_wsl_result():
    return evaluate_wsl_live_state(**ALL_WSL_LIVE_STATE_GOOD)


class EvaluateCombinedGateTest(unittest.TestCase):
    def _good_combined_kwargs(self, now=None):
        now = now or datetime.now(timezone.utc)
        return dict(
            wsl_result=_good_wsl_result(),
            pi_verdict_ok=True,
            pi_verdict_reasons=(),
            pi_verdict_available=True,
            wsl_run_id="run_20260724_153950",
            pi_run_id="run_20260724_153950",
            wsl_evidence_path="/home/eamon/epuck_comm_bags/first_ground_diagnostic_20260724_153950/command_evidence.csv",
            pi_evidence_path="/home/pi/real_robot_avoidance_v1/command_audit_20260724_153950.jsonl",
            pi_verdict_generated_at_utc=now.isoformat(),
            pi_verdict_max_age_s=300.0,
            now=now,
        )

    def test_passes_only_when_both_sides_pass(self):
        result = evaluate_combined_gate(**self._good_combined_kwargs())
        self.assertTrue(result.ok)
        self.assertEqual(result.reasons, ())

    def test_wsl_side_failure_blocks_combined_gate(self):
        bad_wsl = evaluate_wsl_live_state(**dict(ALL_WSL_LIVE_STATE_GOOD, bridge_connected=False))
        kwargs = dict(self._good_combined_kwargs(), wsl_result=bad_wsl)
        result = evaluate_combined_gate(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("BRIDGE_NOT_CONNECTED", result.reasons)

    def test_pi_side_failure_blocks_combined_gate_even_if_wsl_passes(self):
        kwargs = dict(
            self._good_combined_kwargs(),
            pi_verdict_ok=False,
            pi_verdict_reasons=("PI_EVIDENCE_CONTAINS_NONZERO_COMMAND",),
        )
        result = evaluate_combined_gate(**kwargs)
        self.assertFalse(result.ok)
        self.assertTrue(any("PI_EVIDENCE_CONTAINS_NONZERO_COMMAND" in r for r in result.reasons))

    def test_missing_pi_verdict_blocks_with_its_own_distinct_reason(self):
        # A missing/unreachable Pi verdict must never be silently
        # treated as a pass, and must never be labeled as "proven
        # nonzero" -- it gets its own distinct reason.
        kwargs = dict(self._good_combined_kwargs(), pi_verdict_available=False)
        result = evaluate_combined_gate(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("PI_LIVE_AUDIT_NOT_AVAILABLE", result.reasons)
        self.assertFalse(any("NONZERO" in r for r in result.reasons))

    def test_run_id_mismatch_blocks(self):
        kwargs = dict(self._good_combined_kwargs(), pi_run_id="a_different_run_id")
        result = evaluate_combined_gate(**kwargs)
        self.assertFalse(result.ok)
        self.assertTrue(any("RUN_ID_MISMATCH" in r for r in result.reasons))

    def test_stale_pi_verdict_blocks(self):
        now = datetime.now(timezone.utc)
        old_timestamp = (now - timedelta(seconds=600)).isoformat()
        kwargs = dict(self._good_combined_kwargs(now=now), pi_verdict_generated_at_utc=old_timestamp)
        result = evaluate_combined_gate(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("PI_VERDICT_STALE", result.reasons)

    def test_pi_verdict_just_under_max_age_passes(self):
        now = datetime.now(timezone.utc)
        recent_timestamp = (now - timedelta(seconds=299)).isoformat()
        kwargs = dict(self._good_combined_kwargs(now=now), pi_verdict_generated_at_utc=recent_timestamp)
        result = evaluate_combined_gate(**kwargs)
        self.assertTrue(result.ok)

    def test_missing_pi_verdict_timestamp_blocks(self):
        kwargs = dict(self._good_combined_kwargs(), pi_verdict_generated_at_utc=None)
        result = evaluate_combined_gate(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("PI_VERDICT_MISSING_TIMESTAMP", result.reasons)

    def test_unparseable_pi_verdict_timestamp_blocks(self):
        kwargs = dict(self._good_combined_kwargs(), pi_verdict_generated_at_utc="not-a-timestamp")
        result = evaluate_combined_gate(**kwargs)
        self.assertFalse(result.ok)
        self.assertIn("PI_VERDICT_UNPARSEABLE_TIMESTAMP", result.reasons)


if __name__ == "__main__":
    unittest.main()
