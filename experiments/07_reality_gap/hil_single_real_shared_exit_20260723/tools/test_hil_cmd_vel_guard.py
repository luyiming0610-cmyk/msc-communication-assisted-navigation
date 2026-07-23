#!/usr/bin/env python3
"""Unit tests for the pure hil_cmd_vel_guard decision function.

No rclpy import here on purpose -- these tests must run in any plain
Python environment, including outside a sourced ROS workspace, so the
safety-critical logic is exercised even when a full ROS graph is not
available.
"""
import unittest

from hil_cmd_vel_guard import decide_command

# Matches EpuckState's frozen bit assignments (src/epuck2_comm_interfaces/msg/EpuckState.msg):
# FLAG_ODOM_VALID=1, FLAG_IR_VALID=2, FLAG_TOF_VALID=4. Ground motion requires all
# three (value 7) -- confirmed both by cooperative_avoider.py's require_local_sensors=True
# (every N2/N3 controller launch) and by the physical baseline batch's own recorded
# validity_flags_durations_s showing 7 as the actual steady-state value on real hardware.
FLAG_ODOM_VALID = 1
FLAG_IR_VALID = 2
FLAG_TOF_VALID = 4
FLAG_ALL_VALID = FLAG_ODOM_VALID | FLAG_IR_VALID | FLAG_TOF_VALID


def _base_kwargs(**overrides):
    kwargs = dict(
        armed=True,
        target_linear_mps=0.05,
        target_angular_rps=0.3,
        now_s=100.0,
        max_linear_speed_mps=0.02,
        max_angular_speed_rps=0.2,
        last_heartbeat_at_s=99.9,
        heartbeat_timeout_s=0.5,
        last_physical_state_at_s=99.9,
        physical_state_timeout_s=0.5,
        physical_state_protocol_ok=True,
        physical_validity_flags=FLAG_ALL_VALID,
        required_validity_flags=FLAG_ALL_VALID,
        require_virtual_peer=False,
        last_virtual_peer_at_s=None,
        virtual_peer_timeout_s=1.0,
        upstream_cmd_vel_publisher_count=1,
        guarded_cmd_vel_publisher_count=1,
        guarded_publisher_is_self=True,
    )
    kwargs.update(overrides)
    return kwargs


class DefaultDisarmedTest(unittest.TestCase):
    def test_disarmed_by_default_produces_zero(self):
        decision = decide_command(**_base_kwargs(armed=False))
        self.assertEqual(decision.linear_mps, 0.0)
        self.assertEqual(decision.angular_rps, 0.0)
        self.assertIn("DISARMED", decision.blocked_reasons)
        self.assertFalse(decision.is_moving)


class AngularLimitUnconfirmedTest(unittest.TestCase):
    def test_unconfirmed_angular_limit_blocks_motion(self):
        decision = decide_command(**_base_kwargs(max_angular_speed_rps=None))
        self.assertEqual(decision.linear_mps, 0.0)
        self.assertEqual(decision.angular_rps, 0.0)
        self.assertIn("ANGULAR_SPEED_LIMIT_UNCONFIRMED", decision.blocked_reasons)


class ProtocolVersionTest(unittest.TestCase):
    def test_invalid_protocol_version_rejected(self):
        decision = decide_command(**_base_kwargs(physical_state_protocol_ok=False))
        self.assertFalse(decision.is_moving)
        self.assertIn("PHYSICAL_STATE_PROTOCOL_VERSION_MISMATCH", decision.blocked_reasons)


class PublisherCountTest(unittest.TestCase):
    def test_zero_publishers_blocks(self):
        decision = decide_command(**_base_kwargs(upstream_cmd_vel_publisher_count=0))
        self.assertFalse(decision.is_moving)
        self.assertIn("UPSTREAM_CMD_VEL_PUBLISHER_COUNT_INVALID(0)", decision.blocked_reasons)

    def test_multiple_publishers_blocks(self):
        decision = decide_command(**_base_kwargs(upstream_cmd_vel_publisher_count=2))
        self.assertFalse(decision.is_moving)
        self.assertIn("UPSTREAM_CMD_VEL_PUBLISHER_COUNT_INVALID(2)", decision.blocked_reasons)


class GuardedPublisherCountTest(unittest.TestCase):
    """Covers the FINAL, driver-facing cmd_vel topic's own publisher
    check -- distinct from the pre-guard upstream check above. A stray
    old controller, teleop node, or test script publishing directly
    onto the driver topic bypasses the guard entirely and is invisible
    to the upstream-only check, so this is checked independently."""

    def test_correct_one_upstream_one_self_guarded_allows_motion(self):
        decision = decide_command(**_base_kwargs(
            upstream_cmd_vel_publisher_count=1,
            guarded_cmd_vel_publisher_count=1,
            guarded_publisher_is_self=True,
        ))
        self.assertTrue(decision.is_moving)

    def test_zero_guarded_publishers_blocks(self):
        decision = decide_command(**_base_kwargs(
            guarded_cmd_vel_publisher_count=0,
            guarded_publisher_is_self=False,
        ))
        self.assertFalse(decision.is_moving)
        self.assertIn("GUARDED_CMD_VEL_PUBLISHER_COUNT_INVALID(0)", decision.blocked_reasons)

    def test_two_guarded_publishers_blocks(self):
        decision = decide_command(**_base_kwargs(
            guarded_cmd_vel_publisher_count=2,
            guarded_publisher_is_self=False,
        ))
        self.assertFalse(decision.is_moving)
        self.assertIn("GUARDED_CMD_VEL_PUBLISHER_COUNT_INVALID(2)", decision.blocked_reasons)

    def test_single_guarded_publisher_that_is_not_self_blocks(self):
        """Exactly one publisher exists on the driver topic, but it is
        some other node (a stray controller/teleop) instead of this
        guard -- must still block, since the guard's own commands are
        not the ones actually reaching the driver."""
        decision = decide_command(**_base_kwargs(
            guarded_cmd_vel_publisher_count=1,
            guarded_publisher_is_self=False,
        ))
        self.assertFalse(decision.is_moving)
        self.assertIn("GUARDED_CMD_VEL_PUBLISHER_NOT_SELF", decision.blocked_reasons)


class ValidityFlagsTest(unittest.TestCase):
    def test_invalid_validity_flags_forces_zero(self):
        decision = decide_command(**_base_kwargs(physical_validity_flags=0))
        self.assertEqual(decision.linear_mps, 0.0)
        self.assertEqual(decision.angular_rps, 0.0)
        self.assertIn("PHYSICAL_STATE_INVALID_FLAGS", decision.blocked_reasons)

    def test_all_three_flags_set_allows_next_safety_check(self):
        decision = decide_command(**_base_kwargs(
            physical_validity_flags=FLAG_ALL_VALID,
            required_validity_flags=FLAG_ALL_VALID,
        ))
        self.assertNotIn("PHYSICAL_STATE_INVALID_FLAGS", decision.blocked_reasons)
        self.assertTrue(decision.is_moving)

    def test_missing_odom_blocks_even_with_ir_and_tof(self):
        decision = decide_command(**_base_kwargs(
            physical_validity_flags=FLAG_IR_VALID | FLAG_TOF_VALID,
            required_validity_flags=FLAG_ALL_VALID,
        ))
        self.assertFalse(decision.is_moving)
        self.assertIn("PHYSICAL_STATE_INVALID_FLAGS", decision.blocked_reasons)

    def test_missing_ir_blocks_even_with_odom_and_tof(self):
        decision = decide_command(**_base_kwargs(
            physical_validity_flags=FLAG_ODOM_VALID | FLAG_TOF_VALID,
            required_validity_flags=FLAG_ALL_VALID,
        ))
        self.assertFalse(decision.is_moving)
        self.assertIn("PHYSICAL_STATE_INVALID_FLAGS", decision.blocked_reasons)

    def test_missing_tof_blocks_even_with_odom_and_ir(self):
        decision = decide_command(**_base_kwargs(
            physical_validity_flags=FLAG_ODOM_VALID | FLAG_IR_VALID,
            required_validity_flags=FLAG_ALL_VALID,
        ))
        self.assertFalse(decision.is_moving)
        self.assertIn("PHYSICAL_STATE_INVALID_FLAGS", decision.blocked_reasons)

    def test_zero_flags_blocks(self):
        decision = decide_command(**_base_kwargs(
            physical_validity_flags=0,
            required_validity_flags=FLAG_ALL_VALID,
        ))
        self.assertFalse(decision.is_moving)
        self.assertIn("PHYSICAL_STATE_INVALID_FLAGS", decision.blocked_reasons)


class StaleStateTest(unittest.TestCase):
    def test_stale_physical_state_forces_zero(self):
        decision = decide_command(**_base_kwargs(last_physical_state_at_s=90.0))
        self.assertFalse(decision.is_moving)
        self.assertIn("PHYSICAL_STATE_STALE_OR_MISSING", decision.blocked_reasons)

    def test_missing_physical_state_forces_zero(self):
        decision = decide_command(**_base_kwargs(last_physical_state_at_s=None))
        self.assertFalse(decision.is_moving)
        self.assertIn("PHYSICAL_STATE_STALE_OR_MISSING", decision.blocked_reasons)

    def test_stale_virtual_peer_forces_zero_when_required(self):
        decision = decide_command(**_base_kwargs(
            require_virtual_peer=True,
            last_virtual_peer_at_s=90.0,
        ))
        self.assertFalse(decision.is_moving)
        self.assertIn("VIRTUAL_PEER_STALE_OR_MISSING", decision.blocked_reasons)

    def test_virtual_peer_not_required_when_flag_false(self):
        decision = decide_command(**_base_kwargs(
            require_virtual_peer=False,
            last_virtual_peer_at_s=None,
        ))
        self.assertTrue(decision.is_moving)


class HeartbeatTest(unittest.TestCase):
    def test_heartbeat_loss_forces_zero(self):
        decision = decide_command(**_base_kwargs(last_heartbeat_at_s=90.0))
        self.assertFalse(decision.is_moving)
        self.assertIn("HEARTBEAT_STALE_OR_MISSING", decision.blocked_reasons)

    def test_missing_heartbeat_forces_zero(self):
        decision = decide_command(**_base_kwargs(last_heartbeat_at_s=None))
        self.assertFalse(decision.is_moving)
        self.assertIn("HEARTBEAT_STALE_OR_MISSING", decision.blocked_reasons)


class SpeedClampTest(unittest.TestCase):
    def test_linear_speed_over_cap_is_clamped_not_passed_through(self):
        decision = decide_command(**_base_kwargs(target_linear_mps=0.05))
        self.assertTrue(decision.is_moving)
        self.assertLessEqual(decision.linear_mps, 0.02)
        self.assertEqual(decision.linear_mps, 0.02)
        self.assertTrue(decision.clamped)

    def test_negative_linear_speed_clamped_to_zero(self):
        decision = decide_command(**_base_kwargs(target_linear_mps=-0.05))
        self.assertEqual(decision.linear_mps, 0.0)

    def test_angular_speed_clamped_to_confirmed_limit(self):
        decision = decide_command(**_base_kwargs(target_angular_rps=5.0, max_angular_speed_rps=0.2))
        self.assertEqual(decision.angular_rps, 0.2)
        self.assertTrue(decision.clamped)

    def test_angular_speed_clamped_symmetric_negative(self):
        decision = decide_command(**_base_kwargs(target_angular_rps=-5.0, max_angular_speed_rps=0.2))
        self.assertEqual(decision.angular_rps, -0.2)

    def test_within_limits_not_marked_clamped(self):
        decision = decide_command(**_base_kwargs(target_linear_mps=0.01, target_angular_rps=0.1))
        self.assertTrue(decision.is_moving)
        self.assertFalse(decision.clamped)


class HappyPathTest(unittest.TestCase):
    def test_all_checks_pass_produces_nonzero_command(self):
        decision = decide_command(**_base_kwargs(target_linear_mps=0.015, target_angular_rps=0.05))
        self.assertTrue(decision.armed_effective)
        self.assertEqual(decision.blocked_reasons, ())
        self.assertEqual(decision.linear_mps, 0.015)
        self.assertEqual(decision.angular_rps, 0.05)
        self.assertTrue(decision.is_moving)


class DefaultNoMotionWithoutAnyInputTest(unittest.TestCase):
    def test_all_defaults_unset_never_moves(self):
        decision = decide_command(
            armed=False,
            target_linear_mps=0.0,
            target_angular_rps=0.0,
            now_s=0.0,
            max_linear_speed_mps=0.02,
            max_angular_speed_rps=None,
            last_heartbeat_at_s=None,
            heartbeat_timeout_s=0.5,
            last_physical_state_at_s=None,
            physical_state_timeout_s=0.5,
            physical_state_protocol_ok=False,
            physical_validity_flags=0,
            required_validity_flags=FLAG_ODOM_VALID,
            require_virtual_peer=False,
            last_virtual_peer_at_s=None,
            virtual_peer_timeout_s=1.0,
            upstream_cmd_vel_publisher_count=0,
            guarded_cmd_vel_publisher_count=0,
            guarded_publisher_is_self=False,
        )
        self.assertFalse(decision.is_moving)
        self.assertEqual(decision.linear_mps, 0.0)
        self.assertEqual(decision.angular_rps, 0.0)


if __name__ == "__main__":
    unittest.main()
