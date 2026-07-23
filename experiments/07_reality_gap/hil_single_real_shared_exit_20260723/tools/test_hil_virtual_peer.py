#!/usr/bin/env python3
import math
import unittest

from hil_virtual_peer import (
    VirtualNavPlan,
    VirtualState,
    distance_to,
    plan_virtual_command,
    step_virtual_state,
)


class EpuckStateFieldNamesTest(unittest.TestCase):
    """Regression test for a real bug found during the 2026-07-23 Phase 1
    architecture audit: hil_virtual_peer.py's _tick() assigned
    msg.linear_speed_mps/msg.angular_speed_rps (fields that do not
    exist on EpuckState -- the real fields are linear_velocity_mps/
    angular_velocity_rps) and msg.source = "virtual" (a string, but
    `source` is uint8 -- must be EpuckState.SOURCE_VIRTUAL). Both would
    have raised at the first real run; neither was ever caught before
    because hil_virtual_peer.py had never actually been executed.
    Constructs the message exactly as _tick() now does and asserts it
    round-trips correctly, so a future rename of these fields fails
    this test immediately instead of failing silently until first use."""

    def test_epuck_state_fields_used_by_tick_are_real_and_correctly_typed(self):
        from epuck2_comm_interfaces.msg import EpuckState

        msg = EpuckState()
        msg.version = EpuckState.PROTOCOL_VERSION
        msg.robot_id = 99
        msg.sequence = 1
        msg.source = EpuckState.SOURCE_VIRTUAL
        msg.x_m = 0.1
        msg.y_m = 0.2
        msg.yaw_rad = 0.3
        msg.linear_velocity_mps = 0.02
        msg.angular_velocity_rps = 0.05
        msg.validity_flags = EpuckState.FLAG_ODOM_VALID

        self.assertEqual(msg.source, EpuckState.SOURCE_VIRTUAL)
        self.assertAlmostEqual(msg.linear_velocity_mps, 0.02, places=5)
        self.assertAlmostEqual(msg.angular_velocity_rps, 0.05, places=5)
        self.assertFalse(hasattr(msg, "linear_speed_mps"))
        self.assertFalse(hasattr(msg, "angular_speed_rps"))

    def test_goal_announcement_fields_used_by_tick_are_real(self):
        from epuck2_comm_interfaces.msg import GoalAnnouncement

        msg = GoalAnnouncement()
        msg.protocol_version = GoalAnnouncement.PROTOCOL_VERSION
        msg.source_robot_id = 99
        msg.sequence = 1
        msg.goal_id = "shared_exit"
        msg.goal_x_m = 0.86
        msg.goal_y_m = 0.25
        msg.valid = True

        self.assertEqual(msg.source_robot_id, 99)
        self.assertAlmostEqual(msg.goal_x_m, 0.86, places=5)
        self.assertAlmostEqual(msg.goal_y_m, 0.25, places=5)
        self.assertTrue(msg.valid)
        self.assertFalse(hasattr(msg, "robot_id"))
        self.assertFalse(hasattr(msg, "target_x_m"))
        self.assertFalse(hasattr(msg, "target_y_m"))


class StepVirtualStateTest(unittest.TestCase):
    def test_zero_dt_is_a_no_op(self):
        state = VirtualState(1.0, 2.0, 0.0, 0.0, 0.0)
        result = step_virtual_state(state, 0.0, 1.0, 1.0)
        self.assertEqual(result, state)

    def test_negative_dt_is_a_no_op(self):
        state = VirtualState(1.0, 2.0, 0.0, 0.0, 0.0)
        result = step_virtual_state(state, -0.1, 1.0, 1.0)
        self.assertEqual(result, state)

    def test_forward_motion_along_zero_yaw(self):
        state = VirtualState(0.0, 0.0, 0.0, 0.0, 0.0)
        result = step_virtual_state(state, 1.0, 0.02, 0.0)
        self.assertAlmostEqual(result.x_m, 0.02)
        self.assertAlmostEqual(result.y_m, 0.0)

    def test_pure_rotation_updates_yaw_only(self):
        state = VirtualState(0.0, 0.0, 0.0, 0.0, 0.0)
        result = step_virtual_state(state, 1.0, 0.0, 0.5)
        self.assertAlmostEqual(result.yaw_rad, 0.5)
        self.assertAlmostEqual(result.x_m, 0.0)
        self.assertAlmostEqual(result.y_m, 0.0)


class DistanceToTest(unittest.TestCase):
    def test_distance_matches_euclidean(self):
        state = VirtualState(0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(distance_to(state, 3.0, 4.0), 5.0)


class PlanVirtualCommandTest(unittest.TestCase):
    def test_arrival_within_radius_returns_zero_and_arrived(self):
        state = VirtualState(0.99, 0.0, 0.0, 0.0, 0.0)
        plan = VirtualNavPlan(target_x_m=1.0, target_y_m=0.0, cruise_linear_mps=0.02, arrival_radius_m=0.05)
        linear, angular, arrived = plan_virtual_command(state, plan, max_angular_rps=0.5)
        self.assertTrue(arrived)
        self.assertEqual(linear, 0.0)
        self.assertEqual(angular, 0.0)

    def test_facing_away_turns_without_advancing(self):
        state = VirtualState(0.0, 0.0, math.pi, 0.0, 0.0)
        plan = VirtualNavPlan(target_x_m=1.0, target_y_m=0.0, cruise_linear_mps=0.02, arrival_radius_m=0.05)
        linear, angular, arrived = plan_virtual_command(state, plan, max_angular_rps=0.5)
        self.assertFalse(arrived)
        self.assertEqual(linear, 0.0)
        self.assertNotEqual(angular, 0.0)

    def test_facing_target_advances(self):
        state = VirtualState(0.0, 0.0, 0.0, 0.0, 0.0)
        plan = VirtualNavPlan(target_x_m=1.0, target_y_m=0.0, cruise_linear_mps=0.02, arrival_radius_m=0.05)
        linear, angular, arrived = plan_virtual_command(state, plan, max_angular_rps=0.5)
        self.assertFalse(arrived)
        self.assertEqual(linear, 0.02)

    def test_angular_command_clamped_to_max(self):
        state = VirtualState(0.0, 0.0, -math.pi / 2.0, 0.0, 0.0)
        plan = VirtualNavPlan(target_x_m=1.0, target_y_m=1.0, cruise_linear_mps=0.02, arrival_radius_m=0.05)
        _, angular, _ = plan_virtual_command(state, plan, max_angular_rps=0.1)
        self.assertLessEqual(abs(angular), 0.1)


if __name__ == "__main__":
    unittest.main()
