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
