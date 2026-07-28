#!/usr/bin/env python3
"""Tests for new_field_geometry_params.json -- mirrors
test_ground_diagnostic_params.py's exact structure/discipline for the
old field, applied to the new 1.40m x 1.00m field. Reuses
hil_preflight.check_required_fields_ready()/check_required_params_confirmed()
(neither reimplemented here). No rclpy dependency.

This is a genuinely SEPARATE file from ground_diagnostic_params.json --
these tests never read, modify, or assert anything about the old
field's tracked file, and the old field's own test file
(test_ground_diagnostic_params.py) is untouched.
"""
import json
import unittest
from pathlib import Path

from hil_preflight import check_required_fields_ready, check_required_params_confirmed

PARAMS_PATH = Path(__file__).parent / "new_field_geometry_params.json"

ALL_REQUIRED_PATHS = {
    "measured_geometry.start_x_m",
    "measured_geometry.start_y_m",
    "measured_geometry.start_yaw_rad",
    "measured_geometry.travel_direction",
    "measured_geometry.stop_line_distance_m",
    "measured_geometry.min_boundary_clearance_m",
    "measured_geometry.test_area_length_m",
    "measured_geometry.test_area_width_m",
    "measured_geometry.corridor_y_min_m",
    "measured_geometry.corridor_y_max_m",
    "environment.boundaries_and_obstacles_recorded",
    "safety.emergency_stop_position_confirmed",
}

NUMERIC_PATHS = {
    "measured_geometry.start_x_m",
    "measured_geometry.start_y_m",
    "measured_geometry.start_yaw_rad",
    "measured_geometry.travel_direction",
    "measured_geometry.stop_line_distance_m",
    "measured_geometry.min_boundary_clearance_m",
    "measured_geometry.test_area_length_m",
    "measured_geometry.test_area_width_m",
    "measured_geometry.corridor_y_min_m",
    "measured_geometry.corridor_y_max_m",
}

BOOLEAN_CONFIRMATION_PATHS = {
    "environment.boundaries_and_obstacles_recorded",
    "safety.emergency_stop_position_confirmed",
}

SESSION_ONLY_FIELD_NAMES = (
    "floor_condition_confirmed",
    "travel_path_clear_confirmed",
    "operator_present_confirmed",
    "wifi_checked_in_test_area",
)


def _load():
    with open(PARAMS_PATH, encoding="utf-8") as fh:
        return json.load(fh)


def _set_path(params, dotted_path, value):
    node = params
    parts = dotted_path.split(".")
    for part in parts[:-1]:
        node = node[part]
    node[parts[-1]] = value


def _all_unconfirmed_synthetic_params():
    params = {
        "measured_geometry": {path.split(".")[1]: "UNCONFIRMED_PHYSICAL_MEASUREMENT" for path in NUMERIC_PATHS},
        "environment": {"boundaries_and_obstacles_recorded": False},
        "safety": {"emergency_stop_position_confirmed": False},
        "required_before_ground_motion": sorted(ALL_REQUIRED_PATHS),
    }
    return params


class NewFieldGeometryParamsFileTest(unittest.TestCase):
    def test_file_exists_and_is_valid_json(self):
        self.assertTrue(PARAMS_PATH.is_file())
        _load()  # must not raise

    def test_required_before_ground_motion_contains_exactly_the_expected_12_tracked_paths(self):
        params = _load()
        self.assertEqual(set(params["required_before_ground_motion"]), ALL_REQUIRED_PATHS)
        self.assertEqual(len(params["required_before_ground_motion"]), 12)

    def test_session_only_fields_are_not_present_anywhere_in_the_tracked_file(self):
        params = _load()
        self.assertNotIn("network", params)
        for section_name in ("environment", "safety"):
            section = params.get(section_name, {})
            for name in SESSION_ONLY_FIELD_NAMES:
                self.assertNotIn(name, section, f"{name} must not be a key under {section_name}")
        for dotted_path in params["required_before_ground_motion"]:
            for name in SESSION_ONLY_FIELD_NAMES:
                self.assertNotIn(name, dotted_path)

    def test_synthetic_all_unconfirmed_params_are_blocked_on_every_path(self):
        params = _all_unconfirmed_synthetic_params()
        result = check_required_fields_ready(params)
        self.assertFalse(result.ok)
        self.assertEqual(set(result.unconfirmed_paths), ALL_REQUIRED_PATHS)
        self.assertEqual(result.missing_paths, ())

    def test_synthetic_all_unconfirmed_params_numeric_only_check_flags_geometry(self):
        params = _all_unconfirmed_synthetic_params()
        result = check_required_params_confirmed(params)
        self.assertFalse(result.ok)
        self.assertEqual(set(result.unconfirmed_paths), NUMERIC_PATHS)
        self.assertEqual(result.missing_paths, ())

    def test_tracked_file_now_passes_both_stable_venue_confirmations(self):
        # environment.boundaries_and_obstacles_recorded was confirmed
        # true 2026-07-28 after manual on-site obstacle-clearance
        # confirmation; safety.emergency_stop_position_confirmed was
        # confirmed true the same day after manual confirmation of the
        # emergency power-off arrangement (operator position, physical
        # power button within immediate arm's reach, immediate
        # power-removal method). Both tracked-file confirmations are now
        # true, so check_required_fields_ready() passes -- this is NOT
        # the same as PRE_STACK_CHECK passing: the four genuinely
        # per-session confirmations (floor condition, travel path,
        # operator present, Wi-Fi) still live only in
        # hil_ground_diagnostic_session.py and remain false until a live
        # session is explicitly initialized and confirmed.
        params = _load()
        result = check_required_fields_ready(params)
        self.assertTrue(result.ok)
        self.assertEqual(result.unconfirmed_paths, ())
        self.assertEqual(result.missing_paths, ())

    def test_boundaries_and_obstacles_recorded_is_now_confirmed_true(self):
        params = _load()
        self.assertTrue(params["environment"]["boundaries_and_obstacles_recorded"])

    def test_emergency_stop_position_confirmed_is_now_confirmed_true(self):
        params = _load()
        self.assertTrue(params["safety"]["emergency_stop_position_confirmed"])

    def test_external_obstacle_clearance_is_a_documentation_only_lower_bound_not_a_gate(self):
        params = _load()
        self.assertEqual(params["environment"]["external_obstacle_clearance_lower_bound_m"], 0.30)
        for dotted_path in params["required_before_ground_motion"]:
            self.assertNotIn("external_obstacle_clearance", dotted_path)

    def test_all_ten_geometry_fields_are_already_confirmed_real_measurements(self):
        # The 10 numeric/travel-direction geometry fields are already
        # given, real, marked values -- only the two stable-venue
        # booleans remain to be confirmed.
        params = _load()
        geometry_paths = NUMERIC_PATHS
        result = check_required_params_confirmed(params)
        self.assertEqual(result.unconfirmed_paths, ())
        self.assertEqual(result.missing_paths, ())
        for dotted_path in geometry_paths:
            self.assertIn(dotted_path, params["required_before_ground_motion"])

    def test_passes_once_every_required_field_is_confirmed(self):
        params = _all_unconfirmed_synthetic_params()
        for dotted_path in params["required_before_ground_motion"]:
            value = True if dotted_path in BOOLEAN_CONFIRMATION_PATHS else 1.0
            _set_path(params, dotted_path, value)
        result = check_required_fields_ready(params)
        self.assertTrue(result.ok)

    def test_each_new_corridor_field_blocks_alone_when_unconfirmed(self):
        for target in {"measured_geometry.corridor_y_min_m", "measured_geometry.corridor_y_max_m"}:
            params = _all_unconfirmed_synthetic_params()
            for dotted_path in params["required_before_ground_motion"]:
                if dotted_path == target:
                    continue
                value = True if dotted_path in BOOLEAN_CONFIRMATION_PATHS else 1.0
                _set_path(params, dotted_path, value)
            result = check_required_fields_ready(params)
            self.assertFalse(result.ok, f"expected block with {target} unconfirmed")
            self.assertEqual(result.unconfirmed_paths, (target,))

    def test_each_boolean_field_blocks_alone_when_false(self):
        for target in BOOLEAN_CONFIRMATION_PATHS:
            params = _all_unconfirmed_synthetic_params()
            for dotted_path in params["required_before_ground_motion"]:
                if dotted_path == target:
                    continue
                value = True if dotted_path in BOOLEAN_CONFIRMATION_PATHS else 1.0
                _set_path(params, dotted_path, value)
            result = check_required_fields_ready(params)
            self.assertFalse(result.ok, f"expected block with {target} still false")
            self.assertEqual(result.unconfirmed_paths, (target,))

    def test_measured_stopping_clearance_has_no_machine_readable_gating_field(self):
        params = _load()
        text = json.dumps(params)
        self.assertNotIn("stopping_clearance", text)
        for dotted_path in params["required_before_ground_motion"]:
            self.assertNotIn("stopping_clearance", dotted_path)

    def test_angular_speed_is_fixed_zero_and_prohibited(self):
        params = _load()
        limits = params["diagnostic_command_limits"]
        self.assertEqual(limits["max_angular_speed_rps"], 0.0)
        self.assertTrue(limits["angular_speed_prohibited"])

    def test_requested_linear_speed_within_confirmed_diagnostic_cap(self):
        params = _load()
        limits = params["diagnostic_command_limits"]
        self.assertLessEqual(limits["requested_linear_speed_mps"], limits["max_linear_speed_mps"])

    def test_diagnostic_linear_cap_matches_the_already_confirmed_frozen_guard_cap(self):
        params = _load()
        self.assertEqual(
            params["diagnostic_command_limits"]["max_linear_speed_mps"],
            params["reused_frozen_reference"]["guard_hard_linear_cap_mps"],
        )

    def test_pulse_duration_matches_target_travel_over_requested_speed(self):
        params = _load()
        limits = params["diagnostic_command_limits"]
        expected = round(limits["target_commanded_travel_m"] / limits["requested_linear_speed_mps"], 2)
        self.assertAlmostEqual(limits["pulse_s"], expected, places=2)

    def test_target_commanded_travel_is_0p10m_not_0p15m(self):
        params = _load()
        self.assertEqual(params["diagnostic_command_limits"]["target_commanded_travel_m"], 0.10)

    def test_stop_line_distance_is_measured_from_start_not_an_absolute_coordinate(self):
        # Absolute stop line x=1.20m, start_x_m=0.30m -> 0.90m from start.
        params = _load()
        geometry = params["measured_geometry"]
        self.assertAlmostEqual(geometry["stop_line_distance_m"], 0.90, places=6)

    def test_corridor_is_centered_on_start_y_and_field_center(self):
        params = _load()
        geometry = params["measured_geometry"]
        corridor_center = (geometry["corridor_y_min_m"] + geometry["corridor_y_max_m"]) / 2.0
        self.assertAlmostEqual(corridor_center, geometry["start_y_m"], places=6)
        self.assertAlmostEqual(corridor_center, geometry["test_area_width_m"] / 2.0, places=6)

    def test_manual_reference_line_is_documentation_only_never_a_gate(self):
        params = _load()
        self.assertIn("documentation_only_reference", params)
        self.assertIn("manual_reference_line_x_m", params["documentation_only_reference"])
        for dotted_path in params["required_before_ground_motion"]:
            self.assertNotIn("manual_reference_line", dotted_path)

    def test_absolute_stop_line_is_documentation_only_never_a_gate(self):
        params = _load()
        self.assertIn("absolute_stop_line_x_m", params["documentation_only_reference"])
        for dotted_path in params["required_before_ground_motion"]:
            self.assertNotIn("absolute_stop_line", dotted_path)

    def test_absolute_stop_line_traceability_matches_derived_stop_line_distance(self):
        # absolute_stop_line_x_m - start_x_m must equal stop_line_distance_m
        # (1.20 - 0.30 = 0.90), so the two can never silently drift apart.
        params = _load()
        geometry = params["measured_geometry"]
        absolute_stop_line_x_m = params["documentation_only_reference"]["absolute_stop_line_x_m"]
        derived_distance = absolute_stop_line_x_m - geometry["start_x_m"]
        self.assertAlmostEqual(derived_distance, geometry["stop_line_distance_m"], places=9)

    def test_does_not_duplicate_formal_shared_exit_geometry_fields(self):
        params = _load()
        geometry_keys = set(params["measured_geometry"].keys())
        forbidden = {"exit_center_x_m", "exit_center_y_m", "exit_radius_m", "parking_x_m", "parking_y_m", "parking_radius_m", "search_waypoints_m"}
        self.assertEqual(geometry_keys & forbidden, set())

    def test_does_not_read_or_modify_the_old_field_params_file(self):
        old_field_path = Path(__file__).parent / "ground_diagnostic_params.json"
        # Structural check only -- this test file never opens the old
        # field's file at all; this assertion documents that intent
        # and fails loudly if someone ever imports/reads it above.
        self.assertTrue(old_field_path.is_file())
        with open(old_field_path, encoding="utf-8") as fh:
            old_params = json.load(fh)
        self.assertEqual(old_params["measured_geometry"]["test_area_length_m"], 0.65)
        self.assertEqual(old_params["measured_geometry"]["test_area_width_m"], 0.25)


if __name__ == "__main__":
    unittest.main()
