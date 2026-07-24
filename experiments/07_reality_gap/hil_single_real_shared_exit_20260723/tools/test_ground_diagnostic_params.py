#!/usr/bin/env python3
"""Tests for ground_diagnostic_params.json -- reuses
hil_preflight.check_required_params_confirmed() and
hil_preflight.check_required_fields_ready() (neither reimplemented
here) against this diagnostic's own, separate parameter file. No
rclpy dependency.

required_before_ground_motion mixes numeric UNCONFIRMED_PHYSICAL_MEASUREMENT
geometry fields with boolean pre-run safety-confirmation fields
(environment/safety/network sections). The actual gate used by
run_ground_diagnostic_preflight.sh is check_required_fields_ready(),
which is boolean-aware; check_required_params_confirmed() remains as
originally written (numeric/UNCONFIRMED-only) and is exercised here
only to confirm it still behaves as before on the numeric fields.
"""
import json
import tempfile
import unittest
from pathlib import Path

from hil_preflight import check_required_fields_ready, check_required_params_confirmed

PARAMS_PATH = Path(__file__).parent / "ground_diagnostic_params.json"

ALL_REQUIRED_PATHS = {
    "measured_geometry.start_x_m",
    "measured_geometry.start_y_m",
    "measured_geometry.start_yaw_rad",
    "measured_geometry.travel_direction",
    "measured_geometry.stop_line_distance_m",
    "measured_geometry.min_boundary_clearance_m",
    "measured_geometry.test_area_length_m",
    "measured_geometry.test_area_width_m",
    "environment.floor_condition_confirmed",
    "environment.travel_path_clear_confirmed",
    "environment.boundaries_and_obstacles_recorded",
    "safety.emergency_stop_position_confirmed",
    "safety.operator_present_confirmed",
    "network.wifi_checked_in_test_area",
}

NUMERIC_UNCONFIRMED_PATHS = {
    "measured_geometry.start_x_m",
    "measured_geometry.start_y_m",
    "measured_geometry.start_yaw_rad",
    "measured_geometry.travel_direction",
    "measured_geometry.stop_line_distance_m",
    "measured_geometry.min_boundary_clearance_m",
    "measured_geometry.test_area_length_m",
    "measured_geometry.test_area_width_m",
}

BOOLEAN_CONFIRMATION_PATHS = {
    "environment.floor_condition_confirmed",
    "environment.travel_path_clear_confirmed",
    "environment.boundaries_and_obstacles_recorded",
    "safety.emergency_stop_position_confirmed",
    "safety.operator_present_confirmed",
    "network.wifi_checked_in_test_area",
}


def _load():
    with open(PARAMS_PATH, encoding="utf-8") as fh:
        return json.load(fh)


def _set_path(params, dotted_path, value):
    node = params
    parts = dotted_path.split(".")
    for part in parts[:-1]:
        node = node[part]
    node[parts[-1]] = value


class GroundDiagnosticParamsFileTest(unittest.TestCase):
    def test_file_exists_and_is_valid_json(self):
        self.assertTrue(PARAMS_PATH.is_file())
        _load()  # must not raise

    def test_required_before_ground_motion_contains_exactly_the_expected_14_paths(self):
        params = _load()
        self.assertEqual(set(params["required_before_ground_motion"]), ALL_REQUIRED_PATHS)
        self.assertEqual(len(params["required_before_ground_motion"]), 14)

    def test_currently_blocked_because_measurements_are_unconfirmed(self):
        # Honest, current-state check with the original, numeric-only
        # function: unconfirmed numeric fields are flagged; the boolean
        # fields are not literally the UNCONFIRMED string, so this
        # function alone does not flag them -- that is exactly why
        # check_required_fields_ready() exists and is used below.
        params = _load()
        result = check_required_params_confirmed(params)
        self.assertFalse(result.ok)
        self.assertEqual(set(result.unconfirmed_paths), NUMERIC_UNCONFIRMED_PATHS)
        self.assertEqual(result.missing_paths, ())

    def test_fields_ready_currently_blocked_on_every_required_path(self):
        # As shipped (no field measurements taken, all confirmations
        # false), the boolean-aware gate used by the actual preflight
        # script must block on all 14 paths, not just the 6 original
        # geometry fields.
        params = _load()
        result = check_required_fields_ready(params)
        self.assertFalse(result.ok)
        self.assertEqual(set(result.unconfirmed_paths), ALL_REQUIRED_PATHS)
        self.assertEqual(result.missing_paths, ())

    def test_passes_once_every_required_field_is_confirmed(self):
        params = _load()
        for dotted_path in params["required_before_ground_motion"]:
            value = True if dotted_path in BOOLEAN_CONFIRMATION_PATHS else 1.0
            _set_path(params, dotted_path, value)
        result = check_required_fields_ready(params)
        self.assertTrue(result.ok)

    def test_each_new_numeric_field_blocks_alone_when_unconfirmed(self):
        # One at a time: confirm every other required field, leave just
        # this one at UNCONFIRMED_PHYSICAL_MEASUREMENT, and confirm it
        # alone is enough to block.
        for target in NUMERIC_UNCONFIRMED_PATHS & {
            "measured_geometry.test_area_length_m",
            "measured_geometry.test_area_width_m",
        }:
            params = _load()
            for dotted_path in params["required_before_ground_motion"]:
                if dotted_path == target:
                    continue
                value = True if dotted_path in BOOLEAN_CONFIRMATION_PATHS else 1.0
                _set_path(params, dotted_path, value)
            result = check_required_fields_ready(params)
            self.assertFalse(result.ok, f"expected block with {target} unconfirmed")
            self.assertEqual(result.unconfirmed_paths, (target,))

    def test_each_new_boolean_field_blocks_alone_when_false(self):
        # One at a time: confirm every other required field, leave just
        # this one boolean false, and confirm it alone is enough to
        # block -- this is the core of the fix (a False confirmation
        # must block just as surely as an UNCONFIRMED measurement).
        for target in BOOLEAN_CONFIRMATION_PATHS:
            params = _load()
            for dotted_path in params["required_before_ground_motion"]:
                if dotted_path == target:
                    continue
                value = True if dotted_path in BOOLEAN_CONFIRMATION_PATHS else 1.0
                _set_path(params, dotted_path, value)
            result = check_required_fields_ready(params)
            self.assertFalse(result.ok, f"expected block with {target} still false")
            self.assertEqual(result.unconfirmed_paths, (target,))

    def test_each_new_boolean_field_passes_once_true(self):
        # Symmetric check: with every other field confirmed and this
        # one flipped from false to true, overall result is ok.
        for target in BOOLEAN_CONFIRMATION_PATHS:
            params = _load()
            for dotted_path in params["required_before_ground_motion"]:
                value = True if dotted_path in BOOLEAN_CONFIRMATION_PATHS else 1.0
                _set_path(params, dotted_path, value)
            result = check_required_fields_ready(params)
            self.assertTrue(result.ok, f"expected pass with {target} true and all else confirmed")

    def test_measured_stopping_clearance_has_no_machine_readable_gating_field(self):
        # Post-run only, per FIRST_GROUND_DIAGNOSTIC_SPEC.md -- must
        # never appear as a required_before_ground_motion path or
        # anywhere else in this pre-run params file.
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

    def test_suspended_wheel_angular_value_is_not_present_anywhere_in_this_file(self):
        # The suspended-wheel diagnostic's temporary +-0.1 rad/s value
        # must never appear as an actual ground limit in this file.
        text = PARAMS_PATH.read_text(encoding="utf-8")
        self.assertNotIn('"max_angular_speed_rps": 0.1', text)
        self.assertNotIn('"max_angular_speed_rps": -0.1', text)

    def test_does_not_duplicate_formal_shared_exit_geometry_fields(self):
        # This diagnostic's geometry is deliberately smaller than
        # hil_frozen_params.json's field_geometry -- no exit/parking/
        # search-waypoint fields, since this diagnostic never uses them.
        params = _load()
        geometry_keys = set(params["measured_geometry"].keys())
        forbidden = {"exit_center_x_m", "exit_center_y_m", "exit_radius_m", "parking_x_m", "parking_y_m", "parking_radius_m", "search_waypoints_m"}
        self.assertEqual(geometry_keys & forbidden, set())


class NonOverwritePathCheckTest(unittest.TestCase):
    """Mirrors sync_epuck2_comm_logic.py's established pattern: a new
    evidence path must be confirmed NOT to already exist before use."""

    def test_fresh_path_is_available(self):
        with tempfile.TemporaryDirectory() as tmp:
            candidate = Path(tmp) / "command_audit_brand_new.jsonl"
            self.assertFalse(candidate.exists())

    def test_existing_path_is_detected_as_already_present(self):
        with tempfile.TemporaryDirectory() as tmp:
            candidate = Path(tmp) / "command_audit_already_here.jsonl"
            candidate.write_text("{}\n", encoding="utf-8")
            self.assertTrue(candidate.exists())


if __name__ == "__main__":
    unittest.main()
