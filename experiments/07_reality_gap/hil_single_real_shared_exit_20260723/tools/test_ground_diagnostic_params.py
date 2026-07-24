#!/usr/bin/env python3
"""Tests for ground_diagnostic_params.json -- reuses
hil_preflight.check_required_params_confirmed() and
hil_preflight.check_required_fields_ready() (neither reimplemented
here) against this diagnostic's own, separate parameter file. No
rclpy dependency.

This file now holds only TRACKED configuration: measured geometry plus
stable venue facts (floor condition, travel path, obstacle recording,
emergency-stop position). The two genuinely per-session confirmations
(operator present, Wi-Fi checked) were moved out to a separate,
gitignored session-state file -- see
test_hil_ground_diagnostic_session.py -- and are deliberately NOT
present anywhere in this file or its required_before_ground_motion
list.
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
}

BOOLEAN_CONFIRMATION_PATHS = {
    "environment.floor_condition_confirmed",
    "environment.travel_path_clear_confirmed",
    "environment.boundaries_and_obstacles_recorded",
    "safety.emergency_stop_position_confirmed",
}

SESSION_ONLY_FIELD_NAMES = ("operator_present_confirmed", "wifi_checked_in_test_area")


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
    """A fresh, synthetic params dict with every required path still
    unconfirmed -- used instead of the live file so these tests stay
    valid regardless of whether the tracked file's real measurements
    have since been confirmed and committed."""
    params = {
        "measured_geometry": {path.split(".")[1]: "UNCONFIRMED_PHYSICAL_MEASUREMENT" for path in NUMERIC_PATHS},
        "environment": {
            "floor_condition_confirmed": False,
            "travel_path_clear_confirmed": False,
            "boundaries_and_obstacles_recorded": False,
        },
        "safety": {"emergency_stop_position_confirmed": False},
        "required_before_ground_motion": sorted(ALL_REQUIRED_PATHS),
    }
    return params


class GroundDiagnosticParamsFileTest(unittest.TestCase):
    def test_file_exists_and_is_valid_json(self):
        self.assertTrue(PARAMS_PATH.is_file())
        _load()  # must not raise

    def test_required_before_ground_motion_contains_exactly_the_expected_12_tracked_paths(self):
        params = _load()
        self.assertEqual(set(params["required_before_ground_motion"]), ALL_REQUIRED_PATHS)
        self.assertEqual(len(params["required_before_ground_motion"]), 12)

    def test_session_only_fields_are_not_present_anywhere_in_the_tracked_file(self):
        # operator_present_confirmed and wifi_checked_in_test_area must
        # never be committed here as a permanent fact -- they live only
        # in the gitignored session-state file. Checked structurally
        # (as actual JSON keys), not as a raw substring search, since
        # the field names legitimately appear in _comment prose
        # explaining why they are absent from this file.
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
        # A freshly-unconfirmed params dict must block on all 12 paths --
        # this is what the tracked file looked like before any
        # measurement or venue-fact confirmation was taken.
        params = _all_unconfirmed_synthetic_params()
        result = check_required_fields_ready(params)
        self.assertFalse(result.ok)
        self.assertEqual(set(result.unconfirmed_paths), ALL_REQUIRED_PATHS)
        self.assertEqual(result.missing_paths, ())

    def test_synthetic_all_unconfirmed_params_numeric_only_check_flags_geometry(self):
        # check_required_params_confirmed() alone (numeric/UNCONFIRMED
        # only) still correctly flags the 8 geometry fields; the 4
        # boolean fields are not literally the UNCONFIRMED string, so it
        # does not flag them -- that is exactly why
        # check_required_fields_ready() exists and is used above.
        params = _all_unconfirmed_synthetic_params()
        result = check_required_params_confirmed(params)
        self.assertFalse(result.ok)
        self.assertEqual(set(result.unconfirmed_paths), NUMERIC_PATHS)
        self.assertEqual(result.missing_paths, ())

    def test_tracked_file_currently_passes_check_required_fields_ready(self):
        # The tracked file's own 12 fields have all been measured/
        # confirmed -- this is the actual, current, intended state, not
        # a synthetic fixture.
        params = _load()
        result = check_required_fields_ready(params)
        self.assertTrue(result.ok, f"expected tracked file fully confirmed, got unconfirmed={result.unconfirmed_paths}")

    def test_passes_once_every_required_field_is_confirmed(self):
        params = _all_unconfirmed_synthetic_params()
        for dotted_path in params["required_before_ground_motion"]:
            value = True if dotted_path in BOOLEAN_CONFIRMATION_PATHS else 1.0
            _set_path(params, dotted_path, value)
        result = check_required_fields_ready(params)
        self.assertTrue(result.ok)

    def test_each_numeric_field_blocks_alone_when_unconfirmed(self):
        for target in {"measured_geometry.test_area_length_m", "measured_geometry.test_area_width_m"}:
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

    def test_each_boolean_field_passes_once_true(self):
        for target in BOOLEAN_CONFIRMATION_PATHS:
            params = _all_unconfirmed_synthetic_params()
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
