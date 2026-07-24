#!/usr/bin/env python3
"""Tests for ground_diagnostic_params.json -- reuses
hil_preflight.check_required_params_confirmed() (not a reimplemented
check) against this diagnostic's own, separate parameter file. No
rclpy dependency.
"""
import json
import tempfile
import unittest
from pathlib import Path

from hil_preflight import check_required_params_confirmed

PARAMS_PATH = Path(__file__).parent / "ground_diagnostic_params.json"


def _load():
    with open(PARAMS_PATH, encoding="utf-8") as fh:
        return json.load(fh)


class GroundDiagnosticParamsFileTest(unittest.TestCase):
    def test_file_exists_and_is_valid_json(self):
        self.assertTrue(PARAMS_PATH.is_file())
        _load()  # must not raise

    def test_currently_blocked_because_measurements_are_unconfirmed(self):
        # Honest, current-state check: as of this preparation, no field
        # measurement has been taken, so this MUST report blocked, not
        # a false pass.
        params = _load()
        result = check_required_params_confirmed(params)
        self.assertFalse(result.ok)
        self.assertEqual(
            set(result.unconfirmed_paths),
            {
                "measured_geometry.start_x_m",
                "measured_geometry.start_y_m",
                "measured_geometry.start_yaw_rad",
                "measured_geometry.travel_direction",
                "measured_geometry.stop_line_distance_m",
                "measured_geometry.min_boundary_clearance_m",
            },
        )
        self.assertEqual(result.missing_paths, ())

    def test_passes_once_every_required_field_is_confirmed(self):
        params = _load()
        for dotted_path in params["required_before_ground_motion"]:
            node = params
            parts = dotted_path.split(".")
            for part in parts[:-1]:
                node = node[part]
            node[parts[-1]] = 1.0
        result = check_required_params_confirmed(params)
        self.assertTrue(result.ok)

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
