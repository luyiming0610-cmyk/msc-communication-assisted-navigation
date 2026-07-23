#!/usr/bin/env python3
import json
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "hil_frozen_params_to_env.py")


def _write_params(data):
    handle = tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False, encoding="utf-8")
    json.dump(data, handle)
    handle.close()
    return handle.name


CONFIRMED_PARAMS = {
    "hil_guard_limits": {
        "max_linear_speed_mps": 0.02,
        "max_angular_speed_rps": 0.15,
        "heartbeat_timeout_s": 0.5,
        "physical_state_timeout_s": 0.5,
        "virtual_peer_timeout_s": 1.0,
    },
    "coordinate_system": {
        "physical_robot_origin_x_m": 0.0,
        "physical_robot_origin_y_m": 0.0,
        "physical_robot_origin_yaw_rad": 0.0,
    },
    "field_geometry": {
        "arena_length_m": 2.0,
        "arena_width_m": 1.5,
        "start_pose_x_m": 0.1,
        "start_pose_y_m": 0.1,
        "start_pose_yaw_rad": 0.0,
        "exit_center_x_m": 1.5,
        "exit_center_y_m": 0.75,
        "exit_radius_m": 0.2,
        "parking_x_m": 1.7,
        "parking_y_m": 0.75,
        "parking_radius_m": 0.15,
        "min_safety_clearance_m": 0.1,
        "search_waypoints_m": "0.1:0.1,0.8:0.4,1.5:0.75",
    },
}


class HilFrozenParamsToEnvTest(unittest.TestCase):
    def test_confirmed_params_emit_export_lines(self):
        path = _write_params(CONFIRMED_PARAMS)
        self.addCleanup(os.unlink, path)
        result = subprocess.run([sys.executable, SCRIPT, path], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("export MAX_LINEAR_SPEED_MPS=0.02", result.stdout)
        self.assertIn("export EXIT_CENTER_X_M=1.5", result.stdout)
        self.assertIn("export SEARCH_WAYPOINTS_M=0.1:0.1,0.8:0.4,1.5:0.75", result.stdout)

    def test_unconfirmed_search_waypoints_refuses(self):
        params = json.loads(json.dumps(CONFIRMED_PARAMS))
        params["field_geometry"]["search_waypoints_m"] = "UNCONFIRMED_PHYSICAL_MEASUREMENT"
        path = _write_params(params)
        self.addCleanup(os.unlink, path)
        result = subprocess.run([sys.executable, SCRIPT, path], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SEARCH_WAYPOINTS_M", result.stderr)

    def test_unconfirmed_field_refuses_with_nonzero_exit(self):
        params = json.loads(json.dumps(CONFIRMED_PARAMS))
        params["field_geometry"]["exit_center_x_m"] = "UNCONFIRMED_PHYSICAL_MEASUREMENT"
        path = _write_params(params)
        self.addCleanup(os.unlink, path)
        result = subprocess.run([sys.executable, SCRIPT, path], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("UNCONFIRMED_PHYSICAL_MEASUREMENT", result.stderr)
        self.assertEqual(result.stdout, "")

    def test_real_hil_frozen_params_json_currently_refuses(self):
        # The actual project file, as of this test's writing, still has
        # unconfirmed geometry/angular-cap fields -- this test documents
        # and locks in that expectation rather than silently tolerating
        # a future accidental fabrication.
        real_path = os.path.join(HERE, "..", "hil_frozen_params.json")
        result = subprocess.run([sys.executable, SCRIPT, real_path], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
