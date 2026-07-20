"""Guards against exactly the class of bug found in Stage 0 (an analyzer
hardcoding a copy of the goal radius that drifted out of sync with the
orchestrator's own frozen value after a fix). shared_exit_frozen_params.json
is the single source of truth; this test cross-checks it against
verify_shared_exit_geometry.py's own constants (which were used to
derive the frozen file in the first place) and against
frozen_params_to_env.py's output, so any future edit to one without the
other fails a test instead of silently drifting."""
import json
import os
import subprocess
import sys

import verify_shared_exit_geometry as geom


FROZEN_PARAMS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "shared_exit_frozen_params.json"
)


def _load():
    with open(FROZEN_PARAMS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def test_frozen_params_file_is_valid_json():
    params = _load()
    assert params["study"]


def test_goal_region_matches_verified_geometry_script():
    params = _load()
    assert params["exit"]["center_x_m"] == geom.GOAL_CENTER[0]
    assert params["exit"]["center_y_m"] == geom.GOAL_CENTER[1]
    assert params["exit"]["goal_hold_radius_m"] == geom.GOAL_RADIUS_M


def test_obstacle_matches_verified_geometry_script():
    params = _load()
    assert params["obstacle"]["center_x_m"] == geom.OBSTACLE_CENTER[0]
    assert params["obstacle"]["center_y_m"] == geom.OBSTACLE_CENTER[1]


def test_robot_start_poses_match_verified_geometry_script():
    params = _load()
    assert params["robots"]["robot_a"]["start_x_m"] == geom.ROBOT_A_START[0]
    assert params["robots"]["robot_a"]["start_y_m"] == geom.ROBOT_A_START[1]
    assert params["robots"]["robot_b"]["start_x_m"] == geom.ROBOT_B_START[0]
    assert params["robots"]["robot_b"]["start_y_m"] == geom.ROBOT_B_START[1]


def test_search_waypoints_match_verified_geometry_script():
    params = _load()
    frozen_waypoints = [tuple(p) for p in params["robots"]["robot_b"]["search_waypoints_m"]]
    assert frozen_waypoints == geom.ROBOT_B_WAYPOINTS


def test_nominal_speed_is_the_reviewed_value_not_the_original_draft():
    params = _load()
    # The first design draft proposed 0.06; the reviewed, frozen value is
    # 0.04 (local IR/ToF safety behavior was validated at the lower
    # speed). This test fails loudly if that regresses.
    assert params["nominal_speed_mps"] == 0.04


def test_neither_start_pose_is_inside_the_goal_region():
    params = _load()
    import math
    gx, gy, gr = (
        params["exit"]["center_x_m"], params["exit"]["center_y_m"], params["exit"]["goal_hold_radius_m"]
    )
    for robot in ("robot_a", "robot_b"):
        sx = params["robots"][robot]["start_x_m"]
        sy = params["robots"][robot]["start_y_m"]
        distance = math.hypot(sx - gx, sy - gy)
        assert distance > gr, f"{robot} start pose is inside the goal region -- exactly the Stage 0 PILOT04 defect"


def test_frozen_params_to_env_script_emits_matching_values():
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frozen_params_to_env.py")
    result = subprocess.run(
        [sys.executable, script, FROZEN_PARAMS_PATH],
        capture_output=True, text=True, check=True,
    )
    env_lines = dict(
        line.removeprefix("export ").split("=", 1) for line in result.stdout.splitlines() if line.startswith("export ")
    )
    params = _load()
    assert float(env_lines["GOAL_RADIUS_M"]) == params["exit"]["goal_hold_radius_m"]
    assert float(env_lines["MAX_RUNTIME_S"]) == params["max_runtime_s"]
    assert float(env_lines["NOMINAL_SPEED_MPS"]) == params["nominal_speed_mps"]
