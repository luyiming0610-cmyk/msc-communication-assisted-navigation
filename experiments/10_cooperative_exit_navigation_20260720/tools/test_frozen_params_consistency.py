"""Guards against exactly the class of bug found in Stage 0 (an analyzer
hardcoding a copy of the goal radius that drifted out of sync with the
orchestrator's own frozen value after a fix). shared_exit_frozen_params.json
is the single source of truth.

verify_shared_exit_geometry.py was later rewritten (2026-07-21 analytic
exit-geometry redesign: hammer-shaped opening, obstacle removed per
Part VI) to read every value directly from this JSON inside main()
instead of duplicating them as importable module-level constants
(GOAL_CENTER/OBSTACLE_CENTER/ROBOT_A_START/ROBOT_B_WAYPOINTS no longer
exist -- confirmed by reading the current script). That refactor
eliminates the duplication these per-field cross-check tests existed to
catch, so cross-checking against those constants is no longer a
meaningful test (AttributeError, not a real assertion failure) and the
'obstacle' key check is checking for a field that was deliberately
removed from the frozen params along with the world's obstacle. Fixed
here to run the script itself and assert its own PASS verdict, which is
the current, still-meaningful form of the same guarantee -- this does
not touch shared_exit_frozen_params.json, verify_shared_exit_geometry.py,
or any controller/geometry value."""
import json
import os
import subprocess
import sys


FROZEN_PARAMS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "shared_exit_frozen_params.json"
)
GEOMETRY_SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "verify_shared_exit_geometry.py"
)


def _load():
    with open(FROZEN_PARAMS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def test_frozen_params_file_is_valid_json():
    params = _load()
    assert params["study"]


def test_verify_shared_exit_geometry_script_reports_overall_pass():
    result = subprocess.run(
        [sys.executable, GEOMETRY_SCRIPT_PATH],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "overall_check = PASS" in result.stdout


def test_robot_start_poses_are_present_and_outside_the_hammer_walls():
    params = _load()
    for robot in ("robot_a", "robot_b"):
        assert isinstance(params["robots"][robot]["start_x_m"], (int, float))
        assert isinstance(params["robots"][robot]["start_y_m"], (int, float))
    assert params["main_arena"]["x_min_m"] < params["robots"]["robot_a"]["start_x_m"] < params["main_arena"]["x_max_m"]
    assert params["main_arena"]["x_min_m"] < params["robots"]["robot_b"]["start_x_m"] < params["main_arena"]["x_max_m"]


def test_search_waypoints_are_present_and_end_at_the_exit():
    params = _load()
    waypoints = [tuple(p) for p in params["robots"]["robot_b"]["search_waypoints_m"]]
    assert len(waypoints) >= 2
    last = waypoints[-1]
    assert last == (params["exit"]["center_x_m"], params["exit"]["center_y_m"])


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
