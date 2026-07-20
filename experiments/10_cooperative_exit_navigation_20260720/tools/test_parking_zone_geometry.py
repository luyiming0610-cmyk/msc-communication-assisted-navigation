"""Automated (pytest) check that the two post-exit parking zones (Part
V) are non-colliding, per shared_exit_frozen_params.json and
verify_shared_exit_geometry.py's PARKING_A/PARKING_B constants -- the
same numbers verify_shared_exit_geometry.py's standalone script checks,
run here as part of the regular test suite so a future edit that
violates the spacing requirement fails CI, not just a manually-run
script."""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
FROZEN_PARAMS_PATH = os.path.join(HERE, "..", "shared_exit_frozen_params.json")


def _load_params():
    with open(FROZEN_PARAMS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def test_parking_zones_exceed_required_separation():
    params = _load_params()
    zones = params["parking_zones"]
    a = zones["robot_a"]
    b = zones["robot_b"]
    dist = math.hypot(a["center_x_m"] - b["center_x_m"], a["center_y_m"] - b["center_y_m"])
    required = params["safety_radius_m"]
    assert dist > required, f"parking zones {dist}m apart, must exceed {required}m"
    boundary_clearance = dist - a["radius_m"] - b["radius_m"]
    assert boundary_clearance > 0, "parking zone boundaries must not overlap"


def test_parking_zones_outside_exit_completion_region():
    params = _load_params()
    exit_ = params["exit"]
    for name, zone in params["parking_zones"].items():
        if not isinstance(zone, dict) or "center_x_m" not in zone:
            continue
        d = math.hypot(
            zone["center_x_m"] - exit_["center_x_m"], zone["center_y_m"] - exit_["center_y_m"]
        )
        assert d > exit_["goal_hold_radius_m"], f"{name} parking zone must be outside the exit region"


def test_parking_zones_are_not_the_same_point_as_the_shared_exit():
    """Part V: the two zones are not two different task goals -- but they
    also must not collapse onto the single shared exit point, or both
    robots would still converge on the same physical location."""
    params = _load_params()
    exit_center = (params["exit"]["center_x_m"], params["exit"]["center_y_m"])
    for name, zone in params["parking_zones"].items():
        if not isinstance(zone, dict) or "center_x_m" not in zone:
            continue
        assert (zone["center_x_m"], zone["center_y_m"]) != exit_center
