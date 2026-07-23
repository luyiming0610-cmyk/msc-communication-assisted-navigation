#!/usr/bin/env python3
"""Prints hil_frozen_params.json's scalar values as `export KEY=value`
lines, for `source <(python3 hil_frozen_params_to_env.py path.json)` in
the HIL orchestrator -- mirrors the established
10_cooperative_exit_navigation_20260720/tools/frozen_params_to_env.py
pattern exactly: the single source of truth stays the JSON file, never
a second hardcoded copy in bash.

Refuses (nonzero exit, no output) if any required field is still the
literal string UNCONFIRMED_PHYSICAL_MEASUREMENT -- this is a second,
independent check beyond hil_preflight.py's own
check_required_params_confirmed() (which the orchestrator always runs
first), so this script can never silently emit a placeholder as if it
were a real number.
"""
import json
import sys

UNCONFIRMED = "UNCONFIRMED_PHYSICAL_MEASUREMENT"


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "hil_frozen_params.json"
    with open(path, "r", encoding="utf-8") as f:
        p = json.load(f)

    pairs = {
        "MAX_LINEAR_SPEED_MPS": p["hil_guard_limits"]["max_linear_speed_mps"],
        "MAX_ANGULAR_SPEED_RPS": p["hil_guard_limits"]["max_angular_speed_rps"],
        "HEARTBEAT_TIMEOUT_S": p["hil_guard_limits"]["heartbeat_timeout_s"],
        "PHYSICAL_STATE_TIMEOUT_S": p["hil_guard_limits"]["physical_state_timeout_s"],
        "VIRTUAL_PEER_TIMEOUT_S": p["hil_guard_limits"]["virtual_peer_timeout_s"],
        "PHYSICAL_ROBOT_ORIGIN_X_M": p["coordinate_system"]["physical_robot_origin_x_m"],
        "PHYSICAL_ROBOT_ORIGIN_Y_M": p["coordinate_system"]["physical_robot_origin_y_m"],
        "PHYSICAL_ROBOT_ORIGIN_YAW_RAD": p["coordinate_system"]["physical_robot_origin_yaw_rad"],
        "ARENA_LENGTH_M": p["field_geometry"]["arena_length_m"],
        "ARENA_WIDTH_M": p["field_geometry"]["arena_width_m"],
        "START_POSE_X_M": p["field_geometry"]["start_pose_x_m"],
        "START_POSE_Y_M": p["field_geometry"]["start_pose_y_m"],
        "START_POSE_YAW_RAD": p["field_geometry"]["start_pose_yaw_rad"],
        "EXIT_CENTER_X_M": p["field_geometry"]["exit_center_x_m"],
        "EXIT_CENTER_Y_M": p["field_geometry"]["exit_center_y_m"],
        "EXIT_RADIUS_M": p["field_geometry"]["exit_radius_m"],
        "PARKING_X_M": p["field_geometry"]["parking_x_m"],
        "PARKING_Y_M": p["field_geometry"]["parking_y_m"],
        "PARKING_RADIUS_M": p["field_geometry"]["parking_radius_m"],
        "MIN_SAFETY_CLEARANCE_M": p["field_geometry"]["min_safety_clearance_m"],
    }
    # search_waypoints_m is a comma-separated "x:y,x:y,..." string
    # (goal_navigator.py's own --waypoints syntax), not a plain scalar,
    # so it is checked/emitted separately from the scalar `pairs` above.
    search_waypoints = p["field_geometry"].get("search_waypoints_m", UNCONFIRMED)

    unconfirmed = [key for key, value in pairs.items() if value == UNCONFIRMED]
    if search_waypoints == UNCONFIRMED:
        unconfirmed.append("SEARCH_WAYPOINTS_M")
    if unconfirmed:
        print(
            f"ERROR: refusing to emit env vars -- still UNCONFIRMED_PHYSICAL_MEASUREMENT: {unconfirmed}",
            file=sys.stderr,
        )
        return 1

    for key, value in pairs.items():
        print(f"export {key}={value}")
    print(f"export SEARCH_WAYPOINTS_M={search_waypoints}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
