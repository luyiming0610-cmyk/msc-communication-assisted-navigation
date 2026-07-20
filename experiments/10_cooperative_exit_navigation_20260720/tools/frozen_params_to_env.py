#!/usr/bin/env python3
"""Prints shared_exit_frozen_params.json's scalar values as
`export KEY=value` lines, for `source <(python3 frozen_params_to_env.py)`
in the orchestrator -- the single source of truth stays the JSON file,
never a second hardcoded copy in bash."""
import json
import sys


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "shared_exit_frozen_params.json"
    with open(path, "r", encoding="utf-8") as f:
        p = json.load(f)

    pairs = {
        "GOAL_CENTER_X_M": p["exit"]["center_x_m"],
        "GOAL_CENTER_Y_M": p["exit"]["center_y_m"],
        "GOAL_RADIUS_M": p["exit"]["goal_hold_radius_m"],
        "GOAL_HOLD_TIME_S": p["goal_hold_time_s"],
        "PARKING_A_X_M": p["parking_zones"]["robot_a"]["center_x_m"],
        "PARKING_A_Y_M": p["parking_zones"]["robot_a"]["center_y_m"],
        "PARKING_A_RADIUS_M": p["parking_zones"]["robot_a"]["radius_m"],
        "PARKING_B_X_M": p["parking_zones"]["robot_b"]["center_x_m"],
        "PARKING_B_Y_M": p["parking_zones"]["robot_b"]["center_y_m"],
        "PARKING_B_RADIUS_M": p["parking_zones"]["robot_b"]["radius_m"],
        "DIAGNOSTIC_WORLD_FILE": p["diagnostic_world_file"],
        "SAFETY_RADIUS_M": p["safety_radius_m"],
        "COLLISION_CONTACT_DISTANCE_M": p["collision_contact_distance_m"],
        "MAX_RUNTIME_S": p["max_runtime_s"],
        "STARTUP_HOLD_S": p["startup_hold_s"],
        "NOMINAL_SPEED_MPS": p["nominal_speed_mps"],
        "WORLD_FILE": p["world_file"],
        "ROBOT_A_START_X_M": p["robots"]["robot_a"]["start_x_m"],
        "ROBOT_A_START_Y_M": p["robots"]["robot_a"]["start_y_m"],
        "ROBOT_A_START_YAW_RAD": p["robots"]["robot_a"]["start_yaw_rad"],
        "ROBOT_B_START_X_M": p["robots"]["robot_b"]["start_x_m"],
        "ROBOT_B_START_Y_M": p["robots"]["robot_b"]["start_y_m"],
        "ROBOT_B_START_YAW_RAD": p["robots"]["robot_b"]["start_yaw_rad"],
        "ROBOT_B_WAYPOINT_ARRIVAL_RADIUS_M": p["robots"]["robot_b"]["search_waypoint_arrival_radius_m"],
        "GOAL_ID": p["messages"]["goal_announcement"]["fields"] and "shared_exit",
    }
    waypoints = p["robots"]["robot_b"]["search_waypoints_m"]
    waypoints_str = ",".join(f"{x}:{y}" for x, y in waypoints)

    for key, value in pairs.items():
        print(f"export {key}={value}")
    print(f"export ROBOT_B_WAYPOINTS={waypoints_str}")


if __name__ == "__main__":
    main()
