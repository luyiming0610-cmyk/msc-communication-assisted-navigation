#!/usr/bin/env python3
"""Export the frozen three-robot shared-exit parameters for Bash."""
import json
import sys


def main():
    with open(sys.argv[1], "r", encoding="utf-8") as handle:
        params = json.load(handle)
    exit_region = params["exit"]
    robots = params["robots"]
    parking = params["parking_zones"]
    values = {
        "WORLD_FILE": params["world_file"],
        "GOAL_CENTER_X_M": exit_region["center_x_m"],
        "GOAL_CENTER_Y_M": exit_region["center_y_m"],
        "GOAL_RADIUS_M": exit_region["goal_hold_radius_m"],
        "GOAL_HOLD_TIME_S": params["goal_hold_time_s"],
        "COMPLETION_MAX_LINEAR_SPEED_MPS": params["completion_max_linear_speed_mps"],
        "COMPLETION_MAX_ANGULAR_SPEED_RPS": params["completion_max_angular_speed_rps"],
        "SAFETY_RADIUS_M": params["safety_radius_m"],
        "COLLISION_CONTACT_DISTANCE_M": params["collision_contact_distance_m"],
        "MAX_RUNTIME_S": params["max_runtime_s"],
        "STARTUP_HOLD_S": params["startup_hold_s"],
        "NOMINAL_SPEED_MPS": params["nominal_speed_mps"],
        "GOAL_ID": "shared_exit",
    }
    for letter, key in (("A", "robot_a"), ("B", "robot_b"), ("C", "robot_c")):
        robot = robots[key]
        zone = parking[key]
        values.update({
            f"ROBOT_{letter}_START_X_M": robot["start_x_m"],
            f"ROBOT_{letter}_START_Y_M": robot["start_y_m"],
            f"ROBOT_{letter}_START_YAW_RAD": robot["start_yaw_rad"],
            f"PARKING_{letter}_X_M": zone["center_x_m"],
            f"PARKING_{letter}_Y_M": zone["center_y_m"],
            f"PARKING_{letter}_RADIUS_M": zone["radius_m"],
        })
        if "search_waypoints_m" in robot:
            values[f"ROBOT_{letter}_WAYPOINTS"] = ",".join(
                f"{x}:{y}" for x, y in robot["search_waypoints_m"]
            )
            values[f"ROBOT_{letter}_WAYPOINT_ARRIVAL_RADIUS_M"] = robot[
                "search_waypoint_arrival_radius_m"
            ]
    for key, value in values.items():
        print(f"export {key}={value}")


if __name__ == "__main__":
    main()
