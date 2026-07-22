#!/usr/bin/env python3
"""Launch the three unchanged pairwise cooperative controllers."""
import json
import math
import os
import sys

import launch
from launch_ros.actions import Node


def load_params():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "shared_exit_n3_params.json")
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def controller(namespace, robot_id, peer_topic, heading, enable_peer, params):
    return Node(
        package="epuck2_comm",
        executable="cooperative_avoider",
        namespace=namespace,
        output="screen",
        parameters=[{
            "robot_id": robot_id,
            "peer_state_topic": peer_topic,
            "armed": True,
            "desired_heading_rad": heading,
            "enable_peer_avoidance": enable_peer,
            "enable_dynamic_heading": True,
            "enable_dynamic_speed": True,
            "nav_intent_timeout_s": 1.0,
            "enable_local_avoidance": True,
            "require_local_sensors": True,
            "use_sim_time": True,
            "nominal_speed_mps": params["nominal_speed_mps"],
            "safety_radius_m": params["safety_radius_m"],
            "trigger_distance_m": params["peer_trigger_distance_m"],
            "startup_hold_s": params["startup_hold_s"],
            "max_runtime_s": params["max_runtime_s"],
            "stop_after_recovery": False,
            "post_recovery_hold_s": 0.5,
        }],
    )


def main():
    mode = os.environ.get("N3_EXIT_COMM_MODE", "N3_EXIT_COMM_ON")
    if mode not in ("N3_EXIT_COMM_ON", "N3_EXIT_COMM_OFF"):
        print(f"invalid N3_EXIT_COMM_MODE: {mode}", file=sys.stderr)
        return 2
    params = load_params()
    exit_region = params["exit"]
    actions = []
    for letter, robot_id in (("a", 1), ("b", 2), ("c", 3)):
        robot = params["robots"][f"robot_{letter}"]
        if letter == "a":
            target = (exit_region["center_x_m"], exit_region["center_y_m"])
        else:
            target = robot["search_waypoints_m"][1]
        heading = math.atan2(target[1] - robot["start_y_m"], target[0] - robot["start_x_m"])
        actions.append(controller(
            f"epuck{robot_id}", robot_id, f"/epuck{robot_id}/selected_peer_state",
            heading, mode == "N3_EXIT_COMM_ON", params,
        ))
    service = launch.LaunchService()
    service.include_launch_description(launch.LaunchDescription(actions))
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
