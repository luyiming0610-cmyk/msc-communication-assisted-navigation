#!/usr/bin/env python3
"""Launch factory for the two cooperative_avoider controllers in the
shared edge-exit N2 study. Mirrors run_n2_controllers.py's pattern.

Reads N2_EXIT_COMM_MODE from the environment (N2_EXIT_COMM_OFF |
N2_EXIT_COMM_ON) and sets enable_peer_avoidance accordingly -- COMM_OFF
disables peer CPA avoidance entirely (Robot B relies on local IR/ToF
only, per the study's communication definition). enable_dynamic_heading
is always true for both robots in both conditions (both navigate via
goal_navigator's NavigationIntent, not a fixed heading) -- this is
orthogonal to peer_avoidance/GoalAnnouncement, which is the actual
communication channel under study.

All numeric parameters are read from shared_exit_frozen_params.json,
never hardcoded here, so the orchestrator's frozen-params file is the
single source of truth.
"""
from __future__ import annotations

import json
import math
import os
import sys

import launch
from launch_ros.actions import Node


def _load_frozen_params():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "shared_exit_frozen_params.json")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def make_controller(namespace, robot_id, peer_topic, initial_heading, enable_peer_avoidance, params):
    return Node(
        package="epuck2_comm",
        executable="cooperative_avoider",
        namespace=namespace,
        output="screen",
        parameters=[{
            "robot_id": robot_id,
            "peer_state_topic": peer_topic,
            "armed": True,
            "desired_heading_rad": initial_heading,
            "enable_peer_avoidance": enable_peer_avoidance,
            "enable_dynamic_heading": True,
            "enable_dynamic_speed": True,
            "nav_intent_timeout_s": 1.0,
            "enable_local_avoidance": True,
            "require_local_sensors": True,
            "use_sim_time": True,
            "nominal_speed_mps": params["nominal_speed_mps"],
            "safety_radius_m": params["safety_radius_m"],
            "startup_hold_s": params["startup_hold_s"],
            "max_runtime_s": params["max_runtime_s"],
            # PILOT09 (EXCLUDED, preserved) proved stop_after_recovery=True --
            # inherited from the pre-ARRIVED_HOLD design, where "a recovery
            # maneuver just finished" was the only completion proxy available
            # -- is now wrong for this study: it permanently stops the
            # controller the instant ANY local-avoidance encounter resolves,
            # even if the robot has not yet reached its own parking zone.
            # Robot A resolved its one encounter and returned to CRUISE, but
            # was immediately forced to COMPLETE instead of continuing toward
            # ARRIVED_HOLD. Genuine completion is now judged by goal_navigator's
            # own ARRIVED_HOLD latch (which drives desired_linear_speed_mps to
            # 0 via enable_dynamic_speed) and task_completion_monitor.py's
            # live TASK_COMPLETE_GOAL signal -- stop_after_recovery must stay
            # False so the controller keeps navigating after a recovery
            # instead of quitting early. max_runtime_s remains the ultimate
            # backstop.
            "stop_after_recovery": False,
            "post_recovery_hold_s": 0.5,
        }],
    )


def main():
    comm_mode = os.environ.get("N2_EXIT_COMM_MODE", "N2_EXIT_COMM_ON")
    if comm_mode not in ("N2_EXIT_COMM_ON", "N2_EXIT_COMM_OFF"):
        print(f"N2_EXIT_COMM_MODE must be N2_EXIT_COMM_ON or N2_EXIT_COMM_OFF, got {comm_mode!r}", file=sys.stderr)
        return 2
    enable_peer = comm_mode == "N2_EXIT_COMM_ON"

    params = _load_frozen_params()
    robot_a = params["robots"]["robot_a"]
    robot_b = params["robots"]["robot_b"]
    exit_ = params["exit"]

    a_heading = math.atan2(
        exit_["center_y_m"] - robot_a["start_y_m"], exit_["center_x_m"] - robot_a["start_x_m"]
    )
    first_wp = robot_b["search_waypoints_m"][0]
    b_heading = math.atan2(first_wp[1] - robot_b["start_y_m"], first_wp[0] - robot_b["start_x_m"])

    description = launch.LaunchDescription([
        make_controller("epuck1", 1, "/epuck2/state", a_heading, enable_peer, params),
        make_controller("epuck2", 2, "/epuck1/state", b_heading, enable_peer, params),
    ])
    service = launch.LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
