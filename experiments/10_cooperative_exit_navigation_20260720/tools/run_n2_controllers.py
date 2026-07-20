#!/usr/bin/env python3
"""N2 (2-robot) cooperative_avoider launch factory for the exit-navigation
study. Modeled directly on the proven
controller_v4_full_sensor_bypass_20260717/config/comm_baseline_v1/run_comm_baseline_formal_controllers.py
(same make_controller pattern, same frozen cooperative_avoider.py node,
zero controller-code changes) with exactly two differences, both plain
parameters the controller already exposes:

  1. enable_peer_avoidance is threaded through from the COMM_MODE env var
     (COMM_ON -> True, COMM_OFF -> False) instead of being implicitly True.
  2. stop_after_recovery=True (with a short post_recovery_hold_s) so each
     robot autonomously stops shortly after clearing the encounter --
     this produces a stopping POSITION near the shared crossing region of
     the reused two_epuck_head_on_clean_world.wbt geometry (epuck1 at
     (-0.35,0) heading 0, epuck2 at (0.35,0) heading pi), which the
     task_completion_analyzer.py's goal-region check (centered at the
     origin) then verifies from the recorded bag -- not the controller
     itself claiming success.

Reuses the SAME world file and Webots/ros2_control launch stack
(run_dual_head_on_clean.py, dual_namespaced_launch.py,
two_epuck_head_on_clean_world.wbt) as Conditions A-D, unmodified. No new
Webots world file. safety_radius_m stays 0.14 (frozen, unchanged).
"""
import os
import sys

import launch
from launch_ros.actions import Node


def make_controller(namespace, robot_id, peer_topic, desired_heading, enable_peer_avoidance):
    return Node(
        package="epuck2_comm",
        executable="cooperative_avoider",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "robot_id": robot_id,
                "peer_state_topic": peer_topic,
                "armed": True,
                "desired_heading_rad": desired_heading,
                "enable_peer_avoidance": enable_peer_avoidance,
                "enable_local_avoidance": True,
                "require_local_sensors": True,
                "use_sim_time": True,
                "stop_after_recovery": True,
                "post_recovery_hold_s": 0.5,
                "max_runtime_s": 28.0,
            }
        ],
    )


def main():
    # Matches the orchestrator's own COMM_MODE values exactly
    # (N2_COMM_OFF / N2_COMM_ON) -- no translation layer, so there is only
    # one name for this value across both scripts.
    comm_mode = os.environ.get("N2_COMM_MODE", "N2_COMM_ON")
    if comm_mode not in ("N2_COMM_ON", "N2_COMM_OFF"):
        print(f"N2_COMM_MODE must be N2_COMM_ON or N2_COMM_OFF, got {comm_mode!r}", file=sys.stderr)
        return 2
    enable_peer = comm_mode == "N2_COMM_ON"

    description = launch.LaunchDescription(
        [
            make_controller("epuck1", 1, "/epuck2/state", 0.0, enable_peer),
            make_controller("epuck2", 2, "/epuck1/state", 3.141592653589793, enable_peer),
        ]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
