#!/usr/bin/env python3

import sys
from pathlib import Path

import launch
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


CONFIG_DIR = Path(__file__).resolve().parent


def make_controller(
    namespace,
    robot_id,
    peer_topic,
    desired_heading,
    stop_after_recovery,
    max_runtime_s,
    startup_hold_s,
):
    return Node(
        package="epuck2_comm",
        executable="cooperative_avoider",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "robot_id": robot_id,
                "peer_state_topic": peer_topic,
                "desired_heading_rad": desired_heading,
                "armed": True,
                "max_runtime_s": max_runtime_s,
                "startup_hold_s": startup_hold_s,
                "stop_after_recovery": stop_after_recovery,
                "post_recovery_hold_s": 0.5,
                "use_sim_time": True,
                "enable_peer_avoidance": True,
                "enable_local_avoidance": True,
                "require_local_sensors": True,
            }
        ],
    )


def main():
    epuck1 = make_controller(
        "epuck1", 1, "/epuck2/state", 0.0, False, 100.0, 5.0
    )
    epuck2 = make_controller(
        "epuck2", 2, "/epuck1/state", 3.141592653589793, True, 100.0, 42.0
    )
    coordinator = ExecuteProcess(
        cmd=[sys.executable, str(CONFIG_DIR / "combined_task_coordinator.py")],
        output="screen",
    )
    shutdown_on_task_end = RegisterEventHandler(
        OnProcessExit(
            target_action=coordinator,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="combined wooden-box task monitor exited"
                    )
                )
            ],
        )
    )
    description = launch.LaunchDescription(
        [epuck1, epuck2, coordinator, shutdown_on_task_end]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
