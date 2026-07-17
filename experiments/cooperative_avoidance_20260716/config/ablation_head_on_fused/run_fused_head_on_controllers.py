#!/usr/bin/env python3

import sys

import launch
from launch_ros.actions import Node


def make_controller(namespace, robot_id, peer_topic, desired_heading):
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
                "max_runtime_s": 60.0,
                "stop_after_recovery": True,
                "post_recovery_hold_s": 0.5,
                "use_sim_time": True,
                "enable_peer_avoidance": True,
                "enable_local_avoidance": True,
                "require_local_sensors": True,
            }
        ],
    )


def main():
    description = launch.LaunchDescription(
        [
            make_controller("epuck1", 1, "/epuck2/state", 0.0),
            make_controller("epuck2", 2, "/epuck1/state", 3.141592653589793),
        ]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
