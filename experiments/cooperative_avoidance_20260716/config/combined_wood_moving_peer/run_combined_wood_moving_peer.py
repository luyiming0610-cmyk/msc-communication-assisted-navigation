#!/usr/bin/env python3

import os
import sys

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


WORK_DIR = os.path.dirname(os.path.abspath(__file__))
LAUNCH_FILE = os.path.join(WORK_DIR, "dual_namespaced_launch.py")


def main():
    os.environ["EPUCK_WORLD_FILE"] = "combined_wood_moving_peer_world.wbt"
    description = launch.LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(LAUNCH_FILE))]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
