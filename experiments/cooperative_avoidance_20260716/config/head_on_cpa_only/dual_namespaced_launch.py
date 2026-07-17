#!/usr/bin/env python3

import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


WORK_DIR = os.path.dirname(os.path.abspath(__file__))
WORLD_PATH = os.path.join(
    WORK_DIR,
    os.environ.get("EPUCK_WORLD_FILE", "two_epuck_namespaced_world.wbt"),
)


def make_driver(robot_name):
    return WebotsController(
        robot_name=robot_name,
        namespace=robot_name,
        parameters=[
            {
                "robot_description": os.path.join(WORK_DIR, "epuck_namespaced.urdf"),
                "use_sim_time": True,
                "set_robot_state_publisher": True,
            },
            os.path.join(WORK_DIR, f"{robot_name}_ros2_control.yml"),
        ],
        remappings=[
            ("diffdrive_controller/cmd_vel_unstamped", "cmd_vel"),
            ("diffdrive_controller/odom", "odom"),
        ],
        respawn=False,
    )


def make_spawner(robot_name, controller_name):
    return Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        output="screen",
        arguments=[
            controller_name,
            "--controller-manager",
            f"/{robot_name}/controller_manager",
            "--controller-manager-timeout",
            "80",
        ],
    )


def generate_launch_description():
    webots = WebotsLauncher(
        world=WORLD_PATH,
        ros2_supervisor=True,
        mode="realtime",
    )

    epuck1_driver = make_driver("epuck1")
    epuck2_driver = make_driver("epuck2")

    epuck1_joint = make_spawner("epuck1", "joint_state_broadcaster")
    epuck1_diff = make_spawner("epuck1", "diffdrive_controller")
    epuck2_joint = make_spawner("epuck2", "joint_state_broadcaster")
    epuck2_diff = make_spawner("epuck2", "diffdrive_controller")

    epuck1_wait = WaitForControllerConnection(
        target_driver=epuck1_driver,
        nodes_to_start=[
            launch.actions.TimerAction(period=3.0, actions=[epuck1_joint])
        ],
    )
    epuck2_wait = WaitForControllerConnection(
        target_driver=epuck2_driver,
        nodes_to_start=[
            launch.actions.TimerAction(period=3.0, actions=[epuck2_joint])
        ],
    )

    epuck1_start_diff = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=epuck1_joint,
            on_exit=[epuck1_diff],
        )
    )
    epuck2_start_diff = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=epuck2_joint,
            on_exit=[epuck2_diff],
        )
    )

    shutdown_when_webots_exits = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription(
        [
            webots,
            webots._supervisor,
            epuck1_driver,
            epuck2_driver,
            epuck1_wait,
            epuck2_wait,
            epuck1_start_diff,
            epuck2_start_diff,
            shutdown_when_webots_exits,
        ]
    )
