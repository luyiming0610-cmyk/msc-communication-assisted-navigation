from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    armed = LaunchConfiguration("armed")
    max_runtime = LaunchConfiguration("max_runtime_s")
    stop_after_recovery = LaunchConfiguration("stop_after_recovery")
    post_recovery_hold = LaunchConfiguration("post_recovery_hold_s")
    enable_local = LaunchConfiguration("enable_local_avoidance")
    require_local = LaunchConfiguration("require_local_sensors")
    common = {
        "armed": ParameterValue(armed, value_type=bool),
        "max_runtime_s": ParameterValue(max_runtime, value_type=float),
        "stop_after_recovery": ParameterValue(
            stop_after_recovery, value_type=bool
        ),
        "post_recovery_hold_s": ParameterValue(
            post_recovery_hold, value_type=float
        ),
        "use_sim_time": True,
        "enable_local_avoidance": ParameterValue(enable_local, value_type=bool),
        "require_local_sensors": ParameterValue(require_local, value_type=bool),
    }

    epuck1 = Node(
        package="epuck2_comm",
        executable="cooperative_avoider",
        namespace="epuck1",
        output="screen",
        parameters=[
            common,
            {
                "robot_id": 1,
                "peer_state_topic": "/epuck2/state",
                "desired_heading_rad": 0.0,
            },
        ],
    )
    epuck2 = Node(
        package="epuck2_comm",
        executable="cooperative_avoider",
        namespace="epuck2",
        output="screen",
        parameters=[
            common,
            {
                "robot_id": 2,
                "peer_state_topic": "/epuck1/state",
                "desired_heading_rad": 3.141592653589793,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("armed", default_value="false"),
            DeclareLaunchArgument("max_runtime_s", default_value="22.0"),
            DeclareLaunchArgument("stop_after_recovery", default_value="false"),
            DeclareLaunchArgument("post_recovery_hold_s", default_value="0.5"),
            DeclareLaunchArgument("enable_local_avoidance", default_value="true"),
            DeclareLaunchArgument("require_local_sensors", default_value="true"),
            epuck1,
            epuck2,
        ]
    )
