#!/usr/bin/env python3
#
# Objective 5 step 3: communication baseline pilot. Same head_on_cpa_v4
# geometry/CPA parameters (frozen, unchanged) but each robot's peer state
# now flows through a network_impairment_relay configured for ZERO
# impairment (delay_s=jitter_s=drop_probability=0.0), inserted between
# state_publisher (remapped to publish "state_raw") and cooperative_avoider
# (still subscribes to "state", unaware a relay is even present). This
# validates the relay and the comm-performance analyzer before any actual
# delay/jitter/loss is introduced.

import sys
from pathlib import Path

import launch
from launch_ros.actions import Node


CONFIG_DIR = Path(__file__).resolve().parent
LOG_DIR = CONFIG_DIR.parent.parent / "logs"


def make_relay(namespace, seed, log_stem):
    return Node(
        package="epuck2_comm",
        executable="network_impairment_relay",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "delay_s": 0.0,
                "jitter_s": 0.0,
                "drop_probability": 0.0,
                "seed": seed,
                "log_path": str(LOG_DIR / f"{log_stem}_{namespace}_relay.csv"),
                "use_sim_time": True,
            }
        ],
    )


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
    if len(sys.argv) < 2:
        print("usage: run_comm_baseline_controllers.py LOG_STEM", file=sys.stderr)
        return 2
    log_stem = sys.argv[1]

    description = launch.LaunchDescription(
        [
            make_relay("epuck1", seed=1001, log_stem=log_stem),
            make_relay("epuck2", seed=1002, log_stem=log_stem),
            make_controller("epuck1", 1, "/epuck2/state", 0.0),
            make_controller("epuck2", 2, "/epuck1/state", 3.141592653589793),
        ]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
