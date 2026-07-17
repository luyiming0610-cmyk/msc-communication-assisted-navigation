#!/usr/bin/env python3
#
# Measurement-chain isolation diagnostic: launches ONLY the relay (zero
# impairment) and the sequence_counter observer, deliberately BEFORE
# state_publisher starts (see run_comm_baseline_native_diagnostic.sh),
# so their subscriptions exist from sequence 0. No cooperative_avoider is
# launched -- this diagnostic isolates the communication layer only.

import sys

import launch
from launch_ros.actions import Node


def make_relay(namespace, seed, log_path):
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
                "log_path": log_path,
                "use_sim_time": True,
            }
        ],
    )


def make_counter(namespace, output_path):
    return Node(
        package="epuck2_comm",
        executable="sequence_counter",
        namespace=namespace,
        output="screen",
        arguments=["--topics", "state_raw", "state", "--output-path", output_path],
        parameters=[{"use_sim_time": True}],
    )


def main():
    if len(sys.argv) < 3:
        print(
            "usage: run_diagnostic_relay_and_counter.py RELAY_LOG_DIR COUNTER_LOG_DIR",
            file=sys.stderr,
        )
        return 2
    relay_log_dir = sys.argv[1]
    counter_log_dir = sys.argv[2]

    description = launch.LaunchDescription(
        [
            make_relay("epuck1", seed=2001, log_path=f"{relay_log_dir}/epuck1_relay.csv"),
            make_relay("epuck2", seed=2002, log_path=f"{relay_log_dir}/epuck2_relay.csv"),
            make_counter("epuck1", f"{counter_log_dir}/epuck1_counter.json"),
            make_counter("epuck2", f"{counter_log_dir}/epuck2_counter.json"),
        ]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
