#!/usr/bin/env python3
#
# Configurable relay + sequence_counter launcher for the timestamp/latency
# validation pilot (objective5_timestamp_latency_validation_pilot01).
# Unlike run_diagnostic_relay_and_counter.py (always zero-impairment),
# this accepts --delay-s/--jitter-s/--drop-probability/--seed so the same
# scenario can be run at a configured nonzero delay to check the OBSERVED
# latency against the CONFIGURED one. No cooperative_avoider is launched
# here either -- this stays a comm-layer-only diagnostic, never formal.

import argparse
import sys

import launch
from launch_ros.actions import Node


def make_relay(namespace, seed, delay_s, jitter_s, drop_probability, log_path):
    return Node(
        package="epuck2_comm",
        executable="network_impairment_relay",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "delay_s": delay_s,
                "jitter_s": jitter_s,
                "drop_probability": drop_probability,
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--relay-log-dir", required=True)
    parser.add_argument("--counter-log-dir", required=True)
    parser.add_argument("--delay-s", type=float, default=0.0)
    parser.add_argument("--jitter-s", type=float, default=0.0)
    parser.add_argument("--drop-probability", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=3001)
    args = parser.parse_args()

    description = launch.LaunchDescription(
        [
            make_relay("epuck1", args.seed, args.delay_s, args.jitter_s, args.drop_probability,
                       f"{args.relay_log_dir}/epuck1_relay.csv"),
            make_relay("epuck2", args.seed + 1, args.delay_s, args.jitter_s, args.drop_probability,
                       f"{args.relay_log_dir}/epuck2_relay.csv"),
            make_counter("epuck1", f"{args.counter_log_dir}/epuck1_counter.json"),
            make_counter("epuck2", f"{args.counter_log_dir}/epuck2_counter.json"),
        ]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
