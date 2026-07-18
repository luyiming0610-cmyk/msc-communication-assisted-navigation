#!/usr/bin/env python3
#
# Objective5 impairment matrix -- launches BOTH relay instances (one per
# robot's outgoing state stream, each with its OWN seed -- see the
# design doc's matched-but-not-identical seed scheme) plus the
# sequence_counter observer, deliberately BEFORE state_publisher starts,
# matching the proven ordering in run_diagnostic_relay_and_counter.py
# (which this file is directly modeled on -- same Node/parameters/
# arguments shape, only the relay parameters are now CLI-configurable
# instead of hardcoded zero-impairment).

import argparse
import sys

import launch
from launch_ros.actions import Node


def make_relay(namespace, seed, delay_s, jitter_s, drop_probability,
                outage_period_s, outage_duration_s, outage_phase_s, log_path):
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
                "outage_period_s": outage_period_s,
                "outage_duration_s": outage_duration_s,
                "outage_phase_s": outage_phase_s,
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
    parser.add_argument("--diag-log-dir", required=True)
    parser.add_argument("--delay-s", type=float, required=True)
    parser.add_argument("--jitter-s", type=float, required=True)
    parser.add_argument("--drop-probability", type=float, required=True)
    parser.add_argument("--outage-period-s", type=float, default=0.0)
    parser.add_argument("--outage-duration-s", type=float, default=0.0)
    parser.add_argument("--outage-phase-s", type=float, default=0.0)
    parser.add_argument("--seed-epuck1", type=int, required=True)
    parser.add_argument("--seed-epuck2", type=int, required=True)
    args = parser.parse_args()

    description = launch.LaunchDescription(
        [
            make_relay(
                "epuck1", args.seed_epuck1, args.delay_s, args.jitter_s, args.drop_probability,
                args.outage_period_s, args.outage_duration_s, args.outage_phase_s,
                f"{args.diag_log_dir}/epuck1_relay.csv",
            ),
            make_relay(
                "epuck2", args.seed_epuck2, args.delay_s, args.jitter_s, args.drop_probability,
                args.outage_period_s, args.outage_duration_s, args.outage_phase_s,
                f"{args.diag_log_dir}/epuck2_relay.csv",
            ),
            make_counter("epuck1", f"{args.diag_log_dir}/epuck1_counter.json"),
            make_counter("epuck2", f"{args.diag_log_dir}/epuck2_counter.json"),
        ]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
