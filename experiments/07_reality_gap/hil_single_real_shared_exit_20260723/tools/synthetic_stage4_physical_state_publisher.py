#!/usr/bin/env python3
"""TEST-ONLY synthetic stand-in for the real physical state_publisher.py,
used ONLY by the Stage 4 hardware-free live ROS-graph rehearsal
(test_hil_stage4_live_graph_rehearsal.py). Never used by
run_hil_stage4_trial.sh, which always requires exactly one REAL state
publisher on the real /epuck1/state topic (checked explicitly by that
script's preflight).

This node exists because the design review explicitly permits synthetic
test-only publishers for "physical inputs that are unavailable without
hardware" (real-robot state, bridge/liveness state) while still
exercising the real cooperative_avoider, real guard, and real Stage 4
supervisor unmodified. It publishes a single fixed, valid EpuckState
(validity_flags=7, a pose inside the frozen corridor) at a fixed rate,
and nothing else -- no motion, no scripted trajectory, no goal logic.
"""
from __future__ import annotations

import argparse


def main(argv=None):
    import sys
    from pathlib import Path

    import rclpy
    from rclpy.node import Node

    from epuck2_comm_interfaces.msg import EpuckState

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from hil_offline_stage3_harness import apply_synthetic_clear_sensor_fixture

    parser = argparse.ArgumentParser()
    parser.add_argument("--state-topic", required=True)
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--x-m", type=float, default=0.30)
    parser.add_argument("--y-m", type=float, default=0.50)
    parser.add_argument("--yaw-rad", type=float, default=0.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=[])
    node = Node("synthetic_stage4_physical_state_publisher")
    pub = node.create_publisher(EpuckState, args.state_topic, 10)
    sequence = 0

    def tick():
        nonlocal sequence
        sequence += 1
        msg = EpuckState()
        msg.version = EpuckState.PROTOCOL_VERSION
        msg.robot_id = args.robot_id
        msg.sequence = sequence
        msg.stamp = node.get_clock().now().to_msg()
        msg.source = EpuckState.SOURCE_HARDWARE
        msg.x_m = args.x_m
        msg.y_m = args.y_m
        msg.yaw_rad = args.yaw_rad
        msg.validity_flags = (
            EpuckState.FLAG_ODOM_VALID | EpuckState.FLAG_IR_VALID | EpuckState.FLAG_TOF_VALID
        )
        # Reuses the exact same SYNTHETIC_CLEAR_SENSOR_FIXTURE already
        # committed and tested for the Stage 3 harness (+inf, never an
        # implicit 0.0 which decide_local_obstacle() would read as an
        # obstacle at zero range) -- not a new fixture invented here.
        apply_synthetic_clear_sensor_fixture(msg)
        pub.publish(msg)

    node.create_timer(1.0 / args.rate_hz, tick)
    node.get_logger().warn(
        f"SYNTHETIC_STAGE4_PHYSICAL_STATE_PUBLISHER_READY (TEST-ONLY, not a real robot) "
        f"topic={args.state_topic} pose=({args.x_m},{args.y_m},{args.yaw_rad})"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
