#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from epuck2_comm_interfaces.msg import EpuckState


BOX_EAST_FACE_M = -0.22
ROBOT_RADIUS_M = 0.035
CLEARANCE_MARGIN_M = 0.010
PASS_X_M = BOX_EAST_FACE_M + ROBOT_RADIUS_M + CLEARANCE_MARGIN_M
HEADING_TOLERANCE_RAD = 0.10
ENCOUNTER_DISTANCE_M = 0.22
RELEASE_DISTANCE_M = 0.28
STOPPED_SPEED_MPS = 0.003
TIMEOUT_S = 95.0


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = rclpy.create_node("combined_task_coordinator")
    states = {}
    node.create_subscription(
        EpuckState,
        "/epuck1/state",
        lambda message: states.__setitem__("epuck1", message),
        20,
    )
    node.create_subscription(
        EpuckState,
        "/epuck2/state",
        lambda message: states.__setitem__("epuck2", message),
        20,
    )
    started = time.monotonic()
    node.get_logger().info(
        f"TASK_MONITOR_READY pass_x={PASS_X_M:.3f}m "
        f"encounter_distance={ENCOUNTER_DISTANCE_M:.3f}m "
        f"heading_tolerance={HEADING_TOLERANCE_RAD:.3f}rad"
    )
    box_cleared = False
    encounter_seen = False
    exit_code = 2
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.monotonic() - started
        if elapsed >= TIMEOUT_S:
            node.get_logger().error(
                "TASK_TIMEOUT: local-clearance plus cooperative recovery "
                "sequence did not complete"
            )
            break
        first = states.get("epuck1")
        second = states.get("epuck2")
        if first is None or second is None:
            continue
        if float(first.x_m) >= PASS_X_M:
            box_cleared = True
        separation = math.hypot(
            float(first.x_m) - float(second.x_m),
            float(first.y_m) - float(second.y_m),
        )
        if box_cleared and separation <= ENCOUNTER_DISTANCE_M:
            encounter_seen = True
        first_heading_error = abs(normalize_angle(float(first.yaw_rad)))
        second_heading_error = abs(
            normalize_angle(float(second.yaw_rad) - math.pi)
        )
        second_stopped = abs(float(second.linear_velocity_mps)) <= STOPPED_SPEED_MPS
        if (
            box_cleared
            and encounter_seen
            and separation >= RELEASE_DISTANCE_M
            and first_heading_error <= HEADING_TOLERANCE_RAD
            and second_heading_error <= HEADING_TOLERANCE_RAD
            and second_stopped
        ):
            node.get_logger().info(
                "TASK_COMPLETE: epuck1 cleared wooden box and cooperative "
                "encounter recovered; "
                f"epuck1=({float(first.x_m):.3f},{float(first.y_m):.3f},"
                f"{float(first.yaw_rad):.3f}) "
                f"epuck2=({float(second.x_m):.3f},{float(second.y_m):.3f},"
                f"{float(second.yaw_rad):.3f})"
            )
            exit_code = 0
            break

    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
