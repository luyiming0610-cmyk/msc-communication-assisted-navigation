#!/usr/bin/env python3
#
# pilot_v4_combined: task monitor for the combined wooden-box + moving-peer
# scenario under controller_v4_ros_time_consistency. Identical to the
# pre-v4 combined_task_coordinator.py EXCEPT for one pre-diagnosed fix:
# ENCOUNTER_DISTANCE_M/RELEASE_DISTANCE_M are widened. This is NOT an
# avoidance/CPA controller parameter -- it only affects when THIS external
# monitor (sampling the periodic /epuck1,2/state topics at its own 0.1s
# poll rate) declares the encounter "seen". The prior 0.22/0.28 m pair was
# root-caused in combined_wood_moving_peer_README.md (2026-07-17,
# controller_v1 pilot_04): true minimum separation was 0.202 m (via full
# bag replay) but the coordinator's own periodic-topic sampling never saw
# better than 0.272 m, so the encounter was never latched and the run timed
# out even though the underlying CPA avoidance was genuinely correct.

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
ENCOUNTER_DISTANCE_M = 0.30
RELEASE_DISTANCE_M = 0.40
STOPPED_SPEED_MPS = 0.003
TIMEOUT_S = 95.0


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = rclpy.create_node("combined_task_coordinator_v4")
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
