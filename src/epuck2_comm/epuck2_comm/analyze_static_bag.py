"""Extract trajectory and obstacle-clearance metrics from a static trial bag."""

import argparse
import csv
import json
import math
from pathlib import Path

from .analyze_cooperative_bag import _read_bag, command_metrics


ROBOT_STATE_TOPIC = "/epuck1/state"
PEER_STATE_TOPIC = "/epuck2/state"
ROBOT_COMMAND_TOPIC = "/epuck1/cmd_vel"
ODOM_VALID_FLAG = 1


def box_surface_clearance(
    x_m,
    y_m,
    box_x_m,
    box_y_m,
    box_size_x_m,
    box_size_y_m,
    robot_radius_m,
):
    """Signed circle-to-axis-aligned-box clearance; <= 0 means contact."""
    dx = max(abs(float(x_m) - float(box_x_m)) - box_size_x_m / 2.0, 0.0)
    dy = max(abs(float(y_m) - float(box_y_m)) - box_size_y_m / 2.0, 0.0)
    return math.hypot(dx, dy) - float(robot_radius_m)


def trajectory_metrics(
    records,
    *,
    course_heading_rad,
    box_x_m,
    box_y_m,
    box_size_x_m,
    box_size_y_m,
    robot_radius_m,
):
    if not records:
        raise ValueError("At least one valid trajectory record is required.")

    path_length = 0.0
    for previous, current in zip(records, records[1:]):
        path_length += math.hypot(current[1] - previous[1], current[2] - previous[2])

    initial = records[0]
    final = records[-1]
    heading_x = math.cos(course_heading_rad)
    heading_y = math.sin(course_heading_rad)
    normal_x = -heading_y
    normal_y = heading_x
    forward_values = []
    lateral_values = []
    clearances = []
    finite_front = []

    for row in records:
        dx = row[1] - initial[1]
        dy = row[2] - initial[2]
        forward_values.append(dx * heading_x + dy * heading_y)
        lateral_values.append(dx * normal_x + dy * normal_y)
        clearances.append(
            box_surface_clearance(
                row[1],
                row[2],
                box_x_m,
                box_y_m,
                box_size_x_m,
                box_size_y_m,
                robot_radius_m,
            )
        )
        if math.isfinite(row[4]) and row[4] >= 0.0:
            finite_front.append(row[4])

    obstacle_forward = (
        (box_x_m - initial[1]) * heading_x
        + (box_y_m - initial[2]) * heading_y
    )
    obstacle_half_extent = (
        abs(heading_x) * box_size_x_m / 2.0
        + abs(heading_y) * box_size_y_m / 2.0
    )
    passed_threshold = obstacle_forward + obstacle_half_extent + robot_radius_m
    final_progress = forward_values[-1]

    return {
        "state_samples": len(records),
        "path_length_m": path_length,
        "forward_progress_m": final_progress,
        "maximum_forward_progress_m": max(forward_values),
        "path_efficiency": (
            final_progress / path_length if path_length > 1.0e-9 else None
        ),
        "maximum_abs_lateral_deviation_m": max(abs(v) for v in lateral_values),
        "final_x_m": final[1],
        "final_y_m": final[2],
        "final_yaw_rad": final[3],
        "minimum_measured_front_range_m": (
            min(finite_front) if finite_front else None
        ),
        "minimum_box_surface_clearance_m": min(clearances),
        "box_collision_detected": min(clearances) <= 0.0,
        "obstacle_passed": max(forward_values) > passed_threshold,
        "obstacle_pass_threshold_m": passed_threshold,
    }


def _valid_state(message):
    return (
        int(message.validity_flags) & ODOM_VALID_FLAG
        and math.isfinite(float(message.x_m))
        and math.isfinite(float(message.y_m))
        and math.isfinite(float(message.yaw_rad))
    )


def analyze_static_bag(
    bag_path,
    *,
    box_x_m=-0.25,
    box_y_m=0.0,
    box_size_x_m=0.06,
    box_size_y_m=0.06,
    robot_radius_m=0.035,
    course_heading_rad=0.0,
):
    first_timestamp_ns = None
    last_timestamp_ns = None
    robot_states = []
    peer_states = []
    commands = []
    invalid_robot_states = 0

    for topic, message, timestamp_ns in _read_bag(bag_path):
        if first_timestamp_ns is None:
            first_timestamp_ns = timestamp_ns
        last_timestamp_ns = timestamp_ns
        relative_s = (timestamp_ns - first_timestamp_ns) / 1.0e9

        if topic == ROBOT_COMMAND_TOPIC:
            commands.append(
                (relative_s, float(message.linear.x), float(message.angular.z))
            )
        elif topic == ROBOT_STATE_TOPIC:
            if not _valid_state(message):
                invalid_robot_states += 1
                continue
            robot_states.append(
                (
                    relative_s,
                    float(message.x_m),
                    float(message.y_m),
                    float(message.yaw_rad),
                    float(message.front_distance_m),
                    float(message.left_distance_m),
                    float(message.right_distance_m),
                )
            )
        elif topic == PEER_STATE_TOPIC and _valid_state(message):
            peer_states.append(
                (relative_s, float(message.x_m), float(message.y_m))
            )

    if first_timestamp_ns is None or last_timestamp_ns is None:
        raise RuntimeError("No supported state or command messages were found.")

    trajectory = trajectory_metrics(
        robot_states,
        course_heading_rad=course_heading_rad,
        box_x_m=box_x_m,
        box_y_m=box_y_m,
        box_size_x_m=box_size_x_m,
        box_size_y_m=box_size_y_m,
        robot_radius_m=robot_radius_m,
    )
    peer_displacement = None
    if peer_states:
        peer_displacement = math.hypot(
            peer_states[-1][1] - peer_states[0][1],
            peer_states[-1][2] - peer_states[0][2],
        )
    final_command_zero = None
    if commands:
        final_command_zero = (
            abs(commands[-1][1]) <= 1.0e-4
            and abs(commands[-1][2]) <= 1.0e-4
        )

    summary = {
        "bag_path": str(bag_path),
        "bag_duration_s": (last_timestamp_ns - first_timestamp_ns) / 1.0e9,
        "invalid_robot_state_messages": invalid_robot_states,
        "box_geometry": {
            "center_x_m": box_x_m,
            "center_y_m": box_y_m,
            "size_x_m": box_size_x_m,
            "size_y_m": box_size_y_m,
            "robot_radius_m": robot_radius_m,
        },
        "trajectory": trajectory,
        "commands": command_metrics(commands),
        "peer_state_samples": len(peer_states),
        "peer_displacement_m": peer_displacement,
        "final_command_zero": final_command_zero,
    }
    return summary, robot_states, commands


def _write_outputs(output_dir, summary, trajectory, commands):
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "static_summary.json").open(
        "w", encoding="utf-8"
    ) as output:
        json.dump(summary, output, indent=2, ensure_ascii=False)
        output.write("\n")

    with (output_dir / "trajectory.csv").open(
        "w", newline="", encoding="utf-8"
    ) as output:
        writer = csv.writer(output)
        writer.writerow(
            (
                "time_s",
                "x_m",
                "y_m",
                "yaw_rad",
                "front_distance_m",
                "left_distance_m",
                "right_distance_m",
            )
        )
        writer.writerows(trajectory)

    with (output_dir / "static_commands.csv").open(
        "w", newline="", encoding="utf-8"
    ) as output:
        writer = csv.writer(output)
        writer.writerow(("time_s", "linear_mps", "angular_rps"))
        writer.writerows(commands)


def _arguments():
    parser = argparse.ArgumentParser(
        description="Analyze a long-course static-obstacle ROS 2 bag."
    )
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--box-x-m", type=float, default=-0.25)
    parser.add_argument("--box-y-m", type=float, default=0.0)
    parser.add_argument("--box-size-x-m", type=float, default=0.06)
    parser.add_argument("--box-size-y-m", type=float, default=0.06)
    parser.add_argument("--robot-radius-m", type=float, default=0.035)
    parser.add_argument("--course-heading-rad", type=float, default=0.0)
    return parser.parse_args()


def main():
    args = _arguments()
    bag_path = args.bag_path.expanduser().resolve()
    output_dir = args.output_dir or bag_path / "analysis"
    summary, trajectory, commands = analyze_static_bag(
        bag_path,
        box_x_m=args.box_x_m,
        box_y_m=args.box_y_m,
        box_size_x_m=args.box_size_x_m,
        box_size_y_m=args.box_size_y_m,
        robot_radius_m=args.robot_radius_m,
        course_heading_rad=args.course_heading_rad,
    )
    _write_outputs(output_dir, summary, trajectory, commands)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"Static analysis written to: {output_dir}")


if __name__ == "__main__":
    main()
