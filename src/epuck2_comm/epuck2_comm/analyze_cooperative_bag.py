"""Extract reproducible cooperative-avoidance metrics from a ROS 2 bag."""

import argparse
import csv
import json
import math
from pathlib import Path


STATE_TOPICS = ("/epuck1/state", "/epuck2/state")
COMMAND_TOPICS = ("/epuck1/cmd_vel", "/epuck2/cmd_vel")
ODOM_VALID_FLAG = 1


def command_metrics(records, zero_threshold=1.0e-4, turn_threshold=0.05):
    """Summarize command timing and smoothness from (time, linear, angular)."""
    if not records:
        return {
            "message_count": 0,
            "motion_start_s": None,
            "last_motion_command_s": None,
            "peak_abs_linear_mps": 0.0,
            "peak_abs_angular_rps": 0.0,
            "max_linear_step_mps": 0.0,
            "max_angular_step_rps": 0.0,
            "max_linear_slew_mps2": 0.0,
            "max_angular_slew_rps2": 0.0,
            "angular_sign_changes": 0,
        }

    moving = [
        row for row in records
        if abs(row[1]) > zero_threshold or abs(row[2]) > zero_threshold
    ]
    max_linear_step = 0.0
    max_angular_step = 0.0
    max_linear_slew = 0.0
    max_angular_slew = 0.0
    angular_sign_changes = 0
    previous_turn_sign = 0

    for previous, current in zip(records, records[1:]):
        dt = current[0] - previous[0]
        linear_step = abs(current[1] - previous[1])
        angular_step = abs(current[2] - previous[2])
        max_linear_step = max(max_linear_step, linear_step)
        max_angular_step = max(max_angular_step, angular_step)
        if dt > 1.0e-6:
            max_linear_slew = max(max_linear_slew, linear_step / dt)
            max_angular_slew = max(max_angular_slew, angular_step / dt)

    for _, _, angular in records:
        if abs(angular) <= turn_threshold:
            continue
        sign = 1 if angular > 0.0 else -1
        if previous_turn_sign and sign != previous_turn_sign:
            angular_sign_changes += 1
        previous_turn_sign = sign

    return {
        "message_count": len(records),
        "motion_start_s": moving[0][0] if moving else None,
        "last_motion_command_s": moving[-1][0] if moving else None,
        "peak_abs_linear_mps": max(abs(row[1]) for row in records),
        "peak_abs_angular_rps": max(abs(row[2]) for row in records),
        "max_linear_step_mps": max_linear_step,
        "max_angular_step_rps": max_angular_step,
        "max_linear_slew_mps2": max_linear_slew,
        "max_angular_slew_rps2": max_angular_slew,
        "angular_sign_changes": angular_sign_changes,
    }


def _read_bag(bag_path):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "Run this command inside a sourced ROS 2 Humble environment."
        ) from error

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    type_names = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(type_name) for topic, type_name in type_names.items()
    }

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in STATE_TOPICS + COMMAND_TOPICS:
            continue
        yield topic, deserialize_message(data, message_types[topic]), timestamp_ns


def analyze_bag(bag_path, robot_radius_m=0.035):
    first_timestamp_ns = None
    last_timestamp_ns = None
    states = {topic: [] for topic in STATE_TOPICS}
    commands = {topic: [] for topic in COMMAND_TOPICS}
    invalid_state_messages = {topic: 0 for topic in STATE_TOPICS}
    separation_rows = []
    latest_state = {}

    for topic, message, timestamp_ns in _read_bag(bag_path):
        if first_timestamp_ns is None:
            first_timestamp_ns = timestamp_ns
        last_timestamp_ns = timestamp_ns
        relative_s = (timestamp_ns - first_timestamp_ns) / 1.0e9

        if topic in COMMAND_TOPICS:
            commands[topic].append(
                (relative_s, float(message.linear.x), float(message.angular.z))
            )
            continue

        valid = (
            int(message.validity_flags) & ODOM_VALID_FLAG
            and math.isfinite(float(message.x_m))
            and math.isfinite(float(message.y_m))
        )
        if not valid:
            invalid_state_messages[topic] += 1
            continue
        row = (relative_s, float(message.x_m), float(message.y_m))
        states[topic].append(row)
        latest_state[topic] = row
        if all(state_topic in latest_state for state_topic in STATE_TOPICS):
            first = latest_state[STATE_TOPICS[0]]
            second = latest_state[STATE_TOPICS[1]]
            separation_rows.append(
                (
                    relative_s,
                    math.hypot(first[1] - second[1], first[2] - second[2]),
                )
            )

    if first_timestamp_ns is None or last_timestamp_ns is None:
        raise RuntimeError("No cooperative state or command messages were found.")
    if not separation_rows:
        raise RuntimeError("No valid paired /epuck1/state and /epuck2/state samples found.")

    minimum_row = min(separation_rows, key=lambda row: row[1])
    final_row = separation_rows[-1]
    command_summary = {
        topic: command_metrics(records) for topic, records in commands.items()
    }
    starts = [
        metrics["motion_start_s"] for metrics in command_summary.values()
        if metrics["motion_start_s"] is not None
    ]
    stops = [
        metrics["last_motion_command_s"] for metrics in command_summary.values()
        if metrics["last_motion_command_s"] is not None
    ]
    collision_distance_m = 2.0 * robot_radius_m

    summary = {
        "bag_path": str(bag_path),
        "bag_duration_s": (last_timestamp_ns - first_timestamp_ns) / 1.0e9,
        "robot_radius_m": robot_radius_m,
        "collision_distance_m": collision_distance_m,
        "paired_state_samples": len(separation_rows),
        "state_message_count": {
            topic: len(records) for topic, records in states.items()
        },
        "invalid_state_messages": invalid_state_messages,
        "minimum_center_separation_m": minimum_row[1],
        "minimum_separation_time_s": minimum_row[0],
        "minimum_safety_margin_m": minimum_row[1] - collision_distance_m,
        "collision_detected": minimum_row[1] <= collision_distance_m,
        "final_center_separation_m": final_row[1],
        "motion_start_skew_s": max(starts) - min(starts) if len(starts) == 2 else None,
        "last_motion_command_skew_s": (
            max(stops) - min(stops) if len(stops) == 2 else None
        ),
        "commands": command_summary,
    }
    return summary, separation_rows, commands


def _write_outputs(output_dir, summary, separation_rows, commands):
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "summary.json").open("w", encoding="utf-8") as output:
        json.dump(summary, output, indent=2, ensure_ascii=False)
        output.write("\n")

    with (output_dir / "separation.csv").open(
        "w", newline="", encoding="utf-8"
    ) as output:
        writer = csv.writer(output)
        writer.writerow(("time_s", "center_separation_m"))
        writer.writerows(separation_rows)

    with (output_dir / "commands.csv").open(
        "w", newline="", encoding="utf-8"
    ) as output:
        writer = csv.writer(output)
        writer.writerow(("topic", "time_s", "linear_mps", "angular_rps"))
        for topic, records in commands.items():
            for record in records:
                writer.writerow((topic,) + record)


def _arguments():
    parser = argparse.ArgumentParser(
        description="Analyze a two-e-puck cooperative-avoidance ROS 2 bag."
    )
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--robot-radius-m", type=float, default=0.035)
    return parser.parse_args()


def main():
    args = _arguments()
    bag_path = args.bag_path.expanduser().resolve()
    output_dir = args.output_dir or bag_path / "analysis"
    summary, separation_rows, commands = analyze_bag(
        bag_path, robot_radius_m=args.robot_radius_m
    )
    _write_outputs(output_dir, summary, separation_rows, commands)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"Analysis written to: {output_dir}")


if __name__ == "__main__":
    main()
