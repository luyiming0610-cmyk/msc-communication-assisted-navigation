#!/usr/bin/env python3
"""Read-only trajectory/mode audit for Objective 5 matrix trials."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TRANSITION_RE = re.compile(
    r"\[cooperative_avoider-(?P<robot>[12])\].*?TRANSITION "
    r"wall_time=(?P<wall>[0-9.]+) ros_time=(?P<ros>[0-9.]+) "
    r"mode=(?P<old>[^ ]+)->(?P<new>[^ ]+)"
)


def unwrap(values: list[float]) -> list[float]:
    if not values:
        return []
    out = [values[0]]
    for value in values[1:]:
        delta = (value - out[-1] + math.pi) % (2.0 * math.pi) - math.pi
        out.append(out[-1] + delta)
    return out


def integrate_abs(samples: list[tuple[float, float]]) -> float:
    total = 0.0
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        dt = max(0.0, t1 - t0)
        total += 0.5 * (abs(v0) + abs(v1)) * dt
    return total


def integrate_signed(samples: list[tuple[float, float]]) -> float:
    total = 0.0
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        dt = max(0.0, t1 - t0)
        total += 0.5 * (v0 + v1) * dt
    return total


def sign_runs(samples: list[tuple[float, float]], threshold: float = 0.05) -> list[int]:
    runs: list[int] = []
    for _, value in samples:
        sign = 1 if value >= threshold else -1 if value <= -threshold else 0
        if sign and (not runs or runs[-1] != sign):
            runs.append(sign)
    return runs


def sign_run_details(
    samples: list[tuple[float, float]], threshold: float = 0.05
) -> list[dict]:
    if not samples:
        return []
    origin = samples[0][0]
    details: list[dict] = []
    active_sign = 0
    for timestamp, value in samples:
        sign = 1 if value >= threshold else -1 if value <= -threshold else 0
        if not sign or sign == active_sign:
            continue
        active_sign = sign
        details.append(
            {
                "sign": sign,
                "start_offset_s": timestamp - origin,
                "angular_rps_at_start": value,
            }
        )
    return details


def parse_transitions(log_path: Path) -> dict[int, list[dict]]:
    result: dict[int, list[dict]] = {1: [], 2: []}
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = TRANSITION_RE.search(line)
        if not match:
            continue
        item = match.groupdict()
        result[int(item["robot"])].append(
            {
                "wall_time": float(item["wall"]),
                "ros_time": float(item["ros"]),
                "old": item["old"],
                "new": item["new"],
            }
        )
    return result


def mode_intervals(transitions: list[dict]) -> dict[str, list[tuple[float, float]]]:
    intervals: dict[str, list[tuple[float, float]]] = {}
    for current, nxt in zip(transitions, transitions[1:]):
        intervals.setdefault(current["new"], []).append(
            (current["wall_time"], nxt["wall_time"])
        )
    return intervals


def read_bag(bag_path: Path) -> dict[str, list[tuple]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {
        "/epuck1/cmd_vel",
        "/epuck2/cmd_vel",
        "/epuck1/state_raw",
        "/epuck2/state_raw",
    }
    messages: dict[str, list[tuple]] = {topic: [] for topic in wanted}
    classes = {topic: get_message(topic_types[topic]) for topic in wanted}
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(raw, classes[topic])
        t = timestamp_ns / 1e9
        if topic.endswith("cmd_vel"):
            messages[topic].append((t, float(msg.linear.x), float(msg.angular.z)))
        else:
            messages[topic].append(
                (t, float(msg.x_m), float(msg.y_m), float(msg.yaw_rad))
            )
    return messages


def select(samples: list[tuple], start: float, end: float) -> list[tuple]:
    return [item for item in samples if start <= item[0] <= end]


def trajectory_metrics(samples: list[tuple]) -> dict:
    if len(samples) < 2:
        return {"sample_count": len(samples)}
    yaws = unwrap([item[3] for item in samples])
    path_length = sum(
        math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(samples, samples[1:])
    )
    displacement = math.hypot(
        samples[-1][1] - samples[0][1], samples[-1][2] - samples[0][2]
    )
    yaw_variation = sum(abs(b - a) for a, b in zip(yaws, yaws[1:]))
    return {
        "sample_count": len(samples),
        "path_length_m": path_length,
        "displacement_m": displacement,
        "path_efficiency": displacement / path_length if path_length else None,
        "lateral_span_m": max(item[2] for item in samples) - min(item[2] for item in samples),
        "yaw_min_rad": min(yaws),
        "yaw_max_rad": max(yaws),
        "yaw_range_rad": max(yaws) - min(yaws),
        "yaw_total_variation_rad": yaw_variation,
        "yaw_net_change_rad": yaws[-1] - yaws[0],
    }


def command_metrics(samples: list[tuple]) -> dict:
    angular = [(item[0], item[2]) for item in samples]
    runs = sign_runs(angular)
    return {
        "sample_count": len(samples),
        "duration_s": samples[-1][0] - samples[0][0] if len(samples) >= 2 else 0.0,
        "angular_sign_runs_threshold_0_05": runs,
        "angular_sign_run_details": sign_run_details(angular),
        "angular_direction_reversals": max(0, len(runs) - 1),
        "absolute_angular_command_integral_rad": integrate_abs(angular),
        "signed_angular_command_integral_rad": integrate_signed(angular),
        "max_abs_angular_command_rps": max((abs(item[2]) for item in samples), default=0.0),
        "mean_linear_command_mps": (
            sum(item[1] for item in samples) / len(samples) if samples else None
        ),
    }


def analyze_trial(bag: Path, log: Path, label: str) -> dict:
    transitions = parse_transitions(log)
    bag_data = read_bag(bag)
    report = {"label": label, "robots": {}}
    for robot in (1, 2):
        robot_transitions = transitions[robot]
        intervals = mode_intervals(robot_transitions)
        mode_counts: dict[str, int] = {}
        for item in robot_transitions:
            mode_counts[item["new"]] = mode_counts.get(item["new"], 0) + 1
        avoid_pass = intervals.get("AVOID_PASS", [])
        if not avoid_pass:
            pass_metrics = {"present": False}
        else:
            start, end = avoid_pass[0]
            cmd = select(bag_data[f"/epuck{robot}/cmd_vel"], start, end)
            state = select(bag_data[f"/epuck{robot}/state_raw"], start, end)
            pass_metrics = {
                "present": True,
                "start_wall_time": start,
                "end_wall_time": end,
                "duration_s": end - start,
                "command": command_metrics(cmd),
                "trajectory": trajectory_metrics(state),
            }
        local_transitions = {
            mode: count
            for mode, count in mode_counts.items()
            if mode.startswith("LOCAL_") or mode.startswith("SAFE_STOP_LOCAL")
        }
        report["robots"][str(robot)] = {
            "mode_entry_counts": mode_counts,
            "cpa_avoid_turn_entries": mode_counts.get("AVOID_TURN", 0),
            "cpa_avoid_pass_entries": mode_counts.get("AVOID_PASS", 0),
            "cpa_recover_entries": mode_counts.get("RECOVER", 0),
            "local_transition_counts": local_transitions,
            "avoid_pass": pass_metrics,
        }
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--label", required=True)
    args = parser.parse_args()
    print(json.dumps(analyze_trial(args.bag, args.log, args.label), indent=2))


if __name__ == "__main__":
    main()
