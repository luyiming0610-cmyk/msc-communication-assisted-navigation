"""Offline PREDICTED_CPA vs PROXIMITY_FALLBACK trigger classification.

controller_v4_ros_time_consistency did not change (and this script does not
touch) the controller's own trigger condition, which has always been the OR
of two independent conditions (see collision_math.collision_risk):

    predicted_conflict = time_to_cpa_s <= horizon_s and distance_at_cpa_m < safety_radius_m
    proximity_conflict = current_distance_m < trigger_distance_m

This script reconstructs, from already-recorded bag data alone, which of
those two conditions was actually true at the first instant either one
became true, using the exact same collision_math formulas the live
controller runs. It never modifies the controller, never reruns a pilot,
and is read-only analysis over an existing rosbag.

Classification:
    PREDICTED_CPA       -- time_to_cpa_s <= horizon_s AND distance_at_cpa_m < safety_radius_m
    PROXIMITY_FALLBACK  -- current_distance_m < trigger_distance_m, predicted condition false
    NONE                -- neither condition held (or closing_speed too low to be a real risk)
"""

import argparse
import csv
import json
from pathlib import Path

from .collision_math import closest_point_of_approach, collision_risk, velocity_vector, CpaResult


STATE_TOPICS = ("/epuck1/state", "/epuck2/state")
ODOM_VALID_FLAG = 1


def _read_state_bag(bag_path):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "Run this command inside a sourced ROS 2 Humble environment."
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_names = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in type_names.items()
        if topic in STATE_TOPICS
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(STATE_TOPICS)))

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        yield topic, deserialize_message(raw, message_types[topic]), timestamp_ns


def _valid(message) -> bool:
    return bool(int(message.validity_flags) & ODOM_VALID_FLAG)


def classify_trigger(
    bag_path: Path,
    horizon_s: float = 4.0,
    safety_radius_m: float = 0.14,
    trigger_distance_m: float = 0.34,
    minimum_closing_speed_mps: float = 0.001,
):
    latest = {}
    first_ns = None
    rows = []  # (time_s, current_distance_m, closing_speed_mps, tcpa_s, dcpa_m, reason)
    trigger = None

    for topic, message, timestamp_ns in _read_state_bag(bag_path):
        if first_ns is None:
            first_ns = timestamp_ns
        if not _valid(message):
            continue
        time_s = (timestamp_ns - first_ns) / 1.0e9
        latest[topic] = message
        if len(latest) < 2:
            continue

        a = latest[STATE_TOPICS[0]]
        b = latest[STATE_TOPICS[1]]
        a_vx, a_vy = velocity_vector(a.linear_velocity_mps, a.yaw_rad)
        b_vx, b_vy = velocity_vector(b.linear_velocity_mps, b.yaw_rad)
        cpa: CpaResult = closest_point_of_approach(
            a.x_m, a.y_m, a_vx, a_vy, b.x_m, b.y_m, b_vx, b_vy, horizon_s
        )

        predicted_conflict = (
            cpa.closing_speed_mps > minimum_closing_speed_mps
            and cpa.time_to_cpa_s <= horizon_s
            and cpa.distance_at_cpa_m < safety_radius_m
        )
        proximity_conflict = (
            not predicted_conflict and cpa.current_distance_m < trigger_distance_m
        )
        if predicted_conflict:
            reason = "PREDICTED_CPA"
        elif proximity_conflict:
            reason = "PROXIMITY_FALLBACK"
        else:
            reason = "NONE"

        rows.append(
            (time_s, cpa.current_distance_m, cpa.closing_speed_mps, cpa.time_to_cpa_s, cpa.distance_at_cpa_m, reason)
        )

        if trigger is None and reason != "NONE":
            trigger = {
                "trigger_time_s": time_s,
                "trigger_reason": reason,
                "trigger_distance_m": cpa.current_distance_m,
                "tcpa_at_trigger_s": cpa.time_to_cpa_s,
                "dcpa_at_trigger_m": cpa.distance_at_cpa_m,
                "closing_speed_at_trigger_mps": cpa.closing_speed_mps,
            }

    if not rows:
        raise RuntimeError("No valid paired /epuck1/state and /epuck2/state samples found.")

    minimum_distance_row = min(rows, key=lambda row: row[1])
    result = {
        "bag_path": str(bag_path),
        "horizon_s": horizon_s,
        "safety_radius_m": safety_radius_m,
        "trigger_distance_m_threshold": trigger_distance_m,
        "trigger_time_s": None,
        "trigger_reason": "NONE_OBSERVED",
        "trigger_distance_m": None,
        "tcpa_at_trigger_s": None,
        "dcpa_at_trigger_m": None,
        "closing_speed_at_trigger_mps": None,
        "minimum_center_separation_m": minimum_distance_row[1],
        "minimum_center_separation_time_s": minimum_distance_row[0],
    }
    if trigger is not None:
        result.update(trigger)

    return result, rows


def _write_outputs(output_dir: Path, result: dict, rows) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "trigger_reason_summary.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    with (output_dir / "trigger_classification.csv").open(
        "w", newline="", encoding="utf-8"
    ) as fh:
        writer = csv.writer(fh)
        writer.writerow(
            ("time_s", "current_distance_m", "closing_speed_mps", "tcpa_s", "dcpa_m", "trigger_reason")
        )
        writer.writerows(rows)

    lines = [
        "# Trigger reason classification",
        "",
        f"Bag: `{result['bag_path']}`",
        "",
        f"- trigger_reason: **{result['trigger_reason']}**",
        f"- trigger_time_s: {result['trigger_time_s']}",
        f"- trigger_distance_m: {result['trigger_distance_m']}",
        f"- tcpa_at_trigger_s: {result['tcpa_at_trigger_s']}",
        f"- dcpa_at_trigger_m: {result['dcpa_at_trigger_m']}",
        f"- closing_speed_at_trigger_mps: {result['closing_speed_at_trigger_mps']}",
        f"- minimum_center_separation_m: {result['minimum_center_separation_m']}",
        "",
        "Classification thresholds: PREDICTED_CPA requires "
        f"tcpa<={result['horizon_s']}s and dcpa<{result['safety_radius_m']}m; "
        f"PROXIMITY_FALLBACK requires current_distance<{result['trigger_distance_m_threshold']}m "
        "with the predicted condition false. Reconstructed offline from recorded "
        "bag state using the same collision_math formulas the live controller runs; "
        "the controller itself was not modified or rerun to produce this classification.",
        "",
    ]
    (output_dir / "trigger_reason_summary.md").write_text("\n".join(lines), encoding="utf-8")


def _arguments():
    parser = argparse.ArgumentParser(
        description="Classify a cooperative-avoidance bag's CPA trigger as PREDICTED_CPA or PROXIMITY_FALLBACK."
    )
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--horizon-s", type=float, default=4.0)
    parser.add_argument("--safety-radius-m", type=float, default=0.14)
    parser.add_argument("--trigger-distance-m", type=float, default=0.34)
    parser.add_argument("--minimum-closing-speed-mps", type=float, default=0.001)
    return parser.parse_args()


def main():
    args = _arguments()
    bag_path = args.bag_path.expanduser().resolve()
    output_dir = args.output_dir or bag_path / "analysis"
    result, rows = classify_trigger(
        bag_path,
        horizon_s=args.horizon_s,
        safety_radius_m=args.safety_radius_m,
        trigger_distance_m=args.trigger_distance_m,
        minimum_closing_speed_mps=args.minimum_closing_speed_mps,
    )
    _write_outputs(output_dir, result, rows)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"Trigger-reason analysis written to: {output_dir}")


if __name__ == "__main__":
    main()
