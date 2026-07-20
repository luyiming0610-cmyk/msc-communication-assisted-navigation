#!/usr/bin/env python3
"""Post-hoc analyzer for the shared edge-exit N2 study. Reads all scene
geometry and thresholds from shared_exit_frozen_params.json -- never
hardcodes a goal region, safety radius, or hold time. All timestamps
are normalized to trial-relative seconds (time since the trial's own
first sample), never reported as raw absolute sim-clock values.

Usage: analyze_shared_exit_trial.py TRIAL_ID COMM_MODE BAG_DIR DIAG_LOG_DIR [--out OUT_PATH]
  COMM_MODE: N2_EXIT_COMM_OFF | N2_EXIT_COMM_ON
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from epuck2_comm_interfaces.msg import EpuckState, GoalAnnouncement

from task_completion_analyzer import GoalRegion, build_task_verdict, path_length_m, \
    cumulative_absolute_heading_change_rad, stop_duration_s, robot_goal_completion_time
from announcement_metrics import (
    AnnouncementRecord, analyze_announcement_sequence, normalize_trial_relative,
    build_off_communication_summary, build_on_communication_summary,
)

EVENT_LINE_RE = re.compile(
    r"(?P<event>EXIT_KNOWN_AT_START|ANNOUNCEMENT_TX_FIRST|SEARCH_TO_GOAL_SWITCH|WAYPOINT_REACHED) "
    r"robot_id=(?P<robot_id>\d+) t=(?P<t>[0-9.]+)"
)


def _read_state_samples(bag_dir, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    samples = []
    while reader.has_next():
        t, data, _ts = reader.read_next()
        if t == topic:
            msg = deserialize_message(data, EpuckState)
            stamp_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) / 1e9
            samples.append((stamp_s, float(msg.x_m), float(msg.y_m), float(msg.yaw_rad), float(msg.linear_velocity_mps)))
    samples.sort(key=lambda s: s[0])
    return samples


def _read_announcement_records(bag_dir, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    records = []
    while reader.has_next():
        t, data, ts_ns = reader.read_next()
        if t == topic:
            msg = deserialize_message(data, GoalAnnouncement)
            prod_s = float(msg.production_stamp.sec) + float(msg.production_stamp.nanosec) / 1e9
            recv_s = ts_ns / 1e9
            records.append(AnnouncementRecord(
                sequence=int(msg.sequence), production_stamp_s=prod_s, recv_stamp_s=recv_s, valid=bool(msg.valid)
            ))
    records.sort(key=lambda r: r.recv_stamp_s)
    return records


def _parse_navigator_events(log_path):
    events = []
    if not os.path.exists(log_path):
        return events
    with open(log_path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = EVENT_LINE_RE.search(line)
            if m:
                events.append((m.group("event"), int(m.group("robot_id")), float(m.group("t"))))
    return events


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("trial_id")
    parser.add_argument("comm_mode", choices=["N2_EXIT_COMM_OFF", "N2_EXIT_COMM_ON"])
    parser.add_argument("bag_dir")
    parser.add_argument("diag_log_dir")
    parser.add_argument("--frozen-params", default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args(argv)

    here = os.path.dirname(os.path.abspath(__file__))
    frozen_params_path = args.frozen_params or os.path.join(here, "..", "shared_exit_frozen_params.json")
    with open(frozen_params_path, "r", encoding="utf-8") as f:
        params = json.load(f)

    goal = GoalRegion(
        center_x_m=params["exit"]["center_x_m"],
        center_y_m=params["exit"]["center_y_m"],
        radius_m=params["exit"]["goal_hold_radius_m"],
    )
    # Part V (revision 2+): arrival/hold is judged at each robot's OWN
    # post-exit parking zone, not the single shared exit point -- must
    # match exactly what goal_navigator.py's ARRIVED_HOLD latch and
    # task_completion_monitor.py's live verdict use, or this post-hoc
    # analysis could disagree with the actual trial outcome. Falls back
    # to the single shared `goal` above (unchanged) for any frozen_params
    # file that predates parking_zones (Stage 0 / revision 1 pilots).
    per_robot_goals = None
    if "parking_zones" in params:
        per_robot_goals = {
            "robot_a_epuck1": GoalRegion(
                center_x_m=params["parking_zones"]["robot_a"]["center_x_m"],
                center_y_m=params["parking_zones"]["robot_a"]["center_y_m"],
                radius_m=params["parking_zones"]["robot_a"]["radius_m"],
            ),
            "robot_b_epuck2": GoalRegion(
                center_x_m=params["parking_zones"]["robot_b"]["center_x_m"],
                center_y_m=params["parking_zones"]["robot_b"]["center_y_m"],
                radius_m=params["parking_zones"]["robot_b"]["radius_m"],
            ),
        }
    hold_time_s = params["goal_hold_time_s"]
    safety_radius_m = params["safety_radius_m"]
    collision_contact_distance_m = params["collision_contact_distance_m"]

    e1 = _read_state_samples(args.bag_dir, "/epuck1/state")
    e2 = _read_state_samples(args.bag_dir, "/epuck2/state")
    per_robot = {"robot_a_epuck1": e1, "robot_b_epuck2": e2}

    trial_epoch_s = min(s[0] for s in (e1[:1] + e2[:1])) if (e1 or e2) else 0.0

    controller_log = os.path.join(args.diag_log_dir, "controller.log")
    max_runtime_hit = False
    latched_failsafe = False
    if os.path.exists(controller_log):
        with open(controller_log, encoding="utf-8", errors="replace") as f:
            log_text = f.read()
        max_runtime_hit = "maximum runtime reached" in log_text
        failsafe_causes = set(re.findall(r"failsafe_cause=(\S+)", log_text))
        latched_failsafe = any(c != "NONE" for c in failsafe_causes)

    verdict = build_task_verdict(
        per_robot_samples=per_robot, goal=goal, hold_time_s=hold_time_s,
        safety_radius_m=safety_radius_m, collision_contact_distance_m=collision_contact_distance_m,
        data_validity_reasons=[],
        latched_failsafe=latched_failsafe, ended_by_max_runtime=max_runtime_hit,
        per_robot_goals=per_robot_goals,
    )

    individual_completion_time_s = {}
    for name, samples in per_robot.items():
        robot_goal = (per_robot_goals or {}).get(name, goal)
        t_abs, _reason = robot_goal_completion_time(samples, robot_goal, hold_time_s)
        individual_completion_time_s[name] = (
            normalize_trial_relative(t_abs, trial_epoch_s) if t_abs is not None else None
        )
    makespan_trial_relative_s = (
        max(v for v in individual_completion_time_s.values() if v is not None)
        if verdict.all_robots_reached_goal else None
    )

    nav_log_a = os.path.join(args.diag_log_dir, "goal_navigator_epuck1.log")
    nav_log_b = os.path.join(args.diag_log_dir, "goal_navigator_epuck2.log")
    events_a = _parse_navigator_events(nav_log_a)
    events_b = _parse_navigator_events(nav_log_b)

    def _first(events, event_name):
        for e, _rid, t in events:
            if e == event_name:
                return normalize_trial_relative(t, trial_epoch_s)
        return None

    exit_discovery_time_s = _first(events_a, "EXIT_KNOWN_AT_START")

    comm_off = args.comm_mode == "N2_EXIT_COMM_OFF"
    if comm_off:
        # OFF must show ZERO GoalAnnouncement traffic -- check, don't assume.
        ann_records_off_check = _read_announcement_records(args.bag_dir, "/epuck1/goal_announcement")
        communication = build_off_communication_summary(len(ann_records_off_check))
    else:
        tx_time_s = _first(events_a, "ANNOUNCEMENT_TX_FIRST")
        switch_time_s = _first(events_b, "SEARCH_TO_GOAL_SWITCH")
        ann_records = _read_announcement_records(args.bag_dir, "/epuck1/goal_announcement")
        seq_stats = analyze_announcement_sequence(ann_records)
        rx_time_s = (
            normalize_trial_relative(ann_records[0].recv_stamp_s, trial_epoch_s)
            if ann_records else None
        )
        communication = build_on_communication_summary(tx_time_s, rx_time_s, switch_time_s, seq_stats)

    report = {
        "trial_id": args.trial_id,
        "comm_mode": args.comm_mode,
        "trial_epoch_s_absolute": trial_epoch_s,
        "sample_counts": {"robot_a_epuck1": len(e1), "robot_b_epuck2": len(e2)},
        "path_length_m": {name: path_length_m(s) for name, s in per_robot.items()},
        "cumulative_heading_change_rad": {name: cumulative_absolute_heading_change_rad(s) for name, s in per_robot.items()},
        "stop_duration_s": {name: stop_duration_s(s) for name, s in per_robot.items()},
        "individual_completion_time_s": individual_completion_time_s,
        "completion_region_source": "per_robot_parking_zone" if per_robot_goals else "shared_exit_region",
        "makespan_s": makespan_trial_relative_s,
        "exit_discovery_time_s": exit_discovery_time_s,
        "communication": communication,
        "verdict": {
            "data_validity": verdict.data_validity,
            "task_outcome": verdict.task_outcome,
            "task_outcome_reason": verdict.task_outcome_reason,
            "all_robots_reached_goal": verdict.all_robots_reached_goal,
            "completed_robot_count": verdict.completed_robot_count,
            "total_robot_count": verdict.total_robot_count,
            "minimum_pairwise_distance_m": verdict.minimum_pairwise_distance_m,
            "safety_margin_m": verdict.safety_margin_m,
            "collision_count": verdict.collision_count,
        },
        "ended_by_max_runtime": max_runtime_hit,
        "latched_failsafe": latched_failsafe,
    }

    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
