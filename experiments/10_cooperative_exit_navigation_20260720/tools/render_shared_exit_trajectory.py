#!/usr/bin/env python3
"""Read-only top-down trajectory renderer for the shared edge-exit
study, using plain SVG (no matplotlib -- broken in this environment due
to a numpy 2.x ABI mismatch, and this avoids touching system packages).

Reads a trial's bag + frozen_params.json (as recorded for that trial,
never a second hardcoded copy) and produces a single .svg overlaying:
arena bounds, goal/exit region, gate posts, obstacle (if present),
Robot B's waypoints, both robots' full trajectories, start/end markers,
and each robot's goal-entry time (if any). For a trial whose controller
log shows a DURATION_CEILING failsafe, also marks the position at the
moment that failsafe latched.

Does not run any pilot. Read-only over already-recorded evidence.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from epuck2_comm_interfaces.msg import EpuckState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from task_completion_analyzer import GoalRegion, robot_goal_completion_time


ARENA_HALF_EXTENT_M = 0.75
# SVG canvas: arena maps to [MARGIN, MARGIN+SIZE], y flipped (SVG y grows down).
SIZE = 700
MARGIN = 60


def world_to_svg(x, y):
    sx = MARGIN + (x + ARENA_HALF_EXTENT_M) / (2 * ARENA_HALF_EXTENT_M) * SIZE
    sy = MARGIN + (ARENA_HALF_EXTENT_M - y) / (2 * ARENA_HALF_EXTENT_M) * SIZE
    return sx, sy


def read_state_samples(bag_dir, topic):
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
            samples.append((stamp_s, float(msg.x_m), float(msg.y_m)))
    samples.sort(key=lambda s: s[0])
    return samples


def find_failsafe_trigger(controller_log_path):
    """Returns (robot_num:str, ros_time_s:float) for the FIRST transition
    line whose failsafe_cause is not NONE, or None if none found."""
    if not os.path.exists(controller_log_path):
        return None
    pattern = re.compile(
        r"\[cooperative_avoider-(?P<robot>[12])\].*?TRANSITION .*?ros_time=(?P<t>[0-9.]+).*?failsafe_cause=(?P<cause>\S+)"
    )
    with open(controller_log_path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = pattern.search(line)
            if m and m.group("cause") != "NONE":
                return m.group("robot"), float(m.group("t"))
    return None


def polyline_points(samples):
    return " ".join(f"{world_to_svg(x, y)[0]:.1f},{world_to_svg(x, y)[1]:.1f}" for _t, x, y in samples)


def nearest_sample_at(samples, t_s):
    if not samples:
        return None
    return min(samples, key=lambda s: abs(s[0] - t_s))


def render(trial_id, bag_dir, diag_log_dir, frozen_params_path, out_path):
    with open(frozen_params_path, "r", encoding="utf-8") as f:
        params = json.load(f)

    goal = GoalRegion(
        center_x_m=params["goal_center_x_m"], center_y_m=params["goal_center_y_m"],
        radius_m=params["goal_radius_m"],
    )
    hold_time_s = params["goal_hold_time_s"]

    e1 = read_state_samples(bag_dir, "/epuck1/state")
    e2 = read_state_samples(bag_dir, "/epuck2/state")

    a_completion, _ = robot_goal_completion_time(
        [(t, x, y, 0.0, 0.0) for t, x, y in e1], goal, hold_time_s
    )
    b_completion, _ = robot_goal_completion_time(
        [(t, x, y, 0.0, 0.0) for t, x, y in e2], goal, hold_time_s
    )

    controller_log = os.path.join(diag_log_dir, "controller.log")
    failsafe = find_failsafe_trigger(controller_log)
    failsafe_marker_svg = ""
    if failsafe:
        robot_num, t_s = failsafe
        samples = e1 if robot_num == "1" else e2
        nearest = nearest_sample_at(samples, t_s)
        if nearest:
            sx, sy = world_to_svg(nearest[1], nearest[2])
            failsafe_marker_svg = (
                f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="10" fill="none" stroke="red" stroke-width="3"/>'
                f'<text x="{sx+12:.1f}" y="{sy:.1f}" font-size="12" fill="red">'
                f'FAILSAFE robot{robot_num} t={t_s:.1f}s</text>'
            )

    gx, gy = world_to_svg(goal.center_x_m, goal.center_y_m)
    goal_r_px = goal.radius_m / (2 * ARENA_HALF_EXTENT_M) * SIZE

    obstacle_svg = ""
    obstacle = params.get("obstacle")
    # obstacle isn't in the per-trial frozen_params.json (that file only has
    # goal/safety/timing scalars) -- read it from the canonical copy if present.
    canonical_path = os.path.join(diag_log_dir, "frozen_params_canonical_copy.json")
    if os.path.exists(canonical_path):
        with open(canonical_path, "r", encoding="utf-8") as f:
            canonical = json.load(f)
        obs = canonical.get("obstacle")
        if obs:
            ox, oy = world_to_svg(obs["center_x_m"], obs["center_y_m"])
            half_px = obs["size_m"][0] / (2 * ARENA_HALF_EXTENT_M) * SIZE / 2
            obstacle_svg = (
                f'<rect x="{ox-half_px:.1f}" y="{oy-half_px:.1f}" width="{2*half_px:.1f}" '
                f'height="{2*half_px:.1f}" fill="#555" opacity="0.8"/>'
                f'<text x="{ox+half_px+4:.1f}" y="{oy:.1f}" font-size="11" fill="#333">obstacle</text>'
            )
        waypoints = canonical.get("robots", {}).get("robot_b", {}).get("search_waypoints_m", [])
        gate_posts = canonical.get("exit", {}).get("gate_posts_m", [])
    else:
        waypoints = []
        gate_posts = []

    waypoints_svg = ""
    for i, (wx, wy) in enumerate(waypoints):
        sx, sy = world_to_svg(wx, wy)
        waypoints_svg += (
            f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="5" fill="none" stroke="orange" stroke-width="2"/>'
            f'<text x="{sx+6:.1f}" y="{sy-6:.1f}" font-size="10" fill="orange">wp{i}</text>'
        )

    gate_svg = ""
    for gx2, gy2 in gate_posts:
        sx, sy = world_to_svg(gx2, gy2)
        gate_svg += f'<rect x="{sx-4:.1f}" y="{sy-4:.1f}" width="8" height="8" fill="#8B0000"/>'

    a_start_svg = ""
    a_end_svg = ""
    if e1:
        sx, sy = world_to_svg(e1[0][1], e1[0][2])
        a_start_svg = f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="6" fill="blue"/>'
        ex, ey = world_to_svg(e1[-1][1], e1[-1][2])
        a_end_svg = f'<rect x="{ex-5:.1f}" y="{ey-5:.1f}" width="10" height="10" fill="blue"/>'

    b_start_svg = ""
    b_end_svg = ""
    if e2:
        sx, sy = world_to_svg(e2[0][1], e2[0][2])
        b_start_svg = f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="6" fill="green"/>'
        ex, ey = world_to_svg(e2[-1][1], e2[-1][2])
        b_end_svg = f'<rect x="{ex-5:.1f}" y="{ey-5:.1f}" width="10" height="10" fill="green"/>'

    a_pts = polyline_points(e1)
    b_pts = polyline_points(e2)

    svg = f'''<svg viewBox="0 0 {SIZE + 2*MARGIN} {SIZE + 2*MARGIN + 120}" xmlns="http://www.w3.org/2000/svg" font-family="sans-serif">
<rect x="0" y="0" width="{SIZE + 2*MARGIN}" height="{SIZE + 2*MARGIN + 120}" fill="white"/>
<rect x="{MARGIN}" y="{MARGIN}" width="{SIZE}" height="{SIZE}" fill="none" stroke="black" stroke-width="2"/>
<text x="{MARGIN}" y="{MARGIN-15}" font-size="16" font-weight="bold">{trial_id}</text>
<circle cx="{gx:.1f}" cy="{gy:.1f}" r="{goal_r_px:.1f}" fill="#90ee90" opacity="0.5" stroke="green" stroke-width="2"/>
<text x="{gx:.1f}" y="{gy - goal_r_px - 8:.1f}" font-size="11" fill="green" text-anchor="middle">exit/goal region</text>
{obstacle_svg}
{waypoints_svg}
{gate_svg}
<polyline points="{a_pts}" fill="none" stroke="blue" stroke-width="2"/>
<polyline points="{b_pts}" fill="none" stroke="green" stroke-width="2"/>
{a_start_svg}{a_end_svg}
{b_start_svg}{b_end_svg}
{failsafe_marker_svg}
<g transform="translate({MARGIN},{MARGIN+SIZE+30})" font-size="13">
  <circle cx="6" cy="0" r="6" fill="blue"/><text x="16" y="4">Robot A start</text>
  <rect x="200" y="-5" width="10" height="10" fill="blue"/><text x="216" y="4">Robot A end</text>
  <text x="400" y="4">A goal-entry: {"NEVER" if a_completion is None else f"{a_completion:.2f}s (abs sim time)"}</text>
</g>
<g transform="translate({MARGIN},{MARGIN+SIZE+55})" font-size="13">
  <circle cx="6" cy="0" r="6" fill="green"/><text x="16" y="4">Robot B start</text>
  <rect x="200" y="-5" width="10" height="10" fill="green"/><text x="216" y="4">Robot B end</text>
  <text x="400" y="4">B goal-entry: {"NEVER" if b_completion is None else f"{b_completion:.2f}s (abs sim time)"}</text>
</g>
<g transform="translate({MARGIN},{MARGIN+SIZE+80})" font-size="12" fill="#555">
  <text x="0" y="0">Orange circles = Robot B frozen waypoints. Dark-red squares = exit gate posts.</text>
  <text x="0" y="18">Red ring = local-avoidance FAILSAFE trigger location (if any occurred).</text>
</g>
</svg>'''

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(svg)
    print(f"wrote {out_path}")
    print(f"robot_a_completion_time_abs_s={a_completion}")
    print(f"robot_b_completion_time_abs_s={b_completion}")
    if failsafe:
        print(f"failsafe_robot={failsafe[0]} failsafe_ros_time_s={failsafe[1]}")
    else:
        print("failsafe=NONE")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trial_id")
    parser.add_argument("bag_dir")
    parser.add_argument("diag_log_dir")
    parser.add_argument("frozen_params_path")
    parser.add_argument("out_path")
    args = parser.parse_args()
    render(args.trial_id, args.bag_dir, args.diag_log_dir, args.frozen_params_path, args.out_path)


if __name__ == "__main__":
    main()
