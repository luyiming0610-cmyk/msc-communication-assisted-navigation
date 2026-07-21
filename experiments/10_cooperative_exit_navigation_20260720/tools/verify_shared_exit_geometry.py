#!/usr/bin/env python3
"""Deterministic geometry checks for the hammer-shaped shared-exit world."""
from __future__ import annotations

import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, "..", "shared_exit_frozen_params.json")


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_to_segment_distance(point, start, end):
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return distance(point, start)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def main():
    with open(PARAMS, "r", encoding="utf-8") as stream:
        p = json.load(stream)

    ok = True

    def check(label, condition, detail):
        nonlocal ok
        print(f"[{'OK' if condition else 'FAIL'}] {label}: {detail}")
        ok = ok and condition

    robot_radius = p["robot_radius_m"]
    front_release = 0.220
    margin = 0.030
    sensor_wall_requirement = front_release + robot_radius + margin
    peer_requirement = front_release + 2 * robot_radius + margin
    hammer = p["hammer_exit"]
    exit_center = (p["exit"]["center_x_m"], p["exit"]["center_y_m"])
    park_a = (p["parking_zones"]["robot_a"]["center_x_m"], p["parking_zones"]["robot_a"]["center_y_m"])
    park_b = (p["parking_zones"]["robot_b"]["center_x_m"], p["parking_zones"]["robot_b"]["center_y_m"])
    a_start = (p["robots"]["robot_a"]["start_x_m"], p["robots"]["robot_a"]["start_y_m"])
    b_start = (p["robots"]["robot_b"]["start_x_m"], p["robots"]["robot_b"]["start_y_m"])
    b_waypoints = [tuple(value) for value in p["robots"]["robot_b"]["search_waypoints_m"]]

    print("=== Real opening and green neck ===")
    opening_width = hammer["opening_y_max_m"] - hammer["opening_y_min_m"]
    check("opening width matches frozen value", abs(opening_width - hammer["opening_width_m"]) < 1e-9, f"{opening_width:.3f}m")
    check("opening comfortably exceeds robot diameter", opening_width > 4 * (2 * robot_radius), f"opening={opening_width:.3f}m robot_diameter={2*robot_radius:.3f}m")
    check("exit target lies inside neck x", hammer["neck_x_min_m"] < exit_center[0] < hammer["neck_x_max_m"], f"exit_x={exit_center[0]:.3f}")
    check("exit target lies inside opening y", hammer["opening_y_min_m"] < exit_center[1] < hammer["opening_y_max_m"], f"exit_y={exit_center[1]:.3f}")

    print("\n=== Reception and parking regions ===")
    for name, point in (("robot_a", park_a), ("robot_b", park_b)):
        west = point[0] - hammer["reception_x_min_m"]
        east = hammer["reception_x_max_m"] - point[0]
        south = point[1] - hammer["reception_y_min_m"]
        north = hammer["reception_y_max_m"] - point[1]
        minimum = min(west, east, south, north)
        check(f"{name} hold clears every reception wall by sensor-aware distance", minimum > sensor_wall_requirement, f"minimum={minimum:.3f}m required>{sensor_wall_requirement:.3f}m")
    separation = distance(park_a, park_b)
    check("parked robots clear peer sensor-aware distance", separation > peer_requirement, f"separation={separation:.3f}m required>{peer_requirement:.3f}m")

    print("\n=== Routes versus a parked peer ===")
    cases = (
        ("A ingress versus parked B", [a_start, exit_center], park_b),
        ("B OFF search versus parked A", b_waypoints, park_a),
        ("B ON direct route versus parked A", [b_start, exit_center], park_a),
        ("A exit-to-hold versus parked B", [exit_center, park_a], park_b),
        ("B exit-to-hold versus parked A", [exit_center, park_b], park_a),
    )
    for label, route, peer in cases:
        minimum = min(point_to_segment_distance(peer, route[i], route[i + 1]) for i in range(len(route) - 1))
        check(label, minimum > peer_requirement, f"minimum={minimum:.3f}m required>{peer_requirement:.3f}m")

    print("\n=== Travel distances ===")
    a_distance = distance(a_start, exit_center)
    b_direct = distance(b_start, exit_center)
    b_search = sum(distance(b_waypoints[i], b_waypoints[i + 1]) for i in range(len(b_waypoints) - 1))
    longest_hold_leg = max(distance(exit_center, park_a), distance(exit_center, park_b))
    print(f"robot_a_direct_to_opening_m={a_distance:.4f}")
    print(f"robot_b_direct_to_opening_m={b_direct:.4f}")
    print(f"robot_b_off_search_m={b_search:.4f}")
    print(f"longest_opening_to_hold_m={longest_hold_leg:.4f}")
    check("runtime ceiling exceeds nominal route budget", p["max_runtime_s"] > 5 + b_search / p["nominal_speed_mps"] + longest_hold_leg / p["nominal_speed_mps"] + p["goal_hold_time_s"] + 35, f"max_runtime={p['max_runtime_s']:.1f}s")

    print(f"\noverall_check = {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
