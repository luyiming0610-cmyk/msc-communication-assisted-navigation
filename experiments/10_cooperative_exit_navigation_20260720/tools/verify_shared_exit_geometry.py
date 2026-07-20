#!/usr/bin/env python3
"""Read-only geometry verification for the shared-exit scene, run BEFORE
any world file or pilot exists. Every number here is computed, not
eyeballed -- this script's own printed output is the evidence artifact
required before Phase 3 pilots run.

Checks:
  - goal region (with robot-radius margin) fits inside the arena, clear
    of walls
  - gate posts do not overlap the goal region or the arena walls
  - the obstacle does not overlap either robot's start pose or the goal
    region, and DOES intersect Robot B's planned search path (so it is
    a genuine, verified detour requirement, not just believed to be one)
  - neither robot's start pose is inside the goal region
  - exact straight-line and waypoint-path distances for max_runtime_s
    computation
"""
from __future__ import annotations

import math

ARENA_HALF_EXTENT_M = 0.75  # RectangleArena floorSize 1.5 x 1.5
ROBOT_RADIUS_M = 0.037  # e-puck2 body diameter 70mm (GCtronic spec) + small margin

GOAL_CENTER = (0.50, 0.50)
GOAL_RADIUS_M = 0.10

GATE_POSTS = [(0.244, 0.456), (0.456, 0.244)]

OBSTACLE_CENTER = (0.15, -0.15)
OBSTACLE_HALF_SIZE_M = 0.04  # 0.08m box -> 0.04m half-extent, used as a conservative circular proxy

ROBOT_A_START = (0.10, 0.55)
ROBOT_B_START = (-0.20, -0.20)
ROBOT_B_WAYPOINTS = [
    (-0.20, -0.20),
    (0.05, -0.35),
    (0.25, 0.05),
    (0.50, 0.50),
]

NOMINAL_SPEED_MPS = 0.04


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_to_segment_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0:
        return dist(p, a)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    proj = (ax + t * dx, ay + t * dy)
    return dist(p, proj)


def main():
    ok = True

    def check(label, condition, detail):
        nonlocal ok
        status = "OK" if condition else "FAIL"
        if not condition:
            ok = False
        print(f"[{status}] {label}: {detail}")

    print("=== Arena / goal-region wall clearance ===")
    for axis, coord in (("x", GOAL_CENTER[0]), ("y", GOAL_CENTER[1])):
        clearance = ARENA_HALF_EXTENT_M - coord - GOAL_RADIUS_M - ROBOT_RADIUS_M
        check(
            f"goal region + robot radius clears +{axis} wall",
            clearance > 0,
            f"clearance={clearance:.4f}m (arena_half={ARENA_HALF_EXTENT_M}, "
            f"goal_center_{axis}={coord}, goal_radius={GOAL_RADIUS_M}, robot_radius={ROBOT_RADIUS_M})",
        )
    corner = (ARENA_HALF_EXTENT_M, ARENA_HALF_EXTENT_M)
    corner_clearance = dist(GOAL_CENTER, corner) - GOAL_RADIUS_M - ROBOT_RADIUS_M
    check(
        "goal region + robot radius clears the corner point",
        corner_clearance > 0,
        f"clearance={corner_clearance:.4f}m",
    )

    print("\n=== Gate posts vs goal region / walls ===")
    for i, post in enumerate(GATE_POSTS, start=1):
        d = dist(post, GOAL_CENTER)
        check(
            f"gate post {i} does not overlap goal region",
            d > GOAL_RADIUS_M,
            f"post={post} distance_to_goal_center={d:.4f}m > goal_radius={GOAL_RADIUS_M}",
        )
        for axis, coord in (("x", post[0]), ("y", post[1])):
            clearance = ARENA_HALF_EXTENT_M - coord
            check(
                f"gate post {i} clears +{axis} wall",
                clearance > 0.05,
                f"clearance={clearance:.4f}m",
            )
    gate_width = dist(GATE_POSTS[0], GATE_POSTS[1])
    check(
        "gate width comfortably exceeds robot diameter",
        gate_width > 4 * (2 * ROBOT_RADIUS_M),
        f"gate_width={gate_width:.4f}m vs robot_diameter={2 * ROBOT_RADIUS_M:.4f}m "
        f"(ratio={gate_width / (2 * ROBOT_RADIUS_M):.2f}x)",
    )

    print("\n=== Start poses outside goal region ===")
    for name, pose in (("robot_a", ROBOT_A_START), ("robot_b", ROBOT_B_START)):
        d = dist(pose, GOAL_CENTER)
        check(
            f"{name} start pose outside goal region with margin",
            d > GOAL_RADIUS_M + 0.10,
            f"start={pose} distance_to_goal_center={d:.4f}m "
            f"(goal_radius={GOAL_RADIUS_M}, required margin 0.10m)",
        )

    print("\n=== Obstacle placement ===")
    d_a = dist(OBSTACLE_CENTER, ROBOT_A_START)
    check(
        "obstacle does not overlap robot A start pose",
        d_a > OBSTACLE_HALF_SIZE_M + ROBOT_RADIUS_M + 0.05,
        f"distance={d_a:.4f}m",
    )
    d_b = dist(OBSTACLE_CENTER, ROBOT_B_START)
    check(
        "obstacle does not overlap robot B start pose",
        d_b > OBSTACLE_HALF_SIZE_M + ROBOT_RADIUS_M + 0.05,
        f"distance={d_b:.4f}m",
    )
    d_goal = dist(OBSTACLE_CENTER, GOAL_CENTER)
    check(
        "obstacle does not overlap goal region",
        d_goal > OBSTACLE_HALF_SIZE_M + GOAL_RADIUS_M,
        f"distance={d_goal:.4f}m",
    )
    # Robot A path: straight line start -> goal (cooperative_avoider only
    # steers toward a single heading target; this is its actual planned path).
    a_path_dist = point_to_segment_dist(OBSTACLE_CENTER, ROBOT_A_START, GOAL_CENTER)
    check(
        "obstacle does NOT intersect robot A's direct path (asymmetric by design)",
        a_path_dist > OBSTACLE_HALF_SIZE_M + ROBOT_RADIUS_M,
        f"min_distance_to_path={a_path_dist:.4f}m",
    )
    # Robot B path: the waypoint polyline.
    min_seg_dist = min(
        point_to_segment_dist(OBSTACLE_CENTER, ROBOT_B_WAYPOINTS[i], ROBOT_B_WAYPOINTS[i + 1])
        for i in range(len(ROBOT_B_WAYPOINTS) - 1)
    )
    check(
        "obstacle DOES intersect robot B's waypoint path (genuine detour required)",
        min_seg_dist < OBSTACLE_HALF_SIZE_M + ROBOT_RADIUS_M,
        f"min_distance_to_path={min_seg_dist:.4f}m "
        f"(threshold {OBSTACLE_HALF_SIZE_M + ROBOT_RADIUS_M:.4f}m)",
    )

    print("\n=== Distances (for max_runtime_s computation) ===")
    a_straight = dist(ROBOT_A_START, GOAL_CENTER)
    print(f"robot_a straight_line_distance_to_exit_m = {a_straight:.4f}")
    b_straight = dist(ROBOT_B_START, GOAL_CENTER)
    print(f"robot_b straight_line_distance_to_exit_m (ON, immediate switch case) = {b_straight:.4f}")
    b_search_path = sum(
        dist(ROBOT_B_WAYPOINTS[i], ROBOT_B_WAYPOINTS[i + 1])
        for i in range(len(ROBOT_B_WAYPOINTS) - 1)
    )
    print(f"robot_b full waypoint search path length_m (OFF, worst case) = {b_search_path:.4f}")

    a_time_s = a_straight / NOMINAL_SPEED_MPS
    b_time_s = b_search_path / NOMINAL_SPEED_MPS
    print(f"\nrobot_a travel_time_s at {NOMINAL_SPEED_MPS} m/s = {a_time_s:.2f}")
    print(f"robot_b (OFF, full search) travel_time_s at {NOMINAL_SPEED_MPS} m/s = {b_time_s:.2f}")

    print(f"\noverall_check = {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
