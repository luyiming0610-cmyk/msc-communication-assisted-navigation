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

GOAL_CENTER = (0.25, 0.25)
GOAL_RADIUS_M = 0.10

GATE_POSTS = [(0.3561, 0.1439), (0.1439, 0.3561)]

PARKING_A = {"center": (0.3914, 0.1510), "radius": 0.04}
PARKING_B = {"center": (0.1510, 0.3914), "radius": 0.04}
REQUIRED_PARKING_SEPARATION_M = 0.14  # safety_radius_m -- pre-registered minimum
# PILOT04 (EXCLUDED diagnostic) proved 0.14m alone is not sufficient in
# practice -- the local IR/ToF sensor's own detection range must also be
# cleared, or a robot transiting to its own zone repeatedly detects the
# other robot parked nearby and cannot resolve the encounter within the
# frozen turn-ledger budget. Checked separately below.
LOCAL_FRONT_RELEASE_M = 0.220
ROBOT_DIAMETER_M = 2 * ROBOT_RADIUS_M
GEOMETRY_MARGIN_M = 0.03
PARKED_VS_PARKED_REQUIRED_M = LOCAL_FRONT_RELEASE_M + ROBOT_DIAMETER_M + GEOMETRY_MARGIN_M
# PILOT08 (EXCLUDED diagnostic) proved a SEPARATE wall-clearance requirement
# is also needed: physical/CPA-style clearance (parking_radius+robot_radius)
# is not enough to stop the local sensor detecting the WALL itself -- a
# wall can never be "passed" the way a discrete object can, so this
# produces an unresolvable, not merely slow, repeating encounter.
PARKED_VS_WALL_REQUIRED_M = LOCAL_FRONT_RELEASE_M + ROBOT_RADIUS_M + GEOMETRY_MARGIN_M
ROBOT_B_LAST_WAYPOINT_SEGMENT = ((0.25, 0.05), (0.25, 0.25))

OBSTACLE_CENTER = (0.15, -0.15)
OBSTACLE_HALF_SIZE_M = 0.04  # 0.08m box -> 0.04m half-extent, used as a conservative circular proxy

ROBOT_A_START = (0.10, 0.55)
ROBOT_B_START = (-0.20, -0.20)
ROBOT_B_WAYPOINTS = [
    (-0.20, -0.20),
    (0.05, -0.35),
    (0.25, 0.05),
    (0.25, 0.25),
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

    print("\n=== Post-exit parking zones (Part V) ===")
    for name, zone in (("robot_a", PARKING_A), ("robot_b", PARKING_B)):
        cx, cy = zone["center"]
        for axis, coord in (("x", cx), ("y", cy)):
            clearance = ARENA_HALF_EXTENT_M - coord - zone["radius"] - ROBOT_RADIUS_M
            check(
                f"{name} parking zone + robot radius clears +{axis} wall (physical/CPA-style)",
                clearance > 0,
                f"clearance={clearance:.4f}m (center_{axis}={coord}, "
                f"radius={zone['radius']}, robot_radius={ROBOT_RADIUS_M})",
            )
            wall_release_clearance = ARENA_HALF_EXTENT_M - coord - PARKED_VS_WALL_REQUIRED_M
            check(
                f"{name} parking zone clears +{axis} wall by local_front_release_m + robot_radius_m + margin "
                "(PILOT08 finding: physical/CPA-style clearance alone is not sufficient)",
                wall_release_clearance > 0,
                f"clearance={wall_release_clearance:.4f}m (center_{axis}={coord}, "
                f"required={PARKED_VS_WALL_REQUIRED_M:.4f}m = local_front_release_m {LOCAL_FRONT_RELEASE_M} "
                f"+ robot_radius_m {ROBOT_RADIUS_M} + geometry_margin_m {GEOMETRY_MARGIN_M})",
            )
        d_to_exit_center = dist(zone["center"], GOAL_CENTER)
        check(
            f"{name} parking zone center is outside the exit goal region",
            d_to_exit_center > GOAL_RADIUS_M,
            f"distance_to_exit_center={d_to_exit_center:.4f}m > goal_radius={GOAL_RADIUS_M}",
        )
    parking_separation = dist(PARKING_A["center"], PARKING_B["center"])
    boundary_clearance = parking_separation - PARKING_A["radius"] - PARKING_B["radius"]
    check(
        "parking zones are non-colliding (center spacing exceeds safety_radius_m)",
        parking_separation > REQUIRED_PARKING_SEPARATION_M,
        f"center_to_center={parking_separation:.5f}m > required={REQUIRED_PARKING_SEPARATION_M}m "
        f"(boundary-to-boundary clearance={boundary_clearance:.5f}m)",
    )
    check(
        "parking zone boundaries do not overlap each other",
        boundary_clearance > 0,
        f"boundary_clearance={boundary_clearance:.5f}m",
    )
    check(
        "parked-vs-parked spacing clears local_front_release_m + robot_diameter + margin "
        "(PILOT04 finding: >0.14m safety_radius_m alone is not sufficient)",
        parking_separation > PARKED_VS_PARKED_REQUIRED_M,
        f"center_to_center={parking_separation:.5f}m > required={PARKED_VS_PARKED_REQUIRED_M:.5f}m "
        f"(= local_front_release_m {LOCAL_FRONT_RELEASE_M} + robot_diameter {ROBOT_DIAMETER_M:.3f} "
        f"+ geometry_margin_m {GEOMETRY_MARGIN_M})",
    )

    print(
        "\n[INFO] Robot B's real, deterministic transit-path closest approach to "
        "Robot A's parked position (NOT a hard PASS/FAIL check -- an any-angle "
        "worst-case bound of the same magnitude as PARKED_VS_PARKED_REQUIRED_M is "
        "geometrically infeasible in this 1.5x1.5m arena at this exit location; "
        "see shared_exit_frozen_params.json parking_zones.clearance_analysis for "
        "the full reasoning and the max_runtime_s one-encounter allowance that "
        "budgets for this):"
    )
    (wx0, wy0), (wx1, wy1) = ROBOT_B_LAST_WAYPOINT_SEGMENT
    ux, uy = wx1 - wx0, wy1 - wy0
    un = math.hypot(ux, uy)
    ux, uy = ux / un, uy / un
    b_entry = (GOAL_CENTER[0] - GOAL_RADIUS_M * ux, GOAL_CENTER[1] - GOAL_RADIUS_M * uy)
    b_entry_to_a = dist(b_entry, PARKING_A["center"])
    print(
        f"      robot_b_entry_point≈{tuple(round(v, 4) for v in b_entry)} "
        f"distance_to_parked_robot_a={b_entry_to_a:.5f}m "
        f"(vs. required {PARKED_VS_PARKED_REQUIRED_M:.5f}m -- "
        f"{'MEETS' if b_entry_to_a > PARKED_VS_PARKED_REQUIRED_M else 'below'} the ideal target, "
        f"mitigated by max_runtime_s's one_local_encounter_allowance_s)"
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
