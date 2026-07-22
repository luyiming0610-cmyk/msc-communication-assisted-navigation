#!/usr/bin/env python3
"""Deterministic preflight checks for the three-robot hammer arena."""
import json
import math
import os


HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "..", "shared_exit_n3_params.json"), encoding="utf-8") as handle:
    p = json.load(handle)

errors = []


def check(condition, text):
    print(("[OK] " if condition else "[FAIL] ") + text)
    if not condition:
        errors.append(text)


zones = p["parking_zones"]
reception = p["reception_area"]
centres = [(zones[name]["center_x_m"], zones[name]["center_y_m"]) for name in ("robot_a", "robot_b", "robot_c")]
distances = [math.dist(centres[i], centres[j]) for i in range(3) for j in range(i + 1, 3)]
check(min(distances) >= 0.324, f"parking centres maintain sensor-aware separation: min={min(distances):.3f}m")
check(
    min(distances) >= p["peer_trigger_distance_m"] + 0.05,
    f"parking centres clear peer proximity trigger with margin: min={min(distances):.3f}m trigger={p['peer_trigger_distance_m']:.3f}m",
)
for name in ("robot_a", "robot_b", "robot_c"):
    zone = zones[name]
    check(
        reception["x_min_m"] + 0.287 < zone["center_x_m"] < reception["x_max_m"] - 0.287,
        f"{name} parking clears east/west walls",
    )
    check(
        reception["y_min_m"] + 0.287 < zone["center_y_m"] < reception["y_max_m"] - 0.287,
        f"{name} parking clears north/south walls",
    )

starts = []
for name in ("robot_a", "robot_b", "robot_c"):
    r = p["robots"][name]
    starts.append((r["start_x_m"], r["start_y_m"]))
for i in range(3):
    for j in range(i + 1, 3):
        check(math.dist(starts[i], starts[j]) > 0.324, f"start separation robot {i + 1}/{j + 1}")

exit_region = p["exit"]
check(0.75 < exit_region["center_x_m"] < 0.95, "exit centre lies in physical neck")
check(-0.05 < exit_region["center_y_m"] < 0.55, "exit centre lies within wall opening")
check(p["max_runtime_s"] >= 170.0, "pilot runtime accommodates longest frozen search")

for robot_name in ("robot_b", "robot_c"):
    for index, (x_m, y_m) in enumerate(p["robots"][robot_name]["search_waypoints_m"]):
        if index == len(p["robots"][robot_name]["search_waypoints_m"]) - 1:
            continue
        check(
            -0.75 + 0.287 < x_m < 0.75 - 0.287 and -0.75 + 0.287 < y_m < 0.75 - 0.287,
            f"{robot_name} waypoint {index + 1} clears main-arena walls by sensor-aware distance",
        )

print(f"overall_check={'PASS' if not errors else 'FAIL'}")
raise SystemExit(1 if errors else 0)
