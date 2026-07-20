#!/usr/bin/env python3
"""Read-only, dependency-free plain-SVG top-down GEOMETRY diagram for the
shared edge-exit study's revision-3 design -- static scene layout only
(arena, exit region + gate posts, both parking zones, both robots'
start poses, Robot B's frozen waypoints), NOT a trial trajectory (see
render_shared_exit_trajectory.py for that). Reads every coordinate
directly from shared_exit_frozen_params.json, never a second hardcoded
copy. Does not run any pilot or simulation.
"""
from __future__ import annotations

import json
import os
import sys

ARENA_HALF_EXTENT_M = 0.75
SIZE = 700
MARGIN = 60


def world_to_svg(x, y):
    sx = MARGIN + (x + ARENA_HALF_EXTENT_M) / (2 * ARENA_HALF_EXTENT_M) * SIZE
    sy = MARGIN + (ARENA_HALF_EXTENT_M - y) / (2 * ARENA_HALF_EXTENT_M) * SIZE
    return sx, sy


def circle(cx, cy, r_m, **attrs):
    sx, sy = world_to_svg(cx, cy)
    sr = r_m / (2 * ARENA_HALF_EXTENT_M) * SIZE
    attr_str = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in attrs.items())
    return f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{sr:.1f}" {attr_str} />'


def rect_marker(cx, cy, size_m, **attrs):
    sx, sy = world_to_svg(cx, cy)
    half = size_m / (2 * ARENA_HALF_EXTENT_M) * SIZE / 2
    attr_str = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in attrs.items())
    return f'<rect x="{sx-half:.1f}" y="{sy-half:.1f}" width="{half*2:.1f}" height="{half*2:.1f}" {attr_str} />'


def line(x0, y0, x1, y1, **attrs):
    sx0, sy0 = world_to_svg(x0, y0)
    sx1, sy1 = world_to_svg(x1, y1)
    attr_str = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in attrs.items())
    return f'<line x1="{sx0:.1f}" y1="{sy0:.1f}" x2="{sx1:.1f}" y2="{sy1:.1f}" {attr_str} />'


def text(x, y, s, **attrs):
    sx, sy = world_to_svg(x, y)
    attr_str = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in attrs.items())
    return f'<text x="{sx:.1f}" y="{sy:.1f}" {attr_str}>{s}</text>'


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    here = os.path.dirname(os.path.abspath(__file__))
    params_path = argv[0] if argv else os.path.join(here, "..", "shared_exit_frozen_params.json")
    out_path = argv[1] if len(argv) > 1 else os.path.join(
        here, "..", "exit_geometry_diagram_revision3.svg"
    )
    with open(params_path, "r", encoding="utf-8") as f:
        p = json.load(f)

    exit_ = p["exit"]
    robot_a = p["robots"]["robot_a"]
    robot_b = p["robots"]["robot_b"]
    parking = p["parking_zones"]

    elems = []
    ax0, ay0 = world_to_svg(-ARENA_HALF_EXTENT_M, -ARENA_HALF_EXTENT_M)
    ax1, ay1 = world_to_svg(ARENA_HALF_EXTENT_M, ARENA_HALF_EXTENT_M)
    elems.append(
        f'<rect x="{ax0:.1f}" y="{ay1:.1f}" width="{ax1-ax0:.1f}" height="{ay0-ay1:.1f}" '
        f'fill="none" stroke="black" stroke-width="2" />'
    )

    # Exit / goal completion region
    elems.append(circle(exit_["center_x_m"], exit_["center_y_m"], exit_["goal_hold_radius_m"],
                         fill="rgba(0,200,0,0.15)", stroke="green", stroke_width="2"))
    elems.append(text(exit_["center_x_m"], exit_["center_y_m"] - 0.13, "EXIT",
                       font_size="14", text_anchor="middle", fill="green"))

    # Gate posts (elevated, z=0.20m, shown here for x,y reference only)
    for i, (gx, gy) in enumerate(exit_["gate_posts_m"], start=1):
        elems.append(rect_marker(gx, gy, 0.02, fill="red"))
        elems.append(text(gx, gy - 0.03, f"post{i}(z=0.20)", font_size="9",
                           text_anchor="middle", fill="red"))

    # Parking zones
    for name, color in (("robot_a", "blue"), ("robot_b", "orange")):
        zone = parking[name]
        elems.append(circle(zone["center_x_m"], zone["center_y_m"], zone["radius_m"],
                             fill=f"{color}", fill_opacity="0.4", stroke=color, stroke_width="2"))
        elems.append(text(zone["center_x_m"], zone["center_y_m"] + 0.06,
                           f"{name}_park", font_size="11", text_anchor="middle", fill=color))

    # Robot A start + direct path
    elems.append(line(robot_a["start_x_m"], robot_a["start_y_m"],
                       exit_["center_x_m"], exit_["center_y_m"],
                       stroke="blue", stroke_width="1.5", stroke_dasharray="4,3"))
    elems.append(circle(robot_a["start_x_m"], robot_a["start_y_m"], 0.037,
                         fill="blue", stroke="black", stroke_width="1"))
    elems.append(text(robot_a["start_x_m"], robot_a["start_y_m"] - 0.06, "A start",
                       font_size="11", text_anchor="middle", fill="blue"))

    # Robot B start + frozen waypoints
    wps = robot_b["search_waypoints_m"]
    for (x0, y0), (x1, y1) in zip(wps, wps[1:]):
        elems.append(line(x0, y0, x1, y1, stroke="orange", stroke_width="1.5", stroke_dasharray="4,3"))
    elems.append(circle(robot_b["start_x_m"], robot_b["start_y_m"], 0.037,
                         fill="orange", stroke="black", stroke_width="1"))
    elems.append(text(robot_b["start_x_m"], robot_b["start_y_m"] - 0.06, "B start",
                       font_size="11", text_anchor="middle", fill="orange"))
    for i, (wx, wy) in enumerate(wps):
        elems.append(circle(wx, wy, 0.012, fill="orange"))
        elems.append(text(wx + 0.03, wy, f"wp{i}", font_size="9", fill="orange"))

    # Line from exit center to each parking zone (post-exit transit legs)
    elems.append(line(exit_["center_x_m"], exit_["center_y_m"],
                       parking["robot_a"]["center_x_m"], parking["robot_a"]["center_y_m"],
                       stroke="blue", stroke_width="1", stroke_dasharray="2,2"))
    elems.append(line(exit_["center_x_m"], exit_["center_y_m"],
                       parking["robot_b"]["center_x_m"], parking["robot_b"]["center_y_m"],
                       stroke="orange", stroke_width="1", stroke_dasharray="2,2"))

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {SIZE+2*MARGIN} {SIZE+2*MARGIN}">
<rect width="100%" height="100%" fill="white" />
<text x="{MARGIN}" y="30" font-size="16" font-weight="bold">Shared edge-exit N2 geometry -- revision 3</text>
{''.join(elems)}
</svg>"""
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(svg)
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
