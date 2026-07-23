#!/usr/bin/env python3
"""HIL entry point that reuses goal_navigator.py's tested GoalNavigator
class on real hardware, WITHOUT modifying goal_navigator.py.

Why this file exists: goal_navigator.py's own main() unconditionally
calls `rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])`
(confirmed by full source read, 2026-07-23). That is correct and load
-bearing for the sim-only N2/N3 formal studies -- every process in that
study (state_publisher, cooperative_avoider, task_completion_monitor)
is sim-time-based, and forcing it was itself a real, documented bug fix
(see goal_navigator.py's own main() comment). On real hardware there is
no /clock publisher, so a sim-time ROS clock never advances past 0.0 --
any state_publisher.py-style `WAITING_FOR_CLOCK` gate, and any
timestamp-based logic inside GoalNavigator, would hang forever.

Rather than editing the shared file (which the N2/N3 formal results
also depend on, and which the HIL safety rules forbid touching for
anything not on the explicit "new adapter" list), this module imports
the GoalNavigator class directly and constructs it under a real-time
(use_sim_time:=false, i.e. default) rclpy context instead. All of
GoalNavigator's tested navigation-state logic (waypoint advancement,
ARRIVED_HOLD, EXIT_TO_PARKING_SWITCH, GoalAnnouncement TX/RX) is reused
unchanged; only the clock source differs.
"""
from __future__ import annotations

import os
import sys

_TOOLS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "10_cooperative_exit_navigation_20260720",
    "tools",
)


def _import_goal_navigator():
    if _TOOLS_DIR not in sys.path:
        sys.path.insert(0, _TOOLS_DIR)
    import goal_navigator  # noqa: E402  (path adjusted above by design)

    return goal_navigator


def main(argv=None):
    import rclpy

    goal_navigator = _import_goal_navigator()
    args = goal_navigator.parse_args(argv if argv is not None else sys.argv[1:])

    # Deliberately NOT use_sim_time -- see module docstring. This is the
    # one line this adapter exists to change relative to
    # goal_navigator.main().
    rclpy.init(args=[])
    node = goal_navigator.GoalNavigator(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # SIGINT surfaces here as ExternalShutdownException in this
        # rclpy version, not KeyboardInterrupt -- catching only the
        # latter left a clean SIGINT shutdown noisy (uncaught traceback,
        # nonzero exit) even though the finally block below always ran.
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
