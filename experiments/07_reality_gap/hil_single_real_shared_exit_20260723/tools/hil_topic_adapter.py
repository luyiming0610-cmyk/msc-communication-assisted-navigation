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

The node actually constructed is HilGoalAnnouncementEvidenceNavigator
(hil_goal_announcement_evidence.py), a thin subclass of GoalNavigator
adding one bounded, structured evidence log line per received
GoalAnnouncement -- itself still just GoalNavigator's own unmodified
navigation logic, wrapped, not replaced.
"""
from __future__ import annotations

import argparse
import importlib
import math
import sys
from pathlib import Path

# Resolved once, at import time, from this file's own on-disk location
# via pathlib -- never from the current working directory, and never by
# counting `os.path.dirname()` calls (a real hardware-free execution
# attempt exposed exactly that defect: the prior three-call dirname
# chain landed one directory level short of `experiments/`, silently
# computing a nonexistent directory and crashing with
# `ModuleNotFoundError: No module named 'goal_navigator'` before rclpy
# was even touched).
_THIS_FILE = Path(__file__).resolve()
_REQUIRED_RELATIVE_TOOLS_PATH = ("experiments", "10_cooperative_exit_navigation_20260720", "tools")


class GoalNavigatorImportError(RuntimeError):
    """Raised before any ROS/rclpy usage or adapter/node construction if
    goal_navigator.py cannot be located and identity-verified at its one
    intended committed path."""


def _resolve_repo_root() -> Path:
    try:
        return _THIS_FILE.parents[4]
    except IndexError as e:
        raise GoalNavigatorImportError(
            f"this file's location ({_THIS_FILE}) is shallower than expected relative to "
            "the repository root -- repository layout may have changed"
        ) from e


def _import_goal_navigator():
    repo_root = _resolve_repo_root()
    tools_dir = repo_root.joinpath(*_REQUIRED_RELATIVE_TOOLS_PATH)
    goal_navigator_file = tools_dir / "goal_navigator.py"

    try:
        tools_dir = tools_dir.resolve(strict=True)
    except FileNotFoundError as e:
        raise GoalNavigatorImportError(
            f"expected goal_navigator tools directory does not exist: {tools_dir}"
        ) from e
    try:
        goal_navigator_file = goal_navigator_file.resolve(strict=True)
    except FileNotFoundError as e:
        raise GoalNavigatorImportError(
            f"expected goal_navigator.py does not exist: {goal_navigator_file}"
        ) from e

    cached = sys.modules.get("goal_navigator")
    if cached is not None:
        cached_file = getattr(cached, "__file__", None)
        if cached_file is None:
            raise GoalNavigatorImportError(
                "a 'goal_navigator' module is already cached in sys.modules but exposes no "
                "__file__ -- refusing to trust an unidentifiable cached module"
            )
        if Path(cached_file).resolve() != goal_navigator_file:
            raise GoalNavigatorImportError(
                "a 'goal_navigator' module is already cached in sys.modules but resolves to "
                f"{cached_file!r}, not the intended committed file {goal_navigator_file}"
            )
        return cached

    tools_dir_str = str(tools_dir)
    original_sys_path = list(sys.path)
    # Ensure the intended tools directory is at the FRONT of the search
    # path even if the identical string was already present later in
    # sys.path -- never merely skip re-insertion in that case, since a
    # decoy earlier in sys.path would otherwise win the import.
    sys.path = [p for p in sys.path if p != tools_dir_str]
    sys.path.insert(0, tools_dir_str)

    importlib.invalidate_caches()

    try:
        import goal_navigator  # noqa: E402  (path adjusted above by design)
    except Exception as e:
        sys.path = original_sys_path
        sys.modules.pop("goal_navigator", None)
        raise GoalNavigatorImportError(
            f"failed to import 'goal_navigator' from {tools_dir}"
        ) from e

    imported_file = getattr(goal_navigator, "__file__", None)
    if imported_file is None or Path(imported_file).resolve() != goal_navigator_file:
        sys.path = original_sys_path
        sys.modules.pop("goal_navigator", None)
        raise GoalNavigatorImportError(
            f"imported 'goal_navigator' module resolved to {imported_file!r}, "
            f"not the intended committed file {goal_navigator_file}"
        )

    return goal_navigator


def main(argv=None):
    import rclpy

    from hil_goal_announcement_evidence import build_evidence_navigator_class

    goal_navigator = _import_goal_navigator()
    raw_argv = argv if argv is not None else sys.argv[1:]
    adapter_parser = argparse.ArgumentParser(add_help=False)
    adapter_parser.add_argument("--field-origin-x-m", type=float, default=0.0)
    adapter_parser.add_argument("--field-origin-y-m", type=float, default=0.0)
    adapter_parser.add_argument("--field-origin-yaw-rad", type=float, default=0.0)
    adapter_args, navigator_argv = adapter_parser.parse_known_args(raw_argv)
    origin_values = (
        adapter_args.field_origin_x_m,
        adapter_args.field_origin_y_m,
        adapter_args.field_origin_yaw_rad,
    )
    if not all(math.isfinite(value) for value in origin_values):
        raise SystemExit("field-origin values must all be finite")

    args = goal_navigator.parse_args(navigator_argv)
    args.field_origin_x_m = adapter_args.field_origin_x_m
    args.field_origin_y_m = adapter_args.field_origin_y_m
    args.field_origin_yaw_rad = adapter_args.field_origin_yaw_rad
    EvidenceNavigator = build_evidence_navigator_class()

    # Deliberately NOT use_sim_time -- see module docstring. This is the
    # one line this adapter exists to change relative to
    # goal_navigator.main().
    rclpy.init(args=[])
    node = EvidenceNavigator(args)
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
