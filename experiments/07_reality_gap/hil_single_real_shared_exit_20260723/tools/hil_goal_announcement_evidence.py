#!/usr/bin/env python3
"""Adapter-owned GoalAnnouncement reception evidence for the HIL trial.

goal_navigator.py is frozen (shared with the completed N2/N3 formal
batches) and must not be edited to add HIL-specific logging. This module
instead defines a small subclass of GoalNavigator that wraps its
existing, unmodified `_announcement_cb` -- calling it exactly as before
for all navigation logic, then emitting one bounded, structured,
grep-able log line recording what was received and what happened.

Architecture note (important, do not misdescribe this elsewhere):
GoalNavigator does not republish or forward a received GoalAnnouncement
onto any other topic -- it only consumes one to update its own internal
NavigationTargetState. There is therefore no "forwarded" message and no
"forward timestamp" to record here. The evidence below covers reception
and adoption-state at THIS receiving node only.
"""
from __future__ import annotations

import importlib
import sys
from pathlib import Path

# Resolved once, at import time, from this file's own on-disk location
# via pathlib -- never from the current working directory, and never by
# counting `os.path.dirname()` calls. See hil_topic_adapter.py's own
# docstring for the exact defect this replaces (identical dirname-chain
# bug, same file depth, hit live in the same execution attempt).
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


#: Stage-4-only additive interface (see build_evidence_navigator_class()
#: docstring below for why this exists). Not part of the Stage 3 contract.
STAGE4_ADOPTION_EVIDENCE_TOPIC = "/hil/adoption_evidence"

#: Frozen schema version for the /hil/adoption_evidence payload (design
#: review, 2026-07-30, revision 3). Any consumer (the Stage 4 motion
#: supervisor) must reject a payload whose schema_version does not
#: match this exactly -- see
#: hil_stage4_motion_supervisor.ADOPTION_EVIDENCE_SCHEMA_VERSION, which
#: must be kept identical to this constant.
STAGE4_ADOPTION_EVIDENCE_SCHEMA_VERSION = "1.0.0"


def build_evidence_navigator_class():
    """Returns the HilGoalAnnouncementEvidenceNavigator class, importing
    goal_navigator.py lazily (same pattern as hil_topic_adapter.py) so
    this module can be imported for its pure helpers without requiring
    rclpy/epuck2_comm_interfaces to be available."""
    goal_navigator = _import_goal_navigator()
    from std_msgs.msg import String

    class HilGoalAnnouncementEvidenceNavigator(goal_navigator.GoalNavigator):
        """Identical to GoalNavigator in every respect except one
        additional evidence log line, and one additional machine-readable
        adoption-evidence message, per received GoalAnnouncement. No
        navigation-state field, threshold, or timing is changed.

        Stage-4 addition (2026-07-30): the log line below is human-
        readable only and must never be used as a safety gate input.
        Stage 4's motion supervisor needs to know, online and without
        scraping logs, whether THIS receiver adopted the exact announced
        goal_id/coordinates. GoalNavigator itself never republishes a
        received announcement onto any topic (see module docstring), so
        no such signal existed before this addition. This publisher is
        the smallest additive fix: one JSON-encoded std_msgs/String per
        received announcement, carrying the same facts already computed
        for the log line, added after it, never replacing it.
        """

        _stage4_adoption_evidence_pub = None

        def _announcement_cb(self, msg) -> None:
            adapter_receive_time_s = self.get_clock().now().nanoseconds / 1.0e9
            was_switched_before = self.target_state.switched_to_goal

            super()._announcement_cb(msg)

            switched_after = self.target_state.switched_to_goal
            accepted = switched_after and not was_switched_before
            # A duplicate is any subsequent valid announcement arriving
            # after adoption already latched -- receive_announcement()
            # itself is the authority on this (idempotent no-op), this
            # is only evidence describing that same outcome.
            duplicate = bool(msg.valid) and was_switched_before
            source_stamp_s = msg.production_stamp.sec + msg.production_stamp.nanosec / 1.0e9
            tx, ty = self.target_state.current_target
            self.get_logger().info(
                "HIL_GOAL_ANNOUNCEMENT_EVIDENCE "
                f"receiver_robot_id={self.args.robot_id} "
                f"goal_id={msg.goal_id} "
                f"source_robot_id={msg.source_robot_id} "
                f"source_sequence={msg.sequence} "
                f"source_stamp_s={source_stamp_s:.6f} "
                f"adapter_receive_time_s={adapter_receive_time_s:.6f} "
                f"valid={bool(msg.valid)} "
                f"accepted={accepted} "
                f"duplicate={duplicate} "
                f"current_target=({tx:.4f},{ty:.4f})"
            )

            if self._stage4_adoption_evidence_pub is None:
                self._stage4_adoption_evidence_pub = self.create_publisher(
                    String, STAGE4_ADOPTION_EVIDENCE_TOPIC, 10
                )
            import json
            import time as _time

            evidence_msg = String()
            evidence_msg.data = json.dumps({
                "schema_version": STAGE4_ADOPTION_EVIDENCE_SCHEMA_VERSION,
                "receiver_robot_id": self.args.robot_id,
                "goal_id": msg.goal_id,
                "source_robot_id": msg.source_robot_id,
                "source_sequence": int(msg.sequence),
                "source_stamp_s": source_stamp_s,
                "adapter_receive_time_s": adapter_receive_time_s,
                "adapter_receive_monotonic_s": _time.monotonic(),
                "valid": bool(msg.valid),
                "accepted": accepted,
                "duplicate": duplicate,
                "target_x_m": tx,
                "target_y_m": ty,
            })
            self._stage4_adoption_evidence_pub.publish(evidence_msg)

    return HilGoalAnnouncementEvidenceNavigator
