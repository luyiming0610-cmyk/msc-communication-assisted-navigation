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


def build_evidence_navigator_class():
    """Returns the HilGoalAnnouncementEvidenceNavigator class, importing
    goal_navigator.py lazily (same pattern as hil_topic_adapter.py) so
    this module can be imported for its pure helpers without requiring
    rclpy/epuck2_comm_interfaces to be available."""
    goal_navigator = _import_goal_navigator()

    class HilGoalAnnouncementEvidenceNavigator(goal_navigator.GoalNavigator):
        """Identical to GoalNavigator in every respect except one
        additional evidence log line per received GoalAnnouncement. No
        navigation-state field, threshold, or timing is changed."""

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

    return HilGoalAnnouncementEvidenceNavigator
