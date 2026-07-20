#!/usr/bin/env python3
"""Standalone, per-robot goal-directed navigation node for the shared
exit-navigation study.

Two roles, selected by --mode:
  - "informed": the robot already knows its target (Robot A -- this is
    its own a-priori information, not communication) and navigates
    directly toward it from the start of the trial.
  - "search": the robot follows a frozen, pre-registered waypoint
    sequence (Robot B under N2_EXIT_COMM_OFF, and Robot B under
    N2_EXIT_COMM_ON until it receives a valid exit announcement) --
    identical in both conditions up to the point of any switch.

Computes desired_heading_rad = atan2(target_y - y, target_x - x) from
the robot's own /epuckN/state position and publishes it as a
NavigationIntent message at a bounded rate for that SAME robot's own
cooperative_avoider to consume (never cross-robot -- this is not the
communication channel under study).

Optionally (--announce, informed mode only, N2_EXIT_COMM_ON's Robot A):
also periodically publishes GoalAnnouncement with the robot's own known
target, for the peer's goal_navigator (in --goal-announcement-topic
subscriber mode) to receive. GoalAnnouncement is the ONLY channel that
can carry exit information between robots; NavigationIntent never is.

All switch/discovery events are logged via the ROS logger with a fixed,
grep-able key=value line format so the orchestrator's log files are the
evidence record (mirrors task_completion_monitor.py's TASK_COMPLETE_GOAL
line convention) -- never fabricated, never a hardcoded 0 in place of a
real measurement.
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node

from epuck2_comm_interfaces.msg import EpuckState, GoalAnnouncement, NavigationIntent

from goal_hold_tracker import GoalHoldTracker
from navigation_target_state import NavigationTargetState


def _parse_waypoints(text: str) -> list[tuple[float, float]]:
    points = []
    for chunk in text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        x_str, y_str = chunk.split(":")
        points.append((float(x_str), float(y_str)))
    return points


class GoalNavigator(Node):
    def __init__(self, args):
        super().__init__("goal_navigator")
        self.args = args
        self.own_position = None  # (x, y)
        self.nav_seq = 0
        self.announce_seq = 0
        self.exit_known_logged = False

        # The shared exit stays the single navigation/announcement target
        # (informational identity, common to both robots). Each robot's
        # own, distinct, non-colliding parking point (Part V) only takes
        # over as current_target once this robot has geometrically
        # entered the exit region -- handled by
        # NavigationTargetState.check_exit_to_parking_switch().
        parking_target_m = (args.parking_x, args.parking_y)
        exit_center_m = (args.exit_center_x, args.exit_center_y)
        if args.mode == "informed":
            self.target_state = NavigationTargetState(
                mode="informed", current_target=(args.target_x, args.target_y),
                parking_target_m=parking_target_m,
                exit_center_m=exit_center_m, exit_radius_m=args.exit_radius,
            )
        else:
            self.target_state = NavigationTargetState(
                mode="search",
                waypoints=_parse_waypoints(args.waypoints),
                waypoint_arrival_radius_m=args.waypoint_arrival_radius,
                parking_target_m=parking_target_m,
                exit_center_m=exit_center_m, exit_radius_m=args.exit_radius,
            )

        self.create_subscription(EpuckState, args.state_topic, self._state_cb, 20)
        self.nav_pub = self.create_publisher(NavigationIntent, args.nav_intent_topic, 10)

        self.announce_pub = None
        if args.announce:
            self.announce_pub = self.create_publisher(
                GoalAnnouncement, args.announce_topic, 10
            )

        self.announcement_sub = None
        if args.goal_announcement_topic:
            self.announcement_sub = self.create_subscription(
                GoalAnnouncement, args.goal_announcement_topic, self._announcement_cb, 10
            )

        # Arrival/hold is judged at THIS robot's own parking point (Part
        # V), not at the shared exit itself -- geometry and hold time
        # come from frozen_params (via CLI, injected by the
        # orchestrator), never hardcoded here.
        self.goal_hold_tracker = GoalHoldTracker(
            center_x_m=args.parking_x,
            center_y_m=args.parking_y,
            radius_m=args.parking_radius,
            hold_time_s=args.goal_hold_time_s,
        )

        period_s = 1.0 / max(args.rate_hz, 0.1)
        self.create_timer(period_s, self._tick)

        self.get_logger().info(
            f"goal_navigator READY robot_id={args.robot_id} mode={args.mode} "
            f"target={self.target_state.current_target} announce={bool(args.announce)} "
            f"listens_for_announcement={bool(args.goal_announcement_topic)} "
            f"exit_region=({args.exit_center_x:.4f},{args.exit_center_y:.4f},"
            f"r={args.exit_radius:.4f}) "
            f"parking_region=({args.parking_x:.4f},{args.parking_y:.4f},"
            f"r={args.parking_radius:.4f}) "
            f"goal_hold_time_s={args.goal_hold_time_s:.3f}"
        )

    def _state_cb(self, msg: EpuckState) -> None:
        self.own_position = (float(msg.x_m), float(msg.y_m))
        t_s = self._ros_time_s()
        if self.args.mode == "informed" and not self.exit_known_logged:
            self.exit_known_logged = True
            tx, ty = self.target_state.current_target
            self.get_logger().info(
                f"EXIT_KNOWN_AT_START robot_id={self.args.robot_id} t={t_s:.3f} "
                f"x={tx:.4f} y={ty:.4f}"
            )
        if self.args.mode == "search":
            advanced = self.target_state.update_position(*self.own_position)
            if advanced:
                tx, ty = self.target_state.current_target
                self.get_logger().info(
                    f"WAYPOINT_REACHED robot_id={self.args.robot_id} "
                    f"index={self.target_state.waypoint_index} t={t_s:.3f} "
                    f"target=({tx:.4f},{ty:.4f})"
                )

        if not self.target_state.arrived_hold:
            x, y = self.own_position
            switched_parking = self.target_state.check_exit_to_parking_switch(x, y)
            if switched_parking:
                px, py = self.target_state.current_target
                self.get_logger().info(
                    f"EXIT_TO_PARKING_SWITCH robot_id={self.args.robot_id} t={t_s:.3f} "
                    f"parking_target=({px:.4f},{py:.4f})"
                )
            self.goal_hold_tracker.update(t_s, x, y)
            if self.goal_hold_tracker.reached:
                heading_at_latch = self.target_state.desired_heading_rad(x, y)
                latched = self.target_state.latch_arrived_hold(
                    heading_at_latch, self.goal_hold_tracker.completion_time_s
                )
                if latched:
                    self.get_logger().info(
                        f"ARRIVED_HOLD_LATCHED robot_id={self.args.robot_id} "
                        f"t={t_s:.3f} "
                        f"individual_completion_time_s="
                        f"{self.target_state.individual_completion_time_s:.3f} "
                        f"frozen_heading_rad={heading_at_latch:.4f}"
                    )

    def _announcement_cb(self, msg: GoalAnnouncement) -> None:
        switched = self.target_state.receive_announcement(
            float(msg.goal_x_m), float(msg.goal_y_m), bool(msg.valid)
        )
        if switched:
            t_s = self._ros_time_s()
            tx, ty = self.target_state.current_target
            self.get_logger().info(
                f"SEARCH_TO_GOAL_SWITCH robot_id={self.args.robot_id} t={t_s:.3f} "
                f"announcement_sequence={msg.sequence} "
                f"target=({tx:.4f},{ty:.4f})"
            )

    def _ros_time_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _tick(self) -> None:
        if self.own_position is not None:
            x, y = self.own_position
            heading = self.target_state.desired_heading_rad(x, y)
            self.nav_seq += 1
            nav_msg = NavigationIntent()
            nav_msg.protocol_version = NavigationIntent.PROTOCOL_VERSION
            nav_msg.source_robot_id = self.args.robot_id
            nav_msg.sequence = self.nav_seq
            nav_msg.production_stamp = self.get_clock().now().to_msg()
            nav_msg.desired_heading_rad = float(heading)
            nav_msg.desired_linear_speed_mps = float(
                self.target_state.desired_linear_speed_mps(self.args.nominal_speed_mps)
            )
            nav_msg.navigation_phase = self.target_state.navigation_phase
            nav_msg.valid = True
            self.nav_pub.publish(nav_msg)

        if self.announce_pub is not None:
            self.announce_seq += 1
            tx, ty = self.target_state.current_target
            ann_msg = GoalAnnouncement()
            ann_msg.protocol_version = GoalAnnouncement.PROTOCOL_VERSION
            ann_msg.source_robot_id = self.args.robot_id
            ann_msg.sequence = self.announce_seq
            ann_msg.production_stamp = self.get_clock().now().to_msg()
            ann_msg.goal_id = self.args.goal_id
            ann_msg.goal_x_m = float(tx)
            ann_msg.goal_y_m = float(ty)
            ann_msg.valid = True
            self.announce_pub.publish(ann_msg)
            if self.announce_seq == 1:
                t_s = self._ros_time_s()
                self.get_logger().info(
                    f"ANNOUNCEMENT_TX_FIRST robot_id={self.args.robot_id} t={t_s:.3f} "
                    f"target=({tx:.4f},{ty:.4f})"
                )


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-id", type=int, required=True)
    parser.add_argument("--state-topic", required=True)
    parser.add_argument("--nav-intent-topic", required=True)
    parser.add_argument("--mode", choices=["informed", "search"], required=True)
    parser.add_argument("--target-x", type=float, default=0.0)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--waypoints", default="")
    parser.add_argument("--waypoint-arrival-radius", type=float, default=0.10)
    parser.add_argument("--goal-announcement-topic", default="")
    parser.add_argument("--announce", action="store_true")
    parser.add_argument("--announce-topic", default="")
    parser.add_argument("--goal-id", default="shared_exit")
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--nominal-speed-mps", type=float, required=True)
    parser.add_argument("--exit-center-x", type=float, required=True)
    parser.add_argument("--exit-center-y", type=float, required=True)
    parser.add_argument("--exit-radius", type=float, required=True)
    parser.add_argument("--parking-x", type=float, required=True)
    parser.add_argument("--parking-y", type=float, required=True)
    parser.add_argument("--parking-radius", type=float, required=True)
    parser.add_argument("--goal-hold-time-s", type=float, required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    # use_sim_time=true is required so this node's ROS clock follows
    # Webots simulation time (via /clock) instead of the wall clock --
    # every other process in this study (state_publisher,
    # cooperative_avoider, task_completion_monitor) is sim-time-based.
    # Missing this made a real, confirmed bug (PILOT02): logged event
    # timestamps (EXIT_KNOWN_AT_START etc.) were raw wall-clock Unix
    # epoch values, incompatible with the sim-time-stamped state samples
    # the analyzer normalizes against. Position-based navigation itself
    # was unaffected (waypoint advancement uses only x/y, never a
    # clock), only the logged timestamps were wrong.
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = GoalNavigator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
