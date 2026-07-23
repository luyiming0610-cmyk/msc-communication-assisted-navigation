#!/usr/bin/env python3
"""Simulated/virtual peer robot for the single-real-robot HIL trial.

Publishes a synthetic EpuckState (and, once its own simple search/goal
logic decides the exit is "found", a GoalAnnouncement) on a distinct
virtual-peer namespace, so the one real e-puck2 has a second agent to
react to. This does NOT model physical collision dynamics or a genuine
two-body avoidance interaction -- it is a scripted/kinematic stand-in,
and must never be described in results as a physical collision-avoidance
test between two robots.

Kinematics are a trivial unicycle integrator (`step_virtual_state`),
kept pure/testable and deliberately simple: HIL's purpose is to probe
whether the REAL robot's behaviour changes with communication, not to
validate a second physics engine.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class VirtualState:
    x_m: float
    y_m: float
    yaw_rad: float
    linear_mps: float
    angular_rps: float


def step_virtual_state(state: VirtualState, dt_s: float, target_linear_mps: float, target_angular_rps: float) -> VirtualState:
    """Pure unicycle-model integration step, no I/O.

    dt_s must be positive and finite -- callers are responsible for
    discarding non-positive/garbage dt (e.g. first tick, clock jump).
    """
    if dt_s <= 0.0 or not math.isfinite(dt_s):
        return state
    new_yaw = state.yaw_rad + target_angular_rps * dt_s
    new_x = state.x_m + target_linear_mps * math.cos(state.yaw_rad) * dt_s
    new_y = state.y_m + target_linear_mps * math.sin(state.yaw_rad) * dt_s
    return VirtualState(new_x, new_y, new_yaw, target_linear_mps, target_angular_rps)


def distance_to(state: VirtualState, x_m: float, y_m: float) -> float:
    return math.hypot(state.x_m - x_m, state.y_m - y_m)


@dataclass(frozen=True)
class VirtualNavPlan:
    """A minimal scripted plan: drive toward a single target point at a
    fixed speed, then hold. No search behaviour, no dynamic replanning --
    the virtual peer's role in HIL_COMM_ON is only to originate a
    GoalAnnouncement once it reaches (or is configured to already know)
    the exit, mirroring goal_navigator.py's informed-robot role.
    """
    target_x_m: float
    target_y_m: float
    cruise_linear_mps: float
    arrival_radius_m: float


def plan_virtual_command(state: VirtualState, plan: VirtualNavPlan, max_angular_rps: float) -> tuple[float, float, bool]:
    """Returns (linear_mps, angular_rps, arrived). Pure function."""
    remaining = distance_to(state, plan.target_x_m, plan.target_y_m)
    if remaining <= plan.arrival_radius_m:
        return 0.0, 0.0, True
    desired_yaw = math.atan2(plan.target_y_m - state.y_m, plan.target_x_m - state.x_m)
    yaw_error = math.atan2(math.sin(desired_yaw - state.yaw_rad), math.cos(desired_yaw - state.yaw_rad))
    angular = max(-max_angular_rps, min(max_angular_rps, yaw_error))
    linear = plan.cruise_linear_mps if abs(yaw_error) < (math.pi / 4.0) else 0.0
    return linear, angular, False


# --------------------------------------------------------------------
# rclpy wrapper
# --------------------------------------------------------------------
def _build_node():
    import argparse

    import rclpy
    from rclpy.node import Node

    from epuck2_comm_interfaces.msg import EpuckState, GoalAnnouncement

    class HilVirtualPeer(Node):
        def __init__(self, args):
            super().__init__("hil_virtual_peer")
            self.args = args
            self.state = VirtualState(args.start_x_m, args.start_y_m, args.start_yaw_rad, 0.0, 0.0)
            self.plan = VirtualNavPlan(
                target_x_m=args.target_x_m,
                target_y_m=args.target_y_m,
                cruise_linear_mps=args.cruise_linear_mps,
                arrival_radius_m=args.arrival_radius_m,
            )
            self.announced = False
            self.last_tick_s = None
            self.state_seq = 0
            self.announce_seq = 0

            self.state_pub = self.create_publisher(EpuckState, args.state_topic, 20)
            self.announcement_pub = None
            if args.announcement_topic:
                self.announcement_pub = self.create_publisher(GoalAnnouncement, args.announcement_topic, 10)

            self.create_timer(1.0 / args.rate_hz, self._tick)
            self.get_logger().info(
                f"HIL_VIRTUAL_PEER_READY namespace={args.namespace} "
                f"announce_enabled={self.announcement_pub is not None}"
            )

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1.0e9

        def _tick(self) -> None:
            now = self._now()
            dt = 0.0 if self.last_tick_s is None else (now - self.last_tick_s)
            self.last_tick_s = now

            linear, angular, arrived = plan_virtual_command(self.state, self.plan, self.args.max_angular_rps)
            self.state = step_virtual_state(self.state, dt, linear, angular)

            self.state_seq += 1
            msg = EpuckState()
            msg.version = EpuckState.PROTOCOL_VERSION
            msg.robot_id = self.args.robot_id
            msg.sequence = self.state_seq % (2 ** 32)
            msg.stamp = self.get_clock().now().to_msg()
            msg.source = EpuckState.SOURCE_VIRTUAL
            msg.x_m = self.state.x_m
            msg.y_m = self.state.y_m
            msg.yaw_rad = self.state.yaw_rad
            msg.linear_velocity_mps = self.state.linear_mps
            msg.angular_velocity_rps = self.state.angular_rps
            msg.validity_flags = EpuckState.FLAG_ODOM_VALID
            self.state_pub.publish(msg)

            if arrived and not self.announced and self.announcement_pub is not None:
                self.announce_seq += 1
                announcement = GoalAnnouncement()
                announcement.protocol_version = GoalAnnouncement.PROTOCOL_VERSION
                announcement.source_robot_id = self.args.robot_id
                announcement.sequence = self.announce_seq
                announcement.production_stamp = self.get_clock().now().to_msg()
                announcement.goal_id = self.args.goal_id
                announcement.goal_x_m = self.plan.target_x_m
                announcement.goal_y_m = self.plan.target_y_m
                announcement.valid = True
                self.announcement_pub.publish(announcement)
                self.announced = True
                self.get_logger().warn(
                    f"HIL_VIRTUAL_PEER_ANNOUNCEMENT_TX target=({self.plan.target_x_m},{self.plan.target_y_m})"
                )

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--namespace", default="epuck_virtual_peer")
        parser.add_argument("--robot-id", type=int, required=True)
        parser.add_argument("--state-topic", required=True)
        parser.add_argument("--announcement-topic", default="")
        parser.add_argument("--goal-id", default="shared_exit")
        parser.add_argument("--start-x-m", type=float, required=True)
        parser.add_argument("--start-y-m", type=float, required=True)
        parser.add_argument("--start-yaw-rad", type=float, default=0.0)
        parser.add_argument("--target-x-m", type=float, required=True)
        parser.add_argument("--target-y-m", type=float, required=True)
        parser.add_argument("--cruise-linear-mps", type=float, required=True)
        parser.add_argument("--arrival-radius-m", type=float, required=True)
        parser.add_argument("--max-angular-rps", type=float, required=True)
        parser.add_argument("--rate-hz", type=float, default=20.0)
        return parser.parse_args(argv)

    return rclpy, HilVirtualPeer, parse_args


def main(argv=None):
    import sys

    rclpy, HilVirtualPeer, parse_args = _build_node()
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[])
    node = HilVirtualPeer(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # SIGINT surfaces here as ExternalShutdownException in this
        # rclpy version, not KeyboardInterrupt -- catching only the
        # latter let a clean SIGINT shutdown still exit nonzero with an
        # uncaught traceback (functionally harmless: the finally block
        # below still runs either way) but noisy. Found and fixed
        # 2026-07-23 while auditing every hil_*.py tool's shutdown path.
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
