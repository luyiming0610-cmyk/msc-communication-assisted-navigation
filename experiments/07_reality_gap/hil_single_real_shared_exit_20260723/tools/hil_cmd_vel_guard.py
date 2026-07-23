#!/usr/bin/env python3
"""Hardware-in-the-loop cmd_vel safety guard.

This is the ONLY component allowed to publish to the real e-puck2's
driver-facing `/cmd_vel` topic during any HIL work. It sits between the
frozen, UNMODIFIED `cooperative_avoider` node's own `cmd_vel` output
(subscribed here on a distinct, guard-input topic, never the driver
topic directly) and the physical robot.

Design principle: the decision logic (`GuardDecision`/`decide_command()`)
is pure Python, no rclpy dependency, so every fail-closed path can be
unit tested without a ROS graph. The rclpy `Node` wrapper below only
wires real topics/timers to that pure function -- it contains no
additional safety logic of its own, so there is exactly one place the
fail-closed rules live.

Defaults (all binding, per the pre-registered HIL safety design):
  - armed=False until explicitly commanded ARMED (never auto-arms).
  - Publishes a zero Twist by default, before any target command exists.
  - Hard linear speed ceiling (0.02 m/s) enforced independently of
    whatever cooperative_avoider's own nominal_speed_mps parameter is
    set to -- this guard does not trust the upstream node's own clamp.
  - Angular speed ceiling must be an explicitly confirmed, non-UNCONFIRMED
    value or the guard refuses to ever output a nonzero angular command.
  - Heartbeat (freshness of the upstream cmd_vel stream itself), physical
    EpuckState freshness+validity+protocol version, and virtual-peer
    freshness (when required) are all checked every tick; any one being
    stale/invalid/missing forces a zero command.
  - Exactly one upstream cmd_vel publisher is required; zero or multiple
    is refused (multiple ROS2 publishers on one topic interleave
    nondeterministically -- a genuine hazard, not a cosmetic one).
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class GuardDecision:
    linear_mps: float
    angular_rps: float
    armed_effective: bool
    blocked_reasons: tuple[str, ...] = field(default_factory=tuple)
    clamped: bool = False

    @property
    def is_moving(self) -> bool:
        return self.armed_effective and not self.blocked_reasons and (
            self.linear_mps != 0.0 or self.angular_rps != 0.0
        )


def decide_command(
    *,
    armed: bool,
    target_linear_mps: float,
    target_angular_rps: float,
    now_s: float,
    max_linear_speed_mps: float,
    max_angular_speed_rps: float | None,
    last_heartbeat_at_s: float | None,
    heartbeat_timeout_s: float,
    last_physical_state_at_s: float | None,
    physical_state_timeout_s: float,
    physical_state_protocol_ok: bool,
    physical_validity_flags: int,
    required_validity_flags: int,
    require_virtual_peer: bool,
    last_virtual_peer_at_s: float | None,
    virtual_peer_timeout_s: float,
    upstream_cmd_vel_publisher_count: int,
) -> GuardDecision:
    """Pure fail-closed decision function -- no I/O, no ROS dependency.

    Every check below is independently sufficient to force a zero
    command; checks are evaluated in a fixed order so the FIRST blocking
    reason found is reported (informative for logs), but ANY blocking
    condition alone always zeroes the command -- this is not a priority
    system, every check is load-bearing.
    """
    reasons: list[str] = []

    if not armed:
        reasons.append("DISARMED")

    if upstream_cmd_vel_publisher_count != 1:
        reasons.append(
            f"UPSTREAM_CMD_VEL_PUBLISHER_COUNT_INVALID({upstream_cmd_vel_publisher_count})"
        )

    if max_angular_speed_rps is None:
        reasons.append("ANGULAR_SPEED_LIMIT_UNCONFIRMED")

    if max_linear_speed_mps <= 0.0:
        reasons.append("LINEAR_SPEED_LIMIT_NOT_POSITIVE")

    if last_heartbeat_at_s is None or (now_s - last_heartbeat_at_s) > heartbeat_timeout_s:
        reasons.append("HEARTBEAT_STALE_OR_MISSING")

    if last_physical_state_at_s is None or (now_s - last_physical_state_at_s) > physical_state_timeout_s:
        reasons.append("PHYSICAL_STATE_STALE_OR_MISSING")

    if not physical_state_protocol_ok:
        reasons.append("PHYSICAL_STATE_PROTOCOL_VERSION_MISMATCH")

    if (int(physical_validity_flags) & int(required_validity_flags)) != int(required_validity_flags):
        reasons.append("PHYSICAL_STATE_INVALID_FLAGS")

    if require_virtual_peer:
        if last_virtual_peer_at_s is None or (now_s - last_virtual_peer_at_s) > virtual_peer_timeout_s:
            reasons.append("VIRTUAL_PEER_STALE_OR_MISSING")

    if reasons:
        return GuardDecision(0.0, 0.0, armed_effective=False, blocked_reasons=tuple(reasons))

    clamped = False
    linear = target_linear_mps
    if linear < 0.0:
        linear = 0.0
        clamped = True
    if linear > max_linear_speed_mps:
        linear = max_linear_speed_mps
        clamped = True

    angular = target_angular_rps
    if angular > max_angular_speed_rps:
        angular = max_angular_speed_rps
        clamped = True
    elif angular < -max_angular_speed_rps:
        angular = -max_angular_speed_rps
        clamped = True

    return GuardDecision(linear, angular, armed_effective=True, blocked_reasons=(), clamped=clamped)


# --------------------------------------------------------------------
# rclpy wrapper -- thin, no additional safety logic. Only importable
# (and only actually constructed) once rclpy/ROS message packages are
# on the path, so the pure function above stays testable without ROS.
# --------------------------------------------------------------------
def _build_node():
    import argparse

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool

    from epuck2_comm_interfaces.msg import EpuckState

    class HilCmdVelGuard(Node):
        def __init__(self, args):
            super().__init__("hil_cmd_vel_guard")
            self.args = args
            self.armed = False  # NEVER auto-armed; only a live /hil_guard/arm True message arms it.
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.last_heartbeat_at = None
            self.last_physical_state_at = None
            self.last_virtual_peer_at = None
            self.physical_state_protocol_ok = False
            self.physical_validity_flags = 0

            self.guarded_pub = self.create_publisher(Twist, args.guarded_cmd_vel_topic, 10)
            self.create_subscription(Twist, args.upstream_cmd_vel_topic, self._upstream_cb, 10)
            self.create_subscription(Bool, args.arm_topic, self._arm_cb, 10)
            self.create_subscription(EpuckState, args.physical_state_topic, self._physical_state_cb, 20)
            if args.require_virtual_peer:
                self.create_subscription(EpuckState, args.virtual_peer_topic, self._virtual_peer_cb, 20)

            self.create_timer(0.05, self._tick)
            self.get_logger().warn(
                "HIL_CMD_VEL_GUARD_READY armed=False (DISARMED by default) "
                f"max_linear_speed_mps={args.max_linear_speed_mps} "
                f"max_angular_speed_rps={args.max_angular_speed_rps}"
            )

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1.0e9

        def _upstream_cb(self, msg: Twist) -> None:
            self.target_linear = float(msg.linear.x)
            self.target_angular = float(msg.angular.z)
            self.last_heartbeat_at = self._now()

        def _arm_cb(self, msg) -> None:
            if bool(msg.data) != self.armed:
                self.armed = bool(msg.data)
                self.get_logger().warn(f"HIL_GUARD_ARM_STATE_CHANGED armed={self.armed}")

        def _physical_state_cb(self, msg: EpuckState) -> None:
            self.last_physical_state_at = self._now()
            self.physical_state_protocol_ok = int(msg.version) == int(EpuckState.PROTOCOL_VERSION)
            self.physical_validity_flags = int(msg.validity_flags)

        def _virtual_peer_cb(self, msg: EpuckState) -> None:
            self.last_virtual_peer_at = self._now()

        def _upstream_publisher_count(self) -> int:
            try:
                return len(self.get_publishers_info_by_topic(
                    self.resolve_topic_name(self.args.upstream_cmd_vel_topic)
                ))
            except Exception:
                return -1

        def _tick(self) -> None:
            now = self._now()
            decision = decide_command(
                armed=self.armed,
                target_linear_mps=self.target_linear,
                target_angular_rps=self.target_angular,
                now_s=now,
                max_linear_speed_mps=self.args.max_linear_speed_mps,
                max_angular_speed_rps=self.args.max_angular_speed_rps,
                last_heartbeat_at_s=self.last_heartbeat_at,
                heartbeat_timeout_s=self.args.heartbeat_timeout_s,
                last_physical_state_at_s=self.last_physical_state_at,
                physical_state_timeout_s=self.args.physical_state_timeout_s,
                physical_state_protocol_ok=self.physical_state_protocol_ok,
                physical_validity_flags=self.physical_validity_flags,
                required_validity_flags=self.args.required_validity_flags,
                require_virtual_peer=self.args.require_virtual_peer,
                last_virtual_peer_at_s=self.last_virtual_peer_at,
                virtual_peer_timeout_s=self.args.virtual_peer_timeout_s,
                upstream_cmd_vel_publisher_count=self._upstream_publisher_count(),
            )
            command = Twist()
            command.linear.x = float(decision.linear_mps)
            command.angular.z = float(decision.angular_rps)
            self.guarded_pub.publish(command)
            if decision.blocked_reasons:
                self.get_logger().warn(
                    f"HIL_GUARD_BLOCKED reasons={','.join(decision.blocked_reasons)}",
                    throttle_duration_sec=1.0,
                )

        def stop(self) -> None:
            """Called from SIGINT/exit handling -- guarantees zero output."""
            command = Twist()
            for _ in range(3):
                try:
                    self.guarded_pub.publish(command)
                except Exception:
                    break

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--upstream-cmd-vel-topic", default="cmd_vel_unguarded")
        parser.add_argument("--guarded-cmd-vel-topic", default="cmd_vel")
        parser.add_argument("--arm-topic", default="/hil_guard/arm")
        parser.add_argument("--physical-state-topic", required=True)
        parser.add_argument("--virtual-peer-topic", default="")
        parser.add_argument("--require-virtual-peer", action="store_true")
        parser.add_argument("--max-linear-speed-mps", type=float, default=0.02)
        parser.add_argument("--max-angular-speed-rps", type=float, default=None)
        parser.add_argument("--heartbeat-timeout-s", type=float, default=0.5)
        parser.add_argument("--physical-state-timeout-s", type=float, default=0.5)
        parser.add_argument("--virtual-peer-timeout-s", type=float, default=1.0)
        parser.add_argument("--required-validity-flags", type=int, default=EpuckState.FLAG_ODOM_VALID)
        return parser.parse_args(argv)

    return rclpy, Node, HilCmdVelGuard, parse_args


def main(argv=None):
    import sys

    rclpy, _Node, HilCmdVelGuard, parse_args = _build_node()
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[])
    node = HilCmdVelGuard(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
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
