#!/usr/bin/env python3
"""Bounded-duration, straight-line-only, GROUND-motion single-pulse
publisher -- added 2026-07-28 for NEW_FIELD_SINGLE_PULSE_REVALIDATION.

Reuses (imports, never duplicates) `compute_phase()` from
`hil_wheel_suspension_test.py` -- that state machine (ZERO_HOLD ->
PULSE_FORWARD -> POST_HOLD -> DONE, self-terminating, publishes zero
on exit, no CLI angular override) is already exactly correct for a
bounded ground pulse. The only thing that needed to change is the
identity: `hil_wheel_suspension_test.py`'s name, log prefixes, and
module docstring assert a WHEELS-SUSPENDED safety context (its own
established guarantee), which would be misleading -- not merely
cosmetically wrong -- if reused verbatim for a run where the robot's
wheels are actually bearing weight on the ground. This module is that
same, unmodified state machine, wrapped in a clearly ground-named node
so the evidence/log trail for a ground-motion run never claims a
wheels-suspended identity it does not have.

Publishes ONLY to `cmd_vel_unguarded` (never the driver-facing
/cmd_vel directly, and never through anything but
hil_cmd_vel_guard.py) and NEVER requests a nonzero angular velocity --
there is no CLI flag for angular velocity at all, by design, so this
script cannot request a turn even by mistake, exactly matching
hil_wheel_suspension_test.py's own safety pattern.

Each invocation is one bounded, self-terminating run -- there is no
mode that runs indefinitely, and no automatic second pulse from a
single invocation.

Two additional safety properties, both pure-logic and unit-tested
without rclpy (see test_hil_ground_single_pulse_test.py):

- `validate_upstream_cmd_vel_topic()` makes the guard-input topic
  fail-closed by construction, not by runbook convention alone. Only
  `cmd_vel_unguarded`/`/cmd_vel_unguarded` (the guard's own upstream
  input) are accepted; anything else -- especially `cmd_vel`/`/cmd_vel`,
  the driver-facing output the guard exists to protect -- is rejected
  before rclpy is even initialized, so no publisher and no message can
  exist on a rejected topic.
- `best_effort_final_zero_burst()` is called on every exit path (normal
  completion, KeyboardInterrupt, SystemExit, ExternalShutdownException,
  or an unexpected runtime exception) to attempt one final zero Twist
  publish before node destruction/shutdown. It is genuinely best-effort:
  several of those paths (in particular ExternalShutdownException, or
  any exception raised after the ROS context is already torn down) can
  leave publication technically impossible, in which case this
  silently returns False and never raises -- it never masks whatever
  exception is already propagating.
"""
from __future__ import annotations

from hil_wheel_suspension_test import compute_phase  # reused, not duplicated

__all__ = ["compute_phase", "validate_upstream_cmd_vel_topic", "best_effort_final_zero_burst"]

_ALLOWED_UPSTREAM_CMD_VEL_TOPICS = frozenset({"cmd_vel_unguarded", "/cmd_vel_unguarded"})


def validate_upstream_cmd_vel_topic(topic: str) -> str:
    """Fail-closed topic policy for this ground-motion physical tool.

    Only the guard's own upstream input topic may be used. This does
    not rely on the runbook always passing the correct default -- an
    operator (or a future caller) passing any other topic string,
    especially `cmd_vel`/`/cmd_vel`, is rejected here before any
    publisher is created and before the pulse sequence begins.
    """
    if topic not in _ALLOWED_UPSTREAM_CMD_VEL_TOPICS:
        raise ValueError(
            f"REJECTED_UPSTREAM_CMD_VEL_TOPIC={topic!r}; only "
            f"{sorted(_ALLOWED_UPSTREAM_CMD_VEL_TOPICS)} are permitted for this tool"
        )
    return topic


def best_effort_final_zero_burst(pub, build_zero_msg) -> bool:
    """Attempt one final zero-velocity publish. Returns True if the
    publish call completed, False if it was not possible (pub is None,
    or the publish call itself raised -- e.g. a context already being
    torn down). Never raises, so it can never mask whatever exception
    is already propagating in the caller."""
    if pub is None:
        return False
    try:
        pub.publish(build_zero_msg())
        return True
    except Exception:
        return False


def _build_node():
    import argparse

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    def _zero_twist() -> "Twist":
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        return msg

    class HilGroundSinglePulseTest(Node):
        def __init__(self, args):
            super().__init__("hil_ground_single_pulse_test")
            self.args = args
            self.pub = self.create_publisher(Twist, args.upstream_cmd_vel_topic, 10)
            self.start_s = self._now()
            self.last_phase_name = None
            self.create_timer(1.0 / args.rate_hz, self._tick)
            self.get_logger().warn(
                "HIL_GROUND_SINGLE_PULSE_TEST_START "
                f"zero_hold_s={args.zero_hold_s} pulse_s={args.pulse_s} "
                f"pulse_linear_mps={args.pulse_linear_mps} post_hold_s={args.post_hold_s} "
                "-- angular is always 0.0, no exceptions, no CLI override exists for it. "
                "GROUND motion -- wheels bearing weight, not the suspended-wheel diagnostic."
            )

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1.0e9

        def _tick(self) -> None:
            elapsed = self._now() - self.start_s
            phase = compute_phase(
                elapsed, self.args.zero_hold_s, self.args.pulse_s,
                self.args.pulse_linear_mps, self.args.post_hold_s,
            )
            if phase.name != self.last_phase_name:
                self.get_logger().warn(f"HIL_GROUND_SINGLE_PULSE_TEST_PHASE={phase.name} elapsed_s={elapsed:.2f}")
                self.last_phase_name = phase.name

            msg = Twist()
            msg.linear.x = float(phase.linear_mps)
            msg.angular.z = 0.0
            self.pub.publish(msg)

            if phase.done:
                for _ in range(3):
                    best_effort_final_zero_burst(self.pub, _zero_twist)
                self.get_logger().warn("HIL_GROUND_SINGLE_PULSE_TEST_DONE published zero 3x, exiting")
                raise SystemExit(0)

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--upstream-cmd-vel-topic", default="cmd_vel_unguarded")
        parser.add_argument("--rate-hz", type=float, default=10.0)
        parser.add_argument("--zero-hold-s", type=float, default=1.0)
        parser.add_argument("--pulse-s", type=float, required=True)
        parser.add_argument("--pulse-linear-mps", type=float, required=True)
        parser.add_argument("--post-hold-s", type=float, default=1.0)
        return parser.parse_args(argv)

    return rclpy, HilGroundSinglePulseTest, parse_args, _zero_twist


def main(argv=None):
    import sys

    rclpy, HilGroundSinglePulseTest, parse_args, zero_twist_factory = _build_node()
    args = parse_args(argv if argv is not None else sys.argv[1:])

    try:
        validate_upstream_cmd_vel_topic(args.upstream_cmd_vel_topic)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(2)

    rclpy.init(args=[])
    node = HilGroundSinglePulseTest(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, rclpy.executors.ExternalShutdownException):
        best_effort_final_zero_burst(node.pub, zero_twist_factory)
    except Exception:
        best_effort_final_zero_burst(node.pub, zero_twist_factory)
        raise
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
