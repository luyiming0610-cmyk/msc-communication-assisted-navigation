#!/usr/bin/env python3
"""Bounded-duration, rotation-only test publisher for the suspended-wheel
angular-rate diagnostic.

Mirrors hil_wheel_suspension_test.py's design exactly, with linear and
angular swapped: publishes ONLY to `cmd_vel_unguarded` (never the
driver-facing /cmd_vel directly, and never through anything but
hil_cmd_vel_guard.py) and NEVER requests a nonzero linear velocity --
this diagnostic is rotation-only. There is no CLI flag for linear
velocity at all, by design, so this script cannot request forward/
backward motion even by mistake. hil_wheel_suspension_test.py itself is
NOT modified -- it keeps its own "cannot turn" guarantee for any future
straight-line-only reuse; this is a separate, minimal, symmetric file.

Timeline (pure function `compute_angular_phase`, unit-tested
independent of ROS):
  ZERO_HOLD   [0, zero_hold_s)                    -- angular = 0.0
  PULSE       [zero_hold_s, zero_hold_s+pulse_s)  -- angular = --pulse-angular-rps (signed)
  POST_HOLD   [.., .. + post_hold_s)              -- angular = 0.0
  DONE        thereafter                           -- process publishes zero
                                                       three times and exits

Each invocation is one bounded, self-terminating run -- there is no mode
that runs indefinitely. --pulse-angular-rps accepts negative values (for
the opposite-direction pulse); the guard is expected to clamp the
magnitude to its own configured max_angular_speed_rps regardless of
sign, symmetric clamping already covered by hil_cmd_vel_guard.py's own
unit tests.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Phase:
    angular_rps: float
    name: str
    done: bool


def compute_angular_phase(
    elapsed_s: float,
    zero_hold_s: float,
    pulse_s: float,
    pulse_angular_rps: float,
    post_hold_s: float,
) -> Phase:
    if elapsed_s < 0.0:
        elapsed_s = 0.0
    if elapsed_s < zero_hold_s:
        return Phase(0.0, "ZERO_HOLD", False)
    if elapsed_s < zero_hold_s + pulse_s:
        return Phase(pulse_angular_rps, "PULSE_ROTATE", False)
    if elapsed_s < zero_hold_s + pulse_s + post_hold_s:
        return Phase(0.0, "POST_HOLD", False)
    return Phase(0.0, "DONE", True)


def _build_node():
    import argparse

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    class HilAngularSuspensionTest(Node):
        def __init__(self, args):
            super().__init__("hil_angular_suspension_test")
            self.args = args
            self.pub = self.create_publisher(Twist, args.upstream_cmd_vel_topic, 10)
            self.start_s = self._now()
            self.last_phase_name = None
            self.create_timer(1.0 / args.rate_hz, self._tick)
            self.get_logger().warn(
                "HIL_ANGULAR_SUSPENSION_TEST_START "
                f"zero_hold_s={args.zero_hold_s} pulse_s={args.pulse_s} "
                f"pulse_angular_rps={args.pulse_angular_rps} post_hold_s={args.post_hold_s} "
                "-- linear is always 0.0, no exceptions, no CLI override exists for it."
            )

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1.0e9

        def _tick(self) -> None:
            elapsed = self._now() - self.start_s
            phase = compute_angular_phase(
                elapsed, self.args.zero_hold_s, self.args.pulse_s,
                self.args.pulse_angular_rps, self.args.post_hold_s,
            )
            if phase.name != self.last_phase_name:
                self.get_logger().warn(f"HIL_ANGULAR_SUSPENSION_TEST_PHASE={phase.name} elapsed_s={elapsed:.2f}")
                self.last_phase_name = phase.name

            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = float(phase.angular_rps)
            self.pub.publish(msg)

            if phase.done:
                for _ in range(3):
                    zero = Twist()
                    self.pub.publish(zero)
                self.get_logger().warn("HIL_ANGULAR_SUSPENSION_TEST_DONE published zero 3x, exiting")
                raise SystemExit(0)

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--upstream-cmd-vel-topic", default="cmd_vel_unguarded")
        parser.add_argument("--rate-hz", type=float, default=10.0)
        parser.add_argument("--zero-hold-s", type=float, default=3.0)
        parser.add_argument("--pulse-s", type=float, default=2.0)
        parser.add_argument("--pulse-angular-rps", type=float, required=True)
        parser.add_argument("--post-hold-s", type=float, default=3.0)
        return parser.parse_args(argv)

    return rclpy, HilAngularSuspensionTest, parse_args


def main(argv=None):
    import sys

    rclpy, HilAngularSuspensionTest, parse_args = _build_node()
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[])
    node = HilAngularSuspensionTest(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
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
