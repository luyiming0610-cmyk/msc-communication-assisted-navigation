#!/usr/bin/env python3
"""Test-only fake EpuckState publisher, used exclusively by the offline
end-to-end integration test (test_hil_integration_offline.sh). Publishes
on a caller-specified TEST-namespaced topic only -- never /epuck1/state
-- so the integration test can exercise hil_cmd_vel_guard.py's real
freshness/validity logic without any real or virtual robot involved.

Not part of the permanent HIL runtime graph; not started by
run_hil_shared_exit_trial.sh.
"""
from __future__ import annotations


def _build_node():
    import argparse

    import rclpy
    from rclpy.node import Node

    from epuck2_comm_interfaces.msg import EpuckState

    class FakeStatePublisher(Node):
        def __init__(self, args):
            super().__init__("hil_integration_test_fake_state_publisher")
            self.args = args
            self.seq = 0
            self.pub = self.create_publisher(EpuckState, args.state_topic, 10)
            self.create_timer(1.0 / args.rate_hz, self._tick)

        def _tick(self) -> None:
            self.seq += 1
            msg = EpuckState()
            msg.version = EpuckState.PROTOCOL_VERSION if self.args.valid_protocol else 99
            msg.robot_id = 1
            msg.sequence = self.seq % (2 ** 32)
            msg.stamp = self.get_clock().now().to_msg()
            msg.source = EpuckState.SOURCE_HARDWARE
            msg.validity_flags = self.args.validity_flags
            self.pub.publish(msg)

    def parse_args(argv):
        parser = argparse.ArgumentParser()
        parser.add_argument("--state-topic", required=True)
        parser.add_argument("--rate-hz", type=float, default=10.0)
        parser.add_argument("--validity-flags", type=int, default=7)
        parser.add_argument("--valid-protocol", action="store_true", default=True)
        parser.add_argument("--invalid-protocol", dest="valid_protocol", action="store_false")
        return parser.parse_args(argv)

    return rclpy, FakeStatePublisher, parse_args


def main(argv=None):
    import sys

    rclpy, FakeStatePublisher, parse_args = _build_node()
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[])
    node = FakeStatePublisher(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
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
