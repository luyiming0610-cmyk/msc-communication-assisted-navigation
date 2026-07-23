"""Runtime isolation proof, added after the second 2026-07-23
UNEXPECTED_PHYSICAL_MOTION safety incident audit (see
experiments/07_reality_gap/hil_single_real_shared_exit_20260723/
safety_incident_unexpected_motion_2_20260723/SUMMARY.md).

Unlike test_pytest_topic_isolation.py (which only checks that every
test file's source text contains the required `-r __ns:=` remap), this
test actually spins a real CooperativeAvoider under that remap, drives
it to a genuine nonzero command, and confirms via a second, unremapped
observer node -- standing in for "what a live physical bridge would
see" -- that the command:
  (a) DOES arrive on the isolated `/pytest_isolated/cmd_vel` topic, and
  (b) NEVER arrives on the bare `cmd_vel` topic a real bridge
      subscribes to.

This proves the `-r __ns:=/pytest_isolated` remap mechanism itself
actually works, not merely that the text is present in the test files
that use it. It does not and cannot test cross-ROS_DOMAIN_ID isolation
(that would require a second live domain-0 process, which is exactly
what must never exist near a live robot) -- domain isolation is
enforced structurally instead, by conftest.py's pytest_configure hook
and run_isolated_test_suite.sh, and is covered by
test_pytest_topic_isolation.py's static check that conftest.py sets it.

Implementation note, recorded because it was a real bug caught by this
test's own first run: ROS2 remap rules from `rclpy.init(args=...)` are
stored PER-CONTEXT, not per-node -- passing `cli_args=[]` to a `Node()`
constructor does NOT clear them, since rcl still merges the context's
own global arguments in regardless. The only way to get a genuinely
unremapped observer in the same process is a second, separate
`rclpy.Context()` initialized with no args, which is what this test
does.
"""
import math
import time
import unittest

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from epuck2_comm.cooperative_avoider import CooperativeAvoider
from epuck2_comm_interfaces.msg import EpuckState

VALID_ALL = (
    EpuckState.FLAG_ODOM_VALID | EpuckState.FLAG_IR_VALID | EpuckState.FLAG_TOF_VALID
)


def _clear_state():
    msg = EpuckState()
    msg.version = EpuckState.PROTOCOL_VERSION
    msg.validity_flags = VALID_ALL
    msg.x_m = 0.0
    msg.y_m = 0.0
    msg.yaw_rad = 0.0
    msg.front_distance_m = math.inf
    msg.left_distance_m = math.inf
    msg.right_distance_m = math.inf
    return msg


class CooperativeAvoiderTopicIsolationRuntimeTest(unittest.TestCase):
    def test_nonzero_command_appears_only_on_isolated_topic_never_on_bare_cmd_vel(self):
        # CooperativeAvoider's frozen constructor takes no context
        # argument (per HIL_SAFETY_CHECKLIST.md, cooperative_avoider.py
        # must never be modified), so it necessarily uses the default
        # context -- this rclpy.init() call establishes that default,
        # with the isolation remap applied to it.
        rclpy.init(
            args=[
                "--ros-args",
                "-r", "__ns:=/pytest_isolated",
                "-p", "armed:=true",
                "-p", "enable_peer_avoidance:=false",
                "-p", "startup_hold_s:=0.0",
                "-p", "max_runtime_s:=1000.0",
            ]
        )

        # A genuinely separate, unremapped context -- stands in for
        # "what a live physical bridge subscribed to bare `cmd_vel`
        # would receive." Must be a distinct Context, not just a node
        # constructed with cli_args=[]: ROS2 remap rules from
        # rclpy.init(args=...) are stored per-context and still apply
        # to every node in that context regardless of the node's own
        # cli_args.
        observer_context = rclpy.Context()
        rclpy.init(args=[], context=observer_context)

        node = None
        observer = None
        try:
            node = CooperativeAvoider()
            observer = Node(
                "topic_isolation_observer", context=observer_context
            )
            real_cmd_vel_messages = []
            observer.create_subscription(
                Twist, "cmd_vel", lambda msg: real_cmd_vel_messages.append(msg), 10
            )
            isolated_messages = []
            observer.create_subscription(
                Twist,
                "/pytest_isolated/cmd_vel",
                lambda msg: isolated_messages.append(msg),
                10,
            )

            state_msg = _clear_state()
            for _ in range(60):
                node._own_callback(state_msg)
                node._control()
            self.assertGreater(
                node.smoother.linear_mps,
                0.0,
                "sanity: controller should be cruising at nonzero speed",
            )

            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not isolated_messages:
                rclpy.spin_once(observer, timeout_sec=0.1)

            self.assertGreater(
                len(isolated_messages),
                0,
                "expected the controller's command to arrive on /pytest_isolated/cmd_vel",
            )
            self.assertGreater(isolated_messages[-1].linear.x, 0.0)
            self.assertEqual(
                len(real_cmd_vel_messages),
                0,
                "the controller's command must NEVER appear on the bare/real cmd_vel topic",
            )
        finally:
            if node is not None:
                node.destroy_node()
            if observer is not None:
                observer.destroy_node()
            rclpy.shutdown()
            if observer_context.ok():
                rclpy.shutdown(context=observer_context)


if __name__ == "__main__":
    unittest.main()
