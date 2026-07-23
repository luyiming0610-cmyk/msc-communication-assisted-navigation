#!/usr/bin/env python3
"""Regression tests locking down the real HIL topic contract and the
required validity_flags value, as documented in HIL_TOPIC_CONTRACT.md
and HIL_SAFETY_CHECKLIST.md.

Unlike test_hil_cmd_vel_guard.py (which deliberately avoids importing
rclpy so it can run outside a sourced ROS workspace), this file
exercises hil_cmd_vel_guard.py's actual CLI parser defaults via
_build_node(), and therefore requires rclpy/the ROS message packages
to be importable -- run inside a sourced ROS 2 environment, same as
every other live check this session.

These assertions exist specifically to catch a repeat of the
inconsistency found 2026-07-23: HIL_TOPIC_CONTRACT.md and
HIL_SAFETY_CHECKLIST.md had drifted to describe placeholder
/epuck5809/... topic names and an ODOM-only validity requirement that
no longer matched the real, live-confirmed topic names
(/epuck1/state, un-namespaced /cmd_vel_unguarded and /cmd_vel) or the
guard's actual hardened default (FLAG_ODOM_VALID | FLAG_IR_VALID |
FLAG_TOF_VALID, value 7).
"""
import unittest

from epuck2_comm_interfaces.msg import EpuckState

from hil_cmd_vel_guard import _build_node


class HilCmdVelGuardCliDefaultsTest(unittest.TestCase):
    def setUp(self):
        _, _, _, parse_args = _build_node()
        self.parse_args = parse_args

    def test_upstream_cmd_vel_topic_default_is_unnamespaced_cmd_vel_unguarded(self):
        args = self.parse_args(["--physical-state-topic", "/epuck1/state"])
        self.assertEqual(args.upstream_cmd_vel_topic, "cmd_vel_unguarded")

    def test_guarded_cmd_vel_topic_default_is_unnamespaced_cmd_vel(self):
        args = self.parse_args(["--physical-state-topic", "/epuck1/state"])
        self.assertEqual(args.guarded_cmd_vel_topic, "cmd_vel")

    def test_physical_state_topic_has_no_default_and_must_be_supplied_explicitly(self):
        # Confirms there is no live "/epuck5809/state"-style default to
        # drift out of sync with the real topic -- the argument is
        # required, so every invocation must name the real topic itself.
        with self.assertRaises(SystemExit):
            self.parse_args([])

    def test_required_validity_flags_default_is_odom_ir_tof_value_7(self):
        args = self.parse_args(["--physical-state-topic", "/epuck1/state"])
        expected = EpuckState.FLAG_ODOM_VALID | EpuckState.FLAG_IR_VALID | EpuckState.FLAG_TOF_VALID
        self.assertEqual(expected, 7)
        self.assertEqual(args.required_validity_flags, 7)
        self.assertEqual(args.required_validity_flags, expected)


if __name__ == "__main__":
    unittest.main()
