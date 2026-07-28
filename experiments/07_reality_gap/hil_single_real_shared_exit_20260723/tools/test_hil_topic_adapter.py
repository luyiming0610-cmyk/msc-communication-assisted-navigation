#!/usr/bin/env python3
"""Dedicated unit tests for hil_topic_adapter.py.

hil_topic_adapter.py has no publishers or subscriptions of its own --
it only constructs GoalNavigator (via HilGoalAnnouncementEvidenceNavigator)
under a real-time rclpy context instead of goal_navigator.main()'s
hardcoded use_sim_time:=true. These tests therefore verify: (a) that
real-time initialization (never use_sim_time), the evidence-wrapped
class, and the shutdown path are exactly as designed, using mocked
rclpy/goal_navigator/evidence-navigator modules -- no live ROS graph,
no hardware; and (b) a structural, source-level proof that this file
cannot itself publish/subscribe to anything (so no unintended /cmd_vel
publication and no message loop is possible from this file), which
requires no imports at all.

Coverage note: GoalAnnouncement reception, adoption, and idempotence
are properties of GoalNavigator/NavigationTargetState (already tested
in test_navigation_target_state.py) and of
HilGoalAnnouncementEvidenceNavigator (see
test_hil_goal_announcement_adoption.py) -- not of this adapter file,
which never touches message content. This file does not claim to test
behaviour it does not contain.
"""
from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path
from unittest import mock


class NoPublishersOrSubscriptionsTest(unittest.TestCase):
    """Pure source-text inspection -- no imports, no ROS required."""

    def test_module_contains_no_create_publisher_or_create_subscription_call(self):
        source = Path(__file__).with_name("hil_topic_adapter.py").read_text(encoding="utf-8")
        self.assertNotIn("create_publisher", source)
        self.assertNotIn("create_subscription", source)


def _install_fake_modules():
    """Installs minimal fake `rclpy` and `goal_navigator` modules into
    sys.modules so hil_topic_adapter.main() can be exercised without a
    sourced ROS workspace or a live rclpy graph. Returns the fakes so
    tests can assert against them, plus a restore callback."""
    saved = {name: sys.modules.get(name) for name in
              ("rclpy", "rclpy.executors", "goal_navigator", "hil_goal_announcement_evidence")}

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.init_calls = []
    fake_rclpy.ok_value = True

    def _init(args=None):
        fake_rclpy.init_calls.append(args)

    def _ok():
        return fake_rclpy.ok_value

    fake_rclpy.init = _init
    fake_rclpy.ok = _ok
    fake_rclpy.shutdown = mock.Mock()

    fake_executors = types.ModuleType("rclpy.executors")

    class _ExternalShutdownException(Exception):
        pass

    fake_executors.ExternalShutdownException = _ExternalShutdownException
    fake_rclpy.executors = fake_executors

    fake_goal_navigator = types.ModuleType("goal_navigator")
    fake_goal_navigator.parse_args = mock.Mock(return_value=mock.sentinel.parsed_args)

    class _FakeGoalNavigator:
        pass

    fake_goal_navigator.GoalNavigator = _FakeGoalNavigator

    constructed_nodes = []

    class _FakeEvidenceNavigator:
        def __init__(self, args):
            self.args = args
            self.destroy_node = mock.Mock()
            constructed_nodes.append(self)

    fake_evidence_module = types.ModuleType("hil_goal_announcement_evidence")
    fake_evidence_module.build_evidence_navigator_class = mock.Mock(return_value=_FakeEvidenceNavigator)

    sys.modules["rclpy"] = fake_rclpy
    sys.modules["rclpy.executors"] = fake_executors
    sys.modules["goal_navigator"] = fake_goal_navigator
    sys.modules["hil_goal_announcement_evidence"] = fake_evidence_module

    def restore():
        for name, mod in saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod

    return fake_rclpy, fake_goal_navigator, fake_evidence_module, constructed_nodes, restore


class MainRealTimeInitializationTest(unittest.TestCase):
    def setUp(self):
        (self.fake_rclpy, self.fake_goal_navigator, self.fake_evidence_module,
         self.constructed_nodes, self._restore) = _install_fake_modules()
        sys.modules.pop("hil_topic_adapter", None)
        import hil_topic_adapter
        self.hil_topic_adapter = hil_topic_adapter

        def _spin_returns_immediately(node):
            return None

        self.fake_rclpy.spin = mock.Mock(side_effect=_spin_returns_immediately)

    def tearDown(self):
        self._restore()
        sys.modules.pop("hil_topic_adapter", None)

    def test_rclpy_init_called_with_no_sim_time_args(self):
        self.hil_topic_adapter.main(["--fake"])
        self.assertEqual(self.fake_rclpy.init_calls, [[]])

    def test_constructs_evidence_navigator_class_not_plain_goal_navigator(self):
        self.hil_topic_adapter.main(["--fake"])
        self.assertEqual(len(self.constructed_nodes), 1)
        self.assertNotIsInstance(self.constructed_nodes[0], self.fake_goal_navigator.GoalNavigator)

    def test_evidence_navigator_constructed_with_parsed_args(self):
        self.hil_topic_adapter.main(["--fake"])
        self.assertIs(self.constructed_nodes[0].args, mock.sentinel.parsed_args)

    def test_node_destroyed_and_rclpy_shutdown_on_clean_spin_return(self):
        self.hil_topic_adapter.main(["--fake"])
        self.constructed_nodes[0].destroy_node.assert_called_once()
        self.fake_rclpy.shutdown.assert_called_once()


class MainShutdownBehaviourTest(unittest.TestCase):
    def setUp(self):
        (self.fake_rclpy, self.fake_goal_navigator, self.fake_evidence_module,
         self.constructed_nodes, self._restore) = _install_fake_modules()
        sys.modules.pop("hil_topic_adapter", None)
        import hil_topic_adapter
        self.hil_topic_adapter = hil_topic_adapter

    def tearDown(self):
        self._restore()
        sys.modules.pop("hil_topic_adapter", None)

    def test_keyboard_interrupt_during_spin_is_swallowed_and_still_shuts_down(self):
        self.fake_rclpy.spin = mock.Mock(side_effect=KeyboardInterrupt)
        self.hil_topic_adapter.main(["--fake"])  # must not raise
        self.constructed_nodes[0].destroy_node.assert_called_once()
        self.fake_rclpy.shutdown.assert_called_once()

    def test_external_shutdown_exception_during_spin_is_swallowed_and_still_shuts_down(self):
        exc = self.fake_rclpy.executors.ExternalShutdownException
        self.fake_rclpy.spin = mock.Mock(side_effect=exc)
        self.hil_topic_adapter.main(["--fake"])  # must not raise
        self.constructed_nodes[0].destroy_node.assert_called_once()
        self.fake_rclpy.shutdown.assert_called_once()

    def test_other_exception_during_spin_still_destroys_node_before_propagating(self):
        self.fake_rclpy.spin = mock.Mock(side_effect=RuntimeError("unexpected"))
        with self.assertRaises(RuntimeError):
            self.hil_topic_adapter.main(["--fake"])
        self.constructed_nodes[0].destroy_node.assert_called_once()

    def test_rclpy_not_ok_after_spin_skips_shutdown_call(self):
        self.fake_rclpy.ok_value = False
        self.fake_rclpy.spin = mock.Mock(return_value=None)
        self.hil_topic_adapter.main(["--fake"])
        self.fake_rclpy.shutdown.assert_not_called()


class ImportGoalNavigatorSysPathTest(unittest.TestCase):
    def setUp(self):
        (self.fake_rclpy, self.fake_goal_navigator, self.fake_evidence_module,
         self.constructed_nodes, self._restore) = _install_fake_modules()
        sys.modules.pop("hil_topic_adapter", None)
        import hil_topic_adapter
        self.hil_topic_adapter = hil_topic_adapter
        # This module (and others imported earlier in the same test run)
        # may have already inserted _TOOLS_DIR into sys.path -- strip
        # every occurrence so each test starts from a known-clean state
        # rather than asserting against however many prior tests left
        # behind.
        while self.hil_topic_adapter._TOOLS_DIR in sys.path:
            sys.path.remove(self.hil_topic_adapter._TOOLS_DIR)

    def tearDown(self):
        while self.hil_topic_adapter._TOOLS_DIR in sys.path:
            sys.path.remove(self.hil_topic_adapter._TOOLS_DIR)
        self._restore()
        sys.modules.pop("hil_topic_adapter", None)

    def test_sibling_tools_dir_is_the_expected_cooperative_exit_navigation_path(self):
        expected_suffix = str(Path("10_cooperative_exit_navigation_20260720") / "tools")
        self.assertTrue(self.hil_topic_adapter._TOOLS_DIR.endswith(expected_suffix))

    def test_import_goal_navigator_inserts_sibling_dir_into_syspath_once(self):
        self.assertEqual(sys.path.count(self.hil_topic_adapter._TOOLS_DIR), 0)
        self.hil_topic_adapter._import_goal_navigator()
        self.hil_topic_adapter._import_goal_navigator()
        self.assertEqual(sys.path.count(self.hil_topic_adapter._TOOLS_DIR), 1)


if __name__ == "__main__":
    unittest.main()
