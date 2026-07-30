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

import importlib.abc
import importlib.util
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

# Resolved independently of the fix under test, purely for building
# fake-module fixtures below: this test file lives at the same depth as
# hil_topic_adapter.py, so parents[3] is already `experiments/`.
_REAL_GOAL_NAVIGATOR_FILE = str(
    Path(__file__).resolve().parents[3] / "10_cooperative_exit_navigation_20260720" / "tools" / "goal_navigator.py"
)


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
    # Must resolve to the real committed goal_navigator.py so the fix's
    # cached-module identity check (never bypassed here) accepts this
    # fake as legitimate -- a fake with no __file__, or the wrong one,
    # is exactly what the dedicated identity tests below cover instead.
    fake_goal_navigator.__file__ = _REAL_GOAL_NAVIGATOR_FILE
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


class ImportGoalNavigatorRealPathResolutionTest(unittest.TestCase):
    """Exercises the REAL, unmocked `_import_goal_navigator()` path
    resolution and identity-verification logic against synthetic
    filesystem trees mirroring the real relative depth -- never bypasses
    it via a pre-installed fake sys.modules entry (the exact reason the
    original three-dirname-call defect went uncaught by every other test
    in this file, all of which use `_install_fake_modules()`)."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._saved_modules = {
            name: sys.modules.pop(name, None) for name in ("hil_topic_adapter", "goal_navigator")
        }
        self._saved_sys_path = list(sys.path)
        self._saved_meta_path = list(sys.meta_path)

    def tearDown(self):
        # Restore the exact prior list, never a length-based tail slice:
        # the meta_path override test INSERTS at index 0, so a
        # length-based `del sys.meta_path[saved_len:]` would delete the
        # wrong (real) finder from the tail instead of the one actually
        # added at the front, permanently breaking every later dynamic
        # import in the same test process.
        sys.meta_path[:] = self._saved_meta_path
        sys.path[:] = self._saved_sys_path
        for name, mod in self._saved_modules.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod
        self._tmp.cleanup()

    def _build_adapter_module(self, root: Path, depth_ok: bool = True) -> Path:
        if depth_ok:
            adapter_dir = root / "experiments" / "07_reality_gap" / "hil_single_real_shared_exit_20260723" / "tools"
        else:
            adapter_dir = root / "shallow"
        adapter_dir.mkdir(parents=True, exist_ok=True)
        source = Path(__file__).with_name("hil_topic_adapter.py").read_text(encoding="utf-8")
        (adapter_dir / "hil_topic_adapter.py").write_text(source, encoding="utf-8")
        return adapter_dir

    def _build_goal_navigator(self, root: Path, content: str = 'GoalNavigator_marker = "real"\n') -> Path:
        gn_dir = root / "experiments" / "10_cooperative_exit_navigation_20260720" / "tools"
        gn_dir.mkdir(parents=True, exist_ok=True)
        (gn_dir / "goal_navigator.py").write_text(content, encoding="utf-8")
        return gn_dir

    def _import_fresh(self, adapter_dir: Path):
        sys.path.insert(0, str(adapter_dir))
        sys.modules.pop("hil_topic_adapter", None)
        # Defensive against stale FileFinder entries left in
        # sys.path_importer_cache by earlier tests' now-deleted temp
        # directories -- a long-lived pytest process can otherwise
        # occasionally miss a brand-new directory's contents.
        sys.path_importer_cache.clear()
        importlib.invalidate_caches()
        import hil_topic_adapter
        return hil_topic_adapter

    def test_correct_real_layout_resolution(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        self._build_goal_navigator(root)
        mod = self._import_fresh(adapter_dir)
        gn = mod._import_goal_navigator()
        self.assertEqual(gn.GoalNavigator_marker, "real")

    def test_arbitrary_cwd(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        self._build_goal_navigator(root)
        mod = self._import_fresh(adapter_dir)
        old_cwd = os.getcwd()
        try:
            os.chdir(tempfile.gettempdir())
            gn = mod._import_goal_navigator()
            self.assertEqual(gn.GoalNavigator_marker, "real")
        finally:
            os.chdir(old_cwd)

    def test_repo_path_with_spaces_and_unicode(self):
        special_root = Path(self._tmp.name) / "repo with space and 中文"
        special_root.mkdir(parents=True, exist_ok=True)
        adapter_dir = self._build_adapter_module(special_root)
        self._build_goal_navigator(special_root)
        mod = self._import_fresh(adapter_dir)
        gn = mod._import_goal_navigator()
        self.assertEqual(gn.GoalNavigator_marker, "real")

    def test_unexpectedly_shallow_layout_raises_import_error_not_index_error(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root, depth_ok=False)
        mod = self._import_fresh(adapter_dir)
        with self.assertRaises(mod.GoalNavigatorImportError) as ctx:
            mod._import_goal_navigator()
        self.assertIsInstance(ctx.exception.__cause__, IndexError)

    def test_missing_tools_directory_raises(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        mod = self._import_fresh(adapter_dir)
        with self.assertRaises(mod.GoalNavigatorImportError):
            mod._import_goal_navigator()

    def test_missing_goal_navigator_file_raises(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        (root / "experiments" / "10_cooperative_exit_navigation_20260720" / "tools").mkdir(parents=True, exist_ok=True)
        mod = self._import_fresh(adapter_dir)
        with self.assertRaises(mod.GoalNavigatorImportError):
            mod._import_goal_navigator()

    def test_correct_cached_module_is_reused(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        gn_dir = self._build_goal_navigator(root)
        mod = self._import_fresh(adapter_dir)
        sys.path.insert(0, str(gn_dir))
        import goal_navigator
        result = mod._import_goal_navigator()
        self.assertIs(result, goal_navigator)

    def test_cached_module_without_file_raises(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        self._build_goal_navigator(root)
        mod = self._import_fresh(adapter_dir)
        sys.modules["goal_navigator"] = types.ModuleType("goal_navigator")
        with self.assertRaises(mod.GoalNavigatorImportError):
            mod._import_goal_navigator()

    def test_cached_module_wrong_path_raises(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        self._build_goal_navigator(root)
        mod = self._import_fresh(adapter_dir)
        fake = types.ModuleType("goal_navigator")
        fake.__file__ = "/some/other/wrong/location/goal_navigator.py"
        sys.modules["goal_navigator"] = fake
        with self.assertRaises(mod.GoalNavigatorImportError):
            mod._import_goal_navigator()

    def test_wrong_module_earlier_on_sys_path_is_still_corrected(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        gn_dir = self._build_goal_navigator(root)
        decoy_dir = root / "decoy"
        decoy_dir.mkdir()
        (decoy_dir / "goal_navigator.py").write_text('GoalNavigator_marker = "DECOY"\n', encoding="utf-8")
        mod = self._import_fresh(adapter_dir)
        # Correct dir already present but BEHIND a decoy -- the fix must
        # move it to the front, never merely skip re-insertion.
        sys.path.insert(0, str(decoy_dir))
        sys.path.append(str(gn_dir))
        gn = mod._import_goal_navigator()
        self.assertEqual(gn.GoalNavigator_marker, "real")

    def test_partial_failed_import_cleanup_restores_sys_path_and_sys_modules(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        self._build_goal_navigator(root, content="raise RuntimeError('boom')\n")
        mod = self._import_fresh(adapter_dir)
        path_before = list(sys.path)
        with self.assertRaises(mod.GoalNavigatorImportError) as ctx:
            mod._import_goal_navigator()
        self.assertIsInstance(ctx.exception.__cause__, RuntimeError)
        self.assertNotIn("goal_navigator", sys.modules)
        self.assertEqual(sys.path, path_before)

    def test_correct_imported_module(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        self._build_goal_navigator(root)
        mod = self._import_fresh(adapter_dir)
        gn = mod._import_goal_navigator()
        expected = (root / "experiments" / "10_cooperative_exit_navigation_20260720" / "tools" / "goal_navigator.py").resolve()
        self.assertEqual(Path(gn.__file__).resolve(), expected)

    def test_wrong_imported_module_file_via_meta_path_override_raises(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        self._build_goal_navigator(root)
        decoy_dir = root / "decoy2"
        decoy_dir.mkdir()
        decoy_file = decoy_dir / "goal_navigator.py"
        decoy_file.write_text('GoalNavigator_marker = "DECOY"\n', encoding="utf-8")
        mod = self._import_fresh(adapter_dir)

        class _DecoyFinder(importlib.abc.MetaPathFinder):
            def find_spec(self, name, path, target=None):
                if name != "goal_navigator":
                    return None
                return importlib.util.spec_from_file_location(name, str(decoy_file))

        sys.meta_path.insert(0, _DecoyFinder())
        with self.assertRaises(mod.GoalNavigatorImportError):
            mod._import_goal_navigator()

    def test_failure_raised_before_rclpy_import_or_node_construction(self):
        root = Path(self._tmp.name)
        adapter_dir = self._build_adapter_module(root)
        mod = self._import_fresh(adapter_dir)
        with self.assertRaises(mod.GoalNavigatorImportError):
            mod._import_goal_navigator()
        self.assertNotIn("rclpy", sys.modules.get("hil_topic_adapter", mod).__dict__)


class BothGoalNavigatorConsumersUseTheSameCorrectedImportBehaviourTest(unittest.TestCase):
    """hil_topic_adapter.py and hil_goal_announcement_evidence.py each
    define their own copy of `_import_goal_navigator()` (deliberately not
    consolidated into a shared module, per the minimal-fix scope) -- this
    proves the two copies remain byte-for-byte identical in their
    resolution/identity logic, so a future edit to only one cannot
    silently reintroduce a divergence between the two Stage 3 GoalNavigator
    consumers."""

    def test_import_goal_navigator_bodies_are_identical(self):
        adapter_source = Path(__file__).with_name("hil_topic_adapter.py").read_text(encoding="utf-8")
        evidence_source = Path(__file__).with_name("hil_goal_announcement_evidence.py").read_text(encoding="utf-8")

        def _extract(source: str) -> str:
            start = source.index("_THIS_FILE = Path(__file__).resolve()")
            end = source.index("\ndef _import_goal_navigator()") + len("\ndef _import_goal_navigator()")
            body_start = source.index("def _import_goal_navigator()")
            body_end = source.index("\n\n\n", body_start)
            return source[start:end] + source[body_start:body_end]

        self.assertEqual(_extract(adapter_source), _extract(evidence_source))


if __name__ == "__main__":
    unittest.main()
