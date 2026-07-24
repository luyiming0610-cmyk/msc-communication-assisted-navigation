#!/usr/bin/env python3
"""Tests for hil_ground_diagnostic_session.py -- the gitignored,
timestamped, per-session confirmation file for the four confirmations
that can genuinely change between sessions: floor condition, travel
path clear, operator presence, and Wi-Fi checked. No ROS/rclpy
dependency; no filesystem paths outside a temporary directory are ever
touched.
"""
import json
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path

from hil_ground_diagnostic_session import (
    REQUIRED_SESSION_FIELDS,
    TIMESTAMP_KEY,
    check_session_state_ready,
    fresh_session_state,
    load_session_state,
    save_session_state,
)

MODULE_PATH = Path(__file__).parent / "hil_ground_diagnostic_session.py"


class FreshSessionStateTest(unittest.TestCase):
    def test_required_session_fields_are_exactly_the_expected_four(self):
        self.assertEqual(
            set(REQUIRED_SESSION_FIELDS),
            {
                "floor_condition_confirmed",
                "travel_path_clear_confirmed",
                "operator_present_confirmed",
                "wifi_checked_in_test_area",
            },
        )
        self.assertEqual(len(REQUIRED_SESSION_FIELDS), 4)

    def test_fresh_state_has_timestamp_and_all_four_fields_false(self):
        state = fresh_session_state()
        self.assertIn(TIMESTAMP_KEY, state)
        for name in REQUIRED_SESSION_FIELDS:
            self.assertIs(state[name], False)

    def test_fresh_state_timestamp_is_parseable_and_recent(self):
        state = fresh_session_state()
        ts = datetime.fromisoformat(state[TIMESTAMP_KEY])
        age_s = (datetime.now(timezone.utc) - ts).total_seconds()
        self.assertGreaterEqual(age_s, 0)
        self.assertLess(age_s, 5)


class CheckSessionStateReadyTest(unittest.TestCase):
    def test_missing_session_blocks(self):
        result = check_session_state_ready(None)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "SESSION_FILE_MISSING")

    def test_fresh_session_with_all_false_blocks(self):
        result = check_session_state_ready(fresh_session_state())
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "CONFIRMATIONS_NOT_TRUE")
        self.assertEqual(set(result.false_fields), set(REQUIRED_SESSION_FIELDS))

    def test_each_field_blocks_independently_when_it_alone_is_false(self):
        # One at a time: every other field true, only this one false --
        # confirm it alone is enough to block, for every one of the
        # four fields (not just one hardcoded example).
        for target in REQUIRED_SESSION_FIELDS:
            state = fresh_session_state()
            for name in REQUIRED_SESSION_FIELDS:
                if name != target:
                    state[name] = True
            result = check_session_state_ready(state)
            self.assertFalse(result.ok, f"expected block with only {target} false")
            self.assertEqual(result.false_fields, (target,))

    def test_each_field_missing_entirely_blocks_independently(self):
        # A field absent from the dict (not merely False) must be
        # treated identically to False -- this is also what makes the
        # old two-field session format safe (see
        # OldTwoFieldSessionFormatTest below).
        for target in REQUIRED_SESSION_FIELDS:
            state = fresh_session_state()
            for name in REQUIRED_SESSION_FIELDS:
                if name != target:
                    state[name] = True
            del state[target]
            result = check_session_state_ready(state)
            self.assertFalse(result.ok, f"expected block with {target} missing")
            self.assertEqual(result.false_fields, (target,))

    def test_all_fields_true_and_fresh_passes(self):
        state = fresh_session_state()
        for name in REQUIRED_SESSION_FIELDS:
            state[name] = True
        result = check_session_state_ready(state)
        self.assertTrue(result.ok)
        self.assertEqual(result.false_fields, ())

    def test_missing_timestamp_blocks_even_if_fields_true(self):
        state = {name: True for name in REQUIRED_SESSION_FIELDS}
        result = check_session_state_ready(state)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "MISSING_TIMESTAMP")

    def test_unparseable_timestamp_blocks(self):
        state = fresh_session_state()
        for name in REQUIRED_SESSION_FIELDS:
            state[name] = True
        state[TIMESTAMP_KEY] = "not-a-timestamp"
        result = check_session_state_ready(state)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "UNPARSEABLE_TIMESTAMP")

    def test_future_timestamp_blocks(self):
        state = fresh_session_state()
        for name in REQUIRED_SESSION_FIELDS:
            state[name] = True
        future = datetime.now(timezone.utc) + timedelta(hours=1)
        state[TIMESTAMP_KEY] = future.isoformat()
        result = check_session_state_ready(state)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "TIMESTAMP_IN_FUTURE")

    def test_stale_session_blocks_even_if_fields_true(self):
        # A session file from an earlier run, all fields true, must not
        # be silently reused once it is older than max_age_s -- this is
        # the core requirement: a stale file can never be silently
        # treated as still valid.
        state = fresh_session_state()
        for name in REQUIRED_SESSION_FIELDS:
            state[name] = True
        old = datetime.now(timezone.utc) - timedelta(hours=10)
        state[TIMESTAMP_KEY] = old.isoformat()
        result = check_session_state_ready(state, max_age_s=4 * 3600.0)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "SESSION_STALE")
        self.assertGreater(result.age_s, 4 * 3600.0)

    def test_session_just_under_max_age_passes(self):
        state = fresh_session_state()
        for name in REQUIRED_SESSION_FIELDS:
            state[name] = True
        almost_stale = datetime.now(timezone.utc) - timedelta(hours=3, minutes=59)
        state[TIMESTAMP_KEY] = almost_stale.isoformat()
        result = check_session_state_ready(state, max_age_s=4 * 3600.0)
        self.assertTrue(result.ok)


class OldTwoFieldSessionFormatTest(unittest.TestCase):
    """A session file written by the previous two-field version of this
    module (operator_present_confirmed / wifi_checked_in_test_area
    only, both true, no floor/travel-path keys at all) must block
    safely, never silently pass just because the two fields it does
    have are both true."""

    def _old_format_state(self, *, fresh=True):
        state = {
            "_comment": "old format",
            TIMESTAMP_KEY: datetime.now(timezone.utc).isoformat()
            if fresh
            else (datetime.now(timezone.utc) - timedelta(hours=10)).isoformat(),
            "operator_present_confirmed": True,
            "wifi_checked_in_test_area": True,
        }
        return state

    def test_old_format_with_both_legacy_fields_true_still_blocks(self):
        state = self._old_format_state()
        result = check_session_state_ready(state)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "CONFIRMATIONS_NOT_TRUE")
        self.assertEqual(
            set(result.false_fields), {"floor_condition_confirmed", "travel_path_clear_confirmed"}
        )

    def test_old_format_via_cli_check_also_blocks(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "session.json")
            save_session_state(path, self._old_format_state())
            result = subprocess.run(
                [sys.executable, str(MODULE_PATH), "check", "--path", path],
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("SESSION_STATE_CHECK=BLOCKED", result.stdout)
            self.assertIn("floor_condition_confirmed", result.stdout)
            self.assertIn("travel_path_clear_confirmed", result.stdout)


class TrackedValuesCannotBypassSessionGateTest(unittest.TestCase):
    """A tracked ground_diagnostic_params.json value can never
    substitute for a session-file confirmation -- check_session_state_ready
    only ever reads the session dict passed to it, never the tracked
    params file, so there is no code path by which a tracked-file
    "true" could satisfy the session gate."""

    def test_check_session_state_ready_has_no_tracked_params_parameter(self):
        import inspect

        sig = inspect.signature(check_session_state_ready)
        self.assertNotIn("params", sig.parameters)
        self.assertNotIn("tracked", str(sig))

    def test_a_session_dict_with_extra_tracked_looking_keys_still_requires_its_own_fields_true(self):
        # Even if a session dict happened to carry extra keys shaped
        # like tracked-file content, the four REQUIRED_SESSION_FIELDS
        # must still each be independently true -- extra keys are inert.
        state = fresh_session_state()
        state["measured_geometry"] = {"start_x_m": 0.25}
        state["environment"] = {"boundaries_and_obstacles_recorded": True}
        result = check_session_state_ready(state)
        self.assertFalse(result.ok)
        self.assertEqual(set(result.false_fields), set(REQUIRED_SESSION_FIELDS))


class SaveLoadRoundTripTest(unittest.TestCase):
    def test_round_trip_preserves_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "session.json")
            state = fresh_session_state()
            save_session_state(path, state)
            loaded = load_session_state(path)
            self.assertEqual(loaded[TIMESTAMP_KEY], state[TIMESTAMP_KEY])
            for name in REQUIRED_SESSION_FIELDS:
                self.assertEqual(loaded[name], state[name])

    def test_load_missing_file_returns_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "does_not_exist.json")
            self.assertIsNone(load_session_state(path))


class CliTest(unittest.TestCase):
    """Exercises the thin CLI wrapper as a subprocess -- no rclpy
    dependency, no network, no filesystem paths outside a temp dir."""

    def _run(self, *args):
        return subprocess.run(
            [sys.executable, str(MODULE_PATH), *args],
            capture_output=True,
            text=True,
        )

    def test_init_creates_fresh_file_with_all_four_false(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "session.json")
            result = self._run("init", "--path", path)
            self.assertEqual(result.returncode, 0)
            self.assertIn("SESSION_STATE_INITIALIZED", result.stdout)
            with open(path, encoding="utf-8") as fh:
                state = json.load(fh)
            for name in REQUIRED_SESSION_FIELDS:
                self.assertIs(state[name], False)

    def test_check_blocks_before_any_set(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "session.json")
            self._run("init", "--path", path)
            result = self._run("check", "--path", path)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("SESSION_STATE_CHECK=BLOCKED", result.stdout)

    def test_check_passes_after_all_four_set(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "session.json")
            self._run("init", "--path", path)
            for name in REQUIRED_SESSION_FIELDS:
                set_result = self._run("set", "--path", path, "--field", name)
                self.assertEqual(set_result.returncode, 0)
            result = self._run("check", "--path", path)
            self.assertEqual(result.returncode, 0)
            self.assertIn("SESSION_STATE_CHECK=PASS", result.stdout)

    def test_set_without_init_is_blocked(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "does_not_exist.json")
            result = self._run("set", "--path", path, "--field", "operator_present_confirmed")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("SESSION_FILE_MISSING", result.stdout)

    def test_set_on_stale_session_is_blocked_and_never_silently_extends_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "session.json")
            state = fresh_session_state()
            old = datetime.now(timezone.utc) - timedelta(hours=10)
            state[TIMESTAMP_KEY] = old.isoformat()
            save_session_state(path, state)
            result = self._run("set", "--path", path, "--field", "operator_present_confirmed")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("SESSION_STALE", result.stdout)
            with open(path, encoding="utf-8") as fh:
                unchanged = json.load(fh)
            self.assertIs(unchanged["operator_present_confirmed"], False)


if __name__ == "__main__":
    unittest.main()
