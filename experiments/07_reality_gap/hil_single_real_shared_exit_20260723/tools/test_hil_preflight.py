#!/usr/bin/env python3
import json
import os
import tempfile
import unittest

from hil_preflight import (
    UNCONFIRMED,
    check_namespace_conflicts,
    check_protocol_version,
    check_required_params_confirmed,
    run_offline_checks,
)


class RequiredParamsConfirmedTest(unittest.TestCase):
    def test_all_confirmed_passes(self):
        params = {
            "required_before_ground_motion": ["a.b", "c"],
            "a": {"b": 1.0},
            "c": "yes",
        }
        result = check_required_params_confirmed(params)
        self.assertTrue(result.ok)
        self.assertEqual(result.unconfirmed_paths, ())
        self.assertEqual(result.missing_paths, ())

    def test_unconfirmed_value_blocks(self):
        params = {
            "required_before_ground_motion": ["a.b"],
            "a": {"b": UNCONFIRMED},
        }
        result = check_required_params_confirmed(params)
        self.assertFalse(result.ok)
        self.assertIn("a.b", result.unconfirmed_paths)

    def test_missing_path_blocks(self):
        params = {
            "required_before_ground_motion": ["a.missing"],
            "a": {},
        }
        result = check_required_params_confirmed(params)
        self.assertFalse(result.ok)
        self.assertIn("a.missing", result.missing_paths)

    def test_empty_requirement_list_passes_trivially(self):
        result = check_required_params_confirmed({"required_before_ground_motion": []})
        self.assertTrue(result.ok)


class NamespaceConflictTest(unittest.TestCase):
    def test_distinct_namespaces_ok(self):
        result = check_namespace_conflicts("epuck5809", "epuck_virtual_peer")
        self.assertTrue(result.ok)

    def test_physical_equals_virtual_rejected(self):
        result = check_namespace_conflicts("same_ns", "same_ns")
        self.assertFalse(result.ok)

    def test_collision_with_reserved_n2_n3_namespace_rejected(self):
        result = check_namespace_conflicts("epuck1", "epuck_virtual_peer")
        self.assertFalse(result.ok)
        self.assertTrue(any("epuck1" in c for c in result.conflicts))


class ProtocolVersionTest(unittest.TestCase):
    def test_matching_version_ok(self):
        result = check_protocol_version(1, 1)
        self.assertTrue(result.ok)

    def test_mismatched_version_rejected(self):
        result = check_protocol_version(2, 1)
        self.assertFalse(result.ok)


class RunOfflineChecksTest(unittest.TestCase):
    def _write_params(self, data):
        handle = tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False, encoding="utf-8"
        )
        json.dump(data, handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        return handle.name

    def test_unconfirmed_field_yields_blocked_status(self):
        path = self._write_params({
            "frozen_protocol": {"epuck_state_protocol_version": 1},
            "required_before_ground_motion": ["x"],
            "x": UNCONFIRMED,
        })
        result = run_offline_checks(path)
        self.assertEqual(result["status"], "BLOCKED_AWAITING_LAB_MEASUREMENT")

    def test_fully_confirmed_yields_pass_status(self):
        path = self._write_params({
            "frozen_protocol": {"epuck_state_protocol_version": 1},
            "required_before_ground_motion": ["x"],
            "x": 1.23,
        })
        result = run_offline_checks(path)
        self.assertEqual(result["status"], "OFFLINE_CHECKS_PASS")


if __name__ == "__main__":
    unittest.main()
