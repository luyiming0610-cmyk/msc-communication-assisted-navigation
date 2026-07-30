#!/usr/bin/env python3
"""Regression guard for run_isolated_test_suite.sh's isolated-domain and
ROS_LOCALHOST_ONLY contract, added 2026-07-30 after a differential audit
(parent HEAD 3f1e59fecddaf568445d196bcb89841b46f6571f vs. current
working tree) proved 25+2 pre-existing test failures were caused by
TWO independent runner defects, not a regression from Stage 4 work:

1. TEST_ROS_DOMAIN_ID was 89, which collides with
   test_hil_offline_stage3_harness_live.py's own
   FORBIDDEN_ROS_DOMAIN_IDS = frozenset({0, 89, 77}) safety guard (and
   that file separately forbids 91, reserved for the real Stage 3 run).
2. The runner never actually exported ROS_LOCALHOST_ONLY=1, which that
   same test file's setUp() requires exactly.
3. (Found while fixing the above) HIL_EXIT/PI_EXIT/E2E_EXIT/etc. were
   each captured via `$?` AFTER an intervening `echo` statement, so
   `$?` always reflected the echo's own (always-zero) exit status, not
   the actual test suite's -- meaning a real test failure could never
   have been detected by this runner's own FAIL bookkeeping.

This file is a static source guard (never runs live tests itself) so a
future edit cannot silently reintroduce any of the three."""
from __future__ import annotations

import re
import unittest
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parent / "run_isolated_test_suite.sh"
FORBIDDEN_DOMAINS = {0, 89, 77, 91}


class IsolatedDomainAndLocalhostOnlyContractTest(unittest.TestCase):
    def setUp(self):
        self.source = SCRIPT_PATH.read_text(encoding="utf-8")

    def test_test_ros_domain_id_is_not_forbidden(self):
        match = re.search(r"^TEST_ROS_DOMAIN_ID=(\d+)", self.source, re.MULTILINE)
        self.assertIsNotNone(match, "TEST_ROS_DOMAIN_ID assignment not found")
        domain = int(match.group(1))
        self.assertNotIn(
            domain, FORBIDDEN_DOMAINS,
            f"TEST_ROS_DOMAIN_ID={domain} collides with a known-forbidden/reserved domain {FORBIDDEN_DOMAINS}",
        )

    def test_ros_localhost_only_exported_before_hil_suite_runs(self):
        export_idx = self.source.index("export ROS_LOCALHOST_ONLY=1")
        hil_suite_idx = self.source.index("HIL unit test suite")
        self.assertLess(export_idx, hil_suite_idx)

    def test_assert_isolated_domain_checks_localhost_only(self):
        func_start = self.source.index("_assert_isolated_domain() {")
        func_end = self.source.index("\n}\n", func_start)
        func_body = self.source[func_start:func_end]
        self.assertIn("ROS_LOCALHOST_ONLY", func_body)

    def test_ros_localhost_only_is_scoped_not_global(self):
        # Must appear only inside this test-only runner's isolated-domain
        # block, never in any of the physical/production scripts it
        # invokes (this runner never modifies those files, so this test
        # simply confirms none of them mention the variable, i.e. no
        # global leakage was introduced alongside this fix).
        production_scripts = (
            "run_hil_shared_exit_trial.sh", "run_hil_stage4_trial.sh",
            "run_hil_preflight.sh", "run_hil_shutdown.sh",
        )
        tools_dir = SCRIPT_PATH.parent
        for name in production_scripts:
            path = tools_dir / name
            if path.is_file():
                self.assertNotIn(
                    "ROS_LOCALHOST_ONLY", path.read_text(encoding="utf-8"),
                    f"{name} must not set ROS_LOCALHOST_ONLY -- that belongs only to this test runner",
                )

    def test_exit_code_captured_before_echo_for_each_gated_suite(self):
        """Regression guard for the `$?`-after-`echo` bug: for every
        <NAME>_EXIT=$? assignment, the immediately preceding non-blank
        line must be the corresponding output-producing command
        substitution, not an echo of its own output."""
        lines = self.source.splitlines()
        for i, line in enumerate(lines):
            m = re.match(r"^(\w+_EXIT)=\$\?\s*$", line.strip())
            if not m:
                continue
            # Walk backward to the nearest non-blank line.
            j = i - 1
            while j >= 0 and not lines[j].strip():
                j -= 1
            self.assertGreaterEqual(j, 0, f"no preceding command found for {m.group(1)}")
            preceding = lines[j].strip()
            self.assertFalse(
                preceding.startswith("echo "),
                f"{m.group(1)}=$? must be captured immediately after its command, "
                f"before any echo -- found 'echo' immediately before it instead: {preceding!r}",
            )


if __name__ == "__main__":
    unittest.main()
