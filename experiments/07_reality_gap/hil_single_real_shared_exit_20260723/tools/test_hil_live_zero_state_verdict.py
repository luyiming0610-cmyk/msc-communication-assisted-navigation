#!/usr/bin/env python3
"""End-to-end regression test for hil_live_zero_state_verdict.py --
invokes it as a real subprocess with a realistic, full CLI argument
list, exercising the actual argv plumbing that
run_ground_diagnostic_preflight.sh depends on. This is the missing
coverage that let the previous inline heredoc's off-by-one
`sys.argv[1:20]` slice bug (found 2026-07-24 during a live attempt,
run 20260724_172233) go undetected -- every existing test called
evaluate_wsl_live_state()/evaluate_combined_gate() directly in Python,
never through a real subprocess with a real argument list. No
ROS/rclpy dependency; no process other than this CLI is started.
"""
import subprocess
import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path

SCRIPT_PATH = Path(__file__).parent / "hil_live_zero_state_verdict.py"


def _fresh_good_args():
    # Generated at call time (not a fixed literal) -- a hardcoded past
    # timestamp would eventually exceed the default 300s max age and
    # start failing this "all good" fixture for an unrelated reason.
    return [
        "--validity-flags", "7",
        "--bridge-connected", "true",
        "--wsl-csv-growing", "true",
        "--guard-sole-publisher", "true",
        "--guard-armed", "false",
        "--cmd-vel-all-zero", "true",
        "--upstream-zero-or-absent", "true",
        "--forbidden-process-found", "false",
        "--wsl-evidence-all-zero", "true",
        "--pi-verdict-available", "true",
        "--pi-verdict-malformed", "false",
        "--pi-verdict-ok", "true",
        "--pi-verdict-reasons", "[]",
        "--pi-run-id", "20260724_172233",
        "--pi-verdict-generated-at", datetime.now(timezone.utc).isoformat(),
        "--pi-jsonl-path", "/home/pi/real_robot_avoidance_v1/command_audit_20260724_172233.jsonl",
        "--wsl-run-id", "20260724_172233",
        "--wsl-evidence-path", "/home/eamon/epuck_comm_bags/first_ground_diagnostic_20260724_172233/command_evidence.csv",
        "--expected-pi-jsonl-path", "/home/pi/real_robot_avoidance_v1/command_audit_20260724_172233.jsonl",
        "--pi-verdict-max-age-s", "300",
    ]


ALL_GOOD_ARGS = _fresh_good_args()


def _run(args):
    return subprocess.run(
        [sys.executable, str(SCRIPT_PATH), *args],
        capture_output=True,
        text=True,
        cwd=str(SCRIPT_PATH.parent),
    )


class LiveZeroStateVerdictCliTest(unittest.TestCase):
    def test_full_realistic_argument_list_does_not_crash(self):
        # This is exactly the failure mode found live: the previous
        # heredoc raised "ValueError: not enough values to unpack"
        # given this same 20-argument shape. A crash here (non-empty
        # stderr, or a traceback) reproduces that bug.
        result = _run(ALL_GOOD_ARGS)
        self.assertNotIn("Traceback", result.stdout + result.stderr)
        self.assertNotIn("not enough values to unpack", result.stdout + result.stderr)

    def test_all_good_passes(self):
        result = _run(ALL_GOOD_ARGS)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("LIVE_ZERO_STATE_VERDICT=PASS", result.stdout)

    def test_pi_evidence_missing_blocks_without_crashing(self):
        args = list(ALL_GOOD_ARGS)
        idx = args.index("--pi-verdict-available")
        args[idx + 1] = "false"
        result = _run(args)
        self.assertNotIn("Traceback", result.stdout + result.stderr)
        self.assertEqual(result.returncode, 1)
        self.assertIn("LIVE_ZERO_STATE_VERDICT=BLOCKED", result.stdout)
        self.assertIn("PI_LIVE_AUDIT_NOT_AVAILABLE", result.stdout)

    def test_run_id_mismatch_blocks_without_crashing(self):
        args = list(ALL_GOOD_ARGS)
        idx = args.index("--pi-run-id")
        args[idx + 1] = "a_different_run_id"
        result = _run(args)
        self.assertNotIn("Traceback", result.stdout + result.stderr)
        self.assertEqual(result.returncode, 1)
        self.assertIn("RUN_ID_MISMATCH", result.stdout)

    def test_missing_optional_arguments_use_documented_defaults(self):
        # Every flag has a default -- invoking with zero arguments must
        # not crash (this is what a positional-argv off-by-one bug
        # would have masked: silently wrong argument alignment instead
        # of an outright crash, in a different but related failure
        # mode this named-flag design also closes off).
        result = _run([])
        self.assertNotIn("Traceback", result.stdout + result.stderr)
        self.assertIn("LIVE_ZERO_STATE_VERDICT=BLOCKED", result.stdout)


if __name__ == "__main__":
    unittest.main()
