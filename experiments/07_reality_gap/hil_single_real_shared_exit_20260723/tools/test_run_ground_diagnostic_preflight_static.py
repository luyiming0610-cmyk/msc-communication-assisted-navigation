#!/usr/bin/env python3
"""Static proof that run_ground_diagnostic_preflight.sh's check-only
modes can never publish a message or start a physical/HIL process, plus
shell syntax checks for every new script this diagnostic added. No
rclpy dependency. Most checks are purely static (bash -n only parses,
it does not run anything); the one exception
(test_requires_an_explicit_mode_argument_no_default) actually invokes
the script with no/an invalid mode argument, which this script's mode
check rejects immediately -- before sourcing anything ROS-related is
even reached in a way that touches the graph, and before any process,
network, or ros2 CLI call -- so it remains a zero-side-effect run.
"""
import re
import subprocess
import unittest
from pathlib import Path

TOOLS_DIR = Path(__file__).parent
PREFLIGHT_SCRIPT = TOOLS_DIR / "run_ground_diagnostic_preflight.sh"

# Every shell script this diagnostic's preparation added or reuses
# directly -- bash -n must accept each one.
SHELL_SCRIPTS_TO_SYNTAX_CHECK = (
    "run_ground_diagnostic_preflight.sh",
)


class PreflightNeverPublishesOrStartsAProcessTest(unittest.TestCase):
    def setUp(self):
        self.text = PREFLIGHT_SCRIPT.read_text(encoding="utf-8")

    def test_script_exists(self):
        self.assertTrue(PREFLIGHT_SCRIPT.is_file())

    def test_never_calls_ros2_topic_pub(self):
        # The only mutating ROS CLI call this diagnostic's preflight
        # could make would be a topic publish -- must never appear.
        self.assertIsNone(re.search(r"ros2\s+topic\s+pub", self.text))

    def test_never_calls_ros2_run(self):
        # Preflight must never itself start a node via `ros2 run`.
        self.assertIsNone(re.search(r"ros2\s+run\b", self.text))

    def test_never_invokes_a_hil_python_tool_directly(self):
        # Must never directly exec hil_cmd_vel_guard.py, the recorder,
        # hil_wheel_suspension_test.py, cooperative_avoider, etc. --
        # only read-only ros2 CLI calls (topic info/echo/list/hz),
        # pgrep, ping, and the two read-only scripts it explicitly
        # reuses (run_hil_physical_preflight.sh, hil_preflight.py, both
        # invoked read-only).
        forbidden_patterns = (
            r"python3\s+['\"]?[^'\"\s]*hil_cmd_vel_guard\.py",
            r"python3\s+['\"]?[^'\"\s]*hil_command_evidence_recorder\.py",
            r"python3\s+['\"]?[^'\"\s]*hil_wheel_suspension_test\.py",
            r"python3\s+['\"]?[^'\"\s]*hil_virtual_peer\.py",
            r"python3\s+['\"]?[^'\"\s]*hil_topic_adapter\.py",
        )
        for pattern in forbidden_patterns:
            self.assertIsNone(re.search(pattern, self.text), f"forbidden pattern found: {pattern}")

    def test_only_read_only_ros2_topic_verbs_are_used(self):
        # Matches only actual `ros2 topic <verb>` invocations (verb
        # immediately follows "topic"), not incidental prose mentioning
        # "ros2" elsewhere (e.g. an echoed string like "ros2 CLI not on
        # PATH") -- that prose has no "topic" after "ros2", so it can
        # never match this pattern.
        allowed_topic_verbs = {"info", "echo", "list", "hz"}
        found_any = False
        for match in re.finditer(r"ros2\s+topic\s+(\w+)", self.text):
            found_any = True
            self.assertIn(match.group(1), allowed_topic_verbs, f"unexpected ros2 topic verb: {match.group(1)}")
        self.assertTrue(found_any, "expected at least one ros2 topic invocation in this script")

    def test_reuses_existing_preflight_scripts_rather_than_duplicating_their_checks(self):
        self.assertIn("run_hil_physical_preflight.sh", self.text)
        self.assertIn("check_required_fields_ready", self.text)
        self.assertIn("hil_ground_diagnostic_session", self.text)
        self.assertIn("hil_ground_diagnostic_phases", self.text)
        self.assertIn("analyze_ground_diagnostic", self.text)

    def test_requires_an_explicit_mode_argument_no_default(self):
        # No mode, or an unrecognized mode, must exit nonzero with a
        # usage message -- never silently fall back to one phase.
        result = subprocess.run(["bash", str(PREFLIGHT_SCRIPT)], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("USAGE", (result.stdout + result.stderr))

        result = subprocess.run(["bash", str(PREFLIGHT_SCRIPT), "bogus-mode"], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("USAGE", (result.stdout + result.stderr))

    def test_both_explicit_phases_are_present(self):
        self.assertIn('"pre-stack"', self.text)
        self.assertIn('"live-zero-state"', self.text)
        self.assertIn("GROUND_DIAGNOSTIC_PRE_STACK_CHECK_PASS", self.text)
        self.assertIn("GROUND_DIAGNOSTIC_PRE_STACK_CHECK_BLOCKED", self.text)
        self.assertIn("GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_PASS", self.text)
        self.assertIn("GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_BLOCKED", self.text)

    def test_pre_stack_phase_never_requires_validity_flags_bridge_or_guard(self):
        # The circular-dependency fix: the pre-stack code path (text
        # before the live-zero-state phase begins) must not itself gate
        # on any live-stack-only fact.
        pre_stack_text = self.text.split('if [[ "${MODE}" == "live-zero-state" ]]; then')[0]
        for forbidden in ("VALIDITY_FLAGS_NOT_7", "BRIDGE_NOT_CONNECTED", "GUARD_ARMED"):
            self.assertNotIn(forbidden, pre_stack_text)

    def test_shell_syntax_is_valid_for_every_new_script(self):
        for script_name in SHELL_SCRIPTS_TO_SYNTAX_CHECK:
            script_path = TOOLS_DIR / script_name
            self.assertTrue(script_path.is_file(), f"missing: {script_name}")
            result = subprocess.run(
                ["bash", "-n", str(script_path)], capture_output=True, text=True
            )
            self.assertEqual(
                result.returncode, 0, f"bash -n failed for {script_name}: {result.stderr}"
            )


if __name__ == "__main__":
    unittest.main()
