#!/usr/bin/env python3
"""Static, read-only checks on
NEW_FIELD_SINGLE_PULSE_COMMAND_SHEET_20260728_143937.md's text -- no
ROS/rclpy dependency, no process started, the command sheet is only
read, never executed or modified.

Added 2026-07-28 after NEW_FIELD_COMMAND_SHEET_VERDICT=BLOCKED found
that the approved new-field specification recorded no dedicated,
unambiguous executable recorder command at all (only prose mentions of
"recorder"), and its pulse command lacked an explicit
--upstream-cmd-vel-topic flag. This test guards the frozen command
sheet created to close that gap.

The command sheet's own prohibitions section is REQUIRED to name
hil_wheel_suspension_test.py, --pulse-s 2, and --output-csv explicitly
(so a reader sees exactly what not to do, not a paraphrase) -- so this
test must not assert those strings are absent from the whole document.
Instead it extracts the two ```bash fenced blocks (the actual
executable recorder and pulse commands) and checks executable content
against prose content separately: the forbidden strings may appear in
prose, but must never appear inside an executable block.
"""
import re
import unittest
from pathlib import Path

COMMAND_SHEET_PATH = (
    Path(__file__).parent.parent
    / "new_field_single_pulse_revalidation"
    / "NEW_FIELD_SINGLE_PULSE_COMMAND_SHEET_20260728_143937.md"
)

FROZEN_WSL_OUTPUT_ROOT = (
    "/home/eamon/epuck_comm_bags/new_field_single_pulse_revalidation_20260728_143937"
)

BASH_FENCE_RE = re.compile(r"```bash\n(.*?)```", re.DOTALL)


def _extract_bash_blocks(text: str) -> list[str]:
    return BASH_FENCE_RE.findall(text)


class NewFieldSinglePulseCommandSheetStaticTest(unittest.TestCase):
    def setUp(self):
        self.text = COMMAND_SHEET_PATH.read_text(encoding="utf-8")
        self.blocks = _extract_bash_blocks(self.text)

    def test_command_sheet_exists(self):
        self.assertTrue(COMMAND_SHEET_PATH.is_file())

    def test_contains_exactly_two_executable_bash_blocks(self):
        # Exactly one recorder block, one pulse block -- if this ever
        # drifts (a block added/removed/merged), the fixed-index
        # assumptions in the rest of this test class must be revisited.
        self.assertEqual(len(self.blocks), 2)

    def test_recorder_block_is_first_pulse_block_is_second(self):
        recorder_block, pulse_block = self.blocks
        self.assertIn("run_hil_command_evidence_recorder.sh", recorder_block)
        self.assertIn("hil_ground_single_pulse_test.py", pulse_block)

    # -- Executable pulse block: required content --

    def test_pulse_block_uses_the_ground_pulse_tool(self):
        _, pulse_block = self.blocks
        self.assertIn("hil_ground_single_pulse_test.py", pulse_block)

    def test_pulse_block_uses_explicit_upstream_cmd_vel_topic_flag(self):
        _, pulse_block = self.blocks
        self.assertIn("--upstream-cmd-vel-topic cmd_vel_unguarded", pulse_block)

    def test_pulse_block_uses_the_confirmed_pulse_linear_speed(self):
        _, pulse_block = self.blocks
        self.assertIn("--pulse-linear-mps 0.015", pulse_block)

    def test_pulse_block_uses_the_confirmed_pulse_duration(self):
        _, pulse_block = self.blocks
        self.assertIn("--pulse-s 6.67", pulse_block)

    # -- Executable pulse block: forbidden content --

    def test_pulse_block_does_not_use_the_historical_two_second_pulse_duration(self):
        _, pulse_block = self.blocks
        self.assertNotIn("--pulse-s 2", pulse_block)

    def test_pulse_block_does_not_invoke_the_suspended_wheel_tool(self):
        _, pulse_block = self.blocks
        self.assertNotIn("hil_wheel_suspension_test.py", pulse_block)

    # -- Executable recorder block: required content --

    def test_recorder_block_uses_the_frozen_output_root(self):
        recorder_block, _ = self.blocks
        self.assertIn(f"--output-root {FROZEN_WSL_OUTPUT_ROOT}", recorder_block)

    def test_recorder_block_uses_flush_interval_of_one_second(self):
        recorder_block, _ = self.blocks
        self.assertIn("--flush-interval-s 1", recorder_block)

    # -- Executable recorder block: forbidden content --

    def test_recorder_block_does_not_supply_output_csv(self):
        recorder_block, _ = self.blocks
        self.assertNotIn("--output-csv", recorder_block)

    # -- Prohibition prose: forbidden strings must be explicitly named --

    def test_prohibitions_explicitly_name_the_suspended_wheel_tool(self):
        self.assertIn("Do not use `hil_wheel_suspension_test.py`", self.text)

    def test_prohibitions_explicitly_name_the_historical_pulse_duration_flag(self):
        self.assertIn("Do not use `--pulse-s 2`", self.text)

    def test_prohibitions_explicitly_name_the_rejected_csv_flag(self):
        self.assertIn("Do not use `--output-csv`", self.text)

    # -- Frozen identifiers and override, anywhere in the document --

    def test_contains_all_four_frozen_identifiers(self):
        self.assertIn("RUN_ID=20260728_143937", self.text)
        self.assertIn(
            "PI_JSONL=/home/pi/real_robot_avoidance_v1/command_audit_20260728_143937.jsonl",
            self.text,
        )
        self.assertIn(f"WSL_OUTPUT_ROOT={FROZEN_WSL_OUTPUT_ROOT}", self.text)
        self.assertIn(f"WSL_CSV={FROZEN_WSL_OUTPUT_ROOT}/command_evidence.csv", self.text)
        self.assertIn(f"WSL_MANIFEST={FROZEN_WSL_OUTPUT_ROOT}/manifest.json", self.text)

    def test_contains_the_new_field_params_override(self):
        self.assertIn(
            "GROUND_DIAGNOSTIC_PARAMS=<repo>/experiments/07_reality_gap/"
            "hil_single_real_shared_exit_20260723/tools/new_field_geometry_params.json",
            self.text,
        )

    def test_required_gate_and_topic_phrases_are_present(self):
        for phrase in (
            "cmd_vel_unguarded",
            "hil_cmd_vel_guard",
            "EXCLUDED",
            "APPROVED_FOR_SINGLE_PULSE=YES",
            "LIVE_ZERO_STATE_CHECK_PASS",
        ):
            self.assertIn(phrase, self.text)


if __name__ == "__main__":
    unittest.main()
