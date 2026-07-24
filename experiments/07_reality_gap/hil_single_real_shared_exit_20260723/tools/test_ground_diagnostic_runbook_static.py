#!/usr/bin/env python3
"""Static, read-only checks on GROUND_DIAGNOSTIC_RUNBOOK.md's text --
no ROS/rclpy dependency, no process started, the runbook file is only
read, never executed or modified.
"""
import re
import unittest
from pathlib import Path

RUNBOOK_PATH = (
    Path(__file__).parent.parent / "first_ground_diagnostic" / "GROUND_DIAGNOSTIC_RUNBOOK.md"
)

REQUIRED_WINDOW_LABELS = (
    "Pi Window 1 -- physical driver",
    "Pi Window 2 -- audited command server",
    "Pi Window 3 -- Pi read-only verification",
    "WSL Window 1 -- TCP bridge",
    "WSL Window 2 -- state publisher",
    "WSL Window 3 -- command-evidence recorder control",
    "WSL Window 4 -- command guard",
    "WSL Window 5 -- read-only HIL verification",
    "WSL Window 6 -- supervised motion command",
    "PowerShell Window 1 -- operator transfer and host checks",
)

FORBIDDEN_TERMS = ("ChatGPT", "Claude", "Codex", "OpenAI", "Anthropic")


class RecorderDurationTest(unittest.TestCase):
    def setUp(self):
        self.text = RUNBOOK_PATH.read_text(encoding="utf-8")

    def test_runbook_exists(self):
        self.assertTrue(RUNBOOK_PATH.is_file())

    def test_uses_the_extended_3600s_duration(self):
        self.assertIn("--duration-s 3600", self.text)

    def test_does_not_use_the_old_600s_duration_for_the_recorder(self):
        # The old value must not remain anywhere as a recorder duration
        # instruction (a bare "600" appearing in an unrelated context,
        # e.g. a port number, is not what this guards against).
        self.assertNotIn("--duration-s 600", self.text)

    def test_early_recorder_exit_is_documented_as_excluding_the_run(self):
        self.assertIn("EXCLUDED", self.text)
        self.assertTrue(
            re.search(r"recorder exits.{0,200}EXCLUDED", self.text, re.DOTALL)
            or re.search(r"EXCLUDED.{0,200}recorder exits", self.text, re.DOTALL)
        )

    def test_never_silently_restarts_recorder_mid_run(self):
        lowered = self.text.lower()
        self.assertTrue(
            "never silently start a replacement recorder" in lowered
            or "never silently restart" in lowered
        )


class WindowLabelsTest(unittest.TestCase):
    def setUp(self):
        self.text = RUNBOOK_PATH.read_text(encoding="utf-8")

    def test_every_required_window_label_appears_at_least_once(self):
        for label in REQUIRED_WINDOW_LABELS:
            self.assertIn(label, self.text, f"missing window label: {label}")

    def test_every_required_window_label_is_used_as_at_least_one_step_heading(self):
        # Step headings use the bracketed form `[Label]`. Long-running
        # windows are legitimately referenced by more than one step
        # (start, verification, shutdown) -- this only confirms each
        # label is actually used to introduce a command, not left as a
        # table-only definition with nothing pointing to it.
        for label in REQUIRED_WINDOW_LABELS:
            bracketed_count = self.text.count(f"[{label}]")
            self.assertGreaterEqual(
                bracketed_count, 1, f"expected at least one bracketed step heading for: {label}"
            )

    def test_window_map_table_defines_the_role_of_every_window_exactly_once(self):
        # The role/purpose definition (the window-map table row) must
        # appear exactly once per window -- that is the single source
        # of truth for what each window is for, even though the label
        # itself is used repeatedly afterward as a step heading.
        table_section = self.text.split("## Fixed terminal/window map")[1].split("## Emergency procedure")[0]
        for label in REQUIRED_WINDOW_LABELS:
            self.assertEqual(
                table_section.count(label), 1, f"expected exactly one table row for: {label}"
            )

    def test_evidence_summary_table_lists_every_long_running_window(self):
        table_section = self.text.split("## Window/evidence table for the run summary")[1]
        long_running_labels = (
            "Pi Window 1 -- physical driver",
            "Pi Window 2 -- audited command server",
            "WSL Window 1 -- TCP bridge",
            "WSL Window 2 -- state publisher",
            "WSL Window 3 -- command-evidence recorder control",
            "WSL Window 4 -- command guard",
        )
        for label in long_running_labels:
            self.assertIn(label, table_section)

    def test_pulse_step_is_assigned_to_wsl_window_6_only(self):
        # The window-assignment gap is resolved: steps 13/14 use WSL
        # Window 6 exclusively, never any of the other eight windows.
        step_13_section = self.text.split("## 13. Arm and issue one bounded straight pulse")[1].split(
            "## 14. Command immediate zero"
        )[0]
        step_14_section = self.text.split("## 14. Command immediate zero")[1].split("## 15.")[0]
        for section in (step_13_section, step_14_section):
            self.assertIn("[WSL Window 6 -- supervised motion command]", section)
            self.assertNotIn("[Pi Window", section)
            self.assertNotIn("[WSL Window 4", section)
            self.assertNotIn("[WSL Window 5", section)
            self.assertNotIn("[PowerShell Window", section)

    def test_wsl_window_6_gated_on_prior_approvals(self):
        table_section = self.text.split("## Fixed terminal/window map")[1].split("## Emergency procedure")[0]
        window_6_row = next(line for line in table_section.splitlines() if "WSL Window 6" in line)
        self.assertIn("LIVE_ZERO_STATE_CHECK_PASS", window_6_row)
        self.assertIn("APPROVED_FOR_SINGLE_PULSE", window_6_row)

    def test_wsl_window_6_forbidden_processes_documented(self):
        table_section = self.text.split("## Fixed terminal/window map")[1].split("## Emergency procedure")[0]
        window_6_row = next(line for line in table_section.splitlines() if "WSL Window 6" in line)
        for forbidden in ("pytest", "colcon", "cooperative_avoider", "Webots", "rosbag"):
            self.assertIn(forbidden, window_6_row)

    def test_wsl_window_6_pulse_bounds_documented(self):
        step_13_section = self.text.split("## 13. Arm and issue one bounded straight pulse")[1].split(
            "## 14. Command immediate zero"
        )[0]
        normalized = " ".join(step_13_section.split())
        self.assertIn("0.015", normalized)
        self.assertIn("0.0", normalized)
        self.assertIn("never looped or re-run automatically", normalized)
        self.assertIn("no second pulse", normalized.lower())

    def test_shutdown_procedure_references_wsl_window_6(self):
        shutdown_section = self.text.split("## 16. Exact-PID reverse shutdown")[1]
        self.assertIn("[WSL Window 6 -- supervised motion command]", shutdown_section)

    def test_evidence_summary_table_lists_wsl_window_6(self):
        table_section = self.text.split("## Window/evidence table for the run summary")[1]
        self.assertIn("WSL Window 6 -- supervised motion command", table_section)

    def test_no_forbidden_provider_or_tool_wording_anywhere(self):
        for term in FORBIDDEN_TERMS:
            self.assertNotIn(term, self.text)


if __name__ == "__main__":
    unittest.main()
