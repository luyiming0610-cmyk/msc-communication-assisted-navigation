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

    def test_pulse_step_has_no_silently_assigned_window(self):
        # Requirement: do not silently pick a window for the pulse step
        # since none of the nine defined windows is authorized for it.
        step_13_section = self.text.split("## 13. Arm and issue one bounded straight pulse")[1].split(
            "## 14. Command immediate zero"
        )[0]
        self.assertNotIn("[Pi Window", step_13_section)
        self.assertNotIn("[WSL Window", step_13_section)
        self.assertNotIn("[PowerShell Window", step_13_section)
        self.assertIn("not yet assigned", step_13_section.lower())

    def test_no_forbidden_provider_or_tool_wording_anywhere(self):
        for term in FORBIDDEN_TERMS:
            self.assertNotIn(term, self.text)


if __name__ == "__main__":
    unittest.main()
