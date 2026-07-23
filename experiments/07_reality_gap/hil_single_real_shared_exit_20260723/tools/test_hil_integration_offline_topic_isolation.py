#!/usr/bin/env python3
"""Regression test locking down that test_hil_integration_offline.sh can
never publish to or subscribe through a real/global HIL topic.

Written after a 2026-07-23 safety incident audit (UNEXPECTED_PHYSICAL_MOTION)
found that the script started its guard without an --arm-topic override, so
the guard defaulted to hil_cmd_vel_guard.py's GLOBAL --arm-topic default
(/hil_guard/arm) -- the same topic a real or another test's guard instance
could be subscribed to -- and the script's own `ros2 topic pub` armed that
same global topic. The audit did not prove this caused the incident (no
continuous /cmd_vel recording existed to confirm or rule out any command
origin), but it is a real, independently-confirmed design flaw, fixed by
passing an explicit test-namespaced --arm-topic.

This test is static (parses the script text; does not require rclpy or a
sourced ROS workspace) so it can run anywhere, anytime, including while the
physical robot is powered -- it starts no processes and touches no topics
itself.
"""
import re
import unittest
from pathlib import Path

SCRIPT_PATH = Path(__file__).parent / "test_hil_integration_offline.sh"

# Real/global topics this offline test must never reference as a literal
# argument value -- only as prose in comments explaining what NOT to do.
FORBIDDEN_TOPICS = (
    "/hil_guard/arm",
    "/epuck1/state",
    "/cmd_vel_unguarded",
    "/cmd_vel",
)

REQUIRED_TEST_NAMESPACE = "/hil_offline_test/"


def _active_command_lines(script_text: str) -> list[str]:
    """Lines that are actually executed -- strips full-line and
    trailing '#'-comments, keeps everything else verbatim."""
    lines = []
    for raw_line in script_text.splitlines():
        line = raw_line
        # Drop a trailing comment only when a '#' is preceded by whitespace
        # (avoids mangling '#!/usr/bin/env bash' or topic names, none of
        # which contain '#').
        comment_match = re.search(r"(?<!\S)#", line)
        if comment_match:
            line = line[: comment_match.start()]
        stripped = line.strip()
        if stripped:
            lines.append(stripped)
    return lines


class OfflineIntegrationTestTopicIsolationTest(unittest.TestCase):
    def setUp(self):
        self.script_text = SCRIPT_PATH.read_text(encoding="utf-8")
        self.active_lines = _active_command_lines(self.script_text)

    def test_script_exists(self):
        self.assertTrue(SCRIPT_PATH.is_file(), f"expected {SCRIPT_PATH} to exist")

    def test_no_publish_or_subscribe_line_references_a_forbidden_real_topic(self):
        # Only lines that actually start a process or publish -- via this
        # script's own `start` launcher helper or a direct `ros2 topic
        # pub` -- can put a subscriber/publisher on a topic. Read-only
        # `ros2 topic info` checks (used deliberately to confirm the
        # real /cmd_vel's publisher count stays 0) and echo/check
        # diagnostic strings are not a contact surface and must not be
        # flagged.
        contact_lines = [
            line
            for line in self.active_lines
            if re.match(r"start\b", line) or "ros2 topic pub" in line
        ]
        offenders = []
        for line in contact_lines:
            for topic in FORBIDDEN_TOPICS:
                pattern = re.escape(topic) + r"(?![A-Za-z0-9_/])"
                for match in re.finditer(pattern, line):
                    prefix = line[: match.start()]
                    if prefix.endswith(REQUIRED_TEST_NAMESPACE.rstrip("/")):
                        continue
                    offenders.append((topic, line))
        self.assertEqual(
            [],
            offenders,
            f"a process-starting or publish line references a forbidden real/global topic: {offenders}",
        )

    def test_guard_invocation_passes_an_explicit_test_namespaced_arm_topic(self):
        match = re.search(r"--arm-topic\s+(\S+)", self.script_text)
        self.assertIsNotNone(
            match, "expected an explicit --arm-topic override in the guard invocation"
        )
        arm_topic = match.group(1)
        self.assertTrue(
            arm_topic.startswith(REQUIRED_TEST_NAMESPACE),
            f"--arm-topic value {arm_topic!r} must be namespaced under {REQUIRED_TEST_NAMESPACE!r}",
        )
        self.assertNotEqual(arm_topic, "/hil_guard/arm")

    def test_arm_pub_command_targets_the_same_namespaced_arm_topic(self):
        match = re.search(r"--arm-topic\s+(\S+)", self.script_text)
        self.assertIsNotNone(match)
        arm_topic = match.group(1)
        pub_match = re.search(
            r"ros2 topic pub --once (\S+) std_msgs/msg/Bool", self.script_text
        )
        self.assertIsNotNone(pub_match, "expected an 'ros2 topic pub --once ... std_msgs/msg/Bool' arm command")
        self.assertEqual(pub_match.group(1), arm_topic)

    def test_every_hil_cmd_vel_guard_flag_is_test_namespaced(self):
        for flag in (
            "--physical-state-topic",
            "--upstream-cmd-vel-topic",
            "--guarded-cmd-vel-topic",
            "--arm-topic",
        ):
            match = re.search(re.escape(flag) + r"\s+(\S+)", self.script_text)
            self.assertIsNotNone(match, f"expected {flag} to be passed explicitly")
            value = match.group(1)
            self.assertTrue(
                value.startswith(REQUIRED_TEST_NAMESPACE),
                f"{flag} value {value!r} must be namespaced under {REQUIRED_TEST_NAMESPACE!r}",
            )


if __name__ == "__main__":
    unittest.main()
