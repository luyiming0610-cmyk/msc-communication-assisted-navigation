#!/usr/bin/env python3
"""Runtime proof (Part 4 safety-validation requirement, item 16) that
hil_command_evidence_recorder.py creates ZERO publishers and therefore
cannot generate or replay a motor command, no matter what it observes.

Requires rclpy/ROS message packages (unlike every other test_hil_*.py
file in this directory, which are deliberately rclpy-free) -- run
inside a sourced ROS 2 environment, same as test_hil_topic_contract.py.
Runs under the same `-r __ns:=/pytest_isolated` namespace push used
throughout src/epuck2_comm/test/ since 2026-07-23, as defense in depth
even though this node never publishes anything.
"""
import tempfile
import unittest
from pathlib import Path

import rclpy

from hil_command_evidence_recorder import CSV_FIELDS, _build_node


class CommandEvidenceRecorderZeroPublishersTest(unittest.TestCase):
    def setUp(self):
        rclpy.init(
            args=[
                "--ros-args",
                "-r", "__ns:=/pytest_isolated",
            ]
        )
        self._tmpdir = tempfile.TemporaryDirectory()
        self._csv_path = str(Path(self._tmpdir.name) / "evidence.csv")
        _, self.RecorderClass, self.parse_args = _build_node()
        self.args = self.parse_args(["--output-csv", self._csv_path])
        self.node = self.RecorderClass(self.args)

    def tearDown(self):
        self.node.close_csv()
        self.node.destroy_node()
        rclpy.shutdown()
        self._tmpdir.cleanup()

    def test_csv_exists_with_header_immediately_after_construction(self):
        # The specific regression this project hit live 2026-07-24:
        # the CSV must exist (and be a valid, header-only file) the
        # instant the node is constructed, not only once the run ends.
        self.assertTrue(Path(self._csv_path).is_file())
        with open(self._csv_path, encoding="utf-8") as fh:
            header = fh.readline().strip().split(",")
        self.assertEqual(header, CSV_FIELDS)

    def test_recorder_creates_zero_publishers(self):
        # Every rclpy Node automatically gets two standard
        # infrastructure publishers (/parameter_events, /rosout) from
        # the Node base class itself -- these exist on every node in
        # this entire project (the guard, the recorder, every HIL
        # tool) and are not something this recorder's own code
        # creates. Excluded here so the assertion checks what it
        # actually means to check: no command/motion-capable publisher.
        STANDARD_INFRASTRUCTURE_TOPICS = {"/parameter_events", "/rosout"}
        publishers = self.node.get_publisher_names_and_types_by_node(
            self.node.get_name(), self.node.get_namespace()
        )
        non_infrastructure = [
            (name, types)
            for name, types in publishers
            if name not in STANDARD_INFRASTRUCTURE_TOPICS
        ]
        self.assertEqual(
            non_infrastructure,
            [],
            "hil_command_evidence_recorder must never create any publisher "
            f"beyond rclpy's standard infrastructure ones -- found: {non_infrastructure}",
        )

    def test_recorder_has_exactly_the_five_expected_subscriptions_and_no_more(self):
        subscriptions = self.node.get_subscriber_names_and_types_by_node(
            self.node.get_name(), self.node.get_namespace()
        )
        topic_names = {name for name, _ in subscriptions}
        expected = {
            self.node.resolve_topic_name(self.args.upstream_cmd_vel_topic),
            self.node.resolve_topic_name(self.args.guarded_cmd_vel_topic),
            self.node.resolve_topic_name(self.args.arm_topic),
            self.node.resolve_topic_name(self.args.state_topic),
            self.node.resolve_topic_name(self.args.bridge_status_topic),
        }
        self.assertEqual(topic_names, expected)

    def test_source_file_contains_no_create_publisher_call(self):
        # Static, belt-and-braces check alongside the live introspection
        # above -- catches a future edit that adds a publisher even if
        # it is never actually invoked in this specific test run.
        # Matches the actual call pattern (".create_publisher(") rather
        # than the bare substring, which would false-positive on this
        # module's own explanatory comments mentioning the word.
        import inspect
        import re

        import hil_command_evidence_recorder as module

        source = inspect.getsource(module)
        self.assertIsNone(re.search(r"\.create_publisher\(", source))


if __name__ == "__main__":
    unittest.main()
