"""Static regression guarding the ROS-domain/topic isolation fix added
after the 2026-07-23 UNEXPECTED_PHYSICAL_MOTION safety incidents (see
experiments/07_reality_gap/hil_single_real_shared_exit_20260723/
safety_incident_unexpected_motion_20260723/SUMMARY.md and
safety_incident_unexpected_motion_2_20260723/SUMMARY.md).

The second incident's audit found that several test files in this
directory construct real rclpy Node subclasses (CooperativeAvoider,
StatePublisher, NetworkImpairmentRelay, SequenceCounterNode) with no
topic remap and no ROS_DOMAIN_ID isolation -- meaning a routine test
run could place genuine commands onto the real, driver-facing topics
if any part of the physical stack happened to be live on the same DDS
domain at the same time. The fix pushes every such node into a private
`/pytest_isolated` namespace via `-r __ns:=/pytest_isolated` in every
`rclpy.init()` call, in addition to (not instead of) the runner-level
ROS_DOMAIN_ID isolation in conftest.py and run_isolated_test_suite.sh.

This test is purely static (parses source text; does not import rclpy
or start any process) so it can run anywhere, anytime, including while
the physical robot is powered.
"""
import re
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).parent

# Node classes whose constructor creates a real ROS publisher or
# subscription with no test-safe default of its own -- any test file
# that imports and constructs one of these MUST push a private
# namespace via `-r __ns:=` in every rclpy.init() call in that file.
HAZARD_CLASSES = (
    "CooperativeAvoider",
    "StatePublisher",
    "NetworkImpairmentRelay",
    "SequenceCounterNode",
)

REQUIRED_NS_REMAP = "__ns:=/pytest_isolated"

REAL_HARDWARE_TOPIC_NAMES = (
    "cmd_vel",
    "cmd_vel_unguarded",
    "/cmd_vel",
    "/cmd_vel_unguarded",
    "/epuck1/state",
    "/hil_guard/arm",
)


def _test_files():
    return sorted(
        p for p in TEST_DIR.glob("test_*.py") if p.name != Path(__file__).name
    )


class PytestTopicIsolationTest(unittest.TestCase):
    def test_every_rclpy_init_in_a_hazard_test_file_pushes_the_isolated_namespace(self):
        # Checked whole-file, not per call-site: some files build the
        # remap list in a separate `args = [...]` variable and then
        # call `rclpy.init(args=args)`, so the remap text is not
        # textually inside that call's own parentheses even though it
        # genuinely applies to it (confirmed live by
        # test_cooperative_avoider_topic_isolation_runtime.py). A
        # whole-file check still catches the real hazard this guards
        # against -- a hazard-class file with the remap missing
        # entirely -- without false-positiving on the indirection
        # pattern.
        offenders = []
        for path in _test_files():
            text = path.read_text(encoding="utf-8")
            if not any(cls in text for cls in HAZARD_CLASSES):
                continue
            if REQUIRED_NS_REMAP not in text:
                offenders.append(path.name)
        self.assertEqual(
            [],
            offenders,
            "test file(s) construct a real ROS node without the required "
            f"isolated namespace remap ({REQUIRED_NS_REMAP!r}) anywhere in the file: {offenders}",
        )

    def test_no_test_file_remaps_a_topic_directly_to_a_real_hardware_name(self):
        # Defense against a backwards/typo'd remap (e.g. accidentally
        # writing "-r", "/pytest_isolated/cmd_vel:=cmd_vel" instead of
        # the other way round) -- no remap TARGET may equal a real
        # hardware topic name.
        offenders = []
        for path in _test_files():
            text = path.read_text(encoding="utf-8")
            for match in re.finditer(r'"-r",\s*"([^"]+)"', text):
                remap = match.group(1)
                if ":=" not in remap:
                    continue
                _, target = remap.split(":=", 1)
                if target in REAL_HARDWARE_TOPIC_NAMES:
                    offenders.append((path.name, remap))
        self.assertEqual(
            [], offenders, f"remap target(s) point at a real hardware topic: {offenders}"
        )

    def test_conftest_forces_a_dedicated_non_physical_ros_domain_id(self):
        conftest_path = TEST_DIR / "conftest.py"
        self.assertTrue(conftest_path.is_file(), "expected conftest.py to exist")
        text = conftest_path.read_text(encoding="utf-8")
        self.assertIn("ROS_DOMAIN_ID", text)
        self.assertIn("pytest_configure", text)


if __name__ == "__main__":
    unittest.main()
