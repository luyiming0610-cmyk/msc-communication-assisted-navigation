"""pytest-level enforcement of ROS test-domain isolation.

Added after the 2026-07-23 UNEXPECTED_PHYSICAL_MOTION safety incidents
(see experiments/07_reality_gap/hil_single_real_shared_exit_20260723/
safety_incident_unexpected_motion_20260723/SUMMARY.md and
safety_incident_unexpected_motion_2_20260723/SUMMARY.md). Per-test topic
remaps (``-r __ns:=/pytest_isolated`` in the affected test files) are the
first layer; this is the second, independent layer: even a test that
forgets a remap must still be unable to reach a live physical process,
because it is running in a different ROS2 DDS domain entirely.

This must take effect before the FIRST ``rclpy.init()`` call in the
whole pytest session, so it is set in ``pytest_configure`` (runs at
collection start) rather than in a fixture (which only runs once a test
using it actually executes).
"""
import os

# Arbitrary, reserved for automated test runs only. Never set by any
# physical bring-up script (run_hil_*.sh, the WSL bridge, the Pi
# server, state_publisher) -- those all run with ROS_DOMAIN_ID unset
# (default domain 0), so this value is guaranteed distinct from them.
TEST_ROS_DOMAIN_ID = "89"


def pytest_configure(config):
    os.environ["ROS_DOMAIN_ID"] = TEST_ROS_DOMAIN_ID
