#!/usr/bin/env bash
# Permanent safe test runner -- the ONLY sanctioned way to run the HIL
# unit suite and the epuck2_comm/epuck2_comm_interfaces colcon suite
# from 2026-07-23 onward.
#
# Added after two UNEXPECTED_PHYSICAL_MOTION safety incidents on
# 2026-07-23 (see the two safety_incident_unexpected_motion*_20260723/
# SUMMARY.md records). The second incident's audit found that ordinary
# `colcon test` for epuck2_comm instantiates real, unremapped
# CooperativeAvoider/StatePublisher/NetworkImpairmentRelay/
# SequenceCounterNode rclpy nodes with no ROS_DOMAIN_ID isolation from
# any live physical bridge/driver process -- meaning a routine test run
# could place genuine nonzero Twist commands onto the exact `cmd_vel`
# topic a live WSL bridge subscribes to. Per-test topic remaps
# (`-r __ns:=/pytest_isolated`, already added to every affected test
# file) are the first layer of defense; this script is the second,
# independent layer, enforced at the runner level so a test file
# mistake alone can never be sufficient to reach the physical robot:
#   1. Refuses to run at all if any physical/HIL process is detected.
#   2. Checks the real /cmd_vel publisher count in the DEFAULT domain
#      BEFORE touching anything.
#   3. Only then switches to a dedicated, non-physical ROS_DOMAIN_ID for
#      the entire test run (physical bring-up scripts never set this
#      variable, so it is guaranteed distinct from them).
#   4. Runs the HIL suite (offline, no rclpy) and the colcon suite
#      (isolated domain) inside that isolation.
#   5. Switches back to the default domain and re-checks /cmd_vel
#      afterward, to catch anything that appeared during the run.
#
# This script starts no physical process, changes no controller/guard
# logic, and must be run instead of `python3 -m unittest` /
# `pytest` / `colcon test` directly whenever there is any chance the
# physical stack could be live.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash
set -u

# Matches every process name that constitutes "the physical/HIL
# command path" -- the same class of names used by
# run_hil_physical_preflight.sh's own residual-process check, plus the
# WSL bridge and state_publisher (which run_hil_physical_preflight.sh
# deliberately does NOT flag, since it expects them already running for
# a physical HIL session -- but a test run must refuse even then).
PHYSICAL_PATTERN='state_publisher|wsl_epuck_tcp_bridge|hil_cmd_vel_guard|hil_virtual_peer|hil_topic_adapter|cooperative_avoider|goal_navigator'

# Reserved for automated test runs only; never set by any physical
# bring-up script, so it is guaranteed distinct from whatever domain
# (always the unset default, domain 0) any live physical process uses.
TEST_ROS_DOMAIN_ID=89

echo "=== [1/6] Refusing to run if any physical/HIL process is detected ==="
if MATCHED="$(pgrep -af -- "${PHYSICAL_PATTERN}" 2>/dev/null)"; then
    echo "${MATCHED}"
    echo "SAFE_TEST_RUNNER_BLOCKED_PHYSICAL_PROCESS_DETECTED"
    exit 1
fi
echo "No matching physical/HIL process found."

echo ""
echo "=== [2/6] Real /cmd_vel publisher count, default domain, BEFORE ==="
unset ROS_DOMAIN_ID
BEFORE="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || true)"
if [[ -z "${BEFORE}" ]]; then
    BEFORE="TOPIC_NOT_PRESENT"
fi
echo "real_cmd_vel_publisher_count_before=${BEFORE}"
if [[ "${BEFORE}" != "TOPIC_NOT_PRESENT" && "${BEFORE}" != "0" ]]; then
    echo "SAFE_TEST_RUNNER_BLOCKED_REAL_CMD_VEL_PUBLISHER_PRESENT"
    exit 1
fi

echo ""
echo "=== [3/6] Switching to isolated ROS_DOMAIN_ID=${TEST_ROS_DOMAIN_ID} ==="
export ROS_DOMAIN_ID="${TEST_ROS_DOMAIN_ID}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

echo ""
echo "=== [4/6] HIL unit test suite (offline, no rclpy at all) ==="
(cd "${SCRIPT_DIR}" && python3 -m unittest discover -s . -p "test_hil_*.py" -v)
HIL_EXIT=$?

echo ""
echo "=== [5/6] colcon test: epuck2_comm, epuck2_comm_interfaces (isolated domain) ==="
(cd ~/epuck_ws && colcon test --packages-select epuck2_comm epuck2_comm_interfaces --event-handlers console_direct+)
COLCON_EXIT=$?
(cd ~/epuck_ws && colcon test-result --verbose) || true

echo ""
echo "=== [6/6] Real /cmd_vel publisher count, default domain, AFTER ==="
unset ROS_DOMAIN_ID
AFTER="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || true)"
if [[ -z "${AFTER}" ]]; then
    AFTER="TOPIC_NOT_PRESENT"
fi
echo "real_cmd_vel_publisher_count_after=${AFTER}"

FAIL=0
if [[ ${HIL_EXIT} -ne 0 ]]; then
    echo "HIL_SUITE_FAILED"
    FAIL=1
fi
if [[ ${COLCON_EXIT} -ne 0 ]]; then
    echo "COLCON_SUITE_FAILED"
    FAIL=1
fi
if [[ "${AFTER}" != "TOPIC_NOT_PRESENT" && "${AFTER}" != "0" ]]; then
    echo "REAL_CMD_VEL_PUBLISHER_APPEARED_DURING_TEST_RUN"
    FAIL=1
fi

if [[ ${FAIL} -eq 0 ]]; then
    echo "SAFE_ISOLATED_TEST_RUN_PASS"
else
    echo "SAFE_ISOLATED_TEST_RUN_FAIL"
fi
exit ${FAIL}
