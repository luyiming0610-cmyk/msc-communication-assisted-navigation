#!/usr/bin/env bash
# Permanent safe test runner -- the ONLY sanctioned way to run the HIL
# unit suite, the Pi command-audit suite, the sync-script suite, and
# the epuck2_comm/epuck2_comm_interfaces colcon suite from 2026-07-23
# onward.
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
#      variable, so it is guaranteed distinct from them) -- every
#      subsequent rclpy-touching step re-asserts this before running.
#   4. Runs the HIL suite, the Pi command-audit suite, the sync-script
#      suite (offline, no rclpy in any of these), the synthetic
#      sync/build e2e test (also no rclpy, no real directories), and
#      the colcon suite (isolated domain) inside that isolation.
#   5. Asserts every expected safety-critical test module was actually
#      COLLECTED (not silently skipped/zero-collected) before declaring
#      any suite a pass.
#   6. Switches back to the default domain and re-checks /cmd_vel
#      afterward, to catch anything that appeared during the run.
#
# This script starts no physical process, changes no controller/guard
# logic, and must be run instead of `python3 -m unittest` /
# `pytest` / `colcon test` directly whenever there is any chance the
# physical stack could be live.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_AUDIT_DIR="$(cd "${SCRIPT_DIR}/../pi_command_audit" && pwd)"
source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash
set -u

PHYSICAL_PATTERN='state_publisher|wsl_epuck_tcp_bridge|hil_cmd_vel_guard|hil_virtual_peer|hil_topic_adapter|cooperative_avoider|goal_navigator'
TEST_ROS_DOMAIN_ID=89

# Every test module that MUST be collected and run for this suite to
# mean anything -- if any of these is silently missing (a typo'd path,
# a moved file, a collection error swallowed by unittest), the runner
# fails outright rather than reporting a misleadingly clean pass.
declare -a REQUIRED_HIL_TEST_MODULES=(
    "test_hil_command_evidence_recorder"
    "test_hil_command_evidence_recorder_zero_publishers"
    "test_sync_epuck2_comm_logic"
    "test_hil_cmd_vel_guard"
    "test_hil_integration_offline_topic_isolation"
    "test_ground_diagnostic_params"
    "test_analyze_ground_diagnostic"
    "test_run_ground_diagnostic_preflight_static"
    "test_hil_ground_diagnostic_session"
    "test_hil_ground_diagnostic_phases"
    "test_hil_ground_diagnostic_source_identity"
    "test_pi_ground_diagnostic_audit_verifier"
    "test_ground_diagnostic_runbook_static"
    "test_hil_live_zero_state_verdict"
    "test_ground_diagnostic_post_run_verifier"
    "test_hil_motion_repeatability_metrics"
    "test_hil_repeatability_pose_readiness"
    "test_hil_repeatability_batch_aggregator"
    "test_new_field_geometry_params"
    "test_hil_ground_single_pulse_test"
)
declare -a REQUIRED_PI_AUDIT_TEST_MODULES=(
    "test_pi_epuck_tcp_server_sensors_audited"
)

_assert_isolated_domain() {
    if [[ "${ROS_DOMAIN_ID:-}" != "${TEST_ROS_DOMAIN_ID}" ]]; then
        echo "SAFE_TEST_RUNNER_ABORT_ROS_DOMAIN_ID_NOT_ISOLATED(got='${ROS_DOMAIN_ID:-<unset>}', expected='${TEST_ROS_DOMAIN_ID}')"
        exit 1
    fi
    echo "ros_domain_id_confirmed_isolated=${ROS_DOMAIN_ID}"
}

_assert_modules_collected() {
    local label="$1" output="$2"; shift 2
    local missing=()
    for module in "$@"; do
        if [[ "${output}" != *"${module}"* ]]; then
            missing+=("${module}")
        fi
    done
    if [[ ${#missing[@]} -gt 0 ]]; then
        echo "SAFE_TEST_RUNNER_ABORT_EXPECTED_TEST_NOT_COLLECTED(${label}): ${missing[*]}"
        return 1
    fi
    echo "all_expected_${label}_modules_collected: $*"
    return 0
}

echo "=== [1/8] Refusing to run if any physical/HIL process is detected ==="
if MATCHED="$(pgrep -af -- "${PHYSICAL_PATTERN}" 2>/dev/null)"; then
    echo "${MATCHED}"
    echo "SAFE_TEST_RUNNER_BLOCKED_PHYSICAL_PROCESS_DETECTED"
    exit 1
fi
echo "No matching physical/HIL process found."

echo ""
echo "=== [2/8] Real /cmd_vel publisher count, default domain, BEFORE ==="
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
echo "=== [3/8] Switching to isolated ROS_DOMAIN_ID=${TEST_ROS_DOMAIN_ID} ==="
export ROS_DOMAIN_ID="${TEST_ROS_DOMAIN_ID}"
_assert_isolated_domain

FAIL=0

echo ""
echo "=== [4/8] HIL unit test suite (includes command-evidence recorder, zero-publisher proofs, sync-logic tests) ==="
_assert_isolated_domain
HIL_OUTPUT="$(cd "${SCRIPT_DIR}" && python3 -m unittest discover -s . -p "test_*.py" -v 2>&1)"
echo "${HIL_OUTPUT}"
HIL_EXIT=$?
if ! _assert_modules_collected "HIL" "${HIL_OUTPUT}" "${REQUIRED_HIL_TEST_MODULES[@]}"; then
    FAIL=1
fi

echo ""
echo "=== [5/8] Pi command-audit test suite (pure logic, no rclpy) ==="
_assert_isolated_domain
PI_OUTPUT="$(cd "${PI_AUDIT_DIR}" && python3 -m unittest discover -s . -p "test_*.py" -v 2>&1)"
echo "${PI_OUTPUT}"
PI_EXIT=$?
if ! _assert_modules_collected "PI_AUDIT" "${PI_OUTPUT}" "${REQUIRED_PI_AUDIT_TEST_MODULES[@]}"; then
    FAIL=1
fi

echo ""
echo "=== [6/11] Synthetic sync/build script end-to-end test (never touches ~/epuck_ws) ==="
_assert_isolated_domain
E2E_OUTPUT="$(bash "${SCRIPT_DIR}/test_sync_and_build_epuck2_comm_e2e.sh" 2>&1)"
echo "${E2E_OUTPUT}"
E2E_EXIT=$?

echo ""
echo "=== [7/11] Command-evidence recorder end-to-end test (private test-only topics, never the real physical stack) ==="
_assert_isolated_domain
RECORDER_E2E_OUTPUT="$(bash "${SCRIPT_DIR}/test_run_hil_command_evidence_recorder_e2e.sh" 2>&1)"
echo "${RECORDER_E2E_OUTPUT}"
RECORDER_E2E_EXIT=$?

echo ""
echo "=== [8/11] Command-evidence recorder --output-root end-to-end test (private test-only topics, never the real physical stack) ==="
_assert_isolated_domain
RECORDER_OUTPUT_ROOT_E2E_OUTPUT="$(bash "${SCRIPT_DIR}/test_run_hil_command_evidence_recorder_output_root_e2e.sh" 2>&1)"
echo "${RECORDER_OUTPUT_ROOT_E2E_OUTPUT}"
RECORDER_OUTPUT_ROOT_E2E_EXIT=$?

echo ""
echo "=== [9/11] New-field GROUND_DIAGNOSTIC_PARAMS override end-to-end test (read-only, offline, never contacts the Pi) ==="
_assert_isolated_domain
NEW_FIELD_OVERRIDE_E2E_OUTPUT="$(bash "${SCRIPT_DIR}/test_run_ground_diagnostic_preflight_new_field_override_e2e.sh" 2>&1)"
echo "${NEW_FIELD_OVERRIDE_E2E_OUTPUT}"
NEW_FIELD_OVERRIDE_E2E_EXIT=$?

echo ""
echo "=== [10/11] colcon test: epuck2_comm, epuck2_comm_interfaces (isolated domain) ==="
_assert_isolated_domain
(cd ~/epuck_ws && colcon test --packages-select epuck2_comm epuck2_comm_interfaces --event-handlers console_direct+)
COLCON_EXIT=$?
(cd ~/epuck_ws && colcon test-result --verbose) || true

echo ""
echo "=== [11/11] Real /cmd_vel publisher count, default domain, AFTER ==="
unset ROS_DOMAIN_ID
AFTER="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || true)"
if [[ -z "${AFTER}" ]]; then
    AFTER="TOPIC_NOT_PRESENT"
fi
echo "real_cmd_vel_publisher_count_after=${AFTER}"

if [[ ${HIL_EXIT} -ne 0 ]]; then
    echo "HIL_SUITE_FAILED"
    FAIL=1
fi
if [[ ${PI_EXIT} -ne 0 ]]; then
    echo "PI_AUDIT_SUITE_FAILED"
    FAIL=1
fi
if [[ ${E2E_EXIT} -ne 0 ]]; then
    echo "SYNC_E2E_TEST_FAILED"
    FAIL=1
fi
if [[ ${RECORDER_E2E_EXIT} -ne 0 ]]; then
    echo "RECORDER_E2E_TEST_FAILED"
    FAIL=1
fi
if [[ ${RECORDER_OUTPUT_ROOT_E2E_EXIT} -ne 0 ]]; then
    echo "RECORDER_OUTPUT_ROOT_E2E_TEST_FAILED"
    FAIL=1
fi
if [[ ${NEW_FIELD_OVERRIDE_E2E_EXIT} -ne 0 ]]; then
    echo "NEW_FIELD_PARAMS_OVERRIDE_E2E_TEST_FAILED"
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
