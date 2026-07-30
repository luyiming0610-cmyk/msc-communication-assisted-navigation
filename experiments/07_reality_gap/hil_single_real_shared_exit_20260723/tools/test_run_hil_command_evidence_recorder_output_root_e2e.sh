#!/usr/bin/env bash
# End-to-end test for run_hil_command_evidence_recorder.sh's
# --output-root fix -- added 2026-07-27 after SRGRB_20260727 Trial 1
# Attempt 1 was EXCLUDED live: the wrapper always built its own
# --output-csv from an auto-generated timestamped directory and
# prepended it ahead of any caller-supplied extra args, so a caller
# passing its own --output-csv (intending to set the frozen evidence
# path) silently won inside the recorder's own argparse (last-value-
# wins) while the wrapper's manifest/log/reported paths kept referring
# to its own, different, auto-created directory. The recorder crashed
# with FileNotFoundError because the caller's intended directory was
# never created.
#
# Uses only PRIVATE, test-only topics (/e2e_test/...) -- never touches
# the real physical stack. Requires ROS_DOMAIN_ID to already be set to
# the sanctioned isolated test domain (asserted below) before this
# script is ever invoked -- run_isolated_test_suite.sh is the only
# sanctioned caller.
#
# Scenarios:
#   1. --output-root <frozen dir> is honored exactly: the frozen
#      directory (never pre-created by this test) is created by the
#      wrapper itself, the recorder stays alive, and the CSV/manifest/
#      log paths it reports all agree with that exact directory.
#   2. The CSV at the frozen root grows mid-run (proves the recorder
#      is actually writing there, not to some other path).
#   3. `stop` via the manifest and exact PID leaves a valid,
#      SHA-256-verifiable CSV at the frozen root.
#   4. A bare --output-csv is REJECTED outright (nonzero exit, no
#      process started) -- the exact collision this fix closes.
#   5. A duplicate --output-root is also REJECTED outright.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash
set -u

FAIL=0
assert_true() {
    local desc="$1" cond="$2"
    if [[ "${cond}" == "true" ]]; then
        echo "PASS: ${desc}"
    else
        echo "FAIL: ${desc}"
        FAIL=1
    fi
}

# 90, matching run_isolated_test_suite.sh's own TEST_ROS_DOMAIN_ID
# (changed from 89 on 2026-07-30: 89 collides with
# test_hil_offline_stage3_harness_live.py's own
# FORBIDDEN_ROS_DOMAIN_IDS safety guard -- see run_isolated_test_suite.sh
# for the full explanation). This script only asserts isolation from the
# default/production domain, so it must track whatever isolated domain
# the sanctioned runner actually uses, not a hardcoded historical value.
if [[ "${ROS_DOMAIN_ID:-}" != "90" ]]; then
    echo "OUTPUT_ROOT_E2E_TEST_ABORT_ROS_DOMAIN_ID_NOT_ISOLATED(got='${ROS_DOMAIN_ID:-<unset>}', expected='90')"
    exit 1
fi
echo "ros_domain_id_confirmed_isolated=${ROS_DOMAIN_ID}"

SCRATCH_ROOT="$(mktemp -d)"
trap 'rm -rf "${SCRATCH_ROOT}"' EXIT

FROZEN_ROOT="${SCRATCH_ROOT}/srgrb_e2e_test_trial1_attempt1_frozen_root"

echo "=== Scenario 1: --output-root is honored exactly, directory created by the wrapper ==="
if [[ -e "${FROZEN_ROOT}" ]]; then
    echo "FAIL: frozen root already exists before the test even starts -- test fixture bug"
    exit 1
fi

START_OUTPUT="$(bash -c "bash '${SCRIPT_DIR}/run_hil_command_evidence_recorder.sh' start \
    --output-root '${FROZEN_ROOT}' \
    --upstream-cmd-vel-topic /e2e_test/cmd_vel_unguarded \
    --guarded-cmd-vel-topic /e2e_test/cmd_vel \
    --arm-topic /e2e_test/hil_guard/arm \
    --state-topic /e2e_test/epuck1/state \
    --bridge-status-topic /e2e_test/epuck_bridge/status \
    --flush-interval-s 1 \
    --duration-s 60")"
echo "${START_OUTPUT}"
PID="$(echo "${START_OUTPUT}" | grep -oP 'pid=\K[0-9]+')"
MANIFEST="$(echo "${START_OUTPUT}" | grep -oP 'manifest=\K\S+')"
REPORTED_OUT_DIR="$(echo "${START_OUTPUT}" | grep -oP 'output_dir=\K\S+')"
CSV_PATH="${FROZEN_ROOT}/command_evidence.csv"
LOG_PATH="${FROZEN_ROOT}/recorder.log"

assert_true "frozen root directory was created exactly as given" "$([[ -d "${FROZEN_ROOT}" ]] && echo true || echo false)"
assert_true "reported output_dir matches the frozen root exactly" "$([[ "${REPORTED_OUT_DIR}" == "${FROZEN_ROOT}" ]] && echo true || echo false)"
assert_true "manifest is inside the frozen root" "$([[ "${MANIFEST}" == "${FROZEN_ROOT}/manifest.json" ]] && echo true || echo false)"

sleep 1
if ps -p "${PID}" > /dev/null 2>&1; then
    assert_true "recorder (pid=${PID}) is alive after start" "true"
else
    assert_true "recorder (pid=${PID}) is alive after start" "false"
fi

MANIFEST_CSV_PATH="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['csv_path'])" "${MANIFEST}")"
MANIFEST_LOG_PATH="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['log_path'])" "${MANIFEST}")"
assert_true "manifest csv_path agrees with the frozen root" "$([[ "${MANIFEST_CSV_PATH}" == "${CSV_PATH}" ]] && echo true || echo false)"
assert_true "manifest log_path agrees with the frozen root" "$([[ "${MANIFEST_LOG_PATH}" == "${LOG_PATH}" ]] && echo true || echo false)"
assert_true "CSV exists exactly at the frozen root" "$([[ -f "${CSV_PATH}" ]] && echo true || echo false)"
assert_true "recorder.log exists exactly at the frozen root" "$([[ -f "${LOG_PATH}" ]] && echo true || echo false)"

ROWS_BEFORE="$(( $(wc -l < "${CSV_PATH}") - 1 ))"
echo "rows_before_any_publish=${ROWS_BEFORE}"

echo ""
echo "=== Scenario 2: CSV at the frozen root grows mid-run ==="
ros2 topic pub --once /e2e_test/epuck1/state epuck2_comm_interfaces/msg/EpuckState "{validity_flags: 7, sequence: 1}" > /dev/null 2>&1
sleep 2.5
ROWS_AFTER="$(( $(wc -l < "${CSV_PATH}") - 1 ))"
echo "rows_after_publish=${ROWS_AFTER}"
assert_true "row count increased at the frozen root (proves the recorder wrote there, not elsewhere)" "$([[ "${ROWS_AFTER}" -gt "${ROWS_BEFORE}" ]] && echo true || echo false)"

echo ""
echo "=== Scenario 3: stop via manifest and exact PID, valid final CSV at the frozen root ==="
STOP_OUTPUT="$(bash "${SCRIPT_DIR}/run_hil_command_evidence_recorder.sh" stop "${MANIFEST}")"
echo "${STOP_OUTPUT}"
assert_true "stop reports STOPPED" "$([[ "${STOP_OUTPUT}" == *"status=STOPPED"* ]] && echo true || echo false)"
assert_true "stop reports a sha256 for the final CSV" "$([[ "${STOP_OUTPUT}" == *"sha256="* ]] && echo true || echo false)"
if ps -p "${PID}" > /dev/null 2>&1; then
    assert_true "process is gone after stop" "false"
else
    assert_true "process is gone after stop" "true"
fi
FINAL_ROWS="$(( $(wc -l < "${CSV_PATH}") - 1 ))"
echo "final_rows=${FINAL_ROWS}"
assert_true "final CSV at the frozen root still parseable and non-empty" "$([[ "${FINAL_ROWS}" -ge 1 ]] && echo true || echo false)"

echo ""
echo "=== Scenario 4: a bare --output-csv is rejected outright, no process started ==="
set +e
REJECT_OUTPUT="$(bash "${SCRIPT_DIR}/run_hil_command_evidence_recorder.sh" start \
    --output-csv "${SCRATCH_ROOT}/should_never_be_created/command_evidence.csv" \
    --upstream-cmd-vel-topic /e2e_test/cmd_vel_unguarded \
    --guarded-cmd-vel-topic /e2e_test/cmd_vel \
    --arm-topic /e2e_test/hil_guard/arm \
    --state-topic /e2e_test/epuck1/state \
    --bridge-status-topic /e2e_test/epuck_bridge/status \
    --duration-s 5 2>&1)"
REJECT_EXIT=$?
set -e
echo "${REJECT_OUTPUT}"
assert_true "--output-csv is rejected with a nonzero exit code" "$([[ ${REJECT_EXIT} -ne 0 ]] && echo true || echo false)"
assert_true "rejection message names --output-csv" "$([[ "${REJECT_OUTPUT}" == *"HIL_COMMAND_EVIDENCE_RECORDER_REJECTED"* ]] && echo true || echo false)"
assert_true "the rejected directory was never created" "$([[ ! -e "${SCRATCH_ROOT}/should_never_be_created" ]] && echo true || echo false)"

echo ""
echo "=== Scenario 5: a duplicate --output-root is also rejected outright ==="
set +e
DUP_REJECT_OUTPUT="$(bash "${SCRIPT_DIR}/run_hil_command_evidence_recorder.sh" start \
    --output-root "${SCRATCH_ROOT}/dup_one" \
    --output-root "${SCRATCH_ROOT}/dup_two" \
    --upstream-cmd-vel-topic /e2e_test/cmd_vel_unguarded \
    --guarded-cmd-vel-topic /e2e_test/cmd_vel \
    --arm-topic /e2e_test/hil_guard/arm \
    --state-topic /e2e_test/epuck1/state \
    --bridge-status-topic /e2e_test/epuck_bridge/status \
    --duration-s 5 2>&1)"
DUP_REJECT_EXIT=$?
set -e
echo "${DUP_REJECT_OUTPUT}"
assert_true "duplicate --output-root is rejected with a nonzero exit code" "$([[ ${DUP_REJECT_EXIT} -ne 0 ]] && echo true || echo false)"
assert_true "rejection message names --output-root" "$([[ "${DUP_REJECT_OUTPUT}" == *"--output-root given more than once"* ]] && echo true || echo false)"

echo ""
if [[ ${FAIL} -eq 0 ]]; then
    echo "COMMAND_EVIDENCE_RECORDER_OUTPUT_ROOT_E2E_TEST_PASS"
else
    echo "COMMAND_EVIDENCE_RECORDER_OUTPUT_ROOT_E2E_TEST_FAIL"
fi
exit ${FAIL}
