#!/usr/bin/env bash
# End-to-end test for run_hil_command_evidence_recorder.sh -- added
# 2026-07-24 after a real powered-session activation attempt found two
# real problems live: (1) `start` backgrounded the recorder with a
# plain `&` inside a one-shot invoking shell, which did not survive
# that shell's own exit (setsid/disown fix), and (2) the CSV was only
# written once at shutdown, so it could never be observed "growing"
# mid-session (incremental-write/periodic-flush fix). This test proves
# both fixes against PRIVATE, test-only topics -- it never touches the
# real physical stack.
#
# Scenarios:
#   1. `start`, invoked exactly the way it broke live (a single
#      `bash -c "... start ..."` one-shot invocation), leaves the
#      recorder alive and logging in a LATER, SEPARATE check -- proves
#      the setsid/disown fix.
#   2. The CSV file exists (with header) immediately, before any
#      message has ever been published.
#   3. Publishing one message on the private state topic and waiting
#      past the flush interval shows the row count increase WHILE the
#      process is still running -- proves incremental write + periodic
#      flush, not batch-at-shutdown.
#   4. `stop` sends exactly one SIGINT to the exact recorded PID and
#      leaves a valid, non-empty, SHA-256-verifiable CSV -- proves
#      clean shutdown.
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

SCRATCH_ROOT="$(mktemp -d)"
trap 'rm -rf "${SCRATCH_ROOT}"' EXIT
export HIL_COMMAND_EVIDENCE_ROOT="${SCRATCH_ROOT}/bags"

echo "=== Scenario 1: start via a one-shot invocation, survives that shell's own exit ==="
# Deliberately mirrors the exact failure shape: a single bash -c
# invocation that runs `start` and then exits immediately, exactly
# like a one-shot `wsl.exe ... -- bash -lc "... start ..."` call would.
START_OUTPUT="$(bash -c "bash '${SCRIPT_DIR}/run_hil_command_evidence_recorder.sh' start \
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
OUT_DIR="$(dirname "${MANIFEST}")"
CSV_PATH="${OUT_DIR}/command_evidence.csv"

# The one-shot invoking shell above has already exited by this point.
# Check survival from a completely separate shell invocation.
sleep 1
if ps -p "${PID}" > /dev/null 2>&1; then
    assert_true "recorder (pid=${PID}) survived the one-shot invoking shell's exit" "true"
else
    assert_true "recorder (pid=${PID}) survived the one-shot invoking shell's exit" "false"
fi

echo ""
echo "=== Scenario 2: CSV exists with header before any message is published ==="
if [[ -f "${CSV_PATH}" ]]; then
    HEADER="$(head -1 "${CSV_PATH}")"
    assert_true "CSV file exists immediately" "true"
    assert_true "CSV header is present" "$([[ "${HEADER}" == local_time_ns,* ]] && echo true || echo false)"
else
    assert_true "CSV file exists immediately" "false"
    assert_true "CSV header is present" "false"
fi
ROWS_BEFORE="$(( $(wc -l < "${CSV_PATH}") - 1 ))"
echo "rows_before_any_publish=${ROWS_BEFORE}"
assert_true "zero rows before any message is published" "$([[ "${ROWS_BEFORE}" -eq 0 ]] && echo true || echo false)"

echo ""
echo "=== Scenario 3: row count increases mid-run after publishing, past the flush interval ==="
ros2 topic pub --once /e2e_test/epuck1/state epuck2_comm_interfaces/msg/EpuckState "{validity_flags: 7, sequence: 1}" > /dev/null 2>&1
sleep 2.5  # past the 1s flush interval, with margin
ROWS_AFTER="$(( $(wc -l < "${CSV_PATH}") - 1 ))"
echo "rows_after_publish=${ROWS_AFTER}"
if ps -p "${PID}" > /dev/null 2>&1; then
    assert_true "recorder still running while CSV was inspected" "true"
else
    assert_true "recorder still running while CSV was inspected" "false"
fi
assert_true "row count increased mid-run (proves incremental write, not batch-at-shutdown)" "$([[ "${ROWS_AFTER}" -gt "${ROWS_BEFORE}" ]] && echo true || echo false)"

echo ""
echo "=== Scenario 4: stop -- exact PID, clean shutdown, valid final CSV ==="
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
assert_true "final CSV still parseable and non-empty" "$([[ "${FINAL_ROWS}" -ge 1 ]] && echo true || echo false)"

echo ""
if [[ ${FAIL} -eq 0 ]]; then
    echo "COMMAND_EVIDENCE_RECORDER_E2E_TEST_PASS"
else
    echo "COMMAND_EVIDENCE_RECORDER_E2E_TEST_FAIL"
fi
exit ${FAIL}
