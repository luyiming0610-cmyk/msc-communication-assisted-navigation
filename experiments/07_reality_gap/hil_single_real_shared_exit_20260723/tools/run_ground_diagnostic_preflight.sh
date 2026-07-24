#!/usr/bin/env bash
# First-ground-diagnostic preflight -- STRICTLY READ-ONLY, check-only.
# Never starts a process, never publishes a message. Reuses existing
# tooling rather than duplicating it:
#   - hil_preflight.check_required_params_confirmed() (imported, not
#     reimplemented) against ground_diagnostic_params.json.
#   - run_hil_physical_preflight.sh, invoked as a read-only subprocess,
#     for device reachability / driver / bridge / validity_flags /
#     residual-process checks.
#
# This script is meant to be run at more than one point in
# GROUND_DIAGNOSTIC_RUNBOOK.md: early (source identity + measurements,
# before anything physical is started -- the evidence/guard checks
# below correctly report NOT_STARTED at that point) and again later,
# once the evidence recorders and guard are up, as the final gate
# immediately before arming. Every section degrades gracefully to
# NOT_CHECKED/NOT_STARTED rather than crashing when its subject isn't
# running yet -- only genuinely unsafe conditions (blocked params,
# multiple/wrong cmd_vel publishers, a residual test/controller
# process, a non-growing evidence file) produce a BLOCKED verdict.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/epuck_ws/install/setup.bash" 2>/dev/null || true
set -u

PARAMS_FILE="${GROUND_DIAGNOSTIC_PARAMS:-${SCRIPT_DIR}/ground_diagnostic_params.json}"
PI_JSONL_PATH="${GROUND_DIAGNOSTIC_PI_JSONL:-}"
WSL_CSV_PATH="${GROUND_DIAGNOSTIC_WSL_CSV:-}"
STATE_TOPIC="${HIL_STATE_TOPIC:-/epuck1/state}"

FAIL=0
echo "=== [1/8] Source/commit identity (read-only) ==="
if command -v git >/dev/null 2>&1 && [[ -d "${REPO_ROOT}/.git" ]]; then
    GIT_COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo UNKNOWN)"
    GIT_DIRTY="$(git -C "${REPO_ROOT}" status --porcelain -- experiments/07_reality_gap/hil_single_real_shared_exit_20260723 2>/dev/null || true)"
    echo "git_commit=${GIT_COMMIT}"
    if [[ -n "${GIT_DIRTY}" ]]; then
        echo "GIT_TREE_DIRTY_UNDER_HIL_STUDY:"
        echo "${GIT_DIRTY}"
        echo "SOURCE_IDENTITY=BLOCKED_UNCOMMITTED_CHANGES"
        FAIL=1
    else
        echo "SOURCE_IDENTITY=CLEAN(git_commit=${GIT_COMMIT})"
    fi
else
    echo "SOURCE_IDENTITY=NOT_CHECKED(not a git checkout)"
fi

echo ""
echo "=== [2/8] Required ground-diagnostic measurements confirmed ==="
if [[ ! -f "${PARAMS_FILE}" ]]; then
    echo "PARAMS_FILE_NOT_FOUND(${PARAMS_FILE})"
    echo "MEASUREMENTS_CHECK=BLOCKED"
    FAIL=1
else
    # PYTHONPATH (not a shell-interpolated sys.path.insert line) is used
    # so the heredoc's delimiter can stay single-quoted -- nothing from
    # ${SCRIPT_DIR} is ever interpolated into the embedded Python source.
    PARAM_CHECK_OUTPUT="$(PYTHONPATH="${SCRIPT_DIR}" python3 - "${PARAMS_FILE}" <<'PYEOF'
import json
import sys

from hil_preflight import check_required_params_confirmed

with open(sys.argv[1], encoding="utf-8") as fh:
    params = json.load(fh)
result = check_required_params_confirmed(params)
if result.ok:
    print("MEASUREMENTS_CHECK=PASS")
else:
    print("MEASUREMENTS_CHECK=BLOCKED")
    if result.missing_paths:
        print(f"MISSING_PATHS={list(result.missing_paths)}")
    if result.unconfirmed_paths:
        print(f"UNCONFIRMED_PATHS={list(result.unconfirmed_paths)}")
PYEOF
)"
    echo "${PARAM_CHECK_OUTPUT}"
    if [[ "${PARAM_CHECK_OUTPUT}" != *"MEASUREMENTS_CHECK=PASS"* ]]; then
        FAIL=1
    fi
fi

echo ""
echo "=== [3/8] New, non-overwriting evidence paths ==="
if [[ -n "${PI_JSONL_PATH}" ]]; then
    if [[ -e "${PI_JSONL_PATH}" ]]; then
        echo "PI_JSONL_PATH_ALREADY_EXISTS(${PI_JSONL_PATH})"
        FAIL=1
    else
        echo "PI_JSONL_PATH_AVAILABLE(${PI_JSONL_PATH})"
    fi
else
    echo "PI_JSONL_PATH=NOT_PROVIDED(set GROUND_DIAGNOSTIC_PI_JSONL to check)"
fi
if [[ -n "${WSL_CSV_PATH}" ]]; then
    if [[ -e "${WSL_CSV_PATH}" ]]; then
        echo "WSL_CSV_PATH_ALREADY_EXISTS(${WSL_CSV_PATH})"
        FAIL=1
    else
        echo "WSL_CSV_PATH_AVAILABLE(${WSL_CSV_PATH})"
    fi
else
    echo "WSL_CSV_PATH=NOT_PROVIDED(set GROUND_DIAGNOSTIC_WSL_CSV to check)"
fi

echo ""
echo "=== [4/8] Physical state topic + validity_flags (reusing run_hil_physical_preflight.sh) ==="
PHYSICAL_PREFLIGHT_OUTPUT="$(bash "${SCRIPT_DIR}/run_hil_physical_preflight.sh" 2>&1)"
echo "${PHYSICAL_PREFLIGHT_OUTPUT}"
if [[ "${PHYSICAL_PREFLIGHT_OUTPUT}" != *"PHYSICAL_READ_ONLY_PREFLIGHT_DONE"* ]]; then
    echo "PHYSICAL_PREFLIGHT=BLOCKED_OR_DEVICE_UNREACHABLE"
    FAIL=1
elif [[ "${PHYSICAL_PREFLIGHT_OUTPUT}" != *"VALIDITY_FLAGS=7 "* ]]; then
    echo "VALIDITY_FLAGS_NOT_7_OR_NOT_AVAILABLE"
    FAIL=1
else
    echo "PHYSICAL_PREFLIGHT_AND_VALIDITY_FLAGS=PASS"
fi

echo ""
echo "=== [5/8] Pi and WSL evidence streams active and growing ==="
if [[ -n "${PI_JSONL_PATH}" && -f "${PI_JSONL_PATH}" ]]; then
    N1="$(wc -l < "${PI_JSONL_PATH}" 2>/dev/null || echo 0)"
    sleep 1
    N2="$(wc -l < "${PI_JSONL_PATH}" 2>/dev/null || echo 0)"
    if [[ "${N2}" -gt "${N1}" ]]; then
        echo "PI_JSONL_GROWING(${N1}->${N2})"
    else
        echo "PI_JSONL_NOT_GROWING(${N1}->${N2})"
        FAIL=1
    fi
else
    echo "PI_JSONL=NOT_STARTED_OR_NOT_PROVIDED"
fi
if [[ -n "${WSL_CSV_PATH}" && -f "${WSL_CSV_PATH}" ]]; then
    N1="$(wc -l < "${WSL_CSV_PATH}" 2>/dev/null || echo 0)"
    sleep 1
    N2="$(wc -l < "${WSL_CSV_PATH}" 2>/dev/null || echo 0)"
    if [[ "${N2}" -gt "${N1}" ]]; then
        echo "WSL_CSV_GROWING(${N1}->${N2})"
    else
        echo "WSL_CSV_NOT_GROWING(${N1}->${N2})"
        FAIL=1
    fi
else
    echo "WSL_CSV=NOT_STARTED_OR_NOT_PROVIDED"
fi

echo ""
echo "=== [6/8] Guard state ==="
GUARD_PID="$(pgrep -af hil_cmd_vel_guard 2>/dev/null | grep -v 'pgrep -af' || true)"
if [[ -z "${GUARD_PID}" ]]; then
    echo "GUARD=NOT_STARTED"
else
    echo "GUARD_PROCESS_FOUND: ${GUARD_PID}"
    echo "GUARD_ARMED_STATE=NOT_INDEPENDENTLY_VERIFIABLE_VIA_PASSIVE_TOPIC_READ (/hil_guard/arm is not a latched/transient-local topic -- a passive echo would simply hang with no information either way; disarmed-by-default is guaranteed by hil_cmd_vel_guard.py's own tested default (test_disarmed_by_default_produces_zero) and by procedural discipline -- never publish arm=true until the runbook's explicit arm step). Checking the observable consequence instead:"
fi

echo ""
echo "=== [7/8] /cmd_vel and /cmd_vel_unguarded: sole publisher, zero output, no duplicate/test publisher ==="
if command -v ros2 >/dev/null 2>&1; then
    for TOPIC in /cmd_vel cmd_vel_unguarded; do
        PUB_COUNT="$(ros2 topic info "${TOPIC}" 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo "")"
        echo "PUBLISHER_COUNT[${TOPIC}]=${PUB_COUNT:-TOPIC_NOT_PRESENT}"
        if [[ -n "${PUB_COUNT}" && "${PUB_COUNT}" != "0" && "${PUB_COUNT}" != "1" ]]; then
            echo "DUPLICATE_OR_UNEXPECTED_PUBLISHER_COUNT[${TOPIC}]=${PUB_COUNT}"
            FAIL=1
        fi
        if [[ -n "${PUB_COUNT}" && "${PUB_COUNT}" != "0" ]]; then
            VALUE="$(timeout 3 ros2 topic echo "${TOPIC}" --once 2>/dev/null || true)"
            if [[ "${VALUE}" == *"x: 0.0"* || -z "${VALUE}" ]]; then
                echo "OUTPUT_ZERO_CHECK[${TOPIC}]=PASS_OR_NOT_AVAILABLE"
            else
                echo "OUTPUT_ZERO_CHECK[${TOPIC}]=NONZERO_VALUE_DETECTED"
                echo "${VALUE}"
                FAIL=1
            fi
        fi
    done
else
    echo "CMD_VEL_CHECKS=NOT_CHECKED(ros2 CLI not on PATH)"
fi

echo ""
echo "=== [8/8] No controller/virtual-peer/navigator/Webots/pytest/colcon/rosbag process ==="
FORBIDDEN_PATTERN='cooperative_avoider|hil_virtual_peer|hil_topic_adapter|goal_navigator|webots|pytest|colcon|rosbag|hil_wheel_suspension_test|hil_angular_suspension_test'
RESIDUAL="$(pgrep -af -- "${FORBIDDEN_PATTERN}" 2>/dev/null | grep -v 'pgrep -af' || true)"
if [[ -n "${RESIDUAL}" ]]; then
    echo "FORBIDDEN_PROCESS_FOUND:"
    echo "${RESIDUAL}"
    FAIL=1
else
    echo "FORBIDDEN_PROCESS_CHECK=CLEAN"
fi

echo ""
if [[ ${FAIL} -eq 0 ]]; then
    echo "GROUND_DIAGNOSTIC_PREFLIGHT_PASS"
else
    echo "GROUND_DIAGNOSTIC_PREFLIGHT_BLOCKED"
fi
exit ${FAIL}
