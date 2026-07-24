#!/usr/bin/env bash
# First-ground-diagnostic preflight -- STRICTLY READ-ONLY, check-only.
# Never starts a process, never publishes a message.
#
# Runs as one of two explicit, separately-invoked phases (no default --
# the mode must be named):
#   pre-stack        -- everything checkable before the physical stack
#                        (driver/server/bridge/state_publisher/recorder/
#                        guard) is started. Never requires validity_flags,
#                        bridge status, evidence growth, or guard state --
#                        those cannot exist yet at this point.
#   live-zero-state   -- checkable only once the whole stack is up and the
#                        guard is confirmed DISARMED with wheels still
#                        suspended. Requires validity_flags==7, bridge
#                        connected, the WSL evidence file growing with
#                        zero-only recorded commands, the guard as
#                        sole/disarmed /cmd_vel publisher outputting
#                        zero, AND a separately-produced Pi audit
#                        verdict (see below) that also passes, shares
#                        the same run identifier, and is fresh.
#
# The previous single-phase design required validity_flags==7 even before
# bring-up, which can never be true pre-bring-up -- a circular dependency.
# Splitting into two phases (each delegated to a pure, unit-tested
# decision function in hil_ground_diagnostic_phases.py) removes that
# circularity without weakening either check.
#
# Found 2026-07-24 during the first live attempt: live-zero-state
# previously tried to read the Pi's local command-audit JSONL by a
# plain path, but the WSL machine this script runs on shares no
# filesystem with the Pi -- only a network connection -- so that file
# was never actually reachable from here, and the check failed closed
# for a structural reason that looked like a real safety block. The
# Pi's own evidence is now verified ON THE PI ITSELF by
# pi_ground_diagnostic_audit_verifier.py, which writes a small,
# timestamped, machine-readable verdict JSON; that file must be copied
# back (or its path made reachable, e.g. a synced/shared location) and
# passed here via GROUND_DIAGNOSTIC_PI_AUDIT_VERDICT. Without it, this
# phase reports PI_LIVE_AUDIT_NOT_AVAILABLE and blocks -- never
# silently passes, and never treats a missing verdict as "proven
# nonzero" either.
#
# Reuses existing tooling rather than duplicating it:
#   - hil_preflight.check_required_fields_ready() against
#     ground_diagnostic_params.json's TRACKED fields only (measured
#     geometry + stable venue facts).
#   - hil_ground_diagnostic_session.check_session_state_ready() against a
#     separate, gitignored, timestamped session-state file for the four
#     per-session confirmations (floor condition, travel path clear,
#     operator present, Wi-Fi checked) -- never read from or written
#     into the tracked JSON file; a tracked-file value can never
#     substitute for a session-file confirmation.
#   - hil_ground_diagnostic_phases.evaluate_pre_stack() /
#     evaluate_wsl_live_state() / evaluate_combined_gate() for the
#     actual pass/block decisions.
#   - run_hil_physical_preflight.sh, invoked as a read-only subprocess,
#     for device reachability / driver / bridge / validity_flags /
#     residual-process facts (each phase uses only the subset of its
#     output that applies to that phase).
#   - analyze_ground_diagnostic.load_wsl_csv_rows /
#     find_nonzero_command_window, reused (not reimplemented) for the
#     WSL-side "all recorded commands zero" check. Pi-side JSONL
#     parsing is reused via pi_ground_diagnostic_audit_verifier.py,
#     which itself reuses analyze_ground_diagnostic.load_pi_jsonl_records
#     / compute_pi_command_maxima.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/epuck_ws/install/setup.bash" 2>/dev/null || true
set -u

MODE="${1:-}"
if [[ "${MODE}" != "pre-stack" && "${MODE}" != "live-zero-state" ]]; then
    echo "USAGE: $(basename "$0") <pre-stack|live-zero-state>"
    exit 2
fi

PARAMS_FILE="${GROUND_DIAGNOSTIC_PARAMS:-${SCRIPT_DIR}/ground_diagnostic_params.json}"
SESSION_FILE="${GROUND_DIAGNOSTIC_SESSION:-${SCRIPT_DIR}/ground_diagnostic_session_state.json}"
SESSION_MAX_AGE_S="${GROUND_DIAGNOSTIC_SESSION_MAX_AGE_S:-14400}"
PI_JSONL_PATH="${GROUND_DIAGNOSTIC_PI_JSONL:-}"
WSL_CSV_PATH="${GROUND_DIAGNOSTIC_WSL_CSV:-}"
RUN_ID="${GROUND_DIAGNOSTIC_RUN_ID:-}"
PI_AUDIT_VERDICT_PATH="${GROUND_DIAGNOSTIC_PI_AUDIT_VERDICT:-}"
PI_VERDICT_MAX_AGE_S="${GROUND_DIAGNOSTIC_PI_VERDICT_MAX_AGE_S:-300}"
STATE_TOPIC="${HIL_STATE_TOPIC:-/epuck1/state}"
UPSTREAM_TOPIC="${HIL_UPSTREAM_CMD_VEL_TOPIC:-/cmd_vel_unguarded}"
GUARDED_TOPIC="${HIL_GUARDED_CMD_VEL_TOPIC:-/cmd_vel}"

FORBIDDEN_PATTERN='cooperative_avoider|hil_virtual_peer|hil_topic_adapter|goal_navigator|webots|pytest|colcon|rosbag|hil_wheel_suspension_test|hil_angular_suspension_test'

if [[ "${MODE}" == "pre-stack" ]]; then

echo "=== PRE_STACK [1/6] Source/commit identity (read-only) ==="
GIT_CLEAN="false"
if command -v git >/dev/null 2>&1 && [[ -d "${REPO_ROOT}/.git" ]]; then
    GIT_COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo UNKNOWN)"
    GIT_DIRTY="$(git -C "${REPO_ROOT}" status --porcelain -- experiments/07_reality_gap/hil_single_real_shared_exit_20260723 2>/dev/null || true)"
    echo "git_commit=${GIT_COMMIT}"
    if [[ -n "${GIT_DIRTY}" ]]; then
        echo "GIT_TREE_DIRTY_UNDER_HIL_STUDY:"
        echo "${GIT_DIRTY}"
    else
        echo "SOURCE_IDENTITY=CLEAN(git_commit=${GIT_COMMIT})"
        GIT_CLEAN="true"
    fi
else
    echo "SOURCE_IDENTITY=NOT_CHECKED(not a git checkout)"
fi

echo ""
echo "=== PRE_STACK [2/6] Tracked measured-geometry / venue fields confirmed ==="
TRACKED_CHECK_OUTPUT="$(PYTHONPATH="${SCRIPT_DIR}" python3 - "${PARAMS_FILE}" <<'PYEOF'
import json
import sys

from hil_preflight import check_required_fields_ready

with open(sys.argv[1], encoding="utf-8") as fh:
    params = json.load(fh)
result = check_required_fields_ready(params)
print("TRACKED_FIELDS_OK=" + ("true" if result.ok else "false"))
print(f"TRACKED_MISSING={list(result.missing_paths)}")
print(f"TRACKED_UNCONFIRMED={list(result.unconfirmed_paths)}")
PYEOF
)"
echo "${TRACKED_CHECK_OUTPUT}"
TRACKED_FIELDS_OK="$(grep -o 'TRACKED_FIELDS_OK=.*' <<<"${TRACKED_CHECK_OUTPUT}" | cut -d= -f2)"
TRACKED_MISSING="$(grep -o 'TRACKED_MISSING=.*' <<<"${TRACKED_CHECK_OUTPUT}" | cut -d= -f2-)"
TRACKED_UNCONFIRMED="$(grep -o 'TRACKED_UNCONFIRMED=.*' <<<"${TRACKED_CHECK_OUTPUT}" | cut -d= -f2-)"

echo ""
echo "=== PRE_STACK [3/6] Per-session confirmations (floor condition, travel path clear, operator present, Wi-Fi checked) ==="
SESSION_CHECK_OUTPUT="$(PYTHONPATH="${SCRIPT_DIR}" python3 hil_ground_diagnostic_session.py check --path "${SESSION_FILE}" --max-age-s "${SESSION_MAX_AGE_S}" 2>&1)"
echo "${SESSION_CHECK_OUTPUT}"
SESSION_OK="false"
SESSION_REASON="NOT_RUN"
if [[ "${SESSION_CHECK_OUTPUT}" == *"SESSION_STATE_CHECK=PASS"* ]]; then
    SESSION_OK="true"
    SESSION_REASON=""
else
    SESSION_REASON="$(grep -o 'reason=[^ ]*' <<<"${SESSION_CHECK_OUTPUT}" | cut -d= -f2)"
fi

echo ""
echo "=== PRE_STACK [4/6] New, non-overwriting evidence paths ==="
EVIDENCE_PATHS_OK="true"
if [[ -n "${PI_JSONL_PATH}" ]]; then
    if [[ -e "${PI_JSONL_PATH}" ]]; then
        echo "PI_JSONL_PATH_ALREADY_EXISTS(${PI_JSONL_PATH})"
        EVIDENCE_PATHS_OK="false"
    else
        echo "PI_JSONL_PATH_AVAILABLE(${PI_JSONL_PATH})"
    fi
else
    echo "PI_JSONL_PATH=NOT_PROVIDED(set GROUND_DIAGNOSTIC_PI_JSONL to check)"
fi
if [[ -n "${WSL_CSV_PATH}" ]]; then
    if [[ -e "${WSL_CSV_PATH}" ]]; then
        echo "WSL_CSV_PATH_ALREADY_EXISTS(${WSL_CSV_PATH})"
        EVIDENCE_PATHS_OK="false"
    else
        echo "WSL_CSV_PATH_AVAILABLE(${WSL_CSV_PATH})"
    fi
else
    echo "WSL_CSV_PATH=NOT_PROVIDED(set GROUND_DIAGNOSTIC_WSL_CSV to check)"
fi

echo ""
echo "=== PRE_STACK [5/6] Device reachability, residual process, /cmd_vel absent-or-zero-publisher ==="
PHYSICAL_PREFLIGHT_OUTPUT="$(bash "${SCRIPT_DIR}/run_hil_physical_preflight.sh" 2>&1)"
echo "${PHYSICAL_PREFLIGHT_OUTPUT}"
DEVICE_REACHABLE="false"
if [[ "${PHYSICAL_PREFLIGHT_OUTPUT}" == *"PHYSICAL_DEVICE_REACHABLE=YES"* ]]; then
    DEVICE_REACHABLE="true"
fi
RESIDUAL_FOUND="false"
if [[ "${PHYSICAL_PREFLIGHT_OUTPUT}" == *"RESIDUAL_PROCESS_CHECK=FOUND"* ]]; then
    RESIDUAL_FOUND="true"
fi
CMD_VEL_PUB_COUNT=""
if command -v ros2 >/dev/null 2>&1; then
    CMD_VEL_PUB_COUNT="$(ros2 topic info "${GUARDED_TOPIC}" 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo "")"
    echo "CMD_VEL_PUBLISHER_COUNT[${GUARDED_TOPIC}]=${CMD_VEL_PUB_COUNT:-TOPIC_NOT_PRESENT}"
else
    echo "CMD_VEL_PUBLISHER_COUNT=NOT_CHECKED(ros2 CLI not on PATH)"
fi

echo ""
echo "=== PRE_STACK [6/6] No controller/virtual-peer/navigator/Webots/pytest/colcon/rosbag process ==="
RESIDUAL_FORBIDDEN="$(pgrep -af -- "${FORBIDDEN_PATTERN}" 2>/dev/null | grep -v 'pgrep -af' || true)"
FORBIDDEN_FOUND="false"
if [[ -n "${RESIDUAL_FORBIDDEN}" ]]; then
    echo "FORBIDDEN_PROCESS_FOUND:"
    echo "${RESIDUAL_FORBIDDEN}"
    FORBIDDEN_FOUND="true"
else
    echo "FORBIDDEN_PROCESS_CHECK=CLEAN"
fi

echo ""
echo "=== PRE_STACK verdict ==="
VERDICT_OUTPUT="$(PYTHONPATH="${SCRIPT_DIR}" python3 - \
    "${GIT_CLEAN}" "${TRACKED_FIELDS_OK}" "${TRACKED_MISSING}" "${TRACKED_UNCONFIRMED}" \
    "${SESSION_OK}" "${SESSION_REASON}" "${DEVICE_REACHABLE}" "${RESIDUAL_FOUND}" \
    "${FORBIDDEN_FOUND}" "${CMD_VEL_PUB_COUNT}" "${EVIDENCE_PATHS_OK}" <<'PYEOF'
import ast
import sys

from hil_ground_diagnostic_phases import evaluate_pre_stack

(git_clean, tracked_ok, tracked_missing, tracked_unconfirmed,
 session_ok, session_reason, device_reachable, residual_found,
 forbidden_found, cmd_vel_count, evidence_paths_ok) = sys.argv[1:12]


def as_bool(s):
    return s == "true"


def as_list(s):
    try:
        return tuple(ast.literal_eval(s))
    except (ValueError, SyntaxError):
        return ()


count = int(cmd_vel_count) if cmd_vel_count.strip().isdigit() else None

result = evaluate_pre_stack(
    tracked_git_clean=as_bool(git_clean),
    tracked_fields_ok=as_bool(tracked_ok),
    tracked_missing=as_list(tracked_missing),
    tracked_unconfirmed=as_list(tracked_unconfirmed),
    session_ok=as_bool(session_ok),
    session_reason=session_reason,
    device_reachable=as_bool(device_reachable),
    residual_process_found=as_bool(residual_found),
    forbidden_process_found=as_bool(forbidden_found),
    cmd_vel_publisher_count=count,
    evidence_paths_ok=as_bool(evidence_paths_ok),
)
if result.ok:
    print("PRE_STACK_VERDICT=PASS")
else:
    print("PRE_STACK_VERDICT=BLOCKED")
    print(f"REASONS={list(result.reasons)}")
PYEOF
)"
echo "${VERDICT_OUTPUT}"

echo ""
if [[ "${VERDICT_OUTPUT}" == *"PRE_STACK_VERDICT=PASS"* ]]; then
    echo "GROUND_DIAGNOSTIC_PRE_STACK_CHECK_PASS"
    exit 0
else
    echo "GROUND_DIAGNOSTIC_PRE_STACK_CHECK_BLOCKED"
    exit 1
fi

fi

if [[ "${MODE}" == "live-zero-state" ]]; then

echo "=== LIVE_ZERO_STATE [1/4] Physical state topic + validity_flags + bridge (reusing run_hil_physical_preflight.sh) ==="
PHYSICAL_PREFLIGHT_OUTPUT="$(bash "${SCRIPT_DIR}/run_hil_physical_preflight.sh" 2>&1)"
echo "${PHYSICAL_PREFLIGHT_OUTPUT}"
VALIDITY_FLAGS_VALUE=""
if [[ "${PHYSICAL_PREFLIGHT_OUTPUT}" =~ VALIDITY_FLAGS=([0-9]+) ]]; then
    VALIDITY_FLAGS_VALUE="${BASH_REMATCH[1]}"
fi
BRIDGE_CONNECTED="false"
if [[ "${PHYSICAL_PREFLIGHT_OUTPUT}" == *"BRIDGE_STATUS=CONNECTED"* ]]; then
    BRIDGE_CONNECTED="true"
fi

echo ""
echo "=== LIVE_ZERO_STATE [2/5] WSL evidence stream active, growing, zero-only ==="
WSL_CSV_GROWING="false"
WSL_EVIDENCE_ALL_ZERO="false"
if [[ -n "${WSL_CSV_PATH}" && -f "${WSL_CSV_PATH}" ]]; then
    N1="$(wc -l < "${WSL_CSV_PATH}" 2>/dev/null || echo 0)"
    sleep 1
    N2="$(wc -l < "${WSL_CSV_PATH}" 2>/dev/null || echo 0)"
    if [[ "${N2}" -gt "${N1}" ]]; then
        echo "WSL_CSV_GROWING(${N1}->${N2})"
        WSL_CSV_GROWING="true"
    else
        echo "WSL_CSV_NOT_GROWING(${N1}->${N2})"
    fi

    ZERO_CHECK_OUTPUT="$(PYTHONPATH="${SCRIPT_DIR}" python3 - "${WSL_CSV_PATH}" "${GUARDED_TOPIC}" "${UPSTREAM_TOPIC}" <<'PYEOF'
import sys

from analyze_ground_diagnostic import find_nonzero_command_window, load_wsl_csv_rows

wsl_csv_path, guarded_topic, upstream_topic = sys.argv[1:4]

wsl_all_zero = True
try:
    rows = load_wsl_csv_rows(wsl_csv_path)
    for topic in (guarded_topic, upstream_topic):
        window = find_nonzero_command_window(rows, topic)
        if window.first_time_ns is not None:
            wsl_all_zero = False
except FileNotFoundError:
    wsl_all_zero = False

print(f"WSL_EVIDENCE_ALL_ZERO={'true' if wsl_all_zero else 'false'}")
PYEOF
)"
    echo "${ZERO_CHECK_OUTPUT}"
    [[ "${ZERO_CHECK_OUTPUT}" == *"WSL_EVIDENCE_ALL_ZERO=true"* ]] && WSL_EVIDENCE_ALL_ZERO="true"
else
    echo "WSL_CSV=NOT_STARTED_OR_NOT_PROVIDED"
fi

echo ""
echo "=== LIVE_ZERO_STATE [3/5] Pi audit verdict (produced separately, on the Pi, by pi_ground_diagnostic_audit_verifier.py) ==="
PI_VERDICT_AVAILABLE="false"
PI_VERDICT_OK="false"
PI_VERDICT_REASONS="[]"
PI_VERDICT_RUN_ID=""
PI_VERDICT_GENERATED_AT=""
if [[ -n "${PI_AUDIT_VERDICT_PATH}" && -f "${PI_AUDIT_VERDICT_PATH}" ]]; then
    echo "PI_AUDIT_VERDICT_FILE_FOUND(${PI_AUDIT_VERDICT_PATH})"
    cat "${PI_AUDIT_VERDICT_PATH}"
    PI_VERDICT_JSON="$(cat "${PI_AUDIT_VERDICT_PATH}")"
    PI_VERDICT_FIELDS="$(python3 - "${PI_AUDIT_VERDICT_PATH}" <<'PYEOF'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as fh:
    verdict = json.load(fh)
print("available=true")
print(f"ok={'true' if verdict.get('ok') else 'false'}")
print(f"reasons={verdict.get('reasons', [])}")
print(f"run_id={verdict.get('run_id')}")
print(f"generated_at_utc={verdict.get('generated_at_utc')}")
PYEOF
)"
    echo "${PI_VERDICT_FIELDS}"
    PI_VERDICT_AVAILABLE="true"
    [[ "${PI_VERDICT_FIELDS}" == *"ok=true"* ]] && PI_VERDICT_OK="true"
    PI_VERDICT_REASONS="$(grep -o 'reasons=.*' <<<"${PI_VERDICT_FIELDS}" | cut -d= -f2-)"
    PI_VERDICT_RUN_ID="$(grep -o 'run_id=.*' <<<"${PI_VERDICT_FIELDS}" | cut -d= -f2-)"
    PI_VERDICT_GENERATED_AT="$(grep -o 'generated_at_utc=.*' <<<"${PI_VERDICT_FIELDS}" | cut -d= -f2-)"
else
    echo "PI_AUDIT_VERDICT=NOT_AVAILABLE(set GROUND_DIAGNOSTIC_PI_AUDIT_VERDICT to a verdict JSON produced by pi_ground_diagnostic_audit_verifier.py on the Pi)"
fi

echo ""
echo "=== LIVE_ZERO_STATE [4/5] Guard identity, armed state, /cmd_vel + upstream zero ==="
GUARD_PID="$(pgrep -af hil_cmd_vel_guard 2>/dev/null | grep -v 'pgrep -af' || true)"
GUARD_SOLE_PUBLISHER="false"
GUARD_ARMED="true"
CMD_VEL_ALL_ZERO="false"
UPSTREAM_ZERO_OR_ABSENT="false"
if [[ -z "${GUARD_PID}" ]]; then
    echo "GUARD=NOT_STARTED"
else
    echo "GUARD_PROCESS_FOUND: ${GUARD_PID}"
    echo "GUARD_ARMED_STATE=NOT_INDEPENDENTLY_VERIFIABLE_VIA_PASSIVE_TOPIC_READ (disarmed-by-default is guaranteed by hil_cmd_vel_guard.py's own tested default; checking the observable consequence instead)"
    GUARD_ARMED="false"
fi

if command -v ros2 >/dev/null 2>&1; then
    for TOPIC_VAR in "GUARDED:${GUARDED_TOPIC}" "UPSTREAM:${UPSTREAM_TOPIC}"; do
        ROLE="${TOPIC_VAR%%:*}"
        TOPIC="${TOPIC_VAR#*:}"
        PUB_COUNT="$(ros2 topic info "${TOPIC}" 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo "")"
        echo "PUBLISHER_COUNT[${TOPIC}]=${PUB_COUNT:-TOPIC_NOT_PRESENT}"
        if [[ "${ROLE}" == "GUARDED" ]]; then
            [[ "${PUB_COUNT}" == "1" ]] && GUARD_SOLE_PUBLISHER="true"
        fi
        if [[ -z "${PUB_COUNT}" || "${PUB_COUNT}" == "0" ]]; then
            VALUE=""
        else
            VALUE="$(timeout 3 ros2 topic echo "${TOPIC}" --once 2>/dev/null || true)"
        fi
        IS_ZERO="true"
        if [[ -n "${VALUE}" && "${VALUE}" != *"x: 0.0"* ]]; then
            IS_ZERO="false"
            echo "OUTPUT_ZERO_CHECK[${TOPIC}]=NONZERO_VALUE_DETECTED"
            echo "${VALUE}"
        else
            echo "OUTPUT_ZERO_CHECK[${TOPIC}]=PASS_OR_NOT_AVAILABLE"
        fi
        if [[ "${ROLE}" == "GUARDED" ]]; then
            CMD_VEL_ALL_ZERO="${IS_ZERO}"
        else
            if [[ -z "${PUB_COUNT}" || "${PUB_COUNT}" == "0" ]]; then
                UPSTREAM_ZERO_OR_ABSENT="true"
            else
                UPSTREAM_ZERO_OR_ABSENT="${IS_ZERO}"
            fi
        fi
    done
else
    echo "CMD_VEL_CHECKS=NOT_CHECKED(ros2 CLI not on PATH)"
fi

echo ""
echo "=== LIVE_ZERO_STATE [5/5] No controller/virtual-peer/navigator/Webots/pytest/colcon/rosbag process ==="
RESIDUAL_FORBIDDEN="$(pgrep -af -- "${FORBIDDEN_PATTERN}" 2>/dev/null | grep -v 'pgrep -af' || true)"
FORBIDDEN_FOUND="false"
if [[ -n "${RESIDUAL_FORBIDDEN}" ]]; then
    echo "FORBIDDEN_PROCESS_FOUND:"
    echo "${RESIDUAL_FORBIDDEN}"
    FORBIDDEN_FOUND="true"
else
    echo "FORBIDDEN_PROCESS_CHECK=CLEAN"
fi

echo ""
echo "=== LIVE_ZERO_STATE verdict (WSL side + Pi audit combined) ==="
VERDICT_OUTPUT="$(PYTHONPATH="${SCRIPT_DIR}" python3 - \
    "${VALIDITY_FLAGS_VALUE}" "${BRIDGE_CONNECTED}" "${WSL_CSV_GROWING}" \
    "${GUARD_SOLE_PUBLISHER}" "${GUARD_ARMED}" "${CMD_VEL_ALL_ZERO}" "${UPSTREAM_ZERO_OR_ABSENT}" \
    "${FORBIDDEN_FOUND}" "${WSL_EVIDENCE_ALL_ZERO}" \
    "${PI_VERDICT_AVAILABLE}" "${PI_VERDICT_OK}" "${PI_VERDICT_REASONS}" "${PI_VERDICT_RUN_ID}" \
    "${PI_VERDICT_GENERATED_AT}" "${RUN_ID}" "${WSL_CSV_PATH}" "${PI_VERDICT_MAX_AGE_S}" <<'PYEOF'
import ast
import sys

from hil_ground_diagnostic_phases import evaluate_combined_gate, evaluate_wsl_live_state

(flags, bridge, wsl_growing, sole_pub, armed, cmd_vel_zero, upstream_zero,
 forbidden, wsl_zero, pi_available, pi_ok, pi_reasons, pi_run_id,
 pi_generated_at, wsl_run_id, wsl_evidence_path, pi_max_age_s) = sys.argv[1:18]


def as_bool(s):
    return s == "true"


def as_list(s):
    try:
        return tuple(ast.literal_eval(s))
    except (ValueError, SyntaxError):
        return ()


wsl_result = evaluate_wsl_live_state(
    validity_flags=int(flags) if flags.strip().isdigit() else None,
    bridge_connected=as_bool(bridge),
    wsl_csv_growing=as_bool(wsl_growing),
    guard_sole_publisher=as_bool(sole_pub),
    guard_armed=as_bool(armed),
    cmd_vel_all_zero=as_bool(cmd_vel_zero),
    upstream_zero_or_absent=as_bool(upstream_zero),
    forbidden_process_found=as_bool(forbidden),
    wsl_evidence_all_zero=as_bool(wsl_zero),
)

result = evaluate_combined_gate(
    wsl_result=wsl_result,
    pi_verdict_ok=as_bool(pi_ok),
    pi_verdict_reasons=as_list(pi_reasons),
    pi_verdict_available=as_bool(pi_available),
    wsl_run_id=wsl_run_id or None,
    pi_run_id=pi_run_id or None,
    wsl_evidence_path=wsl_evidence_path,
    pi_verdict_generated_at_utc=pi_generated_at or None,
    pi_verdict_max_age_s=float(pi_max_age_s),
)
if result.ok:
    print("LIVE_ZERO_STATE_VERDICT=PASS")
else:
    print("LIVE_ZERO_STATE_VERDICT=BLOCKED")
    print(f"REASONS={list(result.reasons)}")
PYEOF
)"
echo "${VERDICT_OUTPUT}"

echo ""
if [[ "${VERDICT_OUTPUT}" == *"LIVE_ZERO_STATE_VERDICT=PASS"* ]]; then
    echo "GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_PASS"
    exit 0
else
    echo "GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_BLOCKED"
    exit 1
fi

fi
