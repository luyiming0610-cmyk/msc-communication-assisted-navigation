#!/usr/bin/env bash
# End-to-end synthetic test for sync_and_build_epuck2_comm.sh -- per
# the 2026-07-23 hardening request, this NEVER touches ~/epuck_ws or
# any real package. It builds fully synthetic source/destination
# directories (with a synthetic git repo standing in for the real one)
# under a fresh mktemp root, and points the hardened script's
# destination whitelist at those synthetic paths via
# SYNC_EPUCK2_COMM_TEST_EXPECTED_DESTINATIONS_JSON -- an env var the
# real script never reads unless a caller deliberately sets it (see
# sync_epuck2_comm_logic.py's _TEST_OVERRIDE_ENV_VAR).
#
# Scenarios covered:
#   1. Default (no args) is check-only and reports SYNC_CHECK_ONLY_PASS
#      on a clean synthetic tree, with zero destination changes.
#   2. An untracked destination file blocks check-only with
#      SYNC_CHECK_FAIL (i.e., rsync --delete would have removed it).
#   3. --execute without --confirm-token is blocked.
#   4. --execute with the WRONG token is blocked.
#   5. --execute with the correct token on the clean scenario actually
#      syncs, creates a timestamped backup, and passes post-sync
#      verification (colcon build is skipped in this synthetic test --
#      there is no real ROS package here to build).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRATCH="$(mktemp -d)"
trap 'rm -rf "${SCRATCH}"' EXIT

FAIL=0
assert_contains() {
    local haystack="$1" needle="$2" desc="$3"
    if [[ "${haystack}" == *"${needle}"* ]]; then
        echo "PASS: ${desc}"
    else
        echo "FAIL: ${desc} (expected to find '${needle}')"
        FAIL=1
    fi
}
assert_not_contains() {
    local haystack="$1" needle="$2" desc="$3"
    if [[ "${haystack}" != *"${needle}"* ]]; then
        echo "PASS: ${desc}"
    else
        echo "FAIL: ${desc} (did not expect to find '${needle}')"
        FAIL=1
    fi
}

# --- Build a synthetic "git repo" source tree -------------------------
SRC_REPO="${SCRATCH}/synthetic_repo"
mkdir -p "${SRC_REPO}/src/epuck2_comm/epuck2_comm"
mkdir -p "${SRC_REPO}/src/epuck2_comm_interfaces"
echo "print('hello')" > "${SRC_REPO}/src/epuck2_comm/epuck2_comm/example.py"
echo "print('interfaces')" > "${SRC_REPO}/src/epuck2_comm_interfaces/example.py"
(
    cd "${SRC_REPO}"
    git init -q
    git config user.email "test@example.com"
    git config user.name "test"
    git add -A
    git commit -q -m "synthetic initial commit"
)

# --- Build a synthetic "destination workspace" ------------------------
DEST_WS="${SCRATCH}/synthetic_ws"
mkdir -p "${DEST_WS}/src/epuck2_comm/epuck2_comm"
mkdir -p "${DEST_WS}/src/epuck2_comm_interfaces"
cp "${SRC_REPO}/src/epuck2_comm/epuck2_comm/example.py" "${DEST_WS}/src/epuck2_comm/epuck2_comm/example.py"
cp "${SRC_REPO}/src/epuck2_comm_interfaces/example.py" "${DEST_WS}/src/epuck2_comm_interfaces/example.py"

export SYNC_EPUCK2_COMM_TEST_EXPECTED_DESTINATIONS_JSON="$(python3 -c "
import json
print(json.dumps({
    'epuck2_comm': '${DEST_WS}/src/epuck2_comm',
    'epuck2_comm_interfaces': '${DEST_WS}/src/epuck2_comm_interfaces',
}))
")"

RUN() {
    bash "${SCRIPT_DIR}/sync_and_build_epuck2_comm.sh" \
        --source-root "${SRC_REPO}" --dest-root "${DEST_WS}" "$@" 2>&1 || true
}

echo "=== Scenario 1: default (check-only), clean synthetic tree ==="
OUT1="$(RUN)"
echo "${OUT1}"
assert_contains "${OUT1}" "MODE=check-only" "scenario 1: defaults to check-only"
assert_contains "${OUT1}" "SYNC_CHECK_ONLY_PASS" "scenario 1: clean tree passes check-only"
assert_not_contains "${OUT1}" "SYNC_BLOCKED" "scenario 1: clean tree is not blocked"

echo ""
echo "=== Scenario 2: an untracked destination file blocks check-only ==="
echo "print('local scratch, never committed')" > "${DEST_WS}/src/epuck2_comm/epuck2_comm/my_local_scratch.py"
OUT2="$(RUN)"
echo "${OUT2}"
assert_contains "${OUT2}" "SYNC_BLOCKED_UNEXPECTED_DELETIONS_WOULD_OCCUR" "scenario 2: untracked destination file blocks the sync"
assert_contains "${OUT2}" "my_local_scratch.py" "scenario 2: the specific untracked file is named"
if [[ -f "${DEST_WS}/src/epuck2_comm/epuck2_comm/my_local_scratch.py" ]]; then
    echo "PASS: scenario 2: untracked file was NOT deleted (check-only never touches the destination)"
else
    echo "FAIL: scenario 2: untracked file was deleted by a check-only run"
    FAIL=1
fi
rm -f "${DEST_WS}/src/epuck2_comm/epuck2_comm/my_local_scratch.py"

echo ""
echo "=== Scenario 3: --execute without --confirm-token is blocked ==="
OUT3="$(RUN --execute)"
echo "${OUT3}"
assert_contains "${OUT3}" "SYNC_EXECUTE_BLOCKED_GATE_FAILED" "scenario 3: execute without a token is blocked"

echo ""
echo "=== Scenario 4: --execute with the WRONG token is blocked ==="
OUT4="$(RUN --execute --confirm-token WRONG_TOKEN)"
echo "${OUT4}"
assert_contains "${OUT4}" "SYNC_EXECUTE_BLOCKED_GATE_FAILED" "scenario 4: execute with the wrong token is blocked"

echo ""
echo "=== Scenario 5: --execute with the correct token actually syncs and backs up ==="
OUT5="$(RUN --execute --confirm-token CONFIRM_SYNC_EXECUTE)"
echo "${OUT5}"
assert_contains "${OUT5}" "backed_up: epuck2_comm" "scenario 5: a backup was created before the real sync"
assert_contains "${OUT5}" "POST_SYNC_MATCH(epuck2_comm)" "scenario 5: post-sync hash verification passed"
if [[ -d "${DEST_WS}/sync_backups" ]]; then
    echo "PASS: scenario 5: backup directory exists"
else
    echo "FAIL: scenario 5: no backup directory was created"
    FAIL=1
fi

echo ""
if [[ ${FAIL} -eq 0 ]]; then
    echo "SYNTHETIC_E2E_TEST_PASS"
else
    echo "SYNTHETIC_E2E_TEST_FAIL"
fi
exit ${FAIL}
