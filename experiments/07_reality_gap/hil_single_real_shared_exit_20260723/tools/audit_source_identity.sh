#!/usr/bin/env bash
# Read-only source-identity audit -- Part 1 of the command-evidence-chain
# work following the two 2026-07-23 UNEXPECTED_PHYSICAL_MOTION incidents.
#
# Reports SHA-256 for every tracked Python source file across the three
# locations this project has been found to disagree between, without
# ever assuming they match:
#   1. The Windows git repository (the actual source of truth for
#      commits) -- e-puck2-Comm/src/epuck2_comm,
#      e-puck2-Comm/src/epuck2_comm_interfaces.
#   2. The native WSL colcon workspace source
#      (~/epuck_ws/src/epuck2_comm, .../epuck2_comm_interfaces) -- found
#      2026-07-23 to be a SEPARATE physical copy, not a symlink, which
#      silently went stale relative to the git repo earlier this
#      session until manually re-synced.
#   3. The installed copy colcon actually runs from
#      (~/epuck_ws/install/epuck2_comm/...).
#   4. The real_robot_avoidance_v1 project (Pi server, WSL bridge,
#      protocol) -- Windows handover mirror
#      (../实体实验交接包_20260715/real_robot_avoidance_v1) vs the WSL
#      native copy (~/epuck_ws/epuck_comm_project/real_robot_avoidance_v1).
#
# This script only computes and compares SHA-256 hashes. It never
# copies, builds, installs, or starts anything -- see
# sync_and_build_epuck2_comm.sh for the actual sync/build/verify
# procedure (deliberately not executed by this script).
set -euo pipefail

WIN_REPO="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm"
NATIVE_WS="/home/eamon/epuck_ws"
HANDOVER="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/实体实验交接包_20260715"

FAIL=0

_hash_tree() {
    # Prints "relative/path  sha256sum" for every .py file under $1,
    # sorted, so two trees can be diffed directly.
    local root="$1"
    find "${root}" -name '*.py' -type f | sort | while read -r f; do
        rel="${f#${root}/}"
        h="$(sha256sum "${f}" | cut -d' ' -f1)"
        echo "${rel}  ${h}"
    done
}

_compare() {
    local label="$1" root_a="$2" root_b="$3"
    echo ""
    echo "=== ${label} ==="
    echo "A: ${root_a}"
    echo "B: ${root_b}"
    if [[ ! -d "${root_a}" ]]; then
        echo "MISSING: A does not exist"
        FAIL=1
        return
    fi
    if [[ ! -d "${root_b}" ]]; then
        echo "MISSING: B does not exist"
        FAIL=1
        return
    fi
    tree_a="$(_hash_tree "${root_a}")"
    tree_b="$(_hash_tree "${root_b}")"
    if diff <(echo "${tree_a}") <(echo "${tree_b}") > /tmp/audit_source_identity_diff_$$.txt; then
        echo "IDENTICAL ($(echo "${tree_a}" | wc -l) files)"
    else
        echo "MISMATCH:"
        cat /tmp/audit_source_identity_diff_$$.txt
        FAIL=1
    fi
    rm -f /tmp/audit_source_identity_diff_$$.txt
}

echo "SOURCE_IDENTITY_AUDIT_START $(date --iso-8601=seconds)"

_compare "epuck2_comm: git repo vs native WSL source" \
    "${WIN_REPO}/src/epuck2_comm" \
    "${NATIVE_WS}/src/epuck2_comm"

_compare "epuck2_comm_interfaces: git repo vs native WSL source" \
    "${WIN_REPO}/src/epuck2_comm_interfaces" \
    "${NATIVE_WS}/src/epuck2_comm_interfaces"

INSTALLED_PY="${NATIVE_WS}/install/epuck2_comm/lib/python3.10/site-packages/epuck2_comm"
_compare "epuck2_comm: native WSL source vs installed (colcon build output)" \
    "${NATIVE_WS}/src/epuck2_comm/epuck2_comm" \
    "${INSTALLED_PY}"

_compare "real_robot_avoidance_v1: Windows handover mirror vs native WSL copy" \
    "${HANDOVER}/real_robot_avoidance_v1" \
    "${NATIVE_WS}/epuck_comm_project/real_robot_avoidance_v1"

echo ""
if [[ ${FAIL} -eq 0 ]]; then
    echo "SOURCE_IDENTITY_AUDIT_PASS"
else
    echo "SOURCE_IDENTITY_AUDIT_MISMATCH_FOUND"
fi
exit ${FAIL}
