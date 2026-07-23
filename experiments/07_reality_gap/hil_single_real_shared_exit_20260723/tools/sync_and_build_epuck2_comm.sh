#!/usr/bin/env bash
# Deterministic, hash-verified sync + build procedure for epuck2_comm /
# epuck2_comm_interfaces -- Part 1 of the command-evidence-chain work
# following the two 2026-07-23 UNEXPECTED_PHYSICAL_MOTION incidents.
#
# Replaces manual, selective `cp` of individual files (the process gap
# found during the ROS-domain-isolation fix: the native WSL colcon
# workspace silently went stale relative to the git repo until files
# were copied one at a time by hand). This script instead:
#   1. Records the exact git commit the Windows repo is at.
#   2. Reports SHA-256 for every source .py file in the git repo BEFORE
#      touching anything.
#   3. Performs a deterministic full-tree sync (rsync --delete, not a
#      selective copy list) from the git repo into the native WSL
#      colcon workspace source.
#   4. Re-hashes the just-synced native source and asserts it now
#      matches the pre-sync git-repo hashes exactly (proves the sync
#      itself was lossless).
#   5. Runs `colcon build` for both packages.
#   6. Re-hashes the installed output and asserts it matches the
#      synced source hashes -- proving the installed executables
#      genuinely correspond to the intended source commit, not a
#      stale or partial build.
#
# Deliberately NOT executed as part of this turn's work (per explicit
# instruction) -- only bash -n syntax-checked. This script touches no
# physical process, no ROS graph, no hardware -- it is a pure
# filesystem-sync + software-build step -- but running it for real is
# left for a future, explicitly authorized turn.
set -euo pipefail

WIN_REPO="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm"
NATIVE_WS="/home/eamon/epuck_ws"
PACKAGES=(epuck2_comm epuck2_comm_interfaces)

_hash_tree() {
    local root="$1"
    find "${root}" -name '*.py' -type f | sort | while read -r f; do
        rel="${f#${root}/}"
        h="$(sha256sum "${f}" | cut -d' ' -f1)"
        echo "${rel}  ${h}"
    done
}

echo "=== [1/6] Recording git commit identity ==="
GIT_COMMIT="$(git -C "${WIN_REPO}" rev-parse HEAD)"
GIT_DIRTY="$(git -C "${WIN_REPO}" status --porcelain -- src/epuck2_comm src/epuck2_comm_interfaces)"
echo "git_commit=${GIT_COMMIT}"
if [[ -n "${GIT_DIRTY}" ]]; then
    echo "GIT_TREE_DIRTY -- uncommitted changes present under src/:"
    echo "${GIT_DIRTY}"
    echo "SYNC_BUILD_BLOCKED_DIRTY_TREE"
    exit 1
fi

echo ""
echo "=== [2/6] Pre-sync source SHA-256 (git repo) ==="
PRE_SYNC_HASHES=""
for pkg in "${PACKAGES[@]}"; do
    echo "--- ${pkg} ---"
    h="$(_hash_tree "${WIN_REPO}/src/${pkg}")"
    echo "${h}"
    PRE_SYNC_HASHES+="${h}"$'\n'
done

echo ""
echo "=== [3/6] Deterministic full-tree sync (rsync --delete) ==="
for pkg in "${PACKAGES[@]}"; do
    rsync -a --delete "${WIN_REPO}/src/${pkg}/" "${NATIVE_WS}/src/${pkg}/"
    echo "synced: ${pkg}"
done

echo ""
echo "=== [4/6] Post-sync verification: native source must now match git repo exactly ==="
POST_SYNC_HASHES=""
for pkg in "${PACKAGES[@]}"; do
    h="$(_hash_tree "${NATIVE_WS}/src/${pkg}")"
    POST_SYNC_HASHES+="${h}"$'\n'
done
if [[ "${PRE_SYNC_HASHES}" != "${POST_SYNC_HASHES}" ]]; then
    echo "SYNC_BUILD_BLOCKED_POST_SYNC_HASH_MISMATCH"
    diff <(echo "${PRE_SYNC_HASHES}") <(echo "${POST_SYNC_HASHES}") || true
    exit 1
fi
echo "Post-sync hashes match pre-sync git-repo hashes exactly."

echo ""
echo "=== [5/6] colcon build ==="
source /opt/ros/humble/setup.bash
(cd "${NATIVE_WS}" && colcon build --packages-select "${PACKAGES[@]}")

echo ""
echo "=== [6/6] Post-build verification: installed output must match synced source ==="
FAIL=0
for pkg in "${PACKAGES[@]}"; do
    installed_py="${NATIVE_WS}/install/${pkg}/lib/python3.10/site-packages/${pkg}"
    if [[ ! -d "${installed_py}" ]]; then
        echo "SKIP: ${pkg} has no installed python site-packages directory (interface-only package?)"
        continue
    fi
    src_h="$(_hash_tree "${NATIVE_WS}/src/${pkg}/${pkg}")"
    installed_h="$(_hash_tree "${installed_py}")"
    if [[ "${src_h}" != "${installed_h}" ]]; then
        echo "MISMATCH: ${pkg} installed output does not match synced source"
        diff <(echo "${src_h}") <(echo "${installed_h}") || true
        FAIL=1
    else
        echo "MATCH: ${pkg} installed output corresponds to git_commit=${GIT_COMMIT}"
    fi
done

echo ""
if [[ ${FAIL} -eq 0 ]]; then
    echo "SYNC_BUILD_VERIFY_PASS git_commit=${GIT_COMMIT}"
else
    echo "SYNC_BUILD_VERIFY_FAIL"
fi
exit ${FAIL}
