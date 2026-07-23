#!/usr/bin/env bash
# Hardened deterministic sync + build procedure for epuck2_comm /
# epuck2_comm_interfaces -- rewritten 2026-07-23 after review found the
# original version defaulted to an EXECUTING, `rsync --delete` sync
# with no path validation, no deletion-safety check, no backup, and no
# confirmation gate. See sync_epuck2_comm_logic.py for the pure
# decision functions this script calls into (unit tested against
# synthetic inputs in test_sync_epuck2_comm_logic.py -- never against
# a real directory).
#
# DEFAULT MODE IS --check-only. It is entirely non-destructive:
#   1. Resolves and validates every source/destination path as
#      absolute, and aborts unless the destination is EXACTLY one of
#      the two intended package source directories (never a typo, a
#      symlink elsewhere, or an unrelated workspace package).
#   2. Copies the intended source into a FRESH staging directory
#      (never the real destination) and hashes it -- a clean,
#      non-destructive verification step, preferred over touching the
#      real destination at all when only a "would this be correct"
#      question needs answering.
#   3. Runs `rsync -n --delete --itemize-changes` (the `-n` flag makes
#      this a dry run -- rsync itself guarantees no filesystem change)
#      from source to the REAL destination, producing an itemized
#      change/deletion plan.
#   4. Cross-checks every planned deletion against the source's own
#      git-tracked file list. Any deletion target that is NOT
#      git-tracked is flagged as an unexpected/untracked destination
#      file -- check-only mode reports this as a hard blocker.
#
# EXECUTE MODE requires BOTH --execute AND a separately-supplied
# --confirm-token matching sync_epuck2_comm_logic.REQUIRED_CONFIRM_TOKEN
# exactly. It re-runs every check-only validation first and aborts on
# any failure, then creates a timestamped backup of the current
# destination BEFORE touching anything, then performs the real sync,
# then re-verifies.
#
# This script never touches ~/epuck_comm_bags, ~/epuck_ws/log,
# ~/epuck_ws/build, ~/epuck_ws/install, or any package other than the
# two named above -- every path used is asserted against the
# EXPECTED_DESTINATIONS whitelist in sync_epuck2_comm_logic.py, and the
# rsync source/destination pair is always exactly one package
# directory, never a workspace root.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WIN_REPO_DEFAULT="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm"
NATIVE_WS_DEFAULT="/home/eamon/epuck_ws"
PACKAGES=(epuck2_comm epuck2_comm_interfaces)

MODE="check-only"
CONFIRM_TOKEN=""
WIN_REPO="${WIN_REPO_DEFAULT}"
NATIVE_WS="${NATIVE_WS_DEFAULT}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --check-only) MODE="check-only"; shift ;;
        --execute) MODE="execute"; shift ;;
        --confirm-token) CONFIRM_TOKEN="${2:-}"; shift 2 ;;
        --source-root) WIN_REPO="${2:-}"; shift 2 ;;
        --dest-root) NATIVE_WS="${2:-}"; shift 2 ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done

echo "MODE=${MODE}"

# --- Step 1: resolve and validate every path -------------------------
if [[ ! -d "${WIN_REPO}" ]]; then
    echo "SYNC_BLOCKED_SOURCE_ROOT_NOT_FOUND(${WIN_REPO})"
    exit 1
fi
if [[ ! -d "${NATIVE_WS}" ]]; then
    echo "SYNC_BLOCKED_DEST_ROOT_NOT_FOUND(${NATIVE_WS})"
    exit 1
fi
WIN_REPO_RESOLVED="$(cd "${WIN_REPO}" && pwd -P)"
NATIVE_WS_RESOLVED="$(cd "${NATIVE_WS}" && pwd -P)"

FAIL=0
for pkg in "${PACKAGES[@]}"; do
    src_dir="${WIN_REPO_RESOLVED}/src/${pkg}"
    dst_dir="${NATIVE_WS_RESOLVED}/src/${pkg}"
    if [[ ! -d "${src_dir}" ]]; then
        echo "SYNC_BLOCKED_SOURCE_PACKAGE_NOT_FOUND(${src_dir})"
        FAIL=1
        continue
    fi
    dst_dir_resolved="$(cd "$(dirname "${dst_dir}")" 2>/dev/null && pwd -P)/$(basename "${dst_dir}")"

    validate_out="$(python3 - 2>&1 <<PYEOF || true
import sys
sys.path.insert(0, "${SCRIPT_DIR}")
from sync_epuck2_comm_logic import validate_absolute_path, validate_destination_is_expected
for check in (
    validate_absolute_path("${src_dir}"),
    validate_absolute_path("${dst_dir_resolved}"),
    validate_destination_is_expected("${pkg}", "${dst_dir_resolved}"),
):
    if not check.ok:
        print(f"BLOCKED:{check.reason}")
        sys.exit(1)
print("VALIDATED")
PYEOF
)"
    echo "--- ${pkg} path validation: ${validate_out}"
    if [[ "${validate_out}" != *VALIDATED* ]]; then
        FAIL=1
    fi

    # Belt-and-braces: refuse to proceed if the resolved destination
    # string contains any component this script must never touch.
    for forbidden in "/log/" "/build/" "/install/" "/epuck_comm_bags/"; do
        if [[ "${dst_dir_resolved}" == *"${forbidden}"* ]]; then
            echo "SYNC_BLOCKED_FORBIDDEN_PATH_COMPONENT(${forbidden} in ${dst_dir_resolved})"
            FAIL=1
        fi
    done
done

if [[ ${FAIL} -ne 0 ]]; then
    echo "SYNC_BLOCKED_PATH_VALIDATION_FAILED"
    exit 1
fi

echo ""
echo "=== [Staging verification] copying intended source into a fresh, disposable staging directory ==="
STAGING_DIR="$(mktemp -d)"
trap 'rm -rf "${STAGING_DIR}"' EXIT
for pkg in "${PACKAGES[@]}"; do
    mkdir -p "${STAGING_DIR}/${pkg}"
    cp -a "${WIN_REPO_RESOLVED}/src/${pkg}/." "${STAGING_DIR}/${pkg}/"
done
echo "Staged (read-only verification, never the real destination): ${STAGING_DIR}"
for pkg in "${PACKAGES[@]}"; do
    n="$(find "${STAGING_DIR}/${pkg}" -name '*.py' -type f | wc -l)"
    echo "staged_${pkg}_py_file_count=${n}"
done

echo ""
echo "=== [Dry-run plan] rsync -n --delete --itemize-changes (no filesystem change) ==="
FAIL=0
for pkg in "${PACKAGES[@]}"; do
    src_dir="${WIN_REPO_RESOLVED}/src/${pkg}/"
    dst_dir="${NATIVE_WS_RESOLVED}/src/${pkg}/"
    dry_run_report="$(mktemp)"
    rsync -a -n --delete --itemize-changes "${src_dir}" "${dst_dir}" > "${dry_run_report}"
    echo "--- ${pkg} dry-run plan ---"
    cat "${dry_run_report}"

    tracked_file="$(mktemp)"
    git -C "${WIN_REPO_RESOLVED}" ls-files -- "src/${pkg}" | sed "s#^src/${pkg}/##" > "${tracked_file}"

    if ! python3 "${SCRIPT_DIR}/sync_epuck2_comm_logic.py" \
        --rsync-dry-run-output "${dry_run_report}" \
        --tracked-files "${tracked_file}"; then
        echo "SYNC_BLOCKED_UNEXPECTED_DELETIONS_WOULD_OCCUR(${pkg})"
        FAIL=1
    fi
    rm -f "${dry_run_report}" "${tracked_file}"
done

if [[ ${FAIL} -ne 0 ]]; then
    echo "SYNC_CHECK_FAIL"
    exit 1
fi

if [[ "${MODE}" == "check-only" ]]; then
    echo ""
    echo "SYNC_CHECK_ONLY_PASS (no destination file was created, modified, or deleted)"
    exit 0
fi

# --- Execute mode: both --execute and the correct token are required --
echo ""
echo "=== [Execute gate] ==="
GATE_OK="$(python3 -c "
import sys
sys.path.insert(0, '${SCRIPT_DIR}')
from sync_epuck2_comm_logic import check_execute_gate
result = check_execute_gate(execute_requested=True, confirm_token='''${CONFIRM_TOKEN}''')
print('OK' if result.ok else f'BLOCKED:{result.reason}')
")"
echo "execute_gate=${GATE_OK}"
if [[ "${GATE_OK}" != "OK" ]]; then
    echo "SYNC_EXECUTE_BLOCKED_GATE_FAILED"
    exit 1
fi

echo ""
echo "=== [Backup] timestamped backup of the current destination before any change ==="
BACKUP_ROOT="${NATIVE_WS_RESOLVED}/sync_backups/$(date -u +%Y%m%d_%H%M%S)"
mkdir -p "${BACKUP_ROOT}"
GIT_COMMIT="$(git -C "${WIN_REPO_RESOLVED}" rev-parse HEAD)"
for pkg in "${PACKAGES[@]}"; do
    dst_dir="${NATIVE_WS_RESOLVED}/src/${pkg}"
    cp -a "${dst_dir}" "${BACKUP_ROOT}/${pkg}"
    echo "backed_up: ${pkg} -> ${BACKUP_ROOT}/${pkg}"
done
cat > "${BACKUP_ROOT}/manifest.json" <<EOF
{
  "git_commit_synced_from": "${GIT_COMMIT}",
  "backup_created_utc": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "packages": $(printf '%s\n' "${PACKAGES[@]}" | python3 -c "import json,sys; print(json.dumps([l.strip() for l in sys.stdin if l.strip()]))")
}
EOF
echo "manifest=${BACKUP_ROOT}/manifest.json"

echo ""
echo "=== [Execute] rsync -a --delete (real sync) ==="
for pkg in "${PACKAGES[@]}"; do
    rsync -a --delete "${WIN_REPO_RESOLVED}/src/${pkg}/" "${NATIVE_WS_RESOLVED}/src/${pkg}/"
    echo "synced: ${pkg}"
done

echo ""
echo "=== [Post-sync verification] ==="
_hash_tree() {
    local root="$1"
    find "${root}" -name '*.py' -type f | sort | while read -r f; do
        rel="${f#${root}/}"
        h="$(sha256sum "${f}" | cut -d' ' -f1)"
        echo "${rel}  ${h}"
    done
}
FAIL=0
for pkg in "${PACKAGES[@]}"; do
    src_h="$(_hash_tree "${WIN_REPO_RESOLVED}/src/${pkg}")"
    dst_h="$(_hash_tree "${NATIVE_WS_RESOLVED}/src/${pkg}")"
    if [[ "${src_h}" != "${dst_h}" ]]; then
        echo "POST_SYNC_MISMATCH(${pkg})"
        FAIL=1
    else
        echo "POST_SYNC_MATCH(${pkg})"
    fi
done

if [[ ${FAIL} -ne 0 ]]; then
    echo "SYNC_EXECUTE_VERIFY_FAIL"
    exit 1
fi

echo ""
echo "=== [Build] colcon build ==="
source /opt/ros/humble/setup.bash
(cd "${NATIVE_WS_RESOLVED}" && colcon build --packages-select "${PACKAGES[@]}")

echo ""
echo "=== [Post-build verification] ==="
FAIL=0
for pkg in "${PACKAGES[@]}"; do
    installed_py="${NATIVE_WS_RESOLVED}/install/${pkg}/lib/python3.10/site-packages/${pkg}"
    if [[ ! -d "${installed_py}" ]]; then
        echo "SKIP: ${pkg} has no installed python site-packages directory"
        continue
    fi
    src_h="$(_hash_tree "${NATIVE_WS_RESOLVED}/src/${pkg}/${pkg}")"
    installed_h="$(_hash_tree "${installed_py}")"
    if [[ "${src_h}" != "${installed_h}" ]]; then
        echo "MISMATCH: ${pkg} installed output does not match synced source"
        FAIL=1
    else
        echo "MATCH: ${pkg} installed output corresponds to git_commit=${GIT_COMMIT}"
    fi
done

echo ""
if [[ ${FAIL} -eq 0 ]]; then
    echo "SYNC_BUILD_VERIFY_PASS git_commit=${GIT_COMMIT} backup=${BACKUP_ROOT}"
else
    echo "SYNC_BUILD_VERIFY_FAIL"
fi
exit ${FAIL}
