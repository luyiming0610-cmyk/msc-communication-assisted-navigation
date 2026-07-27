#!/usr/bin/env bash
# Start/stop wrapper for hil_command_evidence_recorder.py -- Part 3 of
# the command-evidence-chain work following the two 2026-07-23
# UNEXPECTED_PHYSICAL_MOTION incidents.
#
# `start`: creates a new output directory (auto-timestamped by
# default, or the exact caller-supplied --output-root), starts the
# recorder in the background, writes its exact PID and start time to a
# manifest.json in that directory, and prints both. Per
# HIL_LAB_RUNBOOK.md, this must be the FIRST process started in any
# powered session and the LAST one stopped.
#
# `stop`: takes the manifest.json path, sends exactly one SIGINT to
# the exact recorded PID (never pkill, never a name-based match),
# waits up to 10s for it to exit, then SHA-256-verifies the CSV the
# recorder wrote and records the hash into the same manifest.json for
# provenance.
#
# Usage:
#   run_hil_command_evidence_recorder.sh start [--output-root <dir>] [extra recorder args...]
#   run_hil_command_evidence_recorder.sh stop <manifest.json>
#
# --output-root, not --output-csv (2026-07-27 fix): found live during
# SRGRB_20260727 Trial 1 Attempt 1 -- this wrapper always built its own
# --output-csv from an auto-generated timestamped directory and
# prepended it ahead of any caller-supplied extra args. A caller who
# also passed --output-csv (intending to override the destination) got
# a silent argparse last-value-wins collision: the recorder actually
# tried to open the CALLER's path, while this wrapper's own bash
# variables -- and therefore its manifest.json, its console output, and
# the parent directory it actually mkdir -p'd -- still referred to its
# OWN auto path. The recorder crashed with FileNotFoundError because
# the caller's intended directory was never created; the crash log was
# left behind while the wrapper reported a directory the recorder never
# touched. --output-csv is no longer accepted from a caller at all
# (rejected outright, see cmd_start below); --output-root <dir> is the
# only supported override, and it is this function's single source of
# truth for out_dir/csv_path/log_path/manifest, so every path reported
# is guaranteed to be the one the recorder actually opened.
# set -u deliberately NOT enabled here yet -- ROS2's own setup.bash
# (sourced inside cmd_start) references unset variables internally and
# is not `set -u`-safe. Each function sources ROS first, THEN enables
# `set -u` for its own remaining body, mirroring the fix already
# applied to every other HIL script that hit this exact bug in this
# project (test_hil_integration_offline.sh, run_hil_physical_preflight.sh).
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_ROOT="${HIL_COMMAND_EVIDENCE_ROOT:-$HOME/epuck_comm_bags}"

cmd_start() {
    source /opt/ros/humble/setup.bash
    source "$HOME/epuck_ws/install/setup.bash"
    set -u

    # Parse for an optional --output-root override BEFORE anything else
    # touches "$@" -- see the module docstring above for why a bare
    # --output-csv is rejected rather than silently forwarded.
    output_root=""
    remaining_args=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --output-root)
                if [[ -n "${output_root}" ]]; then
                    echo "HIL_COMMAND_EVIDENCE_RECORDER_REJECTED: --output-root given more than once" >&2
                    exit 2
                fi
                output_root="${2:-}"
                shift 2
                ;;
            --output-root=*)
                if [[ -n "${output_root}" ]]; then
                    echo "HIL_COMMAND_EVIDENCE_RECORDER_REJECTED: --output-root given more than once" >&2
                    exit 2
                fi
                output_root="${1#--output-root=}"
                shift
                ;;
            --output-csv|--output-csv=*)
                echo "HIL_COMMAND_EVIDENCE_RECORDER_REJECTED: --output-csv is not accepted directly -- use --output-root <dir> instead, so this wrapper's manifest/log/reported paths always agree with what the recorder actually opens." >&2
                exit 2
                ;;
            *)
                remaining_args+=("$1")
                shift
                ;;
        esac
    done

    if [[ -n "${output_root}" ]]; then
        out_dir="${output_root}"
    else
        ts="$(date -u +%Y%m%d_%H%M%S)"
        out_dir="${BAGS_ROOT}/hil_command_evidence_${ts}"
    fi
    mkdir -p "${out_dir}"
    csv_path="${out_dir}/command_evidence.csv"
    log_path="${out_dir}/recorder.log"

    # setsid detaches the recorder into its own session (new session
    # leader, no controlling terminal) and disown removes it from this
    # shell's job table -- both are required, not just `&` alone.
    # Found 2026-07-24: when this script itself is invoked as a single
    # one-shot command (e.g. `wsl.exe ... -- bash -lc "run_hil_command_
    # evidence_recorder.sh start ..."`), the invoking shell's own
    # session ends immediately after this function returns; a plain
    # `&` background job is still a member of that session and gets
    # torn down with it (observed: the recorder process and its log
    # file both vanished within about a second, before the first
    # buffered log line could flush to disk). `setsid` breaks that
    # session membership so the recorder survives the invoking shell's
    # exit; `< /dev/null` additionally detaches stdin so nothing about
    # the invoking terminal can signal it either.
    setsid python3 "${SCRIPT_DIR}/hil_command_evidence_recorder.py" \
        --output-csv "${csv_path}" \
        "${remaining_args[@]}" \
        < /dev/null > "${log_path}" 2>&1 &
    pid=$!
    disown 2>/dev/null || true

    manifest="${out_dir}/manifest.json"
    python3 - "$manifest" "$pid" "$csv_path" "$log_path" <<'PYEOF'
import json
import sys
import time

manifest_path, pid, csv_path, log_path = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
manifest = {
    "pid": pid,
    "csv_path": csv_path,
    "log_path": log_path,
    "started_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
}
with open(manifest_path, "w", encoding="utf-8") as fh:
    json.dump(manifest, fh, indent=2)
PYEOF

    echo "HIL_COMMAND_EVIDENCE_RECORDER_STARTED pid=${pid} manifest=${manifest}"
    echo "output_dir=${out_dir}"
}

cmd_stop() {
    set -u
    manifest="${1:-}"
    if [[ -z "${manifest}" || ! -f "${manifest}" ]]; then
        echo "Usage: $0 stop <manifest.json>" >&2
        exit 2
    fi

    python3 - "$manifest" <<'PYEOF'
import json
import os
import signal
import sys
import time

manifest_path = sys.argv[1]
with open(manifest_path, encoding="utf-8") as fh:
    manifest = json.load(fh)
pid = int(manifest["pid"])

try:
    os.kill(pid, 0)
except OSError:
    print(f"pid={pid} status=ALREADY_STOPPED")
    sys.exit(0)

os.kill(pid, signal.SIGINT)
stopped = False
for _ in range(20):
    try:
        os.kill(pid, 0)
    except OSError:
        stopped = True
        break
    time.sleep(0.5)

print(f"pid={pid} status={'STOPPED' if stopped else 'STILL_ALIVE_AFTER_10S'}")
sys.exit(0 if stopped else 1)
PYEOF
    stop_status=$?

    csv_path="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['csv_path'])" "$manifest")"
    if [[ -f "${csv_path}" ]]; then
        hash="$(sha256sum "${csv_path}" | cut -d' ' -f1)"
        rows="$(( $(wc -l < "${csv_path}") - 1 ))"
        echo "csv_path=${csv_path} sha256=${hash} rows=${rows}"
        python3 - "$manifest" "$hash" "$rows" <<'PYEOF'
import json
import sys

manifest_path, sha256, rows = sys.argv[1], sys.argv[2], int(sys.argv[3])
with open(manifest_path, encoding="utf-8") as fh:
    manifest = json.load(fh)
manifest["stopped_status"] = "STOPPED"
manifest["csv_sha256"] = sha256
manifest["csv_row_count"] = rows
with open(manifest_path, "w", encoding="utf-8") as fh:
    json.dump(manifest, fh, indent=2)
PYEOF
    else
        echo "WARNING: csv_path=${csv_path} does not exist -- recorder may not have flushed"
    fi

    exit ${stop_status}
}

case "${1:-}" in
    start)
        shift
        cmd_start "$@"
        ;;
    stop)
        cmd_stop "${2:-}"
        ;;
    *)
        echo "Usage: $0 {start|stop} ..." >&2
        exit 2
        ;;
esac
