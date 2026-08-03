#!/usr/bin/env bash
# Explicit HIL shutdown, using ONLY the exact PIDs recorded in a trial's
# own pid_manifest.json (written by run_hil_shared_exit_trial.sh when
# it starts each process). Never uses pkill or any name-based match --
# if a PID in the manifest no longer belongs to the expected process
# (e.g. reused by an unrelated later process), this script still only
# ever signals the exact PID on record, never guesses a replacement.
#
# Usage: run_hil_shutdown.sh PID_MANIFEST_JSON
set -uo pipefail

MANIFEST="${1:-}"
if [[ -z "$MANIFEST" || ! -f "$MANIFEST" ]]; then
  echo "Usage: $0 PID_MANIFEST_JSON" >&2
  echo "ERROR: manifest file not given or not found: '${MANIFEST}'" >&2
  exit 2
fi

echo "[$(date -Iseconds)] shutting down HIL processes from manifest: $MANIFEST"

# Reverse-order shutdown: guard first (stop new commands from being
# guarded/passed through), then the controllers/supervisor/adapters/peer
# that feed it, then the recorder/monitor last (so they capture the
# clean-stop tail of everything else). Keys must match record_process's
# own naming in run_hil_stage4_trial.sh exactly -- "recorder", not
# "recorder_bag" -- and must list every orchestrator-owned process,
# including hil_stage4_motion_supervisor (both were silently skipped by
# this list before, discovered live during RUN_ID stage4_20260731_151052).
ORDER='["hil_cmd_vel_guard","cooperative_avoider","hil_stage4_motion_supervisor","hil_topic_adapter","hil_virtual_peer","task_completion_monitor","recorder"]'

python3 - "$MANIFEST" "$ORDER" <<'PYEOF'
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

manifest_path, order_json = sys.argv[1], sys.argv[2]
with open(manifest_path, encoding="utf-8") as fh:
    manifest = json.load(fh)
order = json.loads(order_json)

processes = manifest.get("processes", {})
results = {}


def alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def child_pids(pid):
    # Exact PID/PPID process-tree lookup -- never a name-based match.
    # `ros2 run` spawns the real node as a child of its own wrapper
    # process; pid_manifest.json only ever records the wrapper's PID
    # ($!), so a SIGINT to that PID alone can leave the real child
    # orphaned and running (observed live: cooperative_avoider's wrapper
    # PID 963 did not exit within 10s while its real node, child PID
    # 978, kept running under it).
    proc = subprocess.run(
        ["ps", "--ppid", str(pid), "-o", "pid="],
        capture_output=True, text=True, check=False,
    )
    return [int(line) for line in proc.stdout.split() if line.strip()]


for name in order:
    entry = processes.get(name)
    if not entry:
        continue
    pid = int(entry["pid"])
    if not alive(pid):
        results[name] = {"pid": pid, "status": "ALREADY_STOPPED"}
        continue
    targets = child_pids(pid) + [pid]
    kill_failed = None
    for target in targets:
        try:
            os.kill(target, signal.SIGINT)
        except OSError as exc:
            kill_failed = exc
    if kill_failed is not None:
        results[name] = {"pid": pid, "status": f"KILL_FAILED: {kill_failed}"}
        continue
    stopped = False
    for _ in range(20):
        if not alive(pid) and not any(alive(t) for t in targets):
            stopped = True
            break
        time.sleep(0.5)
    still_alive = [t for t in targets if alive(t)]
    results[name] = {
        "pid": pid,
        "status": "STOPPED" if stopped else f"STILL_ALIVE_AFTER_10S:{still_alive}",
    }

for name, result in results.items():
    print(f"{name}: pid={result['pid']} status={result['status']}")

clean_statuses = {"STOPPED", "ALREADY_STOPPED"}
all_clean = all(r["status"] in clean_statuses for r in results.values())
residual_path = os.path.join(os.path.dirname(os.path.abspath(manifest_path)), "residual_check.json")
payload = {
    "residual_process_check": "CLEAN" if all_clean else "NOT_CLEAN",
    "shutdown_results": results,
}
fd, temporary_path = tempfile.mkstemp(
    prefix="residual_check.json.tmp.", dir=os.path.dirname(residual_path), text=True,
)
try:
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary_path, residual_path)
except BaseException:
    try:
        os.unlink(temporary_path)
    except OSError:
        pass
    raise

print(f"residual_check_written={residual_path}")
print("PROCESSES_CLEAN" if all_clean else "PROCESSES_NOT_CLEAN")
sys.exit(0 if all_clean else 1)
PYEOF
STATUS=$?

echo "[$(date -Iseconds)] shutdown sequence complete"
exit "$STATUS"
