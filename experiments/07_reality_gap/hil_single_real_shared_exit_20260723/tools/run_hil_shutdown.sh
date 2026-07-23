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
# guarded/passed through), then the controllers/adapters/peer that
# feed it, then the recorder/monitor last (so they capture the
# clean-stop tail of everything else).
ORDER='["hil_cmd_vel_guard","cooperative_avoider","hil_topic_adapter","hil_virtual_peer","task_completion_monitor","recorder_bag"]'

python3 - "$MANIFEST" "$ORDER" <<'PYEOF'
import json
import os
import signal
import sys
import time

manifest_path, order_json = sys.argv[1], sys.argv[2]
with open(manifest_path, encoding="utf-8") as fh:
    manifest = json.load(fh)
order = json.loads(order_json)

processes = manifest.get("processes", {})
results = {}

for name in order:
    entry = processes.get(name)
    if not entry:
        continue
    pid = int(entry["pid"])
    try:
        os.kill(pid, 0)
    except OSError:
        results[name] = {"pid": pid, "status": "ALREADY_STOPPED"}
        continue
    try:
        os.kill(pid, signal.SIGINT)
    except OSError as exc:
        results[name] = {"pid": pid, "status": f"KILL_FAILED: {exc}"}
        continue
    stopped = False
    for _ in range(20):
        try:
            os.kill(pid, 0)
        except OSError:
            stopped = True
            break
        time.sleep(0.5)
    results[name] = {"pid": pid, "status": "STOPPED" if stopped else "STILL_ALIVE_AFTER_10S"}

for name, result in results.items():
    print(f"{name}: pid={result['pid']} status={result['status']}")

any_still_alive = any(r["status"] == "STILL_ALIVE_AFTER_10S" for r in results.values())
print("PROCESSES_CLEAN" if not any_still_alive else "PROCESSES_NOT_CLEAN")
sys.exit(0 if not any_still_alive else 1)
PYEOF
STATUS=$?

echo "[$(date -Iseconds)] shutdown sequence complete"
exit "$STATUS"
