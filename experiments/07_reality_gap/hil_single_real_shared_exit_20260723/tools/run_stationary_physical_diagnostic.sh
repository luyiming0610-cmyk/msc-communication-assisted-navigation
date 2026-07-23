#!/usr/bin/env bash
set -eo pipefail

# STATIONARY_PHYSICAL_DIAGNOSTIC -- read-only, no publishers, no motion.
#
# Confirms /cmd_vel has zero publishers before starting, then runs a
# fixed-duration, fully concurrent set of read-only observers over the
# real ROS graph: per-topic rate sampling (/odom, /scan, /tof,
# /ps0-/ps7) over the WHOLE window (not a short snapshot), continuous
# validity_flags monitoring, and the existing wsl_expanded_pilot_
# recorder.py bridge-status recorder (reused unmodified).
#
# Never starts hil_cmd_vel_guard, a controller, a virtual peer,
# goal_navigator, Webots, or any /cmd_vel publisher.
#
# Fixes a real bug found in the first live run of this diagnostic
# (2026-07-23): wsl_expanded_pilot_recorder.py has NO internal exit
# condition by design (confirmed by reading its own run() loop -- it
# spins forever until externally signaled) -- every prior script that
# reuses it (run_baseline_v1_trial_v2.sh, run_expanded_bridge_
# epuckstate_pilot.sh) explicitly sends it SIGINT after a fixed sleep.
# The first version of this script omitted that step and instead did
# a bare `wait "$RECORDER_PID"`, which never returned -- the recorder
# ran for ~26 minutes (well past the intended 300s) before being
# stopped manually. The other collectors (each independently wrapped
# in `timeout "$DURATION_S"`) were NOT affected and correctly
# self-terminated at ~300s; this was a single missing-termination bug
# on one process, not a concurrency/serialization defect.
#
# Usage: run_stationary_physical_diagnostic.sh [DURATION_S]

DURATION_S="${1:-300}"

source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash
TOOLS_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/06_physical_pipuck/single_device_bringup/tools"
OUT_DIR="/home/eamon/epuck_comm_bags/stationary_physical_diagnostic_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT_DIR"

echo "[$(date -Iseconds)] pre-flight /cmd_vel publisher count check"
CMD_VEL_PUB_COUNT="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo unknown)"
echo "cmd_vel_publisher_count_preflight=$CMD_VEL_PUB_COUNT"
if [[ "$CMD_VEL_PUB_COUNT" != "0" ]]; then
  echo "ERROR: /cmd_vel has $CMD_VEL_PUB_COUNT publisher(s), expected 0 -- aborting, nothing started." >&2
  exit 1
fi

echo "[$(date -Iseconds)] starting ${DURATION_S}s read-only stationary physical diagnostic (no publishers)"

RECORDER_PID=""
VALIDITY_PID=""
HZ_PIDS=()

cleanup() {
  # Explicit, targeted stop for the recorder (which has no internal
  # exit condition) -- the fix for the bug this script's docstring
  # describes. Never pkill; only the exact PID we started.
  if [[ -n "$RECORDER_PID" ]] && kill -0 "$RECORDER_PID" 2>/dev/null; then
    kill -INT "$RECORDER_PID" 2>/dev/null || true
    wait "$RECORDER_PID" 2>/dev/null || true
  fi
  if [[ -n "$VALIDITY_PID" ]] && kill -0 "$VALIDITY_PID" 2>/dev/null; then
    kill -INT "$VALIDITY_PID" 2>/dev/null || true
    wait "$VALIDITY_PID" 2>/dev/null || true
  fi
  for pid in "${HZ_PIDS[@]:-}"; do
    [[ -n "$pid" ]] || continue
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

# Bridge status + tier-A stats (reused, unmodified, read-only recorder).
# Has NO internal timeout -- this script's own `sleep "$DURATION_S"`
# below plus the `cleanup` trap's explicit `kill -INT` is what bounds
# its runtime, matching the established orchestrator pattern exactly.
python3 "$TOOLS_DIR/wsl_expanded_pilot_recorder.py" \
  --status-csv "$OUT_DIR/wsl_expanded_status.csv" \
  --cmd-vel-checkpoints-json "$OUT_DIR/cmd_vel_checkpoints.json" \
  --checkpoint-schedule-s 0.0 "$(python3 -c "print($DURATION_S/2)")" "$(python3 -c "print($DURATION_S-2)")" \
  > "$OUT_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!

# Continuous validity_flags subscription for the whole window -- this
# one IS self-bounding (timeout wraps the whole `ros2 topic echo`
# invocation, which exits cleanly on SIGTERM), confirmed by the first
# live run finishing at very close to the intended duration.
timeout "$DURATION_S" ros2 topic echo /epuck1/state --field validity_flags \
  > "$OUT_DIR/validity_flags_stream.log" 2>&1 &
VALIDITY_PID=$!

# Per-topic rate over the full window (not a 3s snapshot). Each is
# independently self-bounding via its own `timeout` wrapper.
for t in /odom /scan /tof /ps0 /ps1 /ps2 /ps3 /ps4 /ps5 /ps6 /ps7; do
  safe="$(echo "$t" | tr '/' '_')"
  timeout "$DURATION_S" ros2 topic hz "$t" --window 2000 \
    > "$OUT_DIR/hz${safe}.log" 2>&1 &
  HZ_PIDS+=("$!")
done

sleep 2
CMD_VEL_PUB_COUNT_MID="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo unknown)"
echo "cmd_vel_publisher_count_2s_after_start=$CMD_VEL_PUB_COUNT_MID"

echo "[$(date -Iseconds)] all read-only monitors launched, waiting ${DURATION_S}s (bounded by this script's own sleep + explicit cleanup, not by the recorder's own logic)..."
sleep "$DURATION_S"
echo "[$(date -Iseconds)] window elapsed -- stopping the recorder explicitly (it has no internal exit condition by design)"

trap - EXIT
cleanup

CMD_VEL_PUB_COUNT_END="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo unknown)"
echo "cmd_vel_publisher_count_end=$CMD_VEL_PUB_COUNT_END"

echo "=== OUT_DIR=$OUT_DIR ==="
ls -la "$OUT_DIR"
echo "STATIONARY_PHYSICAL_DIAGNOSTIC_COLLECTION_COMPLETE"
