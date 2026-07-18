#!/usr/bin/env bash
set -eo pipefail

# physical_single_device_zero_impairment_baseline_v1 -- per-trial
# orchestration, v2 (READY-barrier fix).
#
# Fixes the systematic short-window defect found in trial01_attempt01 and
# trial02_attempt01 (both confirmed independently: true bag/status_csv/
# system_csv overlap was only ~294-295s, below the 300s a centered 240s
# main window with >=30s buffers on both sides requires). Root cause: the
# old script started rosbag a fixed 1s after the status recorder (real
# subscription lag pushed the bag's real first message later still), and
# stopped the three processes sequentially (recorder -> sampler -> bag),
# so the status CSV's last row was always several seconds earlier than the
# bag's last message.
#
# v2 fix: explicit READY barriers (process alive + real data flowing +
# monotonic, for the CSV recorders; process alive + "Recording..." + all
# expected topics subscribed + no warn/error, for the bag) BEFORE a single
# unified T0 is recorded and the 315s formal timer starts from T0 (never
# from any process's own spawn instant). On the way down: bag is stopped
# FIRST and its process is waited-on to fully exit and its metadata.yaml/
# .db3 confirmed closed BEFORE the status recorder and system sampler are
# stopped -- so the bag's own window is always contained inside the
# status/system windows, not the other way around as before.
#
# DIAGNOSTIC/FORMAL-INTENT PHYSICAL, stationary only -- no ground motion,
# no controller, robot wheels suspended throughout. Assumes the Pi driver,
# Pi expanded TCP server, WSL expanded bridge, and a FRESH state_publisher
# for this trial are already running (this script does not touch any of
# them). Never publishes /cmd_vel. Never launches a controller. Does not
# touch the batch-level Pi system sampler (started/stopped once, outside
# this script, across the whole n=5 batch).
#
# Usage: run_baseline_v1_trial_v2.sh TRIAL_NAME   (e.g. trial01_attempt02)

if (( $# != 1 )); then
  echo "Usage: $0 TRIAL_NAME" >&2
  exit 2
fi

TRIAL="$1"
TOOLS_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/06_physical_pipuck/single_device_bringup/tools"
NATIVE_ROOT="/home/eamon/epuck_comm_bags"
NATIVE_BAG_DIR="${NATIVE_ROOT}/physical_single_device_zero_impairment_baseline_v1_${TRIAL}"
NATIVE_DIAG_DIR="${NATIVE_ROOT}/physical_single_device_zero_impairment_baseline_v1_${TRIAL}_diag"

FORMAL_DURATION_S=315
READY_TIMEOUT_S=15
EXPECTED_TOPICS="/epuck1/state,/epuck_bridge/status,/odom,/scan,/tof,/ps0,/ps1,/ps2,/ps3,/ps4,/ps5,/ps6,/ps7"

if [[ -e "$NATIVE_BAG_DIR" ]]; then
  echo "Refusing to overwrite existing trial: $NATIVE_BAG_DIR" >&2
  exit 1
fi
mkdir -p "$NATIVE_DIAG_DIR"

source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash

RECORDER_PID=""
BAG_PID=""
SAMPLER_PID=""

abort_cleanup() {
  # Only used for genuine abort paths (READY barrier failure) -- the
  # normal end-of-trial shutdown sequence below is explicit and ordered,
  # not routed through this generic handler.
  echo "[$(date -Iseconds)] ABORT: cleaning up any started processes" >&2
  for p in "$RECORDER_PID" "$SAMPLER_PID" "$BAG_PID"; do
    if [[ -n "$p" ]] && kill -0 "$p" 2>/dev/null; then
      kill -INT "$p" 2>/dev/null || true
    fi
  done
}
trap abort_cleanup ERR

echo "[$(date -Iseconds)] physical_single_device_zero_impairment_baseline_v1_${TRIAL} START (v2 orchestrator)"
echo "[$(date -Iseconds)] robot wheels suspended; no controller; this script never publishes /cmd_vel"

if ! ros2 topic list -t 2>/dev/null | grep -qx '/epuck1/state \[epuck2_comm_interfaces/msg/EpuckState\]'; then
  echo "ERROR: /epuck1/state not present with the expected EpuckState type" >&2
  exit 1
fi
CMD_VEL_PUB_COUNT="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo unknown)"
echo "[$(date -Iseconds)] pre-flight /cmd_vel publisher count: $CMD_VEL_PUB_COUNT"
if [[ "$CMD_VEL_PUB_COUNT" != "0" ]]; then
  echo "ERROR: /cmd_vel has $CMD_VEL_PUB_COUNT publisher(s) on the WSL graph at pre-flight, expected 0" >&2
  exit 1
fi

# --- Phase 1: start status recorder + WSL system sampler, wait for BOTH READY ---
python3 "$TOOLS_DIR/wsl_system_sampler.py" "$NATIVE_DIAG_DIR/wsl_system_metrics.csv" 1.0 "$((FORMAL_DURATION_S + 20))" &
SAMPLER_PID=$!
echo "[$(date -Iseconds)] wsl_system_sampler started, pid=$SAMPLER_PID"

python3 "$TOOLS_DIR/wsl_expanded_pilot_recorder.py" \
  --status-csv "$NATIVE_DIAG_DIR/wsl_expanded_status.csv" \
  --cmd-vel-checkpoints-json "$NATIVE_DIAG_DIR/cmd_vel_checkpoints.json" \
  --checkpoint-schedule-s 0.0 $((FORMAL_DURATION_S / 2)) $((FORMAL_DURATION_S - 2)) &
RECORDER_PID=$!
echo "[$(date -Iseconds)] wsl_expanded_pilot_recorder started, pid=$RECORDER_PID"

echo "[$(date -Iseconds)] waiting for status recorder READY..."
python3 "$TOOLS_DIR/wait_for_ready.py" csv \
  --path "$NATIVE_DIAG_DIR/wsl_expanded_status.csv" \
  --time-column wsl_unix_time_s --min-rows 2 \
  --pid "$RECORDER_PID" --timeout "$READY_TIMEOUT_S"

echo "[$(date -Iseconds)] waiting for WSL system sampler READY..."
python3 "$TOOLS_DIR/wait_for_ready.py" csv \
  --path "$NATIVE_DIAG_DIR/wsl_system_metrics.csv" \
  --time-column unix_time_s --min-rows 2 \
  --pid "$SAMPLER_PID" --timeout "$READY_TIMEOUT_S"

echo "[$(date -Iseconds)] status recorder + system sampler both READY"

# --- Phase 2: start rosbag, wait for READY (recording + all 13 topics subscribed) ---
ros2 bag record -o "$NATIVE_BAG_DIR" \
  /epuck1/state /epuck_bridge/status /odom /scan /tof \
  /ps0 /ps1 /ps2 /ps3 /ps4 /ps5 /ps6 /ps7 /cmd_vel \
  > "$NATIVE_DIAG_DIR/bag_record.log" 2>&1 &
BAG_PID=$!
echo "[$(date -Iseconds)] rosbag record started, pid=$BAG_PID"

echo "[$(date -Iseconds)] waiting for rosbag READY (recording + 13 topics subscribed)..."
python3 "$TOOLS_DIR/wait_for_ready.py" bag \
  --log-path "$NATIVE_DIAG_DIR/bag_record.log" \
  --topics "$EXPECTED_TOPICS" \
  --pid "$BAG_PID" --timeout "$READY_TIMEOUT_S"

echo "[$(date -Iseconds)] rosbag READY"

# --- T0: recorded only after ALL THREE processes are independently confirmed READY ---
T0_UNIX_S="$(date +%s.%N)"
echo "$T0_UNIX_S" > "$NATIVE_DIAG_DIR/t0_unix_s.txt"
echo "[$(date -Iseconds)] T0 = ${T0_UNIX_S} -- starting ${FORMAL_DURATION_S}s formal timer"

STATUS_JSON_START="$(timeout 3 ros2 topic echo /epuck_bridge/status --once --field data 2>/dev/null || true)"
echo "$STATUS_JSON_START" > "$NATIVE_DIAG_DIR/bridge_status_trial_start.json"

sleep "$FORMAL_DURATION_S"
echo "[$(date -Iseconds)] formal ${FORMAL_DURATION_S}s timer elapsed"

trap - ERR  # from here on, shutdown is explicit and ordered, not routed through abort_cleanup

# --- Shutdown order: bag FIRST, fully closed, THEN status recorder + sampler ---
echo "[$(date -Iseconds)] stopping rosbag recorder (FIRST)"
kill -INT "$BAG_PID"
wait "$BAG_PID" 2>/dev/null || true
BAG_PID=""
echo "[$(date -Iseconds)] rosbag recorder process exited"

if [[ ! -s "$NATIVE_BAG_DIR/metadata.yaml" ]]; then
  echo "ERROR: rosbag metadata missing or empty after shutdown" >&2
  exit 1
fi
echo "[$(date -Iseconds)] rosbag metadata.yaml confirmed present and non-empty"

STATUS_JSON_END="$(timeout 3 ros2 topic echo /epuck_bridge/status --once --field data 2>/dev/null || true)"
echo "$STATUS_JSON_END" > "$NATIVE_DIAG_DIR/bridge_status_trial_end.json"
echo "[$(date -Iseconds)] trial-end bridge status snapshot saved (captured right after bag fully closed)"

echo "[$(date -Iseconds)] stopping WSL expanded pilot recorder"
kill -INT "$RECORDER_PID"; wait "$RECORDER_PID" 2>/dev/null || true; RECORDER_PID=""

echo "[$(date -Iseconds)] stopping WSL system sampler"
kill -INT "$SAMPLER_PID" 2>/dev/null || true; wait "$SAMPLER_PID" 2>/dev/null || true; SAMPLER_PID=""

grep -iE 'drop|warn|error' "$NATIVE_DIAG_DIR/bag_record.log" > "$NATIVE_DIAG_DIR/bag_record_warnings.txt" || true
echo "[$(date -Iseconds)] bag_record.log warning/error lines: $(wc -l < "$NATIVE_DIAG_DIR/bag_record_warnings.txt")"

for t in /epuck1/state /epuck_bridge/status /odom /scan /tof /ps0 /ps1 /ps2 /ps3 /ps4 /ps5 /ps6 /ps7 /cmd_vel; do
  safe_name="$(echo "$t" | tr '/' '_')"
  ros2 topic info -v "$t" > "$NATIVE_DIAG_DIR/qos${safe_name}.txt" 2>&1 || true
done

ros2 bag info "$NATIVE_BAG_DIR" > "$NATIVE_DIAG_DIR/bag_info.txt" 2>&1 || true

echo "[$(date -Iseconds)] ${TRIAL} RECORDING_COMPLETE (v2 orchestrator, native path only -- not yet copied to Windows)"
echo "native bag dir: $NATIVE_BAG_DIR"
echo "native diag dir: $NATIVE_DIAG_DIR"
echo "T0_unix_s: $T0_UNIX_S"
