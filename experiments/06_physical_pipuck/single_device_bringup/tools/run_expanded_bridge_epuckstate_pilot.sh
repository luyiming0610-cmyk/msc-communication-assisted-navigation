#!/usr/bin/env bash
set -eo pipefail

# physical_expanded_bridge_epuckstate_integration_pilot01
#
# DIAGNOSTIC_PHYSICAL only -- not a formal baseline, no ground motion.
# Assumes the following are ALREADY running and confirmed healthy (this
# script does not start or touch them): Pi ROS2 driver, Pi expanded TCP
# server (pi_epuck_tcp_server_sensors.py), WSL expanded bridge
# (wsl_epuck_tcp_bridge_sensors.py), state_publisher (robot_id=1,
# source=hardware, use_sim_time=false, -r state:=/epuck1/state).
# Never publishes /cmd_vel. Never launches a controller.
#
# Usage: run_expanded_bridge_epuckstate_pilot.sh ATTEMPT_NAME

if (( $# != 1 )); then
  echo "Usage: $0 ATTEMPT_NAME" >&2
  exit 2
fi

ATTEMPT="$1"
TOOLS_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/06_physical_pipuck/single_device_bringup/tools"
NATIVE_ROOT="/home/eamon/epuck_comm_bags"
NATIVE_BAG_DIR="${NATIVE_ROOT}/physical_expanded_bridge_epuckstate_integration_pilot01_${ATTEMPT}"
NATIVE_DIAG_DIR="${NATIVE_ROOT}/physical_expanded_bridge_epuckstate_integration_pilot01_${ATTEMPT}_diag"

WARMUP_S=30
MAIN_S=240
TAIL_S=30
TOTAL_S=$((WARMUP_S + MAIN_S + TAIL_S))

if [[ -e "$NATIVE_BAG_DIR" ]]; then
  echo "Refusing to overwrite existing attempt: $NATIVE_BAG_DIR" >&2
  exit 1
fi
mkdir -p "$NATIVE_DIAG_DIR"

source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash

RECORDER_PID=""
BAG_PID=""
SAMPLER_PID=""

cleanup() {
  if [[ -n "$RECORDER_PID" ]] && kill -0 "$RECORDER_PID" 2>/dev/null; then
    kill -INT "$RECORDER_PID" 2>/dev/null || true
    wait "$RECORDER_PID" 2>/dev/null || true
  fi
  if [[ -n "$SAMPLER_PID" ]] && kill -0 "$SAMPLER_PID" 2>/dev/null; then
    kill -INT "$SAMPLER_PID" 2>/dev/null || true
    wait "$SAMPLER_PID" 2>/dev/null || true
  fi
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[$(date -Iseconds)] physical_expanded_bridge_epuckstate_integration_pilot01_${ATTEMPT} START (DIAGNOSTIC_PHYSICAL, ${TOTAL_S}s: ${WARMUP_S}s warmup / ${MAIN_S}s main / ${TAIL_S}s tail)"
echo "[$(date -Iseconds)] robot wheels suspended; no controller; this script never publishes /cmd_vel"

# Pre-flight: confirm bridge connected and state_publisher actually
# publishing EpuckState (not just that the process exists).
if ! ros2 topic list -t 2>/dev/null | grep -qx '/epuck1/state \[epuck2_comm_interfaces/msg/EpuckState\]'; then
  echo "ERROR: /epuck1/state not present with the expected EpuckState type" >&2
  exit 1
fi
STATUS_JSON="$(timeout 3 ros2 topic echo /epuck_bridge/status --once --field data 2>/dev/null || true)"
echo "[$(date -Iseconds)] pre-flight /epuck_bridge/status: $STATUS_JSON"
if [[ "$STATUS_JSON" != *'"connected": true'* && "$STATUS_JSON" != *'"connected":true'* ]]; then
  echo "ERROR: expanded bridge not connected at pre-flight check" >&2
  exit 1
fi
CMD_VEL_PUB_COUNT="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo unknown)"
echo "[$(date -Iseconds)] pre-flight /cmd_vel publisher count: $CMD_VEL_PUB_COUNT"
if [[ "$CMD_VEL_PUB_COUNT" != "0" ]]; then
  echo "ERROR: /cmd_vel has $CMD_VEL_PUB_COUNT publisher(s) on the WSL graph at pre-flight, expected 0" >&2
  exit 1
fi

python3 "$TOOLS_DIR/wsl_system_sampler.py" "$NATIVE_DIAG_DIR/wsl_system_metrics.csv" 1.0 "$TOTAL_S" &
SAMPLER_PID=$!

python3 "$TOOLS_DIR/wsl_expanded_pilot_recorder.py" \
  --status-csv "$NATIVE_DIAG_DIR/wsl_expanded_status.csv" \
  --cmd-vel-checkpoints-json "$NATIVE_DIAG_DIR/cmd_vel_checkpoints.json" \
  --checkpoint-schedule-s 0.0 150.0 $((TOTAL_S - 2)) &
RECORDER_PID=$!
sleep 1

ros2 bag record -o "$NATIVE_BAG_DIR" \
  /epuck1/state /epuck_bridge/status /odom /scan /tof \
  /ps0 /ps1 /ps2 /ps3 /ps4 /ps5 /ps6 /ps7 /cmd_vel \
  > "$NATIVE_DIAG_DIR/bag_record.log" 2>&1 &
BAG_PID=$!
sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "ERROR: rosbag recorder exited before recording could start" >&2
  exit 1
fi
echo "[$(date -Iseconds)] rosbag recording to NATIVE path: $NATIVE_BAG_DIR"

echo "[$(date -Iseconds)] WARMUP phase (${WARMUP_S}s)"
sleep "$WARMUP_S"
echo "[$(date -Iseconds)] MAIN phase start"
sleep "$MAIN_S"
echo "[$(date -Iseconds)] MAIN phase end / TAIL phase start (${TAIL_S}s)"
sleep "$TAIL_S"
echo "[$(date -Iseconds)] TAIL phase end"

echo "[$(date -Iseconds)] stopping WSL expanded pilot recorder"
kill -INT "$RECORDER_PID"; wait "$RECORDER_PID" 2>/dev/null || true; RECORDER_PID=""

echo "[$(date -Iseconds)] stopping WSL system sampler"
kill -INT "$SAMPLER_PID" 2>/dev/null || true; wait "$SAMPLER_PID" 2>/dev/null || true; SAMPLER_PID=""

echo "[$(date -Iseconds)] stopping rosbag recorder"
kill -INT "$BAG_PID"; wait "$BAG_PID" 2>/dev/null || true; BAG_PID=""

if [[ ! -s "$NATIVE_BAG_DIR/metadata.yaml" ]]; then
  echo "ERROR: rosbag metadata missing or empty" >&2
  exit 1
fi
echo "[$(date -Iseconds)] rosbag metadata.yaml confirmed present and non-empty"

grep -iE 'drop|warn|error' "$NATIVE_DIAG_DIR/bag_record.log" > "$NATIVE_DIAG_DIR/bag_record_warnings.txt" || true
echo "[$(date -Iseconds)] bag_record.log warning/error lines: $(wc -l < "$NATIVE_DIAG_DIR/bag_record_warnings.txt")"

for t in /epuck1/state /epuck_bridge/status /odom /scan /tof /ps0 /ps1 /ps2 /ps3 /ps4 /ps5 /ps6 /ps7 /cmd_vel; do
  safe_name="$(echo "$t" | tr '/' '_')"
  ros2 topic info -v "$t" > "$NATIVE_DIAG_DIR/qos${safe_name}.txt" 2>&1 || true
done

ros2 bag info "$NATIVE_BAG_DIR" > "$NATIVE_DIAG_DIR/bag_info.txt" 2>&1 || true

echo "[$(date -Iseconds)] $ATTEMPT DIAGNOSTIC RECORDING_COMPLETE (native path only -- not yet copied to Windows)"
echo "native bag dir: $NATIVE_BAG_DIR"
echo "native diag dir: $NATIVE_DIAG_DIR"
trap - EXIT
