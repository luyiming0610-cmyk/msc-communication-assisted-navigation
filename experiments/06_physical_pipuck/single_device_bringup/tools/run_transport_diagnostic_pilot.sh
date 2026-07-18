#!/usr/bin/env bash
set -eo pipefail

# physical_single_device_transport_diagnostic_pilot01
#
# DIAGNOSTIC_PHYSICAL only -- not a formal paper batch, does not validate
# the current EpuckState protocol, does not validate avoidance behaviour.
# Assumes the three processes are ALREADY running and confirmed healthy
# (Pi ROS2 driver, Pi TCP server on 5809, WSL TCP client) -- this script
# does not start or touch any of them. Never publishes /cmd_vel. Robot
# must be stationary (wheels suspended or on a stand) for the whole run.
#
# Usage: run_transport_diagnostic_pilot.sh ATTEMPT_NAME

if (( $# != 1 )); then
  echo "Usage: $0 ATTEMPT_NAME" >&2
  exit 2
fi

ATTEMPT="$1"
TOOLS_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/06_physical_pipuck/single_device_bringup/tools"
FINAL_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/06_physical_pipuck/single_device_bringup/physical_single_device_transport_diagnostic_pilot01_${ATTEMPT}"
NATIVE_ROOT="/home/eamon/epuck_comm_bags"
NATIVE_BAG_DIR="${NATIVE_ROOT}/physical_single_device_transport_diagnostic_pilot01_${ATTEMPT}"
NATIVE_DIAG_DIR="${NATIVE_ROOT}/physical_single_device_transport_diagnostic_pilot01_${ATTEMPT}_diag"

WARMUP_S=30
MAIN_S=240
TAIL_S=30
TOTAL_S=$((WARMUP_S + MAIN_S + TAIL_S))

if [[ -e "$NATIVE_BAG_DIR" || -e "$FINAL_DIR" ]]; then
  echo "Refusing to overwrite existing attempt: $NATIVE_BAG_DIR or $FINAL_DIR" >&2
  exit 1
fi
mkdir -p "$NATIVE_DIAG_DIR"

source /opt/ros/humble/setup.bash

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

echo "[$(date -Iseconds)] physical_single_device_transport_diagnostic_pilot01_${ATTEMPT} START (DIAGNOSTIC_PHYSICAL, ${TOTAL_S}s total: ${WARMUP_S}s warmup / ${MAIN_S}s main / ${TAIL_S}s tail)"
echo "[$(date -Iseconds)] robot must be stationary; this script never publishes /cmd_vel"

# Pre-flight: confirm the three processes are actually healthy before
# starting anything, rather than assuming.
if ! ros2 topic list 2>/dev/null | grep -qx '/epuck_bridge/status'; then
  echo "ERROR: /epuck_bridge/status not present -- WSL bridge not running?" >&2
  exit 1
fi
STATUS_JSON="$(timeout 3 ros2 topic echo /epuck_bridge/status --once --field data 2>/dev/null || true)"
echo "[$(date -Iseconds)] pre-flight /epuck_bridge/status: $STATUS_JSON"
if [[ "$STATUS_JSON" != *'"connected": true'* && "$STATUS_JSON" != *'"connected":true'* ]]; then
  echo "ERROR: bridge not connected at pre-flight check" >&2
  exit 1
fi

python3 "$TOOLS_DIR/wsl_system_sampler.py" "$NATIVE_DIAG_DIR/wsl_system_metrics.csv" 1.0 "$TOTAL_S" &
SAMPLER_PID=$!

python3 "$TOOLS_DIR/wsl_transport_recorder.py" \
  --output-csv "$NATIVE_DIAG_DIR/wsl_transport_status.csv" \
  --totals-json "$NATIVE_DIAG_DIR/wsl_transport_totals.json" &
RECORDER_PID=$!
sleep 1

ros2 bag record -o "$NATIVE_BAG_DIR" \
  /scan /odom /epuck_bridge/status /cmd_vel \
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

echo "[$(date -Iseconds)] stopping WSL transport recorder"
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

ros2 topic info -v /scan  > "$NATIVE_DIAG_DIR/qos_scan.txt" 2>&1 || true
ros2 topic info -v /odom > "$NATIVE_DIAG_DIR/qos_odom.txt" 2>&1 || true
ros2 topic info -v /epuck_bridge/status > "$NATIVE_DIAG_DIR/qos_status.txt" 2>&1 || true
ros2 topic info -v /cmd_vel > "$NATIVE_DIAG_DIR/qos_cmd_vel.txt" 2>&1 || true

ros2 bag info "$NATIVE_BAG_DIR" > "$NATIVE_DIAG_DIR/bag_info.txt" 2>&1 || true

echo "[$(date -Iseconds)] $ATTEMPT DIAGNOSTIC RECORDING_COMPLETE (native path only -- not yet copied to Windows)"
echo "native bag dir: $NATIVE_BAG_DIR"
echo "native diag dir: $NATIVE_DIAG_DIR"
trap - EXIT
