#!/usr/bin/env bash
set -eo pipefail

# Measurement-chain isolation diagnostic (comm_baseline_native_trial01).
# Goal: determine whether the ~40-45% message shortfall seen in
# comm_baseline_v1 Trials 01-03 (all writing their rosbag under /mnt/c) is
# a rosbag/disk-I/O issue, or a genuine loss somewhere in
# publisher->relay->subscriber. Zero impairment throughout (delay=0,
# jitter=0, drop=0). No cooperative_avoider is launched -- this isolates
# the communication layer only, deliberately excluding avoidance behaviour.
#
# Launch order (per diagnostic requirement): relay + sequence_counter
# FIRST, then state_publisher, so their subscriptions exist from
# sequence 0. rosbag writes to a NATIVE WSL ext4 path
# (/home/eamon/epuck_comm_bags/...), not /mnt/c, and is only copied to the
# Windows experiment tree AFTER it has been cleanly stopped and closed.
#
# Usage: run_comm_baseline_native_diagnostic.sh TRIAL_NAME

if (( $# != 1 )); then
  echo "Usage: $0 TRIAL_NAME" >&2
  exit 2
fi

TRIAL="$1"
WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/2-1.仿真通信实验/working"
EXPERIMENT_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/3-3.全传感器避障实验"
CONFIG_DIR="$EXPERIMENT_DIR/config/comm_baseline_v1"
STEM="comm_baseline_native_${TRIAL}"
NATIVE_BAG_ROOT="/home/eamon/epuck_comm_bags"
NATIVE_BAG_DIR="$NATIVE_BAG_ROOT/$STEM"
FINAL_BAG_DIR="$EXPERIMENT_DIR/bags/controller_v4_full_sensor_bypass_20260717_${STEM}"
DIAG_LOG_DIR="$NATIVE_BAG_ROOT/${STEM}_diag_logs"
EXECUTION_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_execution.log"
SIM_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_simulation.log"
STATE1_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_state_epuck1.log"
STATE2_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_state_epuck2.log"
QOS_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_qos_info.log"
BAG_RECORD_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_bag_record.log"
DIAGNOSTIC_DURATION_S=40
RELAY_COUNTER_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_relay_counter.log"

source /opt/ros/humble/setup.bash
source "$HOME/epuck_ws/install/setup.bash"
set -u

if [[ -e "$NATIVE_BAG_DIR" || -e "$FINAL_BAG_DIR" ]]; then
  echo "Refusing to overwrite existing bag: $NATIVE_BAG_DIR or $FINAL_BAG_DIR" >&2
  exit 1
fi
mkdir -p "$DIAG_LOG_DIR"

stop_pid() {
  local pid="${1:-}"
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi
  kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.5
  done
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 10); do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.5
  done
  return 1
}

wait_for_topics() {
  local timeout_s="$1"
  shift
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    local topics
    topics="$(ros2 topic list 2>/dev/null || true)"
    local ready=1
    for expected in "$@"; do
      if ! grep -Fxq "$expected" <<<"$topics"; then
        ready=0
        break
      fi
    done
    if (( ready == 1 )); then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for topics: $*" >&2
  return 1
}

verify_realtime_factor() {
  local stage="$1"
  RATE_STAGE="$stage" python3 - <<'PY'
import os
import sys
import time

import rclpy
from rosgraph_msgs.msg import Clock

stage = os.environ["RATE_STAGE"]
rclpy.init()
node = rclpy.create_node(f"comm_baseline_native_{stage.lower()}_clock_rate_check")
samples = []

def callback(message):
    sim_s = float(message.clock.sec) + float(message.clock.nanosec) / 1.0e9
    samples.append((time.monotonic(), sim_s))

node.create_subscription(Clock, "/clock", callback, 20)
deadline = time.monotonic() + 8.0
first_wall = None
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if samples and first_wall is None:
        first_wall = samples[0][0]
    if first_wall is not None and samples[-1][0] - first_wall >= 3.0:
        break

node.destroy_node()
rclpy.shutdown()
if len(samples) < 2:
    print(f"{stage}_RATE_FAIL no usable /clock samples", file=sys.stderr)
    raise SystemExit(1)
wall_delta = samples[-1][0] - samples[0][0]
sim_delta = samples[-1][1] - samples[0][1]
factor = sim_delta / wall_delta if wall_delta > 0.0 else 0.0
print(f"{stage}_REALTIME_FACTOR={factor:.3f}")
if factor < 0.8 or factor > 1.2:
    print(f"{stage}_RATE_FAIL factor outside 0.8-1.2", file=sys.stderr)
    raise SystemExit(1)
PY
}

SIM_PID=""
RELAY_COUNTER_PID=""
STATE1_PID=""
STATE2_PID=""
BAG_PID=""

cleanup() {
  stop_pid "$STATE1_PID" || true
  stop_pid "$STATE2_PID" || true
  stop_pid "$RELAY_COUNTER_PID" || true
  stop_pid "$BAG_PID" || true
  stop_pid "$SIM_PID" || true
}
trap cleanup EXIT

echo "[$(date -Iseconds)] $STEM START (measurement-chain isolation diagnostic)" | tee "$EXECUTION_LOG"
/mnt/c/Windows/System32/cmd.exe /c echo WSL_INTEROP_OK | tee -a "$EXECUTION_LOG"

echo "no residual process check:" | tee -a "$EXECUTION_LOG"
pgrep -af 'webots-bin|cooperative_avoider|state_publisher|ros2 bag record|network_impairment_relay|sequence_counter' | tee -a "$EXECUTION_LOG" || echo "  (none)" | tee -a "$EXECUTION_LOG"

(
  cd "$WORK_DIR"
  exec python3 run_dual_head_on_clean.py
) >"$SIM_LOG" 2>&1 &
SIM_PID=$!

wait_for_topics 90 /epuck1/odom /epuck2/odom /epuck1/tof /epuck2/tof /epuck1/ps0 /epuck2/ps0
echo "[$(date -Iseconds)] odometry and local sensors ready" | tee -a "$EXECUTION_LOG"
PRELOAD_OUTPUT="$(verify_realtime_factor PRELOAD | tee -a "$EXECUTION_LOG")"
PRELOAD_FACTOR="$(grep -o 'PRELOAD_REALTIME_FACTOR=[0-9.]*' <<<"$PRELOAD_OUTPUT" | cut -d= -f2)"

# Relay + counting subscriber FIRST, before state_publisher exists at all.
stdbuf -oL -eL python3 "$CONFIG_DIR/run_diagnostic_relay_and_counter.py" "$DIAG_LOG_DIR" "$DIAG_LOG_DIR" \
  >"$RELAY_COUNTER_LOG" 2>&1 &
RELAY_COUNTER_PID=$!
sleep 2
echo "[$(date -Iseconds)] relay + sequence_counter subscribed (no state_raw exists yet)" | tee -a "$EXECUTION_LOG"

ros2 run epuck2_comm state_publisher --ros-args \
  -r __ns:=/epuck1 -r state:=state_raw -p robot_id:=1 -p use_sim_time:=true \
  -p mode:=periodic -p origin_x_m:=-0.35 -p origin_y_m:=0.0 \
  -p origin_yaw_rad:=0.0 >"$STATE1_LOG" 2>&1 &
STATE1_PID=$!

ros2 run epuck2_comm state_publisher --ros-args \
  -r __ns:=/epuck2 -r state:=state_raw -p robot_id:=2 -p use_sim_time:=true \
  -p mode:=periodic -p origin_x_m:=0.35 -p origin_y_m:=0.0 \
  -p origin_yaw_rad:=3.141592653589793 >"$STATE2_LOG" 2>&1 &
STATE2_PID=$!

wait_for_topics 30 /epuck1/state_raw /epuck2/state_raw /epuck1/state /epuck2/state
sleep 2
echo "[$(date -Iseconds)] state_raw and relayed state both ready" | tee -a "$EXECUTION_LOG"

echo "=== QoS: /epuck1/state_raw ===" | tee -a "$QOS_LOG"
ros2 topic info -v /epuck1/state_raw 2>&1 | tee -a "$QOS_LOG"
echo "=== QoS: /epuck1/state ===" | tee -a "$QOS_LOG"
ros2 topic info -v /epuck1/state 2>&1 | tee -a "$QOS_LOG"
echo "=== QoS: /epuck2/state_raw ===" | tee -a "$QOS_LOG"
ros2 topic info -v /epuck2/state_raw 2>&1 | tee -a "$QOS_LOG"
echo "=== QoS: /epuck2/state ===" | tee -a "$QOS_LOG"
ros2 topic info -v /epuck2/state 2>&1 | tee -a "$QOS_LOG"
echo "[$(date -Iseconds)] QoS info captured to $QOS_LOG" | tee -a "$EXECUTION_LOG"

mkdir -p "$NATIVE_BAG_ROOT"
ros2 bag record -o "$NATIVE_BAG_DIR" \
  /epuck1/state_raw /epuck2/state_raw /epuck1/state /epuck2/state \
  >"$BAG_RECORD_LOG" 2>&1 &
BAG_PID=$!
sleep 3
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "rosbag recorder exited before recording could start" >&2
  exit 1
fi
echo "[$(date -Iseconds)] rosbag recording to NATIVE path: $NATIVE_BAG_DIR" | tee -a "$EXECUTION_LOG"

FULL_LOAD_OUTPUT="$(verify_realtime_factor FULL_LOAD | tee -a "$EXECUTION_LOG")"
FULL_LOAD_FACTOR="$(grep -o 'FULL_LOAD_REALTIME_FACTOR=[0-9.]*' <<<"$FULL_LOAD_OUTPUT" | cut -d= -f2)"

echo "[$(date -Iseconds)] running fixed diagnostic window: ${DIAGNOSTIC_DURATION_S}s" | tee -a "$EXECUTION_LOG"
sleep "$DIAGNOSTIC_DURATION_S"

# Shutdown order per diagnostic requirement: state_publisher -> relay
# (SIGINT lets sequence_counter write its JSON and the relay close its
# CSV cleanly) -> rosbag (wait for a clean close) -> analysis -> only
# THEN copy the closed bag to the Windows experiment tree.
echo "[$(date -Iseconds)] stopping state_publisher" | tee -a "$EXECUTION_LOG"
stop_pid "$STATE1_PID"; STATE1_PID=""
stop_pid "$STATE2_PID"; STATE2_PID=""

echo "[$(date -Iseconds)] stopping relay + sequence_counter" | tee -a "$EXECUTION_LOG"
stop_pid "$RELAY_COUNTER_PID"; RELAY_COUNTER_PID=""
sleep 1

echo "[$(date -Iseconds)] stopping rosbag recorder" | tee -a "$EXECUTION_LOG"
stop_pid "$BAG_PID"; BAG_PID=""

if [[ ! -s "$NATIVE_BAG_DIR/metadata.yaml" ]]; then
  echo "rosbag metadata is missing or empty (native path)" >&2
  exit 1
fi
echo "[$(date -Iseconds)] rosbag metadata.yaml confirmed present and non-empty" | tee -a "$EXECUTION_LOG"

stop_pid "$SIM_PID"; SIM_PID=""
sleep 2

grep -iE 'drop|warn|error' "$BAG_RECORD_LOG" | tee -a "$EXECUTION_LOG" || echo "no drop/warn/error lines in bag_record.log" | tee -a "$EXECUTION_LOG"

python3 "$CONFIG_DIR/analyze_measurement_chain.py" \
  --native-bag-dir "$NATIVE_BAG_DIR" \
  --diag-log-dir "$DIAG_LOG_DIR" \
  --state1-log "$STATE1_LOG" \
  --state2-log "$STATE2_LOG" \
  --bag-record-log "$BAG_RECORD_LOG" \
  --relay-counter-log "$RELAY_COUNTER_LOG" \
  --preload-factor "${PRELOAD_FACTOR:-0}" \
  --full-load-factor "${FULL_LOAD_FACTOR:-0}" \
  --output-path "$DIAG_LOG_DIR/measurement_chain_verdict.json" \
  | tee -a "$EXECUTION_LOG"

VERDICT="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['verdict'])" "$DIAG_LOG_DIR/measurement_chain_verdict.json" 2>/dev/null || echo UNKNOWN)"
echo "[$(date -Iseconds)] measurement-chain diagnostic verdict=$VERDICT" | tee -a "$EXECUTION_LOG"

# Only now copy the CLOSED native bag (and diagnostic logs) into the
# Windows experiment tree, as required.
mkdir -p "$FINAL_BAG_DIR/analysis"
cp -r "$NATIVE_BAG_DIR"/. "$FINAL_BAG_DIR/"
cp "$DIAG_LOG_DIR"/*.json "$FINAL_BAG_DIR/analysis/" 2>/dev/null || true
cp "$QOS_LOG" "$FINAL_BAG_DIR/analysis/qos_info.log"
cp "$BAG_RECORD_LOG" "$FINAL_BAG_DIR/analysis/bag_record.log"
echo "[$(date -Iseconds)] closed native bag copied to $FINAL_BAG_DIR" | tee -a "$EXECUTION_LOG"

echo "[$(date -Iseconds)] $STEM RECORDING_COMPLETE verdict=$VERDICT" | tee -a "$EXECUTION_LOG"
trap - EXIT
