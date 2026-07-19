#!/usr/bin/env bash
set -eo pipefail

# objective5_timestamp_latency_validation_pilot01 -- diagnostic-only pilot
# whose sole purpose is validating that EpuckState.stamp + the live
# sequence_counter latency measurement actually work as intended, BEFORE
# any impairment-matrix trial is run. NOT a formal trial: no
# cooperative_avoider is launched, and this pilot's results are never
# pooled with formal Objective5 statistics.
#
# Usage: run_timestamp_latency_validation_pilot.sh CONDITION_LABEL DELAY_S JITTER_S DROP_PROB SEED COLLECT_S
# e.g.:  run_timestamp_latency_validation_pilot.sh condition_a_delay0 0.0 0.0 0.0 4001 20
#        run_timestamp_latency_validation_pilot.sh condition_b_delay025 0.25 0.0 0.0 4001 20

if (( $# != 6 )); then
  echo "Usage: $0 CONDITION_LABEL DELAY_S JITTER_S DROP_PROB SEED COLLECT_S" >&2
  exit 2
fi

CONDITION="$1"
DELAY_S="$2"
JITTER_S="$3"
DROP_PROB="$4"
SEED="$5"
COLLECT_S="$6"

WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/2-1.仿真通信实验/working"
EXPERIMENT_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/3-3.全传感器避障实验"
CONFIG_DIR="$EXPERIMENT_DIR/config/comm_baseline_v1"
STEM="objective5_timestamp_latency_validation_pilot01_${CONDITION}"
NATIVE_BAG_ROOT="/home/eamon/epuck_comm_bags"
NATIVE_BAG_DIR="$NATIVE_BAG_ROOT/$STEM"
FINAL_BAG_DIR="$EXPERIMENT_DIR/bags/controller_v4_full_sensor_bypass_20260717_${STEM}"
DIAG_LOG_DIR="$NATIVE_BAG_ROOT/${STEM}_diag_logs"
EXECUTION_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_execution.log"
SIM_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_simulation.log"
STATE1_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_state_epuck1.log"
STATE2_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_state_epuck2.log"
BAG_RECORD_LOG="$EXPERIMENT_DIR/logs/controller_v4_full_sensor_bypass_20260717_${STEM}_bag_record.log"
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

stop_pid_group() {
  local pid="${1:-}"
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi
  kill -INT "-$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    if ! kill -0 "$pid" 2>/dev/null && ! pgrep -g "$pid" >/dev/null 2>&1; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.5
  done
  kill -TERM "-$pid" 2>/dev/null || true
  for _ in $(seq 1 10); do
    if ! kill -0 "$pid" 2>/dev/null && ! pgrep -g "$pid" >/dev/null 2>&1; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.5
  done
  kill -KILL "-$pid" 2>/dev/null || true
  sleep 0.5
  wait "$pid" 2>/dev/null || true
  if kill -0 "$pid" 2>/dev/null || pgrep -g "$pid" >/dev/null 2>&1; then
    return 1
  fi
  return 0
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
node = rclpy.create_node(f"pilot01_{stage.lower()}_clock_rate_check")
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
  stop_pid_group "$RELAY_COUNTER_PID" || true
  stop_pid "$BAG_PID" || true
  stop_pid "$SIM_PID" || true
}
trap cleanup EXIT

echo "[$(date -Iseconds)] $STEM START (delay_s=$DELAY_S jitter_s=$JITTER_S drop=$DROP_PROB seed=$SEED)" | tee "$EXECUTION_LOG"

(
  cd "$WORK_DIR"
  exec python3 run_dual_head_on_clean.py
) >"$SIM_LOG" 2>&1 &
SIM_PID=$!

wait_for_topics 90 /epuck1/odom /epuck2/odom /epuck1/tof /epuck2/tof /epuck1/ps0 /epuck2/ps0
echo "[$(date -Iseconds)] odometry and local sensors ready" | tee -a "$EXECUTION_LOG"
PRELOAD_OUTPUT="$(verify_realtime_factor PRELOAD | tee -a "$EXECUTION_LOG")"
PRELOAD_FACTOR="$(grep -o 'PRELOAD_REALTIME_FACTOR=[0-9.]*' <<<"$PRELOAD_OUTPUT" | cut -d= -f2)"

setsid stdbuf -oL -eL python3 "$CONFIG_DIR/run_relay_counter_configurable.py" \
  --relay-log-dir "$DIAG_LOG_DIR" --counter-log-dir "$DIAG_LOG_DIR" \
  --delay-s "$DELAY_S" --jitter-s "$JITTER_S" --drop-probability "$DROP_PROB" --seed "$SEED" \
  >"$RELAY_COUNTER_LOG" 2>&1 &
RELAY_COUNTER_PID=$!
sleep 2
echo "[$(date -Iseconds)] relay (delay_s=$DELAY_S) + sequence_counter subscribed" | tee -a "$EXECUTION_LOG"

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

echo "[$(date -Iseconds)] collecting for ${COLLECT_S}s" | tee -a "$EXECUTION_LOG"
sleep "$COLLECT_S"

echo "[$(date -Iseconds)] stopping state_publisher" | tee -a "$EXECUTION_LOG"
stop_pid "$STATE1_PID"; STATE1_PID=""
stop_pid "$STATE2_PID"; STATE2_PID=""

echo "[$(date -Iseconds)] stopping relay + sequence_counter" | tee -a "$EXECUTION_LOG"
stop_pid_group "$RELAY_COUNTER_PID"; RELAY_COUNTER_PID=""
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

python3 -m epuck2_comm.analyze_comm_performance "$NATIVE_BAG_DIR" --output-dir "$DIAG_LOG_DIR" >>"$EXECUTION_LOG" 2>&1

mkdir -p "$FINAL_BAG_DIR/analysis"
cp -r "$NATIVE_BAG_DIR"/. "$FINAL_BAG_DIR/"
cp "$DIAG_LOG_DIR"/*.json "$FINAL_BAG_DIR/analysis/" 2>/dev/null || true
cp "$DIAG_LOG_DIR"/*.csv "$FINAL_BAG_DIR/analysis/" 2>/dev/null || true
echo "[$(date -Iseconds)] closed native bag copied to $FINAL_BAG_DIR" | tee -a "$EXECUTION_LOG"

cat > "$DIAG_LOG_DIR/pilot_condition_config.json" <<CFGEOF
{
  "condition": "$CONDITION",
  "configured_delay_s": $DELAY_S,
  "configured_jitter_s": $JITTER_S,
  "configured_drop_probability": $DROP_PROB,
  "seed": $SEED,
  "collect_s": $COLLECT_S,
  "preload_realtime_factor": ${PRELOAD_FACTOR:-0},
  "full_load_realtime_factor": ${FULL_LOAD_FACTOR:-0}
}
CFGEOF
cp "$DIAG_LOG_DIR/pilot_condition_config.json" "$FINAL_BAG_DIR/analysis/" 2>/dev/null || true

echo "[$(date -Iseconds)] $STEM RECORDING_COMPLETE" | tee -a "$EXECUTION_LOG"
trap - EXIT
