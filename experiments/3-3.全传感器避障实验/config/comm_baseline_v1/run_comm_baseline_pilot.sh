#!/usr/bin/env bash
set -eo pipefail

# Objective 5 step 3: exclusionary communication baseline pilot.
# Verifies the network_impairment_relay (zero impairment) and
# analyze_comm_performance.py against a real recorded run before any
# actual delay/jitter/loss trial is attempted. Uses the same frozen
# head_on_cpa_v4-style geometry and CPA parameters as pilot_v4_c (pure
# dual-robot CPA, no box) -- the ONLY difference is that each robot's
# state_publisher now publishes to "state_raw" and a
# network_impairment_relay (configured for zero impairment) republishes
# to "state", which cooperative_avoider still subscribes to unmodified.
#
# This pilot is EXCLUDED from all formal statistics.
#
# Usage: run_comm_baseline_pilot.sh TRIAL_NAME

if (( $# != 1 )); then
  echo "Usage: $0 TRIAL_NAME" >&2
  exit 2
fi

TRIAL="$1"
WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/2-1.仿真通信实验/working"
EXPERIMENT_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/3-3.全传感器避障实验"
CONFIG_DIR="$EXPERIMENT_DIR/config/comm_baseline_v1"
STEM="controller_v4_full_sensor_bypass_20260717_comm_baseline_${TRIAL}"
BAG_DIR="$EXPERIMENT_DIR/bags/$STEM"
CONTROLLER_LOG="$EXPERIMENT_DIR/logs/$STEM.log"
EXECUTION_LOG="$EXPERIMENT_DIR/logs/${STEM}_execution.log"
SIM_LOG="$EXPERIMENT_DIR/logs/${STEM}_simulation.log"
STATE1_LOG="$EXPERIMENT_DIR/logs/${STEM}_state_epuck1.log"
STATE2_LOG="$EXPERIMENT_DIR/logs/${STEM}_state_epuck2.log"

source /opt/ros/humble/setup.bash
source "$HOME/epuck_ws/install/setup.bash"
set -u

if [[ -e "$BAG_DIR" ]]; then
  echo "Refusing to overwrite existing bag: $BAG_DIR" >&2
  exit 1
fi

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
node = rclpy.create_node(f"comm_baseline_{stage.lower()}_clock_rate_check")
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
STATE1_PID=""
STATE2_PID=""
BAG_PID=""
CONTROLLER_PID=""

cleanup() {
  stop_pid "$CONTROLLER_PID" || true
  stop_pid "$BAG_PID" || true
  stop_pid "$STATE2_PID" || true
  stop_pid "$STATE1_PID" || true
  stop_pid "$SIM_PID" || true
}
trap cleanup EXIT

echo "[$(date -Iseconds)] $STEM START" | tee "$EXECUTION_LOG"
/mnt/c/Windows/System32/cmd.exe /c echo WSL_INTEROP_OK | tee -a "$EXECUTION_LOG"

(
  cd "$WORK_DIR"
  exec python3 run_dual_head_on_clean.py
) >"$SIM_LOG" 2>&1 &
SIM_PID=$!

wait_for_topics 90 /epuck1/odom /epuck2/odom /epuck1/tof /epuck2/tof /epuck1/ps0 /epuck2/ps0
echo "[$(date -Iseconds)] odometry and local sensors ready" | tee -a "$EXECUTION_LOG"
PRELOAD_OUTPUT="$(verify_realtime_factor PRELOAD | tee -a "$EXECUTION_LOG")"
PRELOAD_FACTOR="$(grep -o 'PRELOAD_REALTIME_FACTOR=[0-9.]*' <<<"$PRELOAD_OUTPUT" | cut -d= -f2)"

# controller_v4_comm_baseline_20260717 fix: the relays (bundled with the
# controllers in run_comm_baseline_controllers.py) MUST subscribe to
# state_raw before state_publisher starts publishing it -- ROS2's default
# QoS is VOLATILE (no history for late joiners), so a relay that starts
# subscribing after state_publisher has already been running for even a
# few seconds will genuinely miss every message published before its
# subscription connected. This is a launch-ordering artifact, not relay
# loss; the fix is to start the relay+controller bundle FIRST, before
# state_publisher, so the relay's subscription exists from sequence 0.
stdbuf -oL -eL python3 "$CONFIG_DIR/run_comm_baseline_controllers.py" "$STEM" \
  >"$CONTROLLER_LOG" 2>&1 &
CONTROLLER_PID=$!
sleep 2
echo "[$(date -Iseconds)] relays and controllers ready (subscribed before state_raw exists)" | tee -a "$EXECUTION_LOG"

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
echo "[$(date -Iseconds)] communicated state_raw and relayed state both ready" | tee -a "$EXECUTION_LOG"

# controller_v4_comm_baseline_20260717 fix #2: the full 28-topic recording
# list used by other pilots (all 16 per-robot ps0-ps7 IR sensors + tof +
# odom + cmd_vel + state, x2 robots) overloaded rosbag2's recorder under
# this trial's message volume -- proven directly: the relay's own log
# showed a perfect, gap-free 0..759 sequence (spanning ~83s at ~9.1Hz,
# matching the configured 10Hz publish rate), while the bag itself only
# captured ~440-460 of those same messages (~55%). None of the local
# IR/ToF/odom topics are used by analyze_comm_performance or this
# baseline's verification, so they are dropped from the recorded topic
# list here (comm-baseline pilots only) rather than papering over the
# resulting bag-vs-relay count mismatch.
ros2 bag record -o "$BAG_DIR" \
  /epuck1/state_raw /epuck2/state_raw /epuck1/state /epuck2/state \
  /epuck1/cmd_vel /epuck2/cmd_vel \
  >>"$EXECUTION_LOG" 2>&1 &
BAG_PID=$!
sleep 3
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "rosbag recorder exited before recording could start" >&2
  exit 1
fi
echo "[$(date -Iseconds)] rosbag recording" | tee -a "$EXECUTION_LOG"

FULL_LOAD_OUTPUT="$(verify_realtime_factor FULL_LOAD | tee -a "$EXECUTION_LOG")"
FULL_LOAD_FACTOR="$(grep -o 'FULL_LOAD_REALTIME_FACTOR=[0-9.]*' <<<"$FULL_LOAD_OUTPUT" | cut -d= -f2)"

deadline=$((SECONDS + 75))
complete_count=0
while (( SECONDS < deadline )); do
  complete_count="$(grep -c 'COMPLETE: cooperative recovery completed; commanding zero' "$CONTROLLER_LOG" 2>/dev/null || true)"
  if (( complete_count >= 2 )); then
    break
  fi
  if ! kill -0 "$CONTROLLER_PID" 2>/dev/null; then
    echo "Controller exited before both robots completed" >&2
    break
  fi
  sleep 0.5
done
echo "[$(date -Iseconds)] controller stage finished (complete_count=$complete_count)" | tee -a "$EXECUTION_LOG"

sleep 1
stop_pid "$CONTROLLER_PID"
CONTROLLER_PID=""
stop_pid "$BAG_PID"
BAG_PID=""

if [[ ! -s "$BAG_DIR/metadata.yaml" ]]; then
  echo "rosbag metadata is missing or empty" >&2
  exit 1
fi

stop_pid "$STATE2_PID"
STATE2_PID=""
stop_pid "$STATE1_PID"
STATE1_PID=""
stop_pid "$SIM_PID"
SIM_PID=""
sleep 2

python3 -m epuck2_comm.analyze_comm_performance "$BAG_DIR" >>"$EXECUTION_LOG" 2>&1

# Raw-vs-relayed comparison + relay-log cross-check: the actual baseline
# acceptance checks (see comm_baseline_v1 pilot instructions).
PRELOAD_FACTOR="${PRELOAD_FACTOR:-0}" FULL_LOAD_FACTOR="${FULL_LOAD_FACTOR:-0}" \
  BAG_DIR="$BAG_DIR" EXPERIMENT_DIR="$EXPERIMENT_DIR" STEM="$STEM" COMPLETE_COUNT="$complete_count" \
  python3 "$CONFIG_DIR/verify_comm_baseline.py" \
  | tee "$BAG_DIR/analysis/comm_baseline_verdict.json" | tee -a "$EXECUTION_LOG"

VERDICT="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['verdict'])" "$BAG_DIR/analysis/comm_baseline_verdict.json" 2>/dev/null || echo UNKNOWN)"
echo "[$(date -Iseconds)] $STEM RECORDING_COMPLETE verdict=$VERDICT (see analysis/comm_baseline_verdict.json for fail_reasons)" | tee -a "$EXECUTION_LOG"
trap - EXIT
