#!/usr/bin/env bash
set -eo pipefail

if (( $# != 1 )); then
  echo "Usage: $0 TRIAL_NUMBER" >&2
  exit 2
fi

TRIAL="$1"
WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/2-1.仿真通信实验/working"
EXPERIMENT_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/cooperative_avoidance_20260716"
STEM="head_on_centered_realtime_formal_trial_${TRIAL}"
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
node = rclpy.create_node(f"centered_{stage.lower()}_clock_rate_check")
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

wait_for_topics 90 /epuck1/odom /epuck2/odom
echo "[$(date -Iseconds)] odometry ready" | tee -a "$EXECUTION_LOG"
verify_realtime_factor PRELOAD | tee -a "$EXECUTION_LOG"

ros2 run epuck2_comm state_publisher --ros-args \
  -r __ns:=/epuck1 -p robot_id:=1 -p use_sim_time:=true \
  -p mode:=periodic -p origin_x_m:=-0.35 -p origin_y_m:=0.0 \
  -p origin_yaw_rad:=0.0 >"$STATE1_LOG" 2>&1 &
STATE1_PID=$!

ros2 run epuck2_comm state_publisher --ros-args \
  -r __ns:=/epuck2 -p robot_id:=2 -p use_sim_time:=true \
  -p mode:=periodic -p origin_x_m:=0.35 -p origin_y_m:=0.0 \
  -p origin_yaw_rad:=3.141592653589793 >"$STATE2_LOG" 2>&1 &
STATE2_PID=$!

wait_for_topics 30 /epuck1/state /epuck2/state
sleep 2
echo "[$(date -Iseconds)] communicated state ready" | tee -a "$EXECUTION_LOG"

ros2 bag record -o "$BAG_DIR" \
  /epuck1/state /epuck2/state /epuck1/odom /epuck2/odom \
  /epuck1/cmd_vel /epuck2/cmd_vel >>"$EXECUTION_LOG" 2>&1 &
BAG_PID=$!
sleep 3
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "rosbag recorder exited before controller start" >&2
  exit 1
fi
echo "[$(date -Iseconds)] rosbag recording" | tee -a "$EXECUTION_LOG"

stdbuf -oL -eL ros2 launch epuck2_comm \
  dual_cooperative_avoidance.launch.py \
  armed:=true max_runtime_s:=60.0 stop_after_recovery:=true \
  post_recovery_hold_s:=0.5 enable_local_avoidance:=false \
  require_local_sensors:=false >"$CONTROLLER_LOG" 2>&1 &
CONTROLLER_PID=$!

verify_realtime_factor FULL_LOAD | tee -a "$EXECUTION_LOG"

deadline=$((SECONDS + 75))
complete_count=0
while (( SECONDS < deadline )); do
  complete_count="$(grep -c 'COMPLETE: cooperative recovery completed; commanding zero' "$CONTROLLER_LOG" 2>/dev/null || true)"
  if (( complete_count >= 2 )); then
    break
  fi
  if ! kill -0 "$CONTROLLER_PID" 2>/dev/null; then
    echo "Controller exited before both robots completed" >&2
    exit 1
  fi
  sleep 0.5
done
if (( complete_count < 2 )); then
  echo "Timed out waiting for both controller COMPLETE messages" >&2
  exit 1
fi

echo "[$(date -Iseconds)] both controllers complete" | tee -a "$EXECUTION_LOG"
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

python3 -m epuck2_comm.analyze_cooperative_bag "$BAG_DIR" \
  >>"$EXECUTION_LOG" 2>&1
if [[ ! -s "$BAG_DIR/analysis/summary.json" ]]; then
  echo "Bag-derived summary is missing or empty" >&2
  exit 1
fi

python3 - "$BAG_DIR/analysis/summary.json" <<'PY' | tee -a "$EXECUTION_LOG"
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    summary = json.load(stream)
if summary["collision_detected"]:
    raise SystemExit("Collision check failed")
if any(summary["invalid_state_messages"].values()):
    raise SystemExit("Invalid communicated state message detected")
if summary["minimum_center_separation_m"] <= summary["collision_distance_m"]:
    raise SystemExit("Non-positive geometric safety margin")
print(
    "PROTOCOL_PASS "
    f"min_separation={summary['minimum_center_separation_m']:.6f} "
    f"safety_margin={summary['minimum_safety_margin_m']:.6f} "
    f"motion_start_skew={summary['motion_start_skew_s']:.6f}"
)
PY

echo "[$(date -Iseconds)] $STEM PASS" | tee -a "$EXECUTION_LOG"
trap - EXIT
