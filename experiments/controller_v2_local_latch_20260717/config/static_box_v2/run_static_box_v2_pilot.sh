#!/usr/bin/env bash
set -eo pipefail

# controller_v2_local_latch_20260717 exclusionary pilot: single-robot,
# long-duration static-neighbor wooden-box local avoidance. Reuses the
# existing, unmodified controller_v1 simulation infrastructure
# (fusion_static_long_course_world.wbt via run_fusion_static_long_course.py)
# and box geometry, running ONLY epuck1's cooperative_avoider on the new
# controller_v2 code with a deliberately extended max_runtime_s so the
# previously-hidden ~22-28s second-encounter window has a chance to recur
# and be confirmed fixed. epuck2 never moves (no cooperative_avoider is
# launched for it); its state_publisher runs so epuck1's fused
# (local + peer-CPA) code path sees a valid, far-away peer, matching the
# original v1 Phase 1 configuration.
#
# This pilot is EXCLUDED from all formal statistics.

if (( $# != 1 )); then
  echo "Usage: $0 TRIAL_NAME" >&2
  exit 2
fi

TRIAL="$1"
SIM_WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/simulation_comm_experiment_v1/working"
EXPERIMENT_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/controller_v2_local_latch_20260717"
CONFIG_DIR="$EXPERIMENT_DIR/config/static_box_v2"
STEM="controller_v2_local_latch_20260717_static_box_${TRIAL}"
BAG_DIR="$EXPERIMENT_DIR/bags/$STEM"
CONTROLLER_LOG="$EXPERIMENT_DIR/logs/$STEM.log"
EXECUTION_LOG="$EXPERIMENT_DIR/logs/${STEM}_execution.log"
SIM_LOG="$EXPERIMENT_DIR/logs/${STEM}_simulation.log"
STATE1_LOG="$EXPERIMENT_DIR/logs/${STEM}_state_epuck1.log"
STATE2_LOG="$EXPERIMENT_DIR/logs/${STEM}_state_epuck2.log"

MAX_RUNTIME_S=55.0
POST_RECOVERY_HOLD_S=3.0
WATCHDOG_S=90

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
node = rclpy.create_node(f"static_v2_{stage.lower()}_clock_rate_check")
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
  cd "$SIM_WORK_DIR"
  exec python3 run_fusion_static_long_course.py
) >"$SIM_LOG" 2>&1 &
SIM_PID=$!

wait_for_topics 90 /epuck1/odom /epuck2/odom /epuck1/tof /epuck2/tof /epuck1/ps0 /epuck2/ps0
echo "[$(date -Iseconds)] odometry and local sensors ready" | tee -a "$EXECUTION_LOG"
verify_realtime_factor PRELOAD | tee -a "$EXECUTION_LOG"

ros2 run epuck2_comm state_publisher --ros-args \
  -r __ns:=/epuck1 -p robot_id:=1 -p use_sim_time:=true \
  -p mode:=periodic -p origin_x_m:=-0.55 -p origin_y_m:=0.0 \
  -p origin_yaw_rad:=0.0 >"$STATE1_LOG" 2>&1 &
STATE1_PID=$!

ros2 run epuck2_comm state_publisher --ros-args \
  -r __ns:=/epuck2 -p robot_id:=2 -p use_sim_time:=true \
  -p mode:=periodic -p origin_x_m:=0.45 -p origin_y_m:=0.45 \
  -p origin_yaw_rad:=3.141592653589793 >"$STATE2_LOG" 2>&1 &
STATE2_PID=$!

wait_for_topics 30 /epuck1/state /epuck2/state
sleep 2
echo "[$(date -Iseconds)] communicated state ready" | tee -a "$EXECUTION_LOG"

ros2 bag record -o "$BAG_DIR" \
  /epuck1/state /epuck2/state /epuck1/odom /epuck2/odom \
  /epuck1/cmd_vel /epuck2/cmd_vel /epuck1/tof /epuck2/tof \
  /epuck1/ps0 /epuck1/ps1 /epuck1/ps2 /epuck1/ps3 \
  /epuck1/ps4 /epuck1/ps5 /epuck1/ps6 /epuck1/ps7 \
  /epuck2/ps0 /epuck2/ps1 /epuck2/ps2 /epuck2/ps3 \
  /epuck2/ps4 /epuck2/ps5 /epuck2/ps6 /epuck2/ps7 \
  >>"$EXECUTION_LOG" 2>&1 &
BAG_PID=$!
sleep 3
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "rosbag recorder exited before controller start" >&2
  exit 1
fi
echo "[$(date -Iseconds)] rosbag recording" | tee -a "$EXECUTION_LOG"

stdbuf -oL -eL ros2 run epuck2_comm cooperative_avoider --ros-args \
  -r __ns:=/epuck1 \
  -p robot_id:=1 \
  -p peer_state_topic:=/epuck2/state \
  -p armed:=true \
  -p desired_heading_rad:=0.0 \
  -p startup_hold_s:=5.0 \
  -p max_runtime_s:="$MAX_RUNTIME_S" \
  -p stop_after_recovery:=false \
  -p post_recovery_hold_s:="$POST_RECOVERY_HOLD_S" \
  -p enable_peer_avoidance:=true \
  -p enable_local_avoidance:=true \
  -p require_local_sensors:=true \
  -p use_sim_time:=true \
  >"$CONTROLLER_LOG" 2>&1 &
CONTROLLER_PID=$!

verify_realtime_factor FULL_LOAD | tee -a "$EXECUTION_LOG"

deadline=$((SECONDS + WATCHDOG_S))
completed=0
while (( SECONDS < deadline )); do
  if grep -q 'COMPLETE:' "$CONTROLLER_LOG" 2>/dev/null; then
    completed=1
    break
  fi
  if ! kill -0 "$CONTROLLER_PID" 2>/dev/null; then
    echo "Controller exited before COMPLETE was confirmed" >&2
    break
  fi
  sleep 0.5
done
if (( completed == 0 )); then
  echo "TASK_TIMEOUT: no COMPLETE line within ${WATCHDOG_S}s watchdog" | tee -a "$EXECUTION_LOG" >&2
else
  echo "[$(date -Iseconds)] controller COMPLETE" | tee -a "$EXECUTION_LOG"
fi

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

python3 -m epuck2_comm.analyze_cooperative_bag "$BAG_DIR" >>"$EXECUTION_LOG" 2>&1
python3 "/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/cooperative_avoidance_20260716/config/combined_wood_moving_peer/analyze_combined_task.py" "$BAG_DIR" >>"$EXECUTION_LOG" 2>&1
python3 "$CONFIG_DIR/analyze_static_v2_controller_log.py" "$BAG_DIR" "$CONTROLLER_LOG" >>"$EXECUTION_LOG" 2>&1

if (( completed == 0 )); then
  echo "$STEM TASK_TIMEOUT" | tee -a "$EXECUTION_LOG"
  exit 1
fi
echo "[$(date -Iseconds)] $STEM PASS" | tee -a "$EXECUTION_LOG"
trap - EXIT
