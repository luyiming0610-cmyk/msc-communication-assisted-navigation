#!/usr/bin/env bash
set -eo pipefail

WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/2-1.仿真通信实验/working"
EXPERIMENT_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/cooperative_avoidance_20260716"

source /opt/ros/humble/setup.bash
source "$HOME/epuck_ws/install/setup.bash"
set -u
STEM_PREFIX="${STEM_PREFIX:-head_on_offset_040_realtime_formal_trial}"
MAX_RUNTIME_S="${MAX_RUNTIME_S:-60.0}"

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
  echo "Process $pid did not stop after SIGINT/SIGTERM" >&2
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
  python3 - <<'PY'
import sys
import time

import rclpy
from rosgraph_msgs.msg import Clock

rclpy.init()
node = rclpy.create_node("offset_batch_clock_rate_check")
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
    print("No usable /clock samples", file=sys.stderr)
    raise SystemExit(1)
wall_delta = samples[-1][0] - samples[0][0]
sim_delta = samples[-1][1] - samples[0][1]
factor = sim_delta / wall_delta if wall_delta > 0.0 else 0.0
print(f"realtime_factor={factor:.3f}")
if factor < 0.8 or factor > 1.2:
    print("Webots realtime factor is outside the locked 0.8-1.2 range", file=sys.stderr)
    raise SystemExit(1)
PY
}

run_trial() (
  local trial="$1"
  local stem="${STEM_PREFIX}_${trial}"
  local bag_dir="$EXPERIMENT_DIR/bags/$stem"
  local controller_log="$EXPERIMENT_DIR/logs/$stem.log"
  local automation_log="$EXPERIMENT_DIR/logs/${stem}_automation.log"
  local sim_log="$EXPERIMENT_DIR/logs/${stem}_simulation.log"
  local epuck1_log="$EXPERIMENT_DIR/logs/${stem}_state_epuck1.log"
  local epuck2_log="$EXPERIMENT_DIR/logs/${stem}_state_epuck2.log"

  if [[ -e "$bag_dir" ]]; then
    echo "Refusing to overwrite existing bag: $bag_dir" >&2
    return 1
  fi

  local sim_pid=""
  local state1_pid=""
  local state2_pid=""
  local bag_pid=""
  local controller_pid=""

  cleanup_trial() {
    stop_pid "$controller_pid" || true
    stop_pid "$bag_pid" || true
    stop_pid "$state2_pid" || true
    stop_pid "$state1_pid" || true
    stop_pid "$sim_pid" || true
  }
  trap cleanup_trial EXIT

  echo "[$(date -Iseconds)] TRIAL $trial START" | tee "$automation_log"

  (
    cd "$WORK_DIR"
    exec python3 run_dual_head_on_lateral_offset_040.py
  ) >"$sim_log" 2>&1 &
  sim_pid=$!

  wait_for_topics 90 /epuck1/odom /epuck2/odom
  echo "[$(date -Iseconds)] odometry ready" | tee -a "$automation_log"
  verify_realtime_factor | tee -a "$automation_log"
  echo "[$(date -Iseconds)] realtime-factor gate passed" | tee -a "$automation_log"

  ros2 run epuck2_comm state_publisher --ros-args \
    -r __ns:=/epuck1 -p robot_id:=1 -p use_sim_time:=true \
    -p mode:=periodic -p origin_x_m:=-0.35 -p origin_y_m:=-0.02 \
    -p origin_yaw_rad:=0.0 >"$epuck1_log" 2>&1 &
  state1_pid=$!

  ros2 run epuck2_comm state_publisher --ros-args \
    -r __ns:=/epuck2 -p robot_id:=2 -p use_sim_time:=true \
    -p mode:=periodic -p origin_x_m:=0.35 -p origin_y_m:=0.02 \
    -p origin_yaw_rad:=3.141592653589793 >"$epuck2_log" 2>&1 &
  state2_pid=$!

  wait_for_topics 30 /epuck1/state /epuck2/state
  sleep 2
  echo "[$(date -Iseconds)] communicated state ready" | tee -a "$automation_log"

  ros2 bag record -o "$bag_dir" \
    /epuck1/state /epuck2/state /epuck1/odom /epuck2/odom \
    /epuck1/cmd_vel /epuck2/cmd_vel \
    >>"$automation_log" 2>&1 &
  bag_pid=$!
  sleep 3
  if ! kill -0 "$bag_pid" 2>/dev/null; then
    echo "rosbag recorder exited before controller start" >&2
    return 1
  fi
  echo "[$(date -Iseconds)] rosbag recording" | tee -a "$automation_log"

  stdbuf -oL -eL ros2 launch epuck2_comm \
    dual_cooperative_avoidance.launch.py \
    armed:=true max_runtime_s:="$MAX_RUNTIME_S" stop_after_recovery:=true \
    post_recovery_hold_s:=0.5 enable_local_avoidance:=false \
    require_local_sensors:=false >"$controller_log" 2>&1 &
  controller_pid=$!

  verify_realtime_factor | tee -a "$automation_log"
  echo "[$(date -Iseconds)] full-load realtime-factor gate passed" | tee -a "$automation_log"

  local deadline=$((SECONDS + 75))
  local complete_count=0
  while (( SECONDS < deadline )); do
    complete_count="$(grep -c 'COMPLETE: cooperative recovery completed; commanding zero' "$controller_log" 2>/dev/null || true)"
    if (( complete_count >= 2 )); then
      break
    fi
    if ! kill -0 "$controller_pid" 2>/dev/null; then
      echo "Controller exited before both robots completed" >&2
      return 1
    fi
    sleep 0.5
  done
  if (( complete_count < 2 )); then
    echo "Timed out waiting for both controller COMPLETE messages" >&2
    return 1
  fi

  echo "[$(date -Iseconds)] both controllers complete" | tee -a "$automation_log"
  sleep 1
  stop_pid "$controller_pid"
  controller_pid=""
  stop_pid "$bag_pid"
  bag_pid=""

  if [[ ! -s "$bag_dir/metadata.yaml" ]]; then
    echo "rosbag metadata is missing or empty" >&2
    return 1
  fi

  stop_pid "$state2_pid"
  state2_pid=""
  stop_pid "$state1_pid"
  state1_pid=""
  stop_pid "$sim_pid"
  sim_pid=""
  sleep 2

  python3 -m epuck2_comm.analyze_cooperative_bag "$bag_dir" \
    >>"$automation_log" 2>&1
  if [[ ! -s "$bag_dir/analysis/summary.json" ]]; then
    echo "Automated summary is missing or empty" >&2
    return 1
  fi

  python3 - "$bag_dir/analysis/summary.json" <<'PY' | tee -a "$automation_log"
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    summary = json.load(stream)
if summary["collision_detected"]:
    raise SystemExit("Automated collision check failed")
if any(summary["invalid_state_messages"].values()):
    raise SystemExit("Invalid communicated state message detected")
print(
    "AUTOMATED_PASS "
    f"min_separation={summary['minimum_center_separation_m']:.6f} "
    f"safety_margin={summary['minimum_safety_margin_m']:.6f}"
)
PY

  echo "[$(date -Iseconds)] TRIAL $trial PASS" | tee -a "$automation_log"
  trap - EXIT
)

/mnt/c/Windows/System32/cmd.exe /c echo WSL_INTEROP_OK

if (( $# > 0 )); then
  trials=("$@")
else
  trials=(01 02 03 04 05)
fi

for trial in "${trials[@]}"; do
  run_trial "$trial"
done

echo "ALL_OFFSET_040_FORMAL_REPETITIONS_COMPLETE"
