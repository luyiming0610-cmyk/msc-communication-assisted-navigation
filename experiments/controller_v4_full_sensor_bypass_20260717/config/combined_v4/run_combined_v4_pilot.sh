#!/usr/bin/env bash
set -eo pipefail

# controller_v4_full_sensor_bypass_20260717 exclusionary pilot: combined
# wooden-box local avoidance + moving-peer CPA avoidance, single run.
# Geometry, startup holds and every avoidance/CPA parameter are copied
# verbatim from the pre-v4 pilot_04 configuration (the one that already
# proved correct CPA sync before the v1 runaway-turn/box-corner defect was
# found and fixed via the v2-v4 chain). Only the underlying controller
# package is v4, and the task monitor uses one pre-diagnosed
# encounter-distance sampling fix (see combined_task_coordinator_v4.py).
# This pilot is EXCLUDED from all formal statistics.
#
# Usage: run_combined_v4_pilot.sh TRIAL_NAME

if (( $# != 1 )); then
  echo "Usage: $0 TRIAL_NAME" >&2
  exit 2
fi

TRIAL="$1"
WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/simulation_comm_experiment_v1/working"
EXPERIMENT_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/controller_v4_full_sensor_bypass_20260717"
CONFIG_DIR="$EXPERIMENT_DIR/config/combined_v4"
LEGACY_ANALYZER_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/cooperative_avoidance_20260716/config/combined_wood_moving_peer"
STEM="controller_v4_full_sensor_bypass_20260717_combined_${TRIAL}"
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
node = rclpy.create_node(f"combined_v4_{stage.lower()}_clock_rate_check")
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
  exec python3 run_combined_wood_moving_peer.py
) >"$SIM_LOG" 2>&1 &
SIM_PID=$!

wait_for_topics 90 /epuck1/odom /epuck2/odom /epuck1/tof /epuck2/tof /epuck1/ps0 /epuck2/ps0
echo "[$(date -Iseconds)] odometry and local sensors ready" | tee -a "$EXECUTION_LOG"
PRELOAD_OUTPUT="$(verify_realtime_factor PRELOAD | tee -a "$EXECUTION_LOG")"
PRELOAD_FACTOR="$(grep -o 'PRELOAD_REALTIME_FACTOR=[0-9.]*' <<<"$PRELOAD_OUTPUT" | cut -d= -f2)"

ros2 run epuck2_comm state_publisher --ros-args \
  -r __ns:=/epuck1 -p robot_id:=1 -p use_sim_time:=true \
  -p mode:=periodic -p origin_x_m:=-0.55 -p origin_y_m:=0.0 \
  -p origin_yaw_rad:=0.0 >"$STATE1_LOG" 2>&1 &
STATE1_PID=$!

ros2 run epuck2_comm state_publisher --ros-args \
  -r __ns:=/epuck2 -p robot_id:=2 -p use_sim_time:=true \
  -p mode:=periodic -p origin_x_m:=0.45 -p origin_y_m:=0.0 \
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

stdbuf -oL -eL python3 "$CONFIG_DIR/run_combined_v4_controllers.py" \
  >"$CONTROLLER_LOG" 2>&1 &
CONTROLLER_PID=$!

FULL_LOAD_OUTPUT="$(verify_realtime_factor FULL_LOAD | tee -a "$EXECUTION_LOG")"
FULL_LOAD_FACTOR="$(grep -o 'FULL_LOAD_REALTIME_FACTOR=[0-9.]*' <<<"$FULL_LOAD_OUTPUT" | cut -d= -f2)"

deadline=$((SECONDS + 115))
task_complete_count=0
cooperative_complete_count=0
while (( SECONDS < deadline )); do
  task_complete_count="$(grep -c 'TASK_COMPLETE: epuck1 cleared wooden box and cooperative encounter recovered' "$CONTROLLER_LOG" 2>/dev/null || true)"
  cooperative_complete_count="$(grep -c 'COMPLETE: cooperative recovery completed; commanding zero' "$CONTROLLER_LOG" 2>/dev/null || true)"
  if (( task_complete_count >= 1 && cooperative_complete_count >= 1 )); then
    break
  fi
  if ! kill -0 "$CONTROLLER_PID" 2>/dev/null; then
    echo "Controller launch exited before task completion was confirmed" >&2
    break
  fi
  sleep 0.5
done

echo "[$(date -Iseconds)] controller stage finished (task_complete=$task_complete_count cooperative_complete=$cooperative_complete_count) -- this only means the run finished, not that the pilot passed, see below" | tee -a "$EXECUTION_LOG"

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
python3 "$LEGACY_ANALYZER_DIR/analyze_combined_task.py" "$BAG_DIR" >>"$EXECUTION_LOG" 2>&1

PRELOAD_FACTOR="${PRELOAD_FACTOR:-0}" FULL_LOAD_FACTOR="${FULL_LOAD_FACTOR:-0}" \
  CONTROLLER_LOG="$CONTROLLER_LOG" BAG_DIR="$BAG_DIR" \
  TASK_COMPLETE_COUNT="$task_complete_count" COOPERATIVE_COMPLETE_COUNT="$cooperative_complete_count" \
  python3 - <<'PY' | tee "$BAG_DIR/analysis/static_v4_combined_verdict.json" | tee -a "$EXECUTION_LOG"
import json
import os
import re

controller_log = os.environ["CONTROLLER_LOG"]
bag_dir = os.environ["BAG_DIR"]
preload = float(os.environ["PRELOAD_FACTOR"])
full_load = float(os.environ["FULL_LOAD_FACTOR"])
task_complete_count = int(os.environ["TASK_COMPLETE_COUNT"])
cooperative_complete_count = int(os.environ["COOPERATIVE_COMPLETE_COUNT"])

with open(controller_log, encoding="utf-8", errors="replace") as fh:
    log_text = fh.read()

with open(os.path.join(bag_dir, "analysis", "summary.json"), encoding="utf-8") as fh:
    summary = json.load(fh)
with open(os.path.join(bag_dir, "analysis", "combined_task_summary.json"), encoding="utf-8") as fh:
    combined = json.load(fh)

reasons_fail = []

if task_complete_count < 1:
    reasons_fail.append("task monitor never logged TASK_COMPLETE (box pass + cooperative recovery)")
if cooperative_complete_count < 1:
    reasons_fail.append("no robot logged cooperative recovery COMPLETE")

if "COMPLETE: maximum runtime reached" in log_text:
    reasons_fail.append("at least one robot stopped via maximum runtime, not task completion")
if "TASK_TIMEOUT" in log_text:
    reasons_fail.append("task monitor logged TASK_TIMEOUT")

avoid_turn_count = len(re.findall(r"mode=AVOID_TURN", log_text))
if avoid_turn_count < 2:
    reasons_fail.append(f"AVOID_TURN observed only {avoid_turn_count} times (<2, CPA not genuinely triggered by both robots)")

epuck1_local_count = len(re.findall(r"epuck1\.cooperative_avoider.*mode=LOCAL_\S+", log_text))
epuck2_local_count = len(re.findall(r"epuck2\.cooperative_avoider.*mode=LOCAL_\S+", log_text))
if epuck1_local_count < 1:
    reasons_fail.append("epuck1 never entered a LOCAL_* mode -- box bypass not genuinely triggered")
if epuck2_local_count != 0:
    reasons_fail.append(f"epuck2 mis-triggered local avoidance ({epuck2_local_count} LOCAL_* occurrences)")

first_local_match = re.search(r"^.*epuck1\.cooperative_avoider.*mode=LOCAL_\S+.*$", log_text, re.MULTILINE)
first_peer_match = re.search(r"^.*epuck1\.cooperative_avoider.*mode=AVOID_TURN.*$", log_text, re.MULTILINE)
if first_local_match is None or first_peer_match is None or first_local_match.start() >= first_peer_match.start():
    reasons_fail.append("required local-obstacle-then-peer-event order was not observed")

failsafe_count = len(re.findall(r"LOCAL_ENCOUNTER_FAILSAFE", log_text))
failsafe_cause_count = len(re.findall(r"failsafe_cause=(?!NONE)[A-Z_]+", log_text))
if failsafe_count != 0 or failsafe_cause_count != 0:
    reasons_fail.append(f"FAILSAFE observed (failsafe_count={failsafe_count}, failsafe_cause_count={failsafe_cause_count})")

sensor_invalid_count = len(re.findall(r"SAFE_STOP_INVALID_ODOM", log_text))
if sensor_invalid_count != 0:
    reasons_fail.append(f"SENSOR_INVALID/invalid-odom observed ({sensor_invalid_count} occurrences)")

collision = bool(summary.get("collision_detected"))
if collision:
    reasons_fail.append("robot-to-robot collision_detected=true in bag-derived summary")

invalid_state = summary.get("invalid_state_messages", {})
if any(invalid_state.values()):
    reasons_fail.append(f"invalid communicated state messages: {invalid_state}")

min_sep = summary.get("minimum_center_separation_m")
collision_dist = summary.get("collision_distance_m")
if min_sep is None or collision_dist is None or min_sep <= collision_dist:
    reasons_fail.append(f"non-positive robot-robot geometric safety margin (min_sep={min_sep}, collision_dist={collision_dist})")

epuck1_passed_box = bool(combined.get("epuck1_passed_box"))
if not epuck1_passed_box:
    reasons_fail.append("epuck1 did not cross the pass_x_m threshold (did not genuinely pass the box)")

box_collision = combined.get("box_collision_detected", {})
if any(box_collision.values()):
    reasons_fail.append(f"robot-to-box collision detected: {box_collision}")

epuck1_box_clearance = combined.get("minimum_box_clearance_m", {}).get("/epuck1/state")
if epuck1_box_clearance is None or epuck1_box_clearance < 0.005:
    reasons_fail.append(f"epuck1 box clearance below 0.005 m gate (got {epuck1_box_clearance})")

commands = summary.get("commands", {})
for topic, stats in (commands.items() if isinstance(commands, dict) else []):
    count = stats.get("angular_sign_changes") if isinstance(stats, dict) else None
    if isinstance(count, (int, float)) and count > 10:
        reasons_fail.append(f"{topic} shows {count} angular-command sign changes (possible oscillation/spinning)")

realtime_ok = 0.8 <= preload <= 1.2 and 0.8 <= full_load <= 1.2
if not realtime_ok:
    reasons_fail.append(f"realtime factor out of range (preload={preload}, full_load={full_load})")

verdict = "PASS" if not reasons_fail else "FAIL"

result = {
    "verdict": verdict,
    "fail_reasons": reasons_fail,
    "task_complete_count": task_complete_count,
    "cooperative_complete_count": cooperative_complete_count,
    "avoid_turn_count": avoid_turn_count,
    "epuck1_local_count": epuck1_local_count,
    "epuck2_local_count": epuck2_local_count,
    "local_before_peer_order_ok": (
        first_local_match is not None
        and first_peer_match is not None
        and first_local_match.start() < first_peer_match.start()
    ),
    "collision_detected": collision,
    "minimum_center_separation_m": min_sep,
    "collision_distance_m": collision_dist,
    "epuck1_passed_box": epuck1_passed_box,
    "epuck1_box_clearance_m": epuck1_box_clearance,
    "box_collision_detected": box_collision,
    "preload_realtime_factor": preload,
    "full_load_realtime_factor": full_load,
    "realtime_factor_ok": realtime_ok,
}
print(json.dumps(result, indent=2, ensure_ascii=False))
PY

VERDICT="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['verdict'])" "$BAG_DIR/analysis/static_v4_combined_verdict.json" 2>/dev/null || echo UNKNOWN)"
echo "[$(date -Iseconds)] $STEM RECORDING_COMPLETE verdict=$VERDICT (see analysis/static_v4_combined_verdict.json for fail_reasons)" | tee -a "$EXECUTION_LOG"
trap - EXIT
