#!/usr/bin/env bash
set -eo pipefail

# Three-robot shared-exit pilot/formal orchestrator.
# Usage:
#   run_shared_exit_n3_trial.sh [--check-only] N3_EXIT_COMM_OFF|N3_EXIT_COMM_ON TRIAL [ATTEMPT]
#   run_shared_exit_n3_trial.sh MODE TRIAL --pilot LABEL

CHECK_ONLY=false
if [[ "${1:-}" == "--check-only" ]]; then CHECK_ONLY=true; shift; fi
PILOT_LABEL=""
if (( $# == 4 )) && [[ "$3" == "--pilot" ]]; then
  COMM_MODE="$1"; TRIAL_INDEX="$2"; ATTEMPT=1; PILOT_LABEL="$4"
elif (( $# >= 2 && $# <= 3 )); then
  COMM_MODE="$1"; TRIAL_INDEX="$2"; ATTEMPT="${3:-1}"
else
  echo "Usage: $0 [--check-only] MODE TRIAL [ATTEMPT | --pilot LABEL]" >&2; exit 2
fi
if [[ "$COMM_MODE" != "N3_EXIT_COMM_OFF" && "$COMM_MODE" != "N3_EXIT_COMM_ON" ]]; then
  echo "MODE must be N3_EXIT_COMM_OFF or N3_EXIT_COMM_ON" >&2; exit 2
fi

REPO="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm"
EXP_DIR="$REPO/experiments/10_cooperative_exit_navigation_20260720"
TOOLS_DIR="$EXP_DIR/tools"
WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/2-1.仿真通信实验/working"
PARAMS="$EXP_DIR/shared_exit_n3_params.json"

source /opt/ros/humble/setup.bash
source "$HOME/epuck_ws/install/setup.bash"
export WEBOTS_HOME="/mnt/c/Program Files/Webots"
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib/controller:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$WEBOTS_HOME/local/lib/python3.10/dist-packages:$TOOLS_DIR:${PYTHONPATH:-}"
source <(python3 "$TOOLS_DIR/n3_params_to_env.py" "$PARAMS")
set -u

CANONICAL_WORLD="$EXP_DIR/worlds/$WORLD_FILE"
for file in "$PARAMS" "$CANONICAL_WORLD" "$WORK_DIR/$WORLD_FILE" \
  "$WORK_DIR/run_shared_exit_n3.py" "$WORK_DIR/triple_namespaced_launch.py" \
  "$WORK_DIR/epuck3_ros2_control.yml" "$TOOLS_DIR/goal_navigator.py" \
  "$TOOLS_DIR/task_completion_monitor.py" "$TOOLS_DIR/multi_peer_selector.py" \
  "$TOOLS_DIR/run_shared_exit_n3_controllers.py"; do
  [[ -f "$file" ]] || { echo "missing required file: $file" >&2; exit 1; }
done
WORLD_SHA256="$(sha256sum "$WORK_DIR/$WORLD_FILE" | awk '{print $1}')"
CANONICAL_WORLD_SHA256="$(sha256sum "$CANONICAL_WORLD" | awk '{print $1}')"
[[ "$WORLD_SHA256" == "$CANONICAL_WORLD_SHA256" ]] || { echo "working/canonical world mismatch" >&2; exit 1; }
GIT_COMMIT="$(cd "$REPO" && git rev-parse HEAD)"
ORCH_SHA256="$(sha256sum "$0" | awk '{print $1}')"

residuals() {
  pgrep -af 'webots-bin|webots_ros2_driver/driver|cooperative_avoider|state_publisher|ros2 bag record|task_completion_monitor|goal_navigator|multi_peer_selector' 2>/dev/null | grep -v 'bash -lc' || true
}

stop_trial_drivers() {
  local pids
  pids="$(pgrep -f 'webots_ros2_driver/driver.*__ns:=/epuck[123]' 2>/dev/null || true)"
  [[ -n "$pids" ]] || return 0
  for pid in $pids; do kill -INT "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in $pids; do kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true; done
}

if [[ "$CHECK_ONLY" == true ]]; then
  echo "=== N=3 check-only: Webots will not launch ==="
  /mnt/c/Windows/System32/cmd.exe /c echo INTEROP_OK | grep -q INTEROP_OK
  echo "INTEROP_OK"
  echo "git_commit=$GIT_COMMIT"
  echo "orchestrator_sha256=$ORCH_SHA256"
  echo "world_sha256=$WORLD_SHA256 canonical_match=true"
  echo "mode=$COMM_MODE trial=$TRIAL_INDEX pilot=${PILOT_LABEL:-<none>}"
  echo "roles=A informed; B lower search; C upper search"
  echo "event_chain=A discovery -> announcement -> B/C search-to-goal switch"
  FOUND="$(residuals)"
  [[ -z "$FOUND" ]] || { echo "residual processes:" >&2; echo "$FOUND" >&2; exit 1; }
  echo "NO_RESIDUAL_PROCESSES"
  echo "CHECK_ONLY_PASS"
  exit 0
fi

if [[ -n "$PILOT_LABEL" ]]; then
  STEM="shared_exit_n3_${COMM_MODE,,}_EXCLUSIONARY_PILOT_${PILOT_LABEL}"
else
  STEM="shared_exit_n3_${COMM_MODE,,}_trial$(printf '%02d' "$TRIAL_INDEX")_attempt$(printf '%02d' "$ATTEMPT")"
fi
ROOT="/home/eamon/epuck_comm_bags"
BAG_DIR="$ROOT/$STEM"
LOG_DIR="$ROOT/${STEM}_diag_logs"
ANALYSIS_DIR="$EXP_DIR/${STEM}_analysis"
[[ ! -e "$BAG_DIR" && ! -e "$LOG_DIR" && ! -e "$ANALYSIS_DIR" ]] || { echo "refusing to overwrite $STEM" >&2; exit 1; }
mkdir -p "$LOG_DIR"
EXEC_LOG="$LOG_DIR/execution.log"
SIM_LOG="$LOG_DIR/simulation.log"
CONTROLLER_LOG="$LOG_DIR/controller.log"
MONITOR_LOG="$LOG_DIR/task_completion_monitor.log"
MONITOR_VERDICT="$LOG_DIR/monitor_verdict.json"
BAG_LOG="$LOG_DIR/bag_record.log"

FOUND="$(residuals)"
[[ -z "$FOUND" ]] || { echo "residual processes:" >&2; echo "$FOUND" >&2; exit 1; }

PIDS=(); BAG_PID=""
stop_group() {
  local pid="${1:-}"; [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -INT "-$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null || true; return 0; }; sleep 0.5; done
  kill -TERM "-$pid" 2>/dev/null || true; sleep 1; wait "$pid" 2>/dev/null || true
}
cleanup() {
  for pid in "${PIDS[@]:-}"; do stop_group "$pid" || true; done
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then kill -INT "$BAG_PID" 2>/dev/null || true; wait "$BAG_PID" 2>/dev/null || true; fi
  stop_trial_drivers
}
trap cleanup EXIT

wait_topics() {
  local deadline=$((SECONDS + $1)); shift
  while (( SECONDS < deadline )); do
    local list ok=1; list="$(ros2 topic list 2>/dev/null || true)"
    for topic in "$@"; do grep -Fxq "$topic" <<<"$list" || { ok=0; break; }; done
    (( ok == 1 )) && return 0; sleep 1
  done
  echo "timed out waiting for topics: $*" >&2; return 1
}

echo "[$(date -Iseconds)] $STEM START" | tee "$EXEC_LOG"
echo "mode=$COMM_MODE git_commit=$GIT_COMMIT world_sha256=$WORLD_SHA256" | tee -a "$EXEC_LOG"
cp "$PARAMS" "$LOG_DIR/frozen_params_canonical_copy.json"

export EPUCK_WORLD_FILE="$WORLD_FILE"
setsid bash -c "cd '$WORK_DIR' && exec python3 run_shared_exit_n3.py" >"$SIM_LOG" 2>&1 & SIM_PID=$!; PIDS+=("$SIM_PID")
wait_topics 100 /epuck1/odom /epuck2/odom /epuck3/odom /epuck1/tof /epuck2/tof /epuck3/tof /epuck1/ps0 /epuck2/ps0 /epuck3/ps0
echo "[$(date -Iseconds)] three robots and local sensors ready" | tee -a "$EXEC_LOG"

for id in 1 2 3; do
  letter=A; [[ "$id" == 2 ]] && letter=B; [[ "$id" == 3 ]] && letter=C
  eval sx="\$ROBOT_${letter}_START_X_M"; eval sy="\$ROBOT_${letter}_START_Y_M"; eval yaw="\$ROBOT_${letter}_START_YAW_RAD"
  setsid ros2 run epuck2_comm state_publisher --ros-args -r __ns:=/epuck$id \
    -p robot_id:=$id -p use_sim_time:=true -p mode:=periodic \
    -p origin_x_m:=$sx -p origin_y_m:=$sy -p origin_yaw_rad:=$yaw \
    >"$LOG_DIR/state_epuck$id.log" 2>&1 & PIDS+=("$!")
done
wait_topics 30 /epuck1/state /epuck2/state /epuck3/state

if [[ "$COMM_MODE" == "N3_EXIT_COMM_ON" ]]; then
  setsid python3 "$TOOLS_DIR/multi_peer_selector.py" --robot-id=1 --own-topic=/epuck1/state --peer-topics=/epuck2/state,/epuck3/state --output-topic=/epuck1/selected_peer_state --safety-radius-m=$SAFETY_RADIUS_M >"$LOG_DIR/selector_epuck1.log" 2>&1 & PIDS+=("$!")
  setsid python3 "$TOOLS_DIR/multi_peer_selector.py" --robot-id=2 --own-topic=/epuck2/state --peer-topics=/epuck1/state,/epuck3/state --output-topic=/epuck2/selected_peer_state --safety-radius-m=$SAFETY_RADIUS_M >"$LOG_DIR/selector_epuck2.log" 2>&1 & PIDS+=("$!")
  setsid python3 "$TOOLS_DIR/multi_peer_selector.py" --robot-id=3 --own-topic=/epuck3/state --peer-topics=/epuck1/state,/epuck2/state --output-topic=/epuck3/selected_peer_state --safety-radius-m=$SAFETY_RADIUS_M >"$LOG_DIR/selector_epuck3.log" 2>&1 & PIDS+=("$!")
  wait_topics 20 /epuck1/selected_peer_state /epuck2/selected_peer_state /epuck3/selected_peer_state
  echo "[$(date -Iseconds)] multi-peer selectors ready" | tee -a "$EXEC_LOG"
fi

TOPICS=(/epuck1/state /epuck2/state /epuck3/state /epuck1/cmd_vel /epuck2/cmd_vel /epuck3/cmd_vel /epuck1/nav_intent /epuck2/nav_intent /epuck3/nav_intent)
if [[ "$COMM_MODE" == "N3_EXIT_COMM_ON" ]]; then
  TOPICS+=(/epuck1/goal_announcement /epuck1/selected_peer_state /epuck2/selected_peer_state /epuck3/selected_peer_state)
fi
ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}" >"$BAG_LOG" 2>&1 & BAG_PID=$!; sleep 3
kill -0 "$BAG_PID" 2>/dev/null || { echo "bag failed to start" >&2; exit 1; }
echo "[$(date -Iseconds)] rosbag recording: $BAG_DIR" | tee -a "$EXEC_LOG"

setsid python3 "$TOOLS_DIR/task_completion_monitor.py" \
  --robot-ids=epuck1,epuck2,epuck3 --state-topics=/epuck1/state,/epuck2/state,/epuck3/state \
  --goal-centers-x-m=$PARKING_A_X_M,$PARKING_B_X_M,$PARKING_C_X_M \
  --goal-centers-y-m=$PARKING_A_Y_M,$PARKING_B_Y_M,$PARKING_C_Y_M \
  --goal-radii-m=$PARKING_A_RADIUS_M,$PARKING_B_RADIUS_M,$PARKING_C_RADIUS_M \
  --goal-hold-time-s=$GOAL_HOLD_TIME_S --max-linear-speed-mps=$COMPLETION_MAX_LINEAR_SPEED_MPS \
  --max-angular-speed-rps=$COMPLETION_MAX_ANGULAR_SPEED_RPS --verdict-path="$MONITOR_VERDICT" \
  >"$MONITOR_LOG" 2>&1 & MONITOR_PID=$!; PIDS+=("$MONITOR_PID")

NAV_A=(--robot-id=1 --state-topic=/epuck1/state --nav-intent-topic=/epuck1/nav_intent --mode=informed --target-x=$GOAL_CENTER_X_M --target-y=$GOAL_CENTER_Y_M --rate-hz=2 --nominal-speed-mps=$NOMINAL_SPEED_MPS --exit-center-x=$GOAL_CENTER_X_M --exit-center-y=$GOAL_CENTER_Y_M --exit-radius=$GOAL_RADIUS_M --parking-x=$PARKING_A_X_M --parking-y=$PARKING_A_Y_M --parking-radius=$PARKING_A_RADIUS_M --goal-hold-time-s=$GOAL_HOLD_TIME_S)
NAV_B=(--robot-id=2 --state-topic=/epuck2/state --nav-intent-topic=/epuck2/nav_intent --mode=search --waypoints=$ROBOT_B_WAYPOINTS --waypoint-arrival-radius=$ROBOT_B_WAYPOINT_ARRIVAL_RADIUS_M --rate-hz=2 --nominal-speed-mps=$NOMINAL_SPEED_MPS --exit-center-x=$GOAL_CENTER_X_M --exit-center-y=$GOAL_CENTER_Y_M --exit-radius=$GOAL_RADIUS_M --parking-x=$PARKING_B_X_M --parking-y=$PARKING_B_Y_M --parking-radius=$PARKING_B_RADIUS_M --goal-hold-time-s=$GOAL_HOLD_TIME_S)
NAV_C=(--robot-id=3 --state-topic=/epuck3/state --nav-intent-topic=/epuck3/nav_intent --mode=search --waypoints=$ROBOT_C_WAYPOINTS --waypoint-arrival-radius=$ROBOT_C_WAYPOINT_ARRIVAL_RADIUS_M --rate-hz=2 --nominal-speed-mps=$NOMINAL_SPEED_MPS --exit-center-x=$GOAL_CENTER_X_M --exit-center-y=$GOAL_CENTER_Y_M --exit-radius=$GOAL_RADIUS_M --parking-x=$PARKING_C_X_M --parking-y=$PARKING_C_Y_M --parking-radius=$PARKING_C_RADIUS_M --goal-hold-time-s=$GOAL_HOLD_TIME_S)
if [[ "$COMM_MODE" == "N3_EXIT_COMM_ON" ]]; then
  NAV_A+=(--announce --announce-after-exit-entry --announce-topic=/epuck1/goal_announcement --goal-id=$GOAL_ID)
  NAV_B+=(--goal-announcement-topic=/epuck1/goal_announcement)
  NAV_C+=(--goal-announcement-topic=/epuck1/goal_announcement)
fi
setsid python3 "$TOOLS_DIR/goal_navigator.py" "${NAV_A[@]}" >"$LOG_DIR/goal_navigator_epuck1.log" 2>&1 & PIDS+=("$!")
setsid python3 "$TOOLS_DIR/goal_navigator.py" "${NAV_B[@]}" >"$LOG_DIR/goal_navigator_epuck2.log" 2>&1 & PIDS+=("$!")
setsid python3 "$TOOLS_DIR/goal_navigator.py" "${NAV_C[@]}" >"$LOG_DIR/goal_navigator_epuck3.log" 2>&1 & PIDS+=("$!")

N3_EXIT_COMM_MODE="$COMM_MODE" setsid python3 "$TOOLS_DIR/run_shared_exit_n3_controllers.py" >"$CONTROLLER_LOG" 2>&1 & CONTROLLER_PID=$!; PIDS+=("$CONTROLLER_PID")
echo "[$(date -Iseconds)] controllers and navigators running" | tee -a "$EXEC_LOG"

deadline=$((SECONDS + 195)); STOP_REASON=WATCHDOG
while (( SECONDS < deadline )); do
  if grep -q TASK_COMPLETE_GOAL "$MONITOR_LOG" 2>/dev/null; then STOP_REASON=TASK_COMPLETE_GOAL; break; fi
  kill -0 "$CONTROLLER_PID" 2>/dev/null || { STOP_REASON=CONTROLLER_EXITED; break; }
  sleep 0.25
done
echo "[$(date -Iseconds)] stop_reason=$STOP_REASON" | tee -a "$EXEC_LOG"

for pid in "${PIDS[@]}"; do [[ "$pid" == "$SIM_PID" ]] || stop_group "$pid" || true; done
PIDS=("$SIM_PID")
kill -INT "$BAG_PID" 2>/dev/null || true; wait "$BAG_PID" 2>/dev/null || true; BAG_PID=""
stop_group "$SIM_PID" || true; PIDS=()
stop_trial_drivers

DATA_VALIDITY=VALID; TASK_OUTCOME=FAIL; REASON="monitor did not confirm all three parking holds"
[[ -s "$BAG_DIR/metadata.yaml" ]] || DATA_VALIDITY=INVALID
if grep -qiE 'drop|warn|error' "$BAG_LOG"; then DATA_VALIDITY=INVALID; fi
if [[ "$STOP_REASON" == TASK_COMPLETE_GOAL && -s "$MONITOR_VERDICT" ]]; then TASK_OUTCOME=SUCCESS; REASON="monitor confirmed all three parking holds"; fi
cat >"$LOG_DIR/trial_verdict.json" <<EOF
{
  "comm_mode": "$COMM_MODE",
  "trial_index": $TRIAL_INDEX,
  "attempt": $ATTEMPT,
  "pilot_label": "$PILOT_LABEL",
  "data_validity": "$DATA_VALIDITY",
  "task_outcome": "$TASK_OUTCOME",
  "task_outcome_reason": "$REASON",
  "stop_reason": "$STOP_REASON",
  "native_bag_dir": "$BAG_DIR",
  "native_diag_log_dir": "$LOG_DIR",
  "git_commit": "$GIT_COMMIT"
}
EOF
cat "$LOG_DIR/trial_verdict.json"
echo "[$(date -Iseconds)] RECORDING_COMPLETE"
