#!/usr/bin/env bash
set -eo pipefail
#
# N2 (2-robot) cooperative exit-navigation orchestrator. Modeled directly
# on the proven experiments/05_objective5_impairment_matrix/tools/
# run_objective5_impairment_matrix_trial.sh (same interop check, residual
# check, setsid + stop_pid_group process-group discipline, native-WSL bag
# path, queue-drain-before-relay-stop rule). Reuses, UNMODIFIED:
#   - dual_namespaced_launch.py, epuck_namespaced.urdf, epuckN_ros2_control.yml
#     (the Webots working directory, located at 2-1.仿真通信实验/working after
#     the 2026-07-19 folder rename)
# NOT reused for this experiment category: two_epuck_head_on_clean_world.wbt
# (the shared, frozen Objective 5 A-D world) has no visible common
# exit/goal-region marker. Rather than modify that shared file, this
# category uses its own additive copy, two_epuck_cooperative_exit_n2_world.wbt
# (identical geometry/robot poses, plus one purely visual, non-colliding
# marker Solid at the goal region), launched via its own additive entry
# script run_dual_head_on_clean_n2_exit.py. Neither the original world file
# nor run_dual_head_on_clean.py is ever read, written, or referenced here.
#   - epuck2_comm state_publisher, network_impairment_relay,
#     sequence_counter (COMM_ON only), cooperative_avoider (frozen)
# New for this experiment category: run_n2_controllers.py (this dir) --
# the same make_controller pattern as run_comm_baseline_formal_controllers.py,
# parameterized by enable_peer_avoidance + stop_after_recovery.
#
# Usage:
#   run_cooperative_exit_n2_trial.sh COMM_MODE TRIAL_INDEX [ATTEMPT]
#   run_cooperative_exit_n2_trial.sh COMM_MODE TRIAL_INDEX --pilot LABEL
#   run_cooperative_exit_n2_trial.sh --check-only COMM_MODE TRIAL_INDEX [...]
#
#   COMM_MODE: N2_COMM_OFF | N2_COMM_ON
#
# --pilot LABEL produces a directory name containing EXCLUSIONARY_PILOT
# (never mistaken for a formal trial): objective7... no -- this experiment
# category is NOT numbered "Objective 7" (see design doc) --
# cooperative_exit_n2_<COMM_MODE>_EXCLUSIONARY_PILOT<LABEL>.

CHECK_ONLY="false"
if [[ "${1:-}" == "--check-only" ]]; then
  CHECK_ONLY="true"
  shift
fi

PILOT_LABEL=""
if (( $# == 4 )) && [[ "$3" == "--pilot" ]]; then
  COMM_MODE="$1"
  TRIAL_INDEX="$2"
  PILOT_LABEL="$4"
  ATTEMPT=1
elif (( $# >= 2 && $# <= 3 )); then
  COMM_MODE="$1"
  TRIAL_INDEX="$2"
  ATTEMPT="${3:-1}"
else
  echo "Usage: $0 [--check-only] COMM_MODE TRIAL_INDEX [ATTEMPT | --pilot LABEL]" >&2
  echo "       COMM_MODE: N2_COMM_OFF | N2_COMM_ON" >&2
  exit 2
fi

if [[ "$COMM_MODE" != "N2_COMM_OFF" && "$COMM_MODE" != "N2_COMM_ON" ]]; then
  echo "COMM_MODE must be N2_COMM_OFF or N2_COMM_ON, got '$COMM_MODE'" >&2
  exit 2
fi

REPO="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm"
EXP_DIR="$REPO/experiments/10_cooperative_exit_navigation_20260720"
TOOLS_DIR="$EXP_DIR/tools"
WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/2-1.仿真通信实验/working"

source /opt/ros/humble/setup.bash
source "$HOME/epuck_ws/install/setup.bash"
export WEBOTS_HOME="/mnt/c/Program Files/Webots"
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib/controller:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$WEBOTS_HOME/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
set -u

ORCH_SCRIPT="$TOOLS_DIR/run_cooperative_exit_n2_trial.sh"
ORCH_SHA256="$(sha256sum "$ORCH_SCRIPT" | awk '{print $1}')"
CONTROLLER_LAUNCH_SHA256="$(sha256sum "$TOOLS_DIR/run_n2_controllers.py" | awk '{print $1}')"
RELAY_SHA256="$(sha256sum "$REPO/src/epuck2_comm/epuck2_comm/network_impairment_relay.py" | awk '{print $1}')"
IMPAIRMENT_SHA256="$(sha256sum "$REPO/src/epuck2_comm/epuck2_comm/network_impairment.py" | awk '{print $1}')"
SEQCOUNTER_SHA256="$(sha256sum "$REPO/src/epuck2_comm/epuck2_comm/sequence_counter.py" | awk '{print $1}')"
COOPAVOIDER_SHA256="$(sha256sum "$REPO/src/epuck2_comm/epuck2_comm/cooperative_avoider.py" | awk '{print $1}')"
WORLD_SHA256="$(sha256sum "$WORK_DIR/two_epuck_cooperative_exit_n2_world.wbt" | awk '{print $1}')"
LAUNCH_ENTRY_SHA256="$(sha256sum "$WORK_DIR/run_dual_head_on_clean_n2_exit.py" | awk '{print $1}')"
GIT_COMMIT="$(cd "$REPO" && git rev-parse HEAD)"

if [[ "$CHECK_ONLY" == "true" ]]; then
  echo "=== --check-only: verifying interop, working directory, SHA-256 identity, argument passthrough. Webots is NOT launched. ==="
  if ! /mnt/c/Windows/System32/cmd.exe /c echo WSL_INTEROP_OK 2>/dev/null | grep -q WSL_INTEROP_OK; then
    echo "FATAL: WSL interop check failed." >&2
    exit 1
  fi
  echo "INTEROP_OK"
  for f in "$WORK_DIR/run_dual_head_on_clean_n2_exit.py" "$WORK_DIR/dual_namespaced_launch.py" \
           "$WORK_DIR/two_epuck_cooperative_exit_n2_world.wbt" "$WORK_DIR/epuck_namespaced.urdf" \
           "$WORK_DIR/epuck1_ros2_control.yml" "$WORK_DIR/epuck2_ros2_control.yml"; do
    if [[ -f "$f" ]]; then echo "OK    present: $f"; else echo "FAIL  missing: $f" >&2; exit 1; fi
  done
  echo "orchestrator_sha256=$ORCH_SHA256"
  echo "run_n2_controllers.py_sha256=$CONTROLLER_LAUNCH_SHA256"
  echo "network_impairment_relay.py_sha256=$RELAY_SHA256"
  echo "network_impairment.py_sha256=$IMPAIRMENT_SHA256"
  echo "sequence_counter.py_sha256=$SEQCOUNTER_SHA256"
  echo "cooperative_avoider.py_sha256=$COOPAVOIDER_SHA256 (frozen, must equal Condition A-D's own recorded value)"
  echo "two_epuck_cooperative_exit_n2_world.wbt_sha256=$WORLD_SHA256 (new, additive world copy + visual goal marker; A-D's own world file untouched)"
  echo "run_dual_head_on_clean_n2_exit.py_sha256=$LAUNCH_ENTRY_SHA256"
  echo "git_commit=$GIT_COMMIT"
  echo "comm_mode=$COMM_MODE trial_index=$TRIAL_INDEX pilot_label=${PILOT_LABEL:-<none>}"
  RESIDUAL="$(pgrep -af 'webots-bin|cooperative_avoider|state_publisher|ros2 bag record|network_impairment_relay|sequence_counter|task_completion_monitor' 2>/dev/null | grep -v 'bash -lc' || true)"
  if [[ -n "$RESIDUAL" ]]; then
    echo "FAIL  residual processes found:" >&2
    echo "$RESIDUAL" >&2
    exit 1
  fi
  echo "OK    no residual processes"
  echo "--check-only PASSED. No Webots launched."
  exit 0
fi

if [[ -n "$PILOT_LABEL" ]]; then
  STEM="cooperative_exit_${COMM_MODE,,}_EXCLUSIONARY_PILOT${PILOT_LABEL}"
else
  STEM="cooperative_exit_${COMM_MODE,,}_trial$(printf '%02d' "$TRIAL_INDEX")_attempt$(printf '%02d' "$ATTEMPT")"
fi

NATIVE_BAG_ROOT="/home/eamon/epuck_comm_bags"
NATIVE_BAG_DIR="$NATIVE_BAG_ROOT/$STEM"
FINAL_DIR="$EXP_DIR/${STEM}_analysis"
DIAG_LOG_DIR="$NATIVE_BAG_ROOT/${STEM}_diag_logs"
CONTROLLER_LOG="$DIAG_LOG_DIR/controller.log"
SIM_LOG="$DIAG_LOG_DIR/simulation.log"
STATE1_LOG="$DIAG_LOG_DIR/state_epuck1.log"
STATE2_LOG="$DIAG_LOG_DIR/state_epuck2.log"
RELAY_COUNTER_LOG="$DIAG_LOG_DIR/relay_counter.log"
BAG_RECORD_LOG="$DIAG_LOG_DIR/bag_record.log"
EXECUTION_LOG="$DIAG_LOG_DIR/execution.log"
MONITOR_LOG="$DIAG_LOG_DIR/task_completion_monitor.log"
MONITOR_VERDICT="$DIAG_LOG_DIR/monitor_verdict.json"

# Frozen goal/exit-region parameters (also written into frozen_params.json
# below) -- single source of truth shared by the orchestrator's own wait
# loop and the independent task_completion_monitor.py it launches.
# goal_radius_m corrected 2026-07-20 (post-PILOT04): 0.5 was larger than
# the robots' own 0.35m start-distance from the origin, so both robots'
# STATIC start poses were already inside the goal region -- PILOT04's
# "TASK_COMPLETE_GOAL" fired with path_length_m=0.0 for both robots,
# still in STARTUP_HOLD, never having moved. 0.20 requires a genuine
# net displacement (0.35 - 0.20 = 0.15m) before the hold timer can even
# start, while remaining achievable for two robots simultaneously
# (0.40m-diameter region comfortably fits two points >= safety_radius_m
# apart) per prior pilots' trajectory data. See
# cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT04_analysis/NOTE.md.
GOAL_CENTER_X_M=0.0
GOAL_CENTER_Y_M=0.0
GOAL_RADIUS_M=0.20
GOAL_HOLD_TIME_S=2.0

if [[ -e "$NATIVE_BAG_DIR" || -e "$FINAL_DIR" ]]; then
  echo "Refusing to overwrite existing trial: $NATIVE_BAG_DIR or $FINAL_DIR" >&2
  exit 1
fi
mkdir -p "$DIAG_LOG_DIR"

if ! /mnt/c/Windows/System32/cmd.exe /c echo WSL_INTEROP_OK 2>/dev/null | grep -q WSL_INTEROP_OK; then
  echo "WSL_INTEROP_BROKEN" >&2
  exit 1
fi
echo "[$(date -Iseconds)] WSL interop confirmed OK" | tee -a "$EXECUTION_LOG"

RESIDUAL="$(pgrep -af 'webots-bin|cooperative_avoider|state_publisher|ros2 bag record|network_impairment_relay|sequence_counter|task_completion_monitor' 2>/dev/null | grep -v 'bash -lc' || true)"
if [[ -n "$RESIDUAL" ]]; then
  echo "RESIDUAL_PROCESSES_FOUND:" >&2
  echo "$RESIDUAL" >&2
  exit 1
fi
echo "[$(date -Iseconds)] no residual processes found" | tee -a "$EXECUTION_LOG"

echo "[$(date -Iseconds)] $STEM START" | tee "$EXECUTION_LOG"
echo "comm_mode=$COMM_MODE trial_index=$TRIAL_INDEX attempt=$ATTEMPT pilot_label=${PILOT_LABEL:-<none>}" | tee -a "$EXECUTION_LOG"
echo "git_commit=$GIT_COMMIT orchestrator_sha256=$ORCH_SHA256 run_n2_controllers.py_sha256=$CONTROLLER_LAUNCH_SHA256" | tee -a "$EXECUTION_LOG"
echo "cooperative_avoider.py_sha256=$COOPAVOIDER_SHA256 (frozen)" | tee -a "$EXECUTION_LOG"

cat > "$DIAG_LOG_DIR/frozen_params.json" <<EOF
{
  "comm_mode": "$COMM_MODE",
  "trial_index": $TRIAL_INDEX,
  "attempt": $ATTEMPT,
  "pilot_label": "${PILOT_LABEL:-}",
  "goal_center_x_m": $GOAL_CENTER_X_M,
  "goal_center_y_m": $GOAL_CENTER_Y_M,
  "goal_radius_m": $GOAL_RADIUS_M,
  "goal_hold_time_s": $GOAL_HOLD_TIME_S,
  "safety_radius_m": 0.14,
  "collision_contact_distance_m": 0.07,
  "max_runtime_s": 28.0,
  "world_file": "two_epuck_cooperative_exit_n2_world.wbt",
  "world_file_sha256": "$WORLD_SHA256",
  "launch_entry_sha256": "$LAUNCH_ENTRY_SHA256",
  "git_commit": "$GIT_COMMIT",
  "orchestrator_sha256": "$ORCH_SHA256",
  "run_n2_controllers_py_sha256": "$CONTROLLER_LAUNCH_SHA256",
  "network_impairment_relay_py_sha256": "$RELAY_SHA256",
  "network_impairment_py_sha256": "$IMPAIRMENT_SHA256",
  "sequence_counter_py_sha256": "$SEQCOUNTER_SHA256",
  "cooperative_avoider_py_sha256": "$COOPAVOIDER_SHA256"
}
EOF

stop_pid() {
  local pid="${1:-}"
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then return 0; fi
  kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    if ! kill -0 "$pid" 2>/dev/null; then wait "$pid" 2>/dev/null || true; return 0; fi
    sleep 0.5
  done
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 10); do
    if ! kill -0 "$pid" 2>/dev/null; then wait "$pid" 2>/dev/null || true; return 0; fi
    sleep 0.5
  done
  return 1
}

stop_pid_group() {
  local pid="${1:-}"
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then return 0; fi
  kill -INT "-$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    if ! kill -0 "$pid" 2>/dev/null && ! pgrep -g "$pid" >/dev/null 2>&1; then wait "$pid" 2>/dev/null || true; return 0; fi
    sleep 0.5
  done
  kill -TERM "-$pid" 2>/dev/null || true
  for _ in $(seq 1 10); do
    if ! kill -0 "$pid" 2>/dev/null && ! pgrep -g "$pid" >/dev/null 2>&1; then wait "$pid" 2>/dev/null || true; return 0; fi
    sleep 0.5
  done
  kill -KILL "-$pid" 2>/dev/null || true
  sleep 0.5
  wait "$pid" 2>/dev/null || true
  return 0
}

wait_for_topics() {
  local timeout_s="$1"; shift
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    local topics ready=1
    topics="$(ros2 topic list 2>/dev/null || true)"
    for expected in "$@"; do
      grep -Fxq "$expected" <<<"$topics" || { ready=0; break; }
    done
    (( ready == 1 )) && return 0
    sleep 1
  done
  echo "Timed out waiting for topics: $*" >&2
  return 1
}

SIM_PID=""; STATE1_PID=""; STATE2_PID=""; RELAY_COUNTER_PID=""; CONTROLLER_PID=""; BAG_PID=""; MONITOR_PID=""
DATA_VALIDITY="VALID"; INVALID_REASON=""; QUEUE_DRAINED="true"; BAG_LOG_CLEAN="true"

cleanup() {
  stop_pid_group "$STATE1_PID" || true
  stop_pid_group "$STATE2_PID" || true
  stop_pid_group "$CONTROLLER_PID" || true
  stop_pid_group "$MONITOR_PID" || true
  stop_pid_group "$RELAY_COUNTER_PID" || true
  stop_pid "$BAG_PID" || true
  stop_pid_group "$SIM_PID" || true
}
trap cleanup EXIT

setsid bash -c "cd '$WORK_DIR' && exec python3 run_dual_head_on_clean_n2_exit.py" >"$SIM_LOG" 2>&1 &
SIM_PID=$!
wait_for_topics 90 /epuck1/odom /epuck2/odom /epuck1/tof /epuck2/tof /epuck1/ps0 /epuck2/ps0
echo "[$(date -Iseconds)] odometry and local sensors ready" | tee -a "$EXECUTION_LOG"

if [[ "$COMM_MODE" == "N2_COMM_ON" ]]; then
  setsid stdbuf -oL -eL python3 "$REPO/experiments/05_objective5_impairment_matrix/tools/run_matrix_relay_and_counter.py" \
    --diag-log-dir "$DIAG_LOG_DIR" \
    --delay-s 0.0 --jitter-s 0.0 --drop-probability 0.0 \
    --outage-period-s 0.0 --outage-duration-s 0.0 --outage-phase-s 0.0 \
    --seed-epuck1 1 --seed-epuck2 2 \
    >"$RELAY_COUNTER_LOG" 2>&1 &
  RELAY_COUNTER_PID=$!
  sleep 2
  echo "[$(date -Iseconds)] relay+counter subscribed (COMM_ON, zero impairment)" | tee -a "$EXECUTION_LOG"
else
  echo "[$(date -Iseconds)] COMM_OFF: no relay/counter launched -- no cross-robot state topic will exist" | tee -a "$EXECUTION_LOG"
fi

# network_impairment_relay.py bridges /epuckN/state_raw -> /epuckN/state
# WITHIN THE SAME ROBOT'S NAMESPACE (confirmed by reading the relay's own
# module docstring) -- cooperative_avoider's "own state" subscription is
# to the bare relative topic "state" (== /epuckN/state), so /epuckN/state
# is what BOTH that robot's own controller (own state) AND the OTHER
# robot's controller (peer state, via the absolute peer_state_topic
# parameter) subscribe to. This means the relay is NOT optional
# "peer-only" plumbing -- it is required even for a robot's OWN state to
# ever reach its own controller. COMM_OFF must therefore NOT remap
# state:=state_raw (which would leave the bare "state" topic with no
# publisher at all, and BOTH own-state and peer-state deprived) -- it
# publishes state_publisher's output directly to the un-remapped "state"
# topic instead, with NO relay/impairment node running at all, which is
# still a strictly stronger form of "no communication" than A-D's
# zero-impairment relay pass-through (no relay process exists here at
# all, not merely a no-op one).
if [[ "$COMM_MODE" == "N2_COMM_ON" ]]; then
  setsid ros2 run epuck2_comm state_publisher --ros-args \
    -r __ns:=/epuck1 -r state:=state_raw -p robot_id:=1 -p use_sim_time:=true \
    -p mode:=periodic -p origin_x_m:=-0.35 -p origin_y_m:=0.0 -p origin_yaw_rad:=0.0 \
    >"$STATE1_LOG" 2>&1 &
  STATE1_PID=$!
  setsid ros2 run epuck2_comm state_publisher --ros-args \
    -r __ns:=/epuck2 -r state:=state_raw -p robot_id:=2 -p use_sim_time:=true \
    -p mode:=periodic -p origin_x_m:=0.35 -p origin_y_m:=0.0 -p origin_yaw_rad:=3.141592653589793 \
    >"$STATE2_LOG" 2>&1 &
  STATE2_PID=$!
  wait_for_topics 30 /epuck1/state_raw /epuck2/state_raw /epuck1/state /epuck2/state
else
  setsid ros2 run epuck2_comm state_publisher --ros-args \
    -r __ns:=/epuck1 -p robot_id:=1 -p use_sim_time:=true \
    -p mode:=periodic -p origin_x_m:=-0.35 -p origin_y_m:=0.0 -p origin_yaw_rad:=0.0 \
    >"$STATE1_LOG" 2>&1 &
  STATE1_PID=$!
  setsid ros2 run epuck2_comm state_publisher --ros-args \
    -r __ns:=/epuck2 -p robot_id:=2 -p use_sim_time:=true \
    -p mode:=periodic -p origin_x_m:=0.35 -p origin_y_m:=0.0 -p origin_yaw_rad:=3.141592653589793 \
    >"$STATE2_LOG" 2>&1 &
  STATE2_PID=$!
  wait_for_topics 30 /epuck1/state /epuck2/state
fi
sleep 2
echo "[$(date -Iseconds)] state topics ready" | tee -a "$EXECUTION_LOG"

mkdir -p "$NATIVE_BAG_ROOT"
if [[ "$COMM_MODE" == "N2_COMM_ON" ]]; then
  BAG_TOPICS=(/epuck1/state_raw /epuck2/state_raw /epuck1/state /epuck2/state /epuck1/cmd_vel /epuck2/cmd_vel /epuck1/relay_status /epuck2/relay_status)
else
  # COMM_OFF: no state_raw/state distinction (no relay in the loop at
  # all) -- state_publisher's output IS /epuckN/state directly.
  BAG_TOPICS=(/epuck1/state /epuck2/state /epuck1/cmd_vel /epuck2/cmd_vel)
fi
ros2 bag record -o "$NATIVE_BAG_DIR" "${BAG_TOPICS[@]}" >"$BAG_RECORD_LOG" 2>&1 &
BAG_PID=$!
sleep 3
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "rosbag recorder exited before recording could start" >&2
  DATA_VALIDITY="INVALID"; INVALID_REASON="bag recorder failed to start"
  exit 1
fi
echo "[$(date -Iseconds)] rosbag recording to NATIVE path: $NATIVE_BAG_DIR" | tee -a "$EXECUTION_LOG"

# task_completion_monitor.py: independent, read-only observer. Subscribes
# only to /epuck1/state and /epuck2/state (the SAME already-published
# topic each robot's own controller reads its own state from -- unchanged
# in both COMM_OFF and COMM_ON). Never publishes cmd_vel or a nav target,
# never touches Supervisor, never feeds data back into either controller.
# It only writes TASK_COMPLETE_GOAL to its own log once BOTH robots have
# held the frozen goal region for >= goal_hold_time_s -- the wait loop
# below watches for that line so the trial can stop on genuine task
# completion instead of only ever via the controller's own recovery-based
# self-completion or max_runtime_s.
setsid stdbuf -oL -eL python3 "$TOOLS_DIR/task_completion_monitor.py" \
  --robot-ids epuck1,epuck2 \
  --state-topics /epuck1/state,/epuck2/state \
  --goal-center-x-m "$GOAL_CENTER_X_M" --goal-center-y-m "$GOAL_CENTER_Y_M" \
  --goal-radius-m "$GOAL_RADIUS_M" --goal-hold-time-s "$GOAL_HOLD_TIME_S" \
  --verdict-path "$MONITOR_VERDICT" \
  >"$MONITOR_LOG" 2>&1 &
MONITOR_PID=$!
sleep 1
echo "[$(date -Iseconds)] task_completion_monitor launched (read-only, watching /epuck1/state,/epuck2/state)" | tee -a "$EXECUTION_LOG"

N2_COMM_MODE="$COMM_MODE" setsid stdbuf -oL -eL python3 "$TOOLS_DIR/run_n2_controllers.py" \
  >"$CONTROLLER_LOG" 2>&1 &
CONTROLLER_PID=$!

deadline=$((SECONDS + 75))
complete_count=0
STOP_REASON="MAX_RUNTIME"
while (( SECONDS < deadline )); do
  complete_count="$(grep -c 'COMPLETE:' "$CONTROLLER_LOG" 2>/dev/null || true)"
  if grep -q 'TASK_COMPLETE_GOAL' "$MONITOR_LOG" 2>/dev/null; then
    STOP_REASON="TASK_COMPLETE_GOAL"
    echo "[$(date -Iseconds)] task_completion_monitor reports TASK_COMPLETE_GOAL -- stopping trial now, not waiting for max_runtime" | tee -a "$EXECUTION_LOG"
    break
  fi
  if (( complete_count >= 2 )); then
    STOP_REASON="CONTROLLER_SELF_COMPLETE"
    break
  fi
  if ! kill -0 "$CONTROLLER_PID" 2>/dev/null; then
    echo "[$(date -Iseconds)] controller exited (complete_count=$complete_count)" | tee -a "$EXECUTION_LOG"
    STOP_REASON="CONTROLLER_EXITED_EARLY"
    break
  fi
  sleep 0.2
done
echo "[$(date -Iseconds)] controller stage finished (complete_count=$complete_count stop_reason=$STOP_REASON)" | tee -a "$EXECUTION_LOG"

echo "[$(date -Iseconds)] stopping controller (relay/bag/counter/sim remain running for drain)" | tee -a "$EXECUTION_LOG"
# SIGINT via stop_pid_group -- the SAME safe-stop path used at every other
# end-of-trial point. This is what triggers cooperative_avoider.py's own,
# unmodified stop() method (publishes a zero Twist 3x from its
# SIGINT/KeyboardInterrupt handler) -- reused, never a pkill/hard-kill
# substitute for a genuine safe-stop.
stop_pid_group "$CONTROLLER_PID"; CONTROLLER_PID=""
stop_pid_group "$MONITOR_PID"; MONITOR_PID=""
# Settle window: state_publisher (a separate, still-running process) is
# stopped only AFTER this delay, so the bag captures a bit more of the
# robot's REAL post-stop odometry/velocity (Webots-derived, independent
# of cmd_vel) -- this is what verify_state_velocity_settled.py checks,
# not the raw /epuckN/cmd_vel topic (see that script's docstring for why
# the raw cmd_vel topic's last sample is not a reliable signal here).
sleep 1.0
stop_pid_group "$STATE1_PID"; STATE1_PID=""
stop_pid_group "$STATE2_PID"; STATE2_PID=""

if [[ "$COMM_MODE" == "N2_COMM_ON" ]]; then
  DRAIN_DURATION_S=0.3
  echo "[$(date -Iseconds)] draining relay queues for ${DRAIN_DURATION_S}s (zero impairment, small margin)" | tee -a "$EXECUTION_LOG"
  sleep "$DRAIN_DURATION_S"
  for ns in epuck1 epuck2; do
    STATUS_RAW=""
    for _attempt in 1 2; do
      STATUS_RAW="$(timeout 6 ros2 topic echo "/$ns/relay_status" --once --field data 2>/dev/null || true)"
      [[ -n "$STATUS_RAW" ]] && break
    done
    STATUS_JSON="$(head -n1 <<<"$STATUS_RAW")"
    echo "[$(date -Iseconds)] $ns relay_status after drain wait: $STATUS_JSON" | tee -a "$EXECUTION_LOG"
    PENDING="$(python3 -c "
import json,sys
try:
    print(json.loads(sys.argv[1])['pending_queue_depth'])
except Exception:
    print('UNKNOWN')
" "$STATUS_JSON" 2>/dev/null || echo UNKNOWN)"
    if [[ "$PENDING" != "0" ]]; then
      echo "[$(date -Iseconds)] $ns relay queue NOT drained (pending=$PENDING)" | tee -a "$EXECUTION_LOG"
      DATA_VALIDITY="INVALID"; QUEUE_DRAINED="false"
      INVALID_REASON="${INVALID_REASON}${ns} relay queue not drained (pending=$PENDING); "
    fi
  done
  echo "[$(date -Iseconds)] stopping relay+counter" | tee -a "$EXECUTION_LOG"
  stop_pid_group "$RELAY_COUNTER_PID"; RELAY_COUNTER_PID=""
  sleep 1
fi

echo "[$(date -Iseconds)] stopping rosbag recorder" | tee -a "$EXECUTION_LOG"
stop_pid "$BAG_PID"; BAG_PID=""

if [[ ! -s "$NATIVE_BAG_DIR/metadata.yaml" ]]; then
  echo "[$(date -Iseconds)] rosbag metadata is missing or empty" | tee -a "$EXECUTION_LOG" >&2
  DATA_VALIDITY="INVALID"; INVALID_REASON="${INVALID_REASON}bag metadata missing/empty; "
fi
echo "[$(date -Iseconds)] rosbag metadata.yaml check done" | tee -a "$EXECUTION_LOG"

# Post-hoc, read-only checks on the now-closed bag.
#
# 1) /epuckN/cmd_vel's raw last sample -- INFORMATIONAL ONLY, not a
#    DATA_VALIDITY gate. PILOT05 showed this can legitimately be
#    non-zero even after a clean SIGINT/stop() shutdown: the frozen
#    stop() method publishes zero 3x then immediately destroy_node()s,
#    and those final publishes can be lost to a DDS-teardown race before
#    the bag recorder captures them (the controller's own code comment
#    acknowledges this: "Motion is also bounded by the driver watchdog,
#    so cleanup must remain quiet"). Logged for traceability only.
CMD_VEL_ZERO="true"
if [[ -s "$NATIVE_BAG_DIR/metadata.yaml" ]]; then
  if ! python3 "$TOOLS_DIR/verify_cmd_vel_zero.py" "$NATIVE_BAG_DIR" /epuck1/cmd_vel /epuck2/cmd_vel 2>&1 | tee -a "$EXECUTION_LOG"; then
    CMD_VEL_ZERO="false"
  fi
else
  CMD_VEL_ZERO="false"
fi
echo "[$(date -Iseconds)] cmd_vel raw-last-sample check done, INFORMATIONAL ONLY (cmd_vel_zero=$CMD_VEL_ZERO)" | tee -a "$EXECUTION_LOG"

# 2) /epuckN/state.linear_velocity_mps's last sample -- the REAL
#    DATA_VALIDITY gate. state_publisher keeps running (and the trial
#    keeps recording) for the 1.0s settle window above, capturing the
#    robot's ACTUAL physical odometry-derived velocity, independent of
#    whether a specific cmd_vel message reached the bag before the
#    controller process exited.
VELOCITY_SETTLED="true"
if [[ -s "$NATIVE_BAG_DIR/metadata.yaml" ]]; then
  if ! python3 "$TOOLS_DIR/verify_state_velocity_settled.py" "$NATIVE_BAG_DIR" /epuck1/state /epuck2/state 2>&1 | tee -a "$EXECUTION_LOG"; then
    VELOCITY_SETTLED="false"
    DATA_VALIDITY="INVALID"
    INVALID_REASON="${INVALID_REASON}robot velocity not settled near zero at trial end; "
  fi
else
  VELOCITY_SETTLED="false"
fi
echo "[$(date -Iseconds)] state-velocity-settled check done (velocity_settled=$VELOCITY_SETTLED)" | tee -a "$EXECUTION_LOG"

stop_pid_group "$SIM_PID"; SIM_PID=""
sleep 2

BAG_WARN_LINES="$(grep -icE 'drop|warn|error' "$BAG_RECORD_LOG" 2>/dev/null)" || true
BAG_WARN_LINES="${BAG_WARN_LINES:-0}"
grep -iE 'drop|warn|error' "$BAG_RECORD_LOG" | tee -a "$EXECUTION_LOG" || echo "no drop/warn/error lines in bag_record.log" | tee -a "$EXECUTION_LOG"
if [[ "$BAG_WARN_LINES" != "0" ]]; then
  DATA_VALIDITY="INVALID"; BAG_LOG_CLEAN="false"
  INVALID_REASON="${INVALID_REASON}bag_record.log has $BAG_WARN_LINES drop/warn/error line(s); "
fi

CONTROLLER_CRASHED="false"
# STOP_REASON=TASK_COMPLETE_GOAL means the orchestrator deliberately
# SIGINT'd a still-healthy controller before it reached its own internal
# _complete() -- that controller.log therefore legitimately shows neither
# "COMPLETE:" nor a Traceback/ExternalShutdownException signature. This is
# the expected shape of a genuine early stop, not a crash.
if [[ "$STOP_REASON" != "TASK_COMPLETE_GOAL" ]] && ! grep -q 'COMPLETE:' "$CONTROLLER_LOG" 2>/dev/null && (( complete_count < 2 )); then
  if ! grep -qE 'Traceback|rclpy\.executors\.ExternalShutdownException' "$CONTROLLER_LOG" 2>/dev/null; then
    CONTROLLER_CRASHED="true"
    DATA_VALIDITY="INVALID"
    INVALID_REASON="${INVALID_REASON}controller stage ended without COMPLETE and without an expected shutdown signature; "
  fi
fi

# ended_by_max_runtime: true only if BOTH robots' controller.log shows the
# max-runtime completion message specifically (never inferred from any
# other exit path) -- read by finalize_n2_pilot.py, not decided here.
MAX_RUNTIME_HITS="$(grep -c 'maximum runtime reached' "$CONTROLLER_LOG" 2>/dev/null || true)"
MAX_RUNTIME_HITS="${MAX_RUNTIME_HITS:-0}"

MONITOR_VERDICT_PRESENT="false"
[[ -s "$MONITOR_VERDICT" ]] && MONITOR_VERDICT_PRESENT="true"

cat > "$DIAG_LOG_DIR/trial_verdict.json" <<EOF
{
  "comm_mode": "$COMM_MODE",
  "trial_index": $TRIAL_INDEX,
  "attempt": $ATTEMPT,
  "data_validity": "$DATA_VALIDITY",
  "data_validity_reason": "$INVALID_REASON",
  "stop_reason": "$STOP_REASON",
  "controller_complete_count": $complete_count,
  "controller_crashed": $CONTROLLER_CRASHED,
  "ended_by_max_runtime_hits": $MAX_RUNTIME_HITS,
  "cmd_vel_zero_at_end": $CMD_VEL_ZERO,
  "cmd_vel_zero_at_end_informational_only": true,
  "velocity_settled_at_end": $VELOCITY_SETTLED,
  "monitor_verdict_present": $MONITOR_VERDICT_PRESENT,
  "monitor_verdict_path": "$MONITOR_VERDICT",
  "queue_drained": $QUEUE_DRAINED,
  "bag_log_clean": $BAG_LOG_CLEAN,
  "native_bag_dir": "$NATIVE_BAG_DIR",
  "native_diag_log_dir": "$DIAG_LOG_DIR",
  "git_commit": "$GIT_COMMIT"
}
EOF
echo "[$(date -Iseconds)] $STEM RECORDING_COMPLETE data_validity=$DATA_VALIDITY stop_reason=$STOP_REASON complete_count=$complete_count" | tee -a "$EXECUTION_LOG"
cat "$DIAG_LOG_DIR/trial_verdict.json"
echo "native bag dir: $NATIVE_BAG_DIR"
echo "diag log dir: $DIAG_LOG_DIR"
echo "stem: $STEM"
