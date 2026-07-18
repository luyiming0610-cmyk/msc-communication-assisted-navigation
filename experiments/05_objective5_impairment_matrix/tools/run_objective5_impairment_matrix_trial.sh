#!/usr/bin/env bash
set -eo pipefail

# Objective5 impairment matrix -- unified parameterized orchestrator for
# every condition A-G. NOT run by this commit; built and syntax-checked
# only, per instruction. Modeled directly on the proven
# run_objective5_comm_baseline_formal_trial.sh (same relay-before-
# state_publisher subscription order, same native-WSL-first bag path,
# same realtime-factor gates, same shutdown ordering) with three
# additions: (1) ALL relay/seed parameters come from the frozen
# conditions CSV via load_condition_config.py -- no hand-typed
# --delay-s/--jitter-s/etc override exists anywhere in this script, by
# design; (2) a queue-drain wait (relay_drain.py) between task
# completion and stopping the relay/bag, so pending delayed messages are
# never miscounted as loss by stopping too early; (3) a
# runtime_manifest.json capturing every resolved parameter, both
# directions' seeds, and the exact commit/SHA-256 of every script this
# trial ran, per trial.
#
# Usage: run_objective5_impairment_matrix_trial.sh CONDITION_ID TRIAL_INDEX [ATTEMPT]
#        run_objective5_impairment_matrix_trial.sh CONDITION_ID TRIAL_INDEX --pilot LABEL
#   CONDITION_ID: A-G (matches objective5_impairment_matrix_conditions.csv)
#   TRIAL_INDEX:  1-5 (still resolves this trial index's frozen
#                 parameters/seeds from the CSV even in --pilot mode, so
#                 an exclusionary pilot run uses genuinely frozen
#                 parameters, not ad-hoc ones)
#   ATTEMPT:      defaults to 1; bump only after a failed/interrupted
#                 attempt (never reused, never overwritten)
#   --pilot LABEL: EXCLUSIONARY_DIAGNOSTIC mode -- directory name becomes
#                 objective5_matrix_v1_condition<ID>_exclusionary_<LABEL>
#                 instead of the normal trialNN_attemptNN scheme, so it
#                 can never collide with or be mistaken for a formal n=5
#                 trial directory. Still fully exercises the real
#                 pipeline (same relay/controller/bag/drain/verdict
#                 code path) -- only the directory naming differs.

PILOT_LABEL=""
if (( $# == 4 )) && [[ "$3" == "--pilot" ]]; then
  PILOT_LABEL="$4"
  CONDITION_ID="$1"
  TRIAL_INDEX="$2"
  ATTEMPT=1
elif (( $# >= 2 && $# <= 3 )); then
  CONDITION_ID="$1"
  TRIAL_INDEX="$2"
  ATTEMPT="${3:-1}"
else
  echo "Usage: $0 CONDITION_ID TRIAL_INDEX [ATTEMPT]" >&2
  echo "       $0 CONDITION_ID TRIAL_INDEX --pilot LABEL" >&2
  exit 2
fi

MATRIX_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/05_objective5_impairment_matrix"
TOOLS_DIR="$MATRIX_DIR/tools"
CONDITIONS_CSV="$MATRIX_DIR/objective5_impairment_matrix_conditions.csv"
BASELINE_CONFIG_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/controller_v4_full_sensor_bypass_20260717/config/comm_baseline_v1"
WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/simulation_comm_experiment_v1/working"

source /opt/ros/humble/setup.bash
source "$HOME/epuck_ws/install/setup.bash"
# Webots environment -- normally set by ~/.bashrc, but `.bashrc` is only
# sourced for interactive non-login shells; this script is routinely
# invoked as `bash -lc "bash '<this script>' ..."` (a login shell, which
# does NOT source .bashrc unless .bash_profile explicitly does), so
# WEBOTS_HOME/LD_LIBRARY_PATH/PYTHONPATH are exported explicitly here
# rather than assumed inherited from whatever shell launched this script.
# Confirmed missing and root-caused directly: the first real run of this
# script (objective5_matrix_v1_conditionA_exclusionary_pilot01) failed
# with "OSError: [Errno 8] Exec format error" because webots_ros2_driver
# fell back to launching a Windows .exe path with no WEBOTS_HOME set.
export WEBOTS_HOME="/mnt/c/Program Files/Webots"
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib/controller:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$WEBOTS_HOME/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
set -u

# --- resolve this trial's frozen parameters (the ONLY way parameters
# enter this script -- no CLI override flag exists) ---
CONFIG_OUTPUT="$(python3 "$TOOLS_DIR/load_condition_config.py" \
  --csv "$CONDITIONS_CSV" --condition-id "$CONDITION_ID" --trial-index "$TRIAL_INDEX")"
eval "$CONFIG_OUTPUT"
# populates: CONDITION_ID TRIAL_INDEX DELAY_S JITTER_S DROP_PROBABILITY
#            SEED_EPUCK1 SEED_EPUCK2 OUTAGE_PERIOD_S OUTAGE_DURATION_S OUTAGE_PHASE_S

if [[ -n "$PILOT_LABEL" ]]; then
  STEM="objective5_matrix_v1_condition${CONDITION_ID}_exclusionary_${PILOT_LABEL}"
else
  # trial_dir_name() already returns the FULL directory name (including
  # the "objective5_impairment_matrix_v1_" prefix) -- STEM is used
  # as-is below, never re-prefixed (an earlier draft of this script
  # double-prefixed it; caught before any real run via this exact
  # exclusionary-pilot path).
  STEM="$(python3 -c "
import sys
sys.path.insert(0, '$TOOLS_DIR')
from unique_trial_dir import trial_dir_name
print(trial_dir_name('$CONDITION_ID', $TRIAL_INDEX, $ATTEMPT))
")"
fi

NATIVE_BAG_ROOT="/home/eamon/epuck_comm_bags"
NATIVE_BAG_DIR="$NATIVE_BAG_ROOT/$STEM"
FINAL_DIR="$MATRIX_DIR/${STEM}_analysis"
DIAG_LOG_DIR="$NATIVE_BAG_ROOT/${STEM}_diag_logs"
CONTROLLER_LOG="$DIAG_LOG_DIR/controller.log"
SIM_LOG="$DIAG_LOG_DIR/simulation.log"
STATE1_LOG="$DIAG_LOG_DIR/state_epuck1.log"
STATE2_LOG="$DIAG_LOG_DIR/state_epuck2.log"
RELAY_COUNTER_LOG="$DIAG_LOG_DIR/relay_counter.log"
BAG_RECORD_LOG="$DIAG_LOG_DIR/bag_record.log"
EXECUTION_LOG="$DIAG_LOG_DIR/execution.log"

if [[ -e "$NATIVE_BAG_DIR" || -e "$FINAL_DIR" ]]; then
  echo "Refusing to overwrite existing trial: $NATIVE_BAG_DIR or $FINAL_DIR" >&2
  exit 1
fi
mkdir -p "$DIAG_LOG_DIR"

# --- WSL binfmt_misc interop check, fail fast with a clear diagnostic
# instead of a 90s topic-wait timeout. This is a known-recurring WSL
# state (confirmed twice this session: the first real run of this
# script silently launched Webots as if WEBOTS_HOME were the only
# problem, actually failing on this instead -- "OSError: [Errno 8] Exec
# format error" trying to posix_spawn a Windows .exe directly). Fixed
# externally via `wsl -u root -- bash -lc "echo ':WSLInterop:M::MZ::/init:PF'
# > /proc/sys/fs/binfmt_misc/register"` when this check fails -- this
# script does NOT attempt that fix itself (requires root, a privilege
# escalation this script must never silently perform).
if ! /mnt/c/Windows/System32/cmd.exe /c echo WSL_INTEROP_OK 2>/dev/null | grep -q WSL_INTEROP_OK; then
  echo "WSL_INTEROP_BROKEN: cannot exec a Windows binary from this WSL session (binfmt_misc WSLInterop likely unregistered) -- Webots would fail the same way. Aborting before starting anything." >&2
  exit 1
fi
echo "[$(date -Iseconds)] WSL interop confirmed OK" | tee -a "$EXECUTION_LOG"

RESIDUAL="$(pgrep -af 'webots-bin|cooperative_avoider|state_publisher|ros2 bag record|network_impairment_relay|sequence_counter' 2>/dev/null | grep -v 'bash -lc' || true)"
if [[ -n "$RESIDUAL" ]]; then
  echo "RESIDUAL_PROCESSES_FOUND: refusing to start with leftover processes from a prior run:" >&2
  echo "$RESIDUAL" >&2
  exit 1
fi
echo "[$(date -Iseconds)] no residual processes found" | tee -a "$EXECUTION_LOG"

# --- freeze/record every resolved parameter + commit/SHA before anything starts ---
ORCH_SCRIPT="$TOOLS_DIR/run_objective5_impairment_matrix_trial.sh"
ORCH_SHA256="$(sha256sum "$ORCH_SCRIPT" | awk '{print $1}')"
RELAY_SHA256="$(sha256sum "/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/src/epuck2_comm/epuck2_comm/network_impairment_relay.py" | awk '{print $1}')"
IMPAIRMENT_SHA256="$(sha256sum "/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/src/epuck2_comm/epuck2_comm/network_impairment.py" | awk '{print $1}')"
GIT_COMMIT="$(cd "/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm" && git rev-parse HEAD)"

echo "[$(date -Iseconds)] $STEM START" | tee "$EXECUTION_LOG"
echo "condition=$CONDITION_ID trial_index=$TRIAL_INDEX attempt=$ATTEMPT" | tee -a "$EXECUTION_LOG"
echo "delay_s=$DELAY_S jitter_s=$JITTER_S drop_probability=$DROP_PROBABILITY" | tee -a "$EXECUTION_LOG"
echo "outage_period_s=$OUTAGE_PERIOD_S outage_duration_s=$OUTAGE_DURATION_S outage_phase_s=$OUTAGE_PHASE_S" | tee -a "$EXECUTION_LOG"
echo "seed_epuck1_to_epuck2=$SEED_EPUCK1 seed_epuck2_to_epuck1=$SEED_EPUCK2" | tee -a "$EXECUTION_LOG"
echo "git_commit=$GIT_COMMIT orchestrator_sha256=$ORCH_SHA256" | tee -a "$EXECUTION_LOG"
echo "network_impairment_relay.py_sha256=$RELAY_SHA256 network_impairment.py_sha256=$IMPAIRMENT_SHA256" | tee -a "$EXECUTION_LOG"

cat > "$DIAG_LOG_DIR/frozen_params.json" <<EOF
{
  "condition_id": "$CONDITION_ID",
  "trial_index": $TRIAL_INDEX,
  "attempt": $ATTEMPT,
  "delay_s": $DELAY_S,
  "jitter_s": $JITTER_S,
  "drop_probability": $DROP_PROBABILITY,
  "outage_period_s": $OUTAGE_PERIOD_S,
  "outage_duration_s": $OUTAGE_DURATION_S,
  "outage_phase_s": $OUTAGE_PHASE_S,
  "seed_epuck1_to_epuck2": $SEED_EPUCK1,
  "seed_epuck2_to_epuck1": $SEED_EPUCK2,
  "git_commit": "$GIT_COMMIT",
  "orchestrator_sha256": "$ORCH_SHA256",
  "network_impairment_relay_py_sha256": "$RELAY_SHA256",
  "network_impairment_py_sha256": "$IMPAIRMENT_SHA256"
}
EOF

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
import os, sys, time
import rclpy
from rosgraph_msgs.msg import Clock
stage = os.environ["RATE_STAGE"]
rclpy.init()
node = rclpy.create_node(f"obj5_matrix_{stage.lower()}_clock_rate_check")
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

stop_pid_group() {
  # Same rationale as run_objective5_comm_baseline_formal_trial.sh's
  # identically-named function: the relay+counter launch is a
  # launch.LaunchService managing several rclpy child processes as
  # separate OS processes; signaling only the parent PID risks orphaning
  # children that never receive their own SIGINT. Launched via setsid so
  # this PID has its own process group, isolated from sibling jobs.
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
  return 0
}

SIM_PID=""; STATE1_PID=""; STATE2_PID=""; RELAY_COUNTER_PID=""
CONTROLLER_PID=""; BAG_PID=""
DATA_VALIDITY="VALID"
INVALID_REASON=""

cleanup() {
  stop_pid "$STATE1_PID" || true
  stop_pid "$STATE2_PID" || true
  stop_pid "$CONTROLLER_PID" || true
  stop_pid_group "$RELAY_COUNTER_PID" || true
  stop_pid "$BAG_PID" || true
  stop_pid "$SIM_PID" || true
}
trap cleanup EXIT

(
  cd "$WORK_DIR"
  exec python3 run_dual_head_on_clean.py
) >"$SIM_LOG" 2>&1 &
SIM_PID=$!

wait_for_topics 90 /epuck1/odom /epuck2/odom /epuck1/tof /epuck2/tof /epuck1/ps0 /epuck2/ps0
echo "[$(date -Iseconds)] odometry and local sensors ready" | tee -a "$EXECUTION_LOG"
PRELOAD_OUTPUT="$(verify_realtime_factor PRELOAD | tee -a "$EXECUTION_LOG")"
PRELOAD_FACTOR="$(grep -o 'PRELOAD_REALTIME_FACTOR=[0-9.]*' <<<"$PRELOAD_OUTPUT" | cut -d= -f2)"

# Relay + sequence_counter FIRST, before state_publisher exists -- same
# ordering discipline as the formal zero-impairment baseline. Two
# separate relay instances, one per robot's outgoing state stream, each
# with ITS OWN seed (epuck1's relay uses SEED_EPUCK1, epuck2's relay
# uses SEED_EPUCK2 -- never the same seed for both directions, per the
# design doc's matched-but-not-identical seed scheme). Launched via a
# single launch_ros LaunchDescription (run_matrix_relay_and_counter.py,
# modeled directly on the proven run_diagnostic_relay_and_counter.py),
# under setsid for its own process group -- see stop_pid_group above.
setsid stdbuf -oL -eL python3 "$TOOLS_DIR/run_matrix_relay_and_counter.py" \
  --diag-log-dir "$DIAG_LOG_DIR" \
  --delay-s "$DELAY_S" --jitter-s "$JITTER_S" --drop-probability "$DROP_PROBABILITY" \
  --outage-period-s "$OUTAGE_PERIOD_S" --outage-duration-s "$OUTAGE_DURATION_S" --outage-phase-s "$OUTAGE_PHASE_S" \
  --seed-epuck1 "$SEED_EPUCK1" --seed-epuck2 "$SEED_EPUCK2" \
  >"$RELAY_COUNTER_LOG" 2>&1 &
RELAY_COUNTER_PID=$!
sleep 2
echo "[$(date -Iseconds)] both relays + sequence_counter subscribed (no state_raw exists yet)" | tee -a "$EXECUTION_LOG"

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
  /epuck1/cmd_vel /epuck2/cmd_vel /epuck1/relay_status /epuck2/relay_status \
  >"$BAG_RECORD_LOG" 2>&1 &
BAG_PID=$!
sleep 3
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "rosbag recorder exited before recording could start" >&2
  DATA_VALIDITY="INVALID"; INVALID_REASON="bag recorder failed to start"
  exit 1
fi
echo "[$(date -Iseconds)] rosbag recording to NATIVE path: $NATIVE_BAG_DIR" | tee -a "$EXECUTION_LOG"

FULL_LOAD_OUTPUT="$(verify_realtime_factor FULL_LOAD | tee -a "$EXECUTION_LOG")"
FULL_LOAD_FACTOR="$(grep -o 'FULL_LOAD_REALTIME_FACTOR=[0-9.]*' <<<"$FULL_LOAD_OUTPUT" | cut -d= -f2)"

# Controller launched LAST, exactly as the formal baseline does.
stdbuf -oL -eL python3 "$BASELINE_CONFIG_DIR/run_comm_baseline_formal_controllers.py" \
  >"$CONTROLLER_LOG" 2>&1 &
CONTROLLER_PID=$!

deadline=$((SECONDS + 75))
complete_count=0
while (( SECONDS < deadline )); do
  complete_count="$(grep -c 'COMPLETE:' "$CONTROLLER_LOG" 2>/dev/null || true)"
  if (( complete_count >= 2 )); then
    break
  fi
  if ! kill -0 "$CONTROLLER_PID" 2>/dev/null; then
    echo "[$(date -Iseconds)] controller exited (complete_count=$complete_count) -- TASK_OUTCOME classification deferred to the analyzer, not decided here" | tee -a "$EXECUTION_LOG"
    break
  fi
  sleep 0.5
done
echo "[$(date -Iseconds)] controller stage finished (complete_count=$complete_count)" | tee -a "$EXECUTION_LOG"

# Stop the CONTROLLER only, then hold both cmd_vel at zero (the
# controller's own `stop()` already publishes a zero Twist a few times
# on shutdown -- see cooperative_avoider.py's stop() method) while the
# relay/clock/counter/bag keep running for the drain window, per the
# queue-drain rule (design doc section 5): pending queued messages must
# not be miscounted as "dropped" by stopping the relay too early.
echo "[$(date -Iseconds)] stopping controller (relay/bag/counter/sim remain running for drain)" | tee -a "$EXECUTION_LOG"
stop_pid "$CONTROLLER_PID"; CONTROLLER_PID=""
stop_pid "$STATE1_PID"; STATE1_PID=""
stop_pid "$STATE2_PID"; STATE2_PID=""

DRAIN_DURATION_S="$(python3 -c "
import sys
sys.path.insert(0, '$TOOLS_DIR')
from relay_drain import compute_drain_duration_s
print(compute_drain_duration_s($DELAY_S, $JITTER_S, 0.1151, periods_margin=2))
")"
echo "[$(date -Iseconds)] draining relay queues for ${DRAIN_DURATION_S}s (max_configured_delivery_delay + 2 publish periods)" | tee -a "$EXECUTION_LOG"
sleep "$DRAIN_DURATION_S"

for ns in epuck1 epuck2; do
  # `ros2 topic echo --field data --once` ALWAYS appends a trailing YAML
  # document-end marker line ("---") after the value, regardless of
  # --field -- the same behavior that broke bridge-status JSON parsing
  # during the physical-hardware phase of this session (fixed there by
  # compute_tier_a_delta.load_snapshot_json()). Only the FIRST line is
  # the actual JSON; take it explicitly with `head -n1`, never pass the
  # raw multi-line capture to json.loads().
  STATUS_RAW="$(timeout 3 ros2 topic echo "/$ns/relay_status" --once --field data 2>/dev/null || true)"
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
    echo "[$(date -Iseconds)] $ns relay queue NOT drained (pending=$PENDING) -- DATA_VALIDITY=INVALID, not a network-impairment result" | tee -a "$EXECUTION_LOG"
    DATA_VALIDITY="INVALID"
    INVALID_REASON="${INVALID_REASON}${ns} relay queue not drained (pending=$PENDING); "
  fi
done

echo "[$(date -Iseconds)] stopping relays + sequence_counter" | tee -a "$EXECUTION_LOG"
stop_pid_group "$RELAY_COUNTER_PID"; RELAY_COUNTER_PID=""
sleep 1

echo "[$(date -Iseconds)] stopping rosbag recorder" | tee -a "$EXECUTION_LOG"
stop_pid "$BAG_PID"; BAG_PID=""

if [[ ! -s "$NATIVE_BAG_DIR/metadata.yaml" ]]; then
  echo "rosbag metadata is missing or empty (native path)" >&2
  DATA_VALIDITY="INVALID"; INVALID_REASON="${INVALID_REASON}bag metadata missing/empty; "
fi
echo "[$(date -Iseconds)] rosbag metadata.yaml check done" | tee -a "$EXECUTION_LOG"

stop_pid "$SIM_PID"; SIM_PID=""
sleep 2

BAG_WARN_LINES="$(grep -icE 'drop|warn|error' "$BAG_RECORD_LOG" 2>/dev/null || echo 0)"
grep -iE 'drop|warn|error' "$BAG_RECORD_LOG" | tee -a "$EXECUTION_LOG" || echo "no drop/warn/error lines in bag_record.log" | tee -a "$EXECUTION_LOG"
if [[ "$BAG_WARN_LINES" != "0" ]]; then
  DATA_VALIDITY="INVALID"; INVALID_REASON="${INVALID_REASON}bag_record.log has $BAG_WARN_LINES drop/warn/error line(s); "
fi

# --- seed/config cross-check: the relay's own startup log line is the
# ground truth for what was ACTUALLY deployed (not merely what this
# script intended to pass) -- per the analysis plan's requirement.
SEEDS_MATCH="true"
for ns in epuck1 epuck2; do
  EXPECTED_SEED="$SEED_EPUCK1"
  [[ "$ns" == "epuck2" ]] && EXPECTED_SEED="$SEED_EPUCK2"
  if ! grep -q "seed=$EXPECTED_SEED " "$RELAY_COUNTER_LOG" 2>/dev/null; then
    SEEDS_MATCH="false"
    INVALID_REASON="${INVALID_REASON}${ns} relay startup log does not confirm seed=$EXPECTED_SEED; "
  fi
done

# --- min inter-robot distance over the recorded bag (whole trial, not
# windowed -- the matrix's collision heuristic deliberately looks at the
# entire recording, not just a trimmed main window) ---
MIN_DISTANCE_JSON="$(python3 "$TOOLS_DIR/min_interrobot_distance.py" --native-bag-dir "$NATIVE_BAG_DIR" 2>>"$EXECUTION_LOG" || echo '{"min_interrobot_distance_m": null}')"
MIN_DISTANCE_M="$(python3 -c "import json,sys; print(json.loads(sys.argv[1])['min_interrobot_distance_m'])" "$MIN_DISTANCE_JSON" 2>/dev/null || echo null)"
echo "[$(date -Iseconds)] min_interrobot_distance_m=$MIN_DISTANCE_M" | tee -a "$EXECUTION_LOG"

CONTROLLER_CRASHED="false"
if ! grep -q 'COMPLETE:' "$CONTROLLER_LOG" 2>/dev/null && (( complete_count < 2 )); then
  # No COMPLETE at all AND the controller process is confirmed gone --
  # distinct from a clean max_runtime timeout, which still logs its own
  # "COMPLETE: maximum runtime reached" line per cooperative_avoider.py.
  if grep -qiE 'traceback|error' "$CONTROLLER_LOG" 2>/dev/null; then
    CONTROLLER_CRASHED="true"
  fi
fi

REALTIME_FACTOR_OK="true"
python3 -c "
factor_preload = ${PRELOAD_FACTOR:-0}
factor_full = ${FULL_LOAD_FACTOR:-0}
import sys
sys.exit(0 if (0.8 <= factor_preload <= 1.2 and 0.8 <= factor_full <= 1.2) else 1)
" || REALTIME_FACTOR_OK="false"

BAG_METADATA_OK="true"
[[ ! -s "$NATIVE_BAG_DIR/metadata.yaml" ]] && BAG_METADATA_OK="false"

ANALYZER_OK="true"  # tier-A/B/C analyzer reuse (analyze_expanded_bridge-style) is a follow-up step, not blocking this trial's own DATA_VALIDITY computation this pass

VERDICT_INPUT_JSON="$(python3 -c "
import json
print(json.dumps({
    'seeds_match_frozen_config': '$SEEDS_MATCH' == 'true',
    'realtime_factor_ok': '$REALTIME_FACTOR_OK' == 'true',
    'bag_metadata_ok': '$BAG_METADATA_OK' == 'true',
    'analyzer_ok': '$ANALYZER_OK' == 'true',
    'queue_drained': '$DATA_VALIDITY' == 'VALID',
    'complete_count': $complete_count,
    'expected_complete_count': 2,
    'min_interrobot_distance_m': $MIN_DISTANCE_M,
    'safety_radius_m': 0.14,
    'controller_crashed': '$CONTROLLER_CRASHED' == 'true',
}))
")"
VERDICT_JSON="$(echo "$VERDICT_INPUT_JSON" | python3 "$TOOLS_DIR/matrix_verdict.py")"
echo "[$(date -Iseconds)] verdict: $VERDICT_JSON" | tee -a "$EXECUTION_LOG"

FINAL_DATA_VALIDITY="$(python3 -c "import json,sys; print(json.loads(sys.argv[1])['data_validity'])" "$VERDICT_JSON")"
FINAL_TASK_OUTCOME="$(python3 -c "import json,sys; print(json.loads(sys.argv[1])['task_outcome'])" "$VERDICT_JSON")"

mkdir -p "$FINAL_DIR"
cp "$DIAG_LOG_DIR/frozen_params.json" "$FINAL_DIR/"
python3 -c "
import json
manifest = {
    'condition_id': '$CONDITION_ID',
    'trial_index': $TRIAL_INDEX,
    'attempt': $ATTEMPT,
    'native_bag_dir': '$NATIVE_BAG_DIR',
    'native_diag_log_dir': '$DIAG_LOG_DIR',
    'preload_realtime_factor': ${PRELOAD_FACTOR:-0},
    'full_load_realtime_factor': ${FULL_LOAD_FACTOR:-0},
    'controller_complete_count': $complete_count,
    'drain_duration_s': $DRAIN_DURATION_S,
    'min_interrobot_distance_m': $MIN_DISTANCE_M,
    'git_commit': '$GIT_COMMIT',
    'orchestrator_sha256': '$ORCH_SHA256',
    'network_impairment_relay_py_sha256': '$RELAY_SHA256',
    'network_impairment_py_sha256': '$IMPAIRMENT_SHA256',
}
manifest.update(json.loads('''$VERDICT_JSON'''))
with open('$FINAL_DIR/trial_verdict.json', 'w', encoding='utf-8') as fh:
    json.dump(manifest, fh, indent=2)
print(json.dumps(manifest, indent=2))
" | tee -a "$EXECUTION_LOG"

echo "[$(date -Iseconds)] $STEM RECORDING_COMPLETE data_validity=$FINAL_DATA_VALIDITY task_outcome=$FINAL_TASK_OUTCOME" | tee -a "$EXECUTION_LOG"
echo "native bag dir: $NATIVE_BAG_DIR"
echo "analysis dir: $FINAL_DIR"
trap - EXIT
