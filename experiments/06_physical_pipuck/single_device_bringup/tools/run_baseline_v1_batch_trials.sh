#!/usr/bin/env bash
set -eo pipefail

# Drives trial02_attempt02 .. trial05_attempt01 of
# physical_single_device_zero_impairment_baseline_v1, one at a time, using
# the fixed run_baseline_v1_trial_v2.sh orchestrator. Stops the whole batch
# immediately on any single trial's failure (window short, tier-A delta
# nonzero missing/out_of_order, tier-B/C/field/cmd_vel/NaN/reconnect/CRC
# failure via the existing analyzer, or a recorder traceback) -- does not
# retry, does not modify thresholds, does not continue past a failure.
#
# Usage: run_baseline_v1_batch_trials.sh
#   (trial list and starting state_publisher PIDs are hardcoded below,
#    matching this batch's known state at invocation time)

TOOLS_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/06_physical_pipuck/single_device_bringup/tools"
NATIVE_ROOT="/home/eamon/epuck_comm_bags"
V2_SCRIPT="$TOOLS_DIR/run_baseline_v1_trial_v2.sh"
V2_SHA256_EXPECTED="dddff1f51a51a871cc86667ec7a213dc38bc6aa49ae22b528b4a86a2193796db"
V2_COMMIT="a7f2a7e"

source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash

# current state_publisher, from trial01_attempt02 (must be replaced fresh
# before trial02_attempt02 starts)
CURRENT_SP_WRAPPER_PID=5765
CURRENT_SP_ACTUAL_PID=5766

TRIALS=(trial02_attempt02 trial03_attempt01 trial04_attempt01 trial05_attempt01)

for TRIAL in "${TRIALS[@]}"; do
  echo "############################################################"
  echo "[$(date -Iseconds)] === BEGIN $TRIAL ==="
  echo "############################################################"

  echo "[$(date -Iseconds)] boundary gap: 12s stable pause before this trial"
  sleep 12

  echo "[$(date -Iseconds)] pre-trial health check"
  V2_SHA256_NOW="$(sha256sum "$V2_SCRIPT" | awk '{print $1}')"
  if [[ "$V2_SHA256_NOW" != "$V2_SHA256_EXPECTED" ]]; then
    echo "ABORT_BATCH: run_baseline_v1_trial_v2.sh SHA-256 changed ($V2_SHA256_NOW != $V2_SHA256_EXPECTED)" >&2
    exit 1
  fi

  SCAN_PUB_COUNT="$(ros2 topic info /scan 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*')"
  if [[ "$SCAN_PUB_COUNT" != "1" ]]; then
    echo "ABORT_BATCH: /scan publisher count = $SCAN_PUB_COUNT (expected 1, Pi driver unhealthy)" >&2
    exit 1
  fi

  BRIDGE_STATUS="$(timeout 3 ros2 topic echo /epuck_bridge/status --once --field data 2>/dev/null)"
  if [[ "$BRIDGE_STATUS" != *'"connected": true'* && "$BRIDGE_STATUS" != *'"connected":true'* ]]; then
    echo "ABORT_BATCH: expanded bridge not connected at pre-trial check: $BRIDGE_STATUS" >&2
    exit 1
  fi

  if ! pgrep -f wsl_epuck_tcp_bridge_sensors >/dev/null; then
    echo "ABORT_BATCH: WSL expanded bridge process not found" >&2
    exit 1
  fi

  CMD_VEL_PUB_COUNT="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo unknown)"
  if [[ "$CMD_VEL_PUB_COUNT" != "0" ]]; then
    echo "ABORT_BATCH: /cmd_vel has $CMD_VEL_PUB_COUNT publisher(s) (expected 0)" >&2
    exit 1
  fi

  NATIVE_BAG_DIR="${NATIVE_ROOT}/physical_single_device_zero_impairment_baseline_v1_${TRIAL}"
  if [[ -e "$NATIVE_BAG_DIR" ]]; then
    echo "ABORT_BATCH: target directory already exists: $NATIVE_BAG_DIR" >&2
    exit 1
  fi

  echo "[$(date -Iseconds)] pre-trial health check PASSED"

  echo "[$(date -Iseconds)] stopping previous state_publisher ($CURRENT_SP_WRAPPER_PID / $CURRENT_SP_ACTUAL_PID)"
  kill -INT "$CURRENT_SP_WRAPPER_PID" "$CURRENT_SP_ACTUAL_PID" 2>/dev/null || true
  sleep 2

  nohup ros2 run epuck2_comm state_publisher --ros-args -p robot_id:=1 -p source:=hardware -p use_sim_time:=false -p mode:=periodic -r state:=/epuck1/state \
    > "/home/eamon/state_publisher_${TRIAL}.log" 2>&1 &
  disown
  sleep 2
  SP_PIDS="$(pgrep -f 'state_publisher --ros-args' | tr '\n' ' ')"
  read -r NEW_SP_WRAPPER_PID NEW_SP_ACTUAL_PID <<< "$SP_PIDS"
  if [[ -z "$NEW_SP_ACTUAL_PID" ]]; then
    echo "ABORT_BATCH: fresh state_publisher failed to start for $TRIAL" >&2
    exit 1
  fi
  echo "[$(date -Iseconds)] FRESH state_publisher for $TRIAL: wrapper=$NEW_SP_WRAPPER_PID actual=$NEW_SP_ACTUAL_PID"
  CURRENT_SP_WRAPPER_PID="$NEW_SP_WRAPPER_PID"
  CURRENT_SP_ACTUAL_PID="$NEW_SP_ACTUAL_PID"

  TRIAL_LOG="${NATIVE_ROOT}/${TRIAL}_orchestrator.log"
  echo "[$(date -Iseconds)] launching run_baseline_v1_trial_v2.sh $TRIAL"
  set +e
  bash "$V2_SCRIPT" "$TRIAL" > "$TRIAL_LOG" 2>&1
  V2_EXIT=$?
  set -e
  echo "[$(date -Iseconds)] run_baseline_v1_trial_v2.sh exit code: $V2_EXIT"
  if [[ $V2_EXIT -ne 0 ]]; then
    echo "ABORT_BATCH: $TRIAL orchestrator exited nonzero ($V2_EXIT); see $TRIAL_LOG" >&2
    tail -40 "$TRIAL_LOG" >&2
    exit 1
  fi

  NATIVE_DIAG_DIR="${NATIVE_ROOT}/physical_single_device_zero_impairment_baseline_v1_${TRIAL}_diag"
  mkdir -p "$NATIVE_DIAG_DIR"
  cp "$TRIAL_LOG" "$NATIVE_DIAG_DIR/orchestrator.log"

  ANALYSIS_DIR="${NATIVE_ROOT}/physical_single_device_zero_impairment_baseline_v1_${TRIAL}_analysis"
  EXTRA_JSON="$NATIVE_DIAG_DIR/runtime_manifest_extra.json"
  cat > "$EXTRA_JSON" <<EOF
{
  "trial": "$TRIAL",
  "orchestrator_script": "$V2_SCRIPT",
  "orchestrator_sha256": "$V2_SHA256_EXPECTED",
  "orchestrator_git_commit": "$V2_COMMIT",
  "state_publisher_wrapper_pid": "$NEW_SP_WRAPPER_PID",
  "state_publisher_actual_pid": "$NEW_SP_ACTUAL_PID",
  "state_publisher_status": "FRESH",
  "pi_driver_status": "REUSED (unchanged since batch start, PIDs 813/814 on the Pi, verified via /scan publisher-count proxy this trial)",
  "pi_expanded_server_status": "REUSED (unchanged since batch start, Pi PID 1168, verified via /epuck_bridge/status connected=true this trial)",
  "wsl_expanded_bridge_status": "REUSED (unchanged since batch start, WSL PID 2535, verified alive this trial)",
  "pi_batch_sampler_status": "REUSED across the whole batch, Pi PID 1290, not touched by this script",
  "recorded_at_unix_s": $(date +%s.%N)
}
EOF

  echo "[$(date -Iseconds)] running per-trial analysis (window + tier A delta + tier B/C analyzer)"
  set +e
  python3 "$TOOLS_DIR/run_one_trial_analysis.py" \
    --trial "$TRIAL" \
    --bag-dir "$NATIVE_BAG_DIR" \
    --diag-dir "$NATIVE_DIAG_DIR" \
    --analysis-dir "$ANALYSIS_DIR" \
    --analyzer-script "$TOOLS_DIR/analyze_expanded_bridge_epuckstate_pilot.py" \
    --runtime-manifest-extra "$EXTRA_JSON" \
    --trial-log "$TRIAL_LOG"
  ANALYSIS_EXIT=$?
  set -e
  echo "[$(date -Iseconds)] per-trial analysis exit code: $ANALYSIS_EXIT"
  if [[ $ANALYSIS_EXIT -ne 0 ]]; then
    echo "ABORT_BATCH: $TRIAL FAILED per-trial verdict; see $ANALYSIS_DIR/trial_verdict.json" >&2
    cat "$ANALYSIS_DIR/trial_verdict.json" >&2
    exit 1
  fi

  echo "[$(date -Iseconds)] === $TRIAL: THREE_SOURCE_PROVISIONAL_PASS ==="
done

echo "[$(date -Iseconds)] ALL TRIALS (trial02_attempt02..trial05_attempt01) COMPLETED: 4/4 THREE_SOURCE_PROVISIONAL_PASS"
echo "FINAL_STATE_PUBLISHER_WRAPPER_PID=$CURRENT_SP_WRAPPER_PID"
echo "FINAL_STATE_PUBLISHER_ACTUAL_PID=$CURRENT_SP_ACTUAL_PID"
