#!/usr/bin/env bash
# Bounded, exact-PID, self-rolling-back substitution of the diagnostic-
# only instrumented bridge for the currently-running, unmodified WSL
# bridge, to capture ~130s of GC/timer-gap evidence for the ~33s
# validity_flags periodicity investigation.
#
# Modes:
#   --check-only (DEFAULT, no argument)  Verifies preconditions only.
#                                          Starts/stops nothing.
#   --dry-run                             Prints the planned step
#                                          sequence only. Starts/stops
#                                          nothing.
#   --execute ORIGINAL_BRIDGE_PID          Performs the real, bounded
#                                          substitution. Requires the
#                                          exact PID of the currently
#                                          running original bridge.
#
# Never starts hil_cmd_vel_guard, a controller, a virtual peer,
# goal_navigator, Webots, rosbag, or any /cmd_vel publisher -- this
# script is entirely about the sensor/bridge layer, upstream of all of
# those, and touches none of them.
#
# The cleanup trap is unconditional: on ANY exit path (success, error,
# Ctrl+C), if the instrumented bridge was started it is stopped by its
# exact PID, and if the original bridge was stopped but rollback has
# not yet completed, the original bridge is restarted -- the script
# never leaves the robot with no bridge running, and never leaves two
# bridges running at once.
set -o pipefail

BRIDGE_DIR="/home/eamon/epuck_ws/epuck_comm_project/real_robot_avoidance_v1"
CAPTURE_DURATION_S=130
STATE_TOPIC="/epuck1/state"
BRIDGE_STATUS_TOPIC="/epuck_bridge/status"

MODE="${1:---check-only}"
ORIGINAL_BRIDGE_PID="${2:-}"

# ROS2's own setup.bash is not `set -u`-safe (it references unset
# variables internally) -- source it BEFORE enabling nounset, not
# wrapped in set +u/-u around it, so a real unbound-variable bug
# anywhere else in this script is still caught.
source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash
set -u

# --------------------------------------------------------------------
# Shared, mode-independent checks
# --------------------------------------------------------------------
check_cmd_vel_zero() {
    local count
    count="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo unknown)"
    echo "cmd_vel_publisher_count=$count"
    [[ "$count" == "0" ]]
}

check_no_motion_processes() {
    local found
    found="$(pgrep -af 'hil_cmd_vel_guard|cooperative_avoider|goal_navigator|hil_virtual_peer|webots' 2>/dev/null | grep -v 'bash -lc' || true)"
    if [[ -n "$found" ]]; then
        echo "MOTION_RELATED_PROCESS_FOUND:"
        echo "$found"
        return 1
    fi
    echo "no_motion_related_processes=true"
    return 0
}

check_bridge_process_count() {
    # Prints ONLY the integer count to stdout (the caller captures this
    # via command substitution) -- any human-readable context goes to
    # stderr instead, so the two are never accidentally concatenated.
    local count
    count="$(pgrep -af 'wsl_epuck_tcp_bridge_sensors' 2>/dev/null | grep -v 'bash -lc' | wc -l)"
    echo "bridge_process_count=$count" >&2
    echo "$count"
}

check_bridge_health() {
    # Calls `ros2 topic list` exactly ONCE and checks every expected
    # topic against that single snapshot -- calling it once per topic
    # in a loop was observed to be flaky (a different, genuinely-
    # present topic "missing" on each run), consistent with normal
    # ros2 CLI/daemon discovery-cache timing, not a real absence.
    local status_json flags_raw flags_value topic_list
    status_json="$(timeout 3 ros2 topic echo "$BRIDGE_STATUS_TOPIC" --once --field data 2>/dev/null || true)"
    echo "bridge_status_raw=$status_json"
    if [[ "$status_json" != *'"connected": true'* && "$status_json" != *'"connected":true'* ]]; then
        echo "BRIDGE_HEALTH_CHECK_FAIL: connected != true"
        return 1
    fi
    flags_raw="$(timeout 4 ros2 topic echo "$STATE_TOPIC" --field validity_flags --once 2>/dev/null || true)"
    flags_value="$(grep -m1 -E '^[[:space:]]*[0-9]+[[:space:]]*$' <<<"$flags_raw" | tr -d '[:space:]' || true)"
    echo "validity_flags=$flags_value"
    if [[ "$flags_value" != "7" ]]; then
        echo "BRIDGE_HEALTH_CHECK_FAIL: validity_flags != 7 (got '${flags_value:-<none>}')"
        return 1
    fi
    topic_list="$(ros2 topic list 2>/dev/null || true)"
    for t in /odom /scan /tof /ps0 /ps1 /ps2 /ps3 /ps4 /ps5 /ps6 /ps7; do
        grep -qx "$t" <<<"$topic_list" || { echo "BRIDGE_HEALTH_CHECK_FAIL: $t missing"; return 1; }
    done
    echo "sensor_topics_present=true"
    return 0
}

start_original_bridge() {
    cd "$BRIDGE_DIR"
    setsid python3 wsl_epuck_tcp_bridge_sensors.py \
        > "$OUT_DIR/original_bridge_restart.log" 2>&1 &
    ORIGINAL_RESTARTED_PID=$!
    disown
    echo "original_bridge_restarted_pid=$ORIGINAL_RESTARTED_PID"
}

# --------------------------------------------------------------------
# --check-only: preconditions only, nothing started or stopped.
# --------------------------------------------------------------------
if [[ "$MODE" == "--check-only" ]]; then
    echo "=== --check-only: verifying preconditions, starting/stopping nothing ==="
    OK=true
    check_cmd_vel_zero || OK=false
    check_no_motion_processes || OK=false
    BRIDGE_COUNT="$(check_bridge_process_count)"
    if [[ "$BRIDGE_COUNT" != "1" ]]; then
        echo "PRECONDITION_FAIL: expected exactly 1 running bridge process, found $BRIDGE_COUNT"
        OK=false
    fi
    check_bridge_health || OK=false
    if [[ "$OK" == "true" ]]; then
        echo "BRIDGE_SUBSTITUTION_CHECK_ONLY_PASS"
        exit 0
    else
        echo "BRIDGE_SUBSTITUTION_CHECK_ONLY_FAIL"
        exit 1
    fi
fi

# --------------------------------------------------------------------
# --dry-run: print the planned sequence only.
# --------------------------------------------------------------------
if [[ "$MODE" == "--dry-run" ]]; then
    echo "=== --dry-run: printing planned steps only, nothing executed ==="
    cat <<'EOF'
Planned steps for --execute ORIGINAL_BRIDGE_PID (NONE executed in --dry-run):
  1. Verify /cmd_vel Publisher count == 0.
  2. Verify no hil_cmd_vel_guard/cooperative_avoider/goal_navigator/hil_virtual_peer/webots process exists.
  3. Verify exactly one bridge process is currently running.
  4. Verify bridge health: connected=true, validity_flags=7, all sensor topics present.
  5. Stop the original bridge by the exact given PID (kill -INT, never pkill); confirm it exited.
  6. Start exactly one instrumented bridge with diagnostic_mode:=true and a new timestamped diagnostic_output_csv.
  7. Verify exactly one bridge process is running again (the instrumented one).
  8. Re-verify health: connected=true, validity_flags=7, sensor topics present.
  9. Capture for exactly 130 seconds (~4 x 33s cycles).
 10. Stop the instrumented bridge by its exact PID; confirm its diagnostic CSV is closed and non-empty.
 11. Immediately restart the original, unmodified bridge (identical parameters, no diagnostic flags).
 12. Re-verify health: connected=true, validity_flags=7, /cmd_vel Publisher count == 0.
 13. Report the new original-bridge PID (a fresh OS-assigned PID, since the old one already exited).
A cleanup trap runs on any exit path (success, error, or interrupt): if
the instrumented bridge is still running it is stopped by its exact
PID, and if the original bridge was stopped but rollback has not
completed, the original bridge is restarted -- the robot is never left
without exactly one bridge running.
EOF
    exit 0
fi

# --------------------------------------------------------------------
# --execute: the real, bounded substitution.
# --------------------------------------------------------------------
if [[ "$MODE" == "--execute" ]]; then
    if [[ -z "$ORIGINAL_BRIDGE_PID" ]]; then
        echo "ERROR: --execute requires the exact original bridge PID as the second argument." >&2
        exit 2
    fi

    OUT_DIR="/home/eamon/epuck_comm_bags/bridge_instrumentation_substitution_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$OUT_DIR"
    DIAG_CSV="$OUT_DIR/bridge_diagnostic_events.csv"

    INSTRUMENTED_PID=""
    INSTRUMENTED_STARTED="false"
    ORIGINAL_STOPPED="false"
    ROLLBACK_DONE="false"

    cleanup() {
        local exit_code=$?
        if [[ "$INSTRUMENTED_STARTED" == "true" ]] && [[ -n "$INSTRUMENTED_PID" ]] && kill -0 "$INSTRUMENTED_PID" 2>/dev/null; then
            echo "[cleanup] stopping instrumented bridge PID $INSTRUMENTED_PID"
            kill -INT "$INSTRUMENTED_PID" 2>/dev/null || true
            wait "$INSTRUMENTED_PID" 2>/dev/null || true
        fi
        if [[ "$ORIGINAL_STOPPED" == "true" ]] && [[ "$ROLLBACK_DONE" == "false" ]]; then
            echo "[cleanup] rollback: restoring original bridge (was stopped, rollback not yet recorded as done)"
            start_original_bridge
        fi
        exit "$exit_code"
    }
    trap cleanup EXIT

    echo "[$(date -Iseconds)] step 1-4: preconditions"
    check_cmd_vel_zero || { echo "ABORT: cmd_vel publisher count != 0"; exit 1; }
    check_no_motion_processes || { echo "ABORT: motion-related process found"; exit 1; }
    BRIDGE_COUNT="$(check_bridge_process_count)"
    [[ "$BRIDGE_COUNT" == "1" ]] || { echo "ABORT: expected exactly 1 bridge process, found $BRIDGE_COUNT"; exit 1; }
    kill -0 "$ORIGINAL_BRIDGE_PID" 2>/dev/null || { echo "ABORT: given original bridge PID $ORIGINAL_BRIDGE_PID is not running"; exit 1; }
    check_bridge_health || { echo "ABORT: bridge health check failed before substitution"; exit 1; }

    echo "[$(date -Iseconds)] step 5: stopping original bridge PID $ORIGINAL_BRIDGE_PID"
    kill -INT "$ORIGINAL_BRIDGE_PID"
    for _ in $(seq 1 20); do
        kill -0 "$ORIGINAL_BRIDGE_PID" 2>/dev/null || break
        sleep 0.5
    done
    if kill -0 "$ORIGINAL_BRIDGE_PID" 2>/dev/null; then
        echo "ABORT: original bridge PID $ORIGINAL_BRIDGE_PID did not exit"
        exit 1
    fi
    ORIGINAL_STOPPED="true"
    echo "original_bridge_stopped=true"

    echo "[$(date -Iseconds)] step 6: starting instrumented bridge"
    cd "$BRIDGE_DIR"
    setsid python3 wsl_epuck_tcp_bridge_sensors_instrumented.py --ros-args \
        -p diagnostic_mode:=true -p diagnostic_output_csv:="$DIAG_CSV" \
        > "$OUT_DIR/instrumented_bridge.log" 2>&1 &
    INSTRUMENTED_PID=$!
    disown
    INSTRUMENTED_STARTED="true"
    echo "instrumented_bridge_pid=$INSTRUMENTED_PID"
    sleep 2

    echo "[$(date -Iseconds)] step 7: verifying exactly one bridge process"
    BRIDGE_COUNT_AFTER_START="$(check_bridge_process_count)"
    [[ "$BRIDGE_COUNT_AFTER_START" == "1" ]] || { echo "ABORT: expected exactly 1 bridge process after start, found $BRIDGE_COUNT_AFTER_START"; exit 1; }

    echo "[$(date -Iseconds)] step 8: verifying health with instrumented bridge running"
    check_bridge_health || { echo "ABORT: bridge health check failed after starting instrumented bridge"; exit 1; }

    echo "[$(date -Iseconds)] step 9: capturing for ${CAPTURE_DURATION_S}s"
    sleep "$CAPTURE_DURATION_S"

    echo "[$(date -Iseconds)] step 10: stopping instrumented bridge PID $INSTRUMENTED_PID"
    kill -INT "$INSTRUMENTED_PID"
    for _ in $(seq 1 20); do
        kill -0 "$INSTRUMENTED_PID" 2>/dev/null || break
        sleep 0.5
    done
    if kill -0 "$INSTRUMENTED_PID" 2>/dev/null; then
        echo "ABORT: instrumented bridge PID $INSTRUMENTED_PID did not exit"
        exit 1
    fi
    INSTRUMENTED_STARTED="false"
    if [[ ! -s "$DIAG_CSV" ]]; then
        echo "ABORT: diagnostic CSV missing or empty: $DIAG_CSV"
        exit 1
    fi
    echo "diagnostic_csv_confirmed_nonempty=$DIAG_CSV"

    echo "[$(date -Iseconds)] step 11: restarting original unmodified bridge"
    start_original_bridge
    ROLLBACK_DONE="true"
    sleep 2

    echo "[$(date -Iseconds)] step 12: verifying rollback health"
    check_bridge_health || { echo "ROLLBACK_HEALTH_CHECK_FAIL -- reported, not auto-retried"; }
    check_cmd_vel_zero || echo "ROLLBACK_CMD_VEL_CHECK_FAIL"

    echo "=== OUT_DIR=$OUT_DIR ==="
    ls -la "$OUT_DIR"
    echo "BRIDGE_INSTRUMENTATION_SUBSTITUTION_COMPLETE"
    exit 0
fi

echo "Usage: $0 [--check-only | --dry-run | --execute ORIGINAL_BRIDGE_PID]" >&2
exit 2
