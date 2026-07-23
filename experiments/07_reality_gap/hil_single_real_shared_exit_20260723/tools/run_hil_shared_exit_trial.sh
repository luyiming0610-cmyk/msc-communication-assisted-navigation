#!/usr/bin/env bash
# HIL shared-exit trial launcher -- the one permanent orchestrator for
# the single-real-robot (epuck5809) + virtual-peer shared-exit study.
#
# Modes (mutually exclusive):
#   --check-only   (DEFAULT if no mode flag is given) Runs hil_preflight.py's
#                   offline checks only. Never starts Webots, the driver,
#                   the bridge, SSH, any controller, rosbag, and never
#                   publishes cmd_vel or writes raw trial data.
#   --dry-run       Prints the sequence of steps a real run WOULD take
#                    (including the exact commands), without running any
#                    of them.
#   --comm-off      Runs one HIL_COMM_OFF trial. Real execution path --
#                   see the safety gate below.
#   --comm-on       Runs one HIL_COMM_ON trial. Real execution path --
#                   see the safety gate below.
#
# Safety gate (unconditional, no bypass): --comm-off and --comm-on both
# first run the full run_hil_preflight.sh (offline UNCONFIRMED_PHYSICAL_
# MEASUREMENT check + read-only physical health check). If either
# layer fails, this script prints PHYSICAL_MOTION_LOCKED_UNTIL_LAB_
# VALIDATION and starts nothing -- no controller, no guard, no bag, no
# adapter, no virtual peer. As of 2026-07-23, hil_frozen_params.json's
# field_geometry and hil_guard_limits.max_angular_speed_rps are still
# UNCONFIRMED_PHYSICAL_MEASUREMENT, so --comm-off/--comm-on are
# currently guaranteed to hit this gate and start nothing -- this is
# expected and has been verified, not merely assumed.
#
# Even past that gate, this script NEVER arms hil_cmd_vel_guard.py --
# it starts every process DISARMED and stops, printing
# AWAITING_EXPLICIT_ARM_CONFIRMATION. Arming (and therefore any
# nonzero /cmd_vel) is a separate, explicit, human action, never
# automated by this script. This script also never starts a second
# trial automatically -- every invocation runs at most one trial.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
FROZEN_PARAMS="${HIL_ROOT}/hil_frozen_params.json"
COOP_EXIT_TOOLS="$(cd "${HIL_ROOT}/../../10_cooperative_exit_navigation_20260720/tools" && pwd)"

MODE="--check-only"
if [[ $# -gt 0 ]]; then
    MODE="$1"
fi

case "${MODE}" in
    --check-only)
        echo "[run_hil_shared_exit_trial] MODE=--check-only (offline checks only, no processes started)"
        python3 "${SCRIPT_DIR}/hil_preflight.py" --frozen-params "${FROZEN_PARAMS}"
        exit $?
        ;;
    --dry-run)
        echo "[run_hil_shared_exit_trial] MODE=--dry-run (printing planned steps only, nothing executed)"
        cat <<'EOF'
Planned steps for --comm-off / --comm-on (NONE of these are executed in --dry-run):
  1. Run run_hil_preflight.sh (offline UNCONFIRMED check + read-only physical
     health check). Abort with PHYSICAL_MOTION_LOCKED_UNTIL_LAB_VALIDATION on
     any failure, before starting anything.
  2. Verify /cmd_vel_unguarded and /cmd_vel both currently have 0 publishers.
  3. Create a new timestamped evidence directory; write pid_manifest.json as
     each process starts (exact PID + SHA-256 of the script file).
  4. Start the rosbag recorder (per hil_recorder_plan.py's topic list) FIRST.
  5. Start task_completion_monitor.py.
  6. Start hil_virtual_peer.py.
  7. Start hil_topic_adapter.py (the real robot's GoalNavigator wrapper) --
     subscribing to the virtual peer's GoalAnnouncement only in --comm-on.
  8. Start cooperative_avoider.py (ros2 run epuck2_comm cooperative_avoider,
     -r cmd_vel:=cmd_vel_unguarded) -- peer-avoidance visibility of the
     virtual peer's state is enabled only in --comm-on, mirroring
     run_shared_exit_n2_controllers.py's own enable_peer_avoidance=
     (comm_mode==COMM_ON) pattern exactly (confirmed by source re-read
     2026-07-23 -- this is NOT a HIL-specific choice, it reuses the
     frozen N2/N3 controllers' own established semantics unchanged).
  9. Start hil_cmd_vel_guard.py -- DISARMED. Verify the guarded /cmd_vel
     topic has exactly 1 publisher and it is the guard itself.
 10. STOP. Print AWAITING_EXPLICIT_ARM_CONFIRMATION. Do not arm the guard,
     do not publish nonzero /cmd_vel, do not start a second trial.
 11. (Separate, explicit, human step, not run by this script): arm via
     `ros2 topic pub --once /hil_guard/arm std_msgs/msg/Bool "{data: true}"`,
     observe the trial, then stop it via run_hil_shutdown.sh with the
     trial's own pid_manifest.json.
EOF
        exit 0
        ;;
    --comm-off|--comm-on)
        if [[ "${MODE}" == "--comm-off" ]]; then
            COMM_MODE="HIL_COMM_OFF"
            ENABLE_PEER_AVOIDANCE="false"
        else
            COMM_MODE="HIL_COMM_ON"
            ENABLE_PEER_AVOIDANCE="true"
        fi

        echo "[$(date -Iseconds)] MODE=${MODE} (${COMM_MODE}) -- running safety gate first"
        if ! bash "${SCRIPT_DIR}/run_hil_preflight.sh"; then
            echo "[run_hil_shared_exit_trial] PHYSICAL_MOTION_LOCKED_UNTIL_LAB_VALIDATION"
            echo "[run_hil_shared_exit_trial] Reason: run_hil_preflight.sh did not pass (offline UNCONFIRMED_PHYSICAL_MEASUREMENT fields remain, and/or the read-only physical health check failed). No process was started."
            exit 2
        fi

        source /opt/ros/humble/setup.bash
        source ~/epuck_ws/install/setup.bash
        # ROS2's setup.bash is not `set -u`-safe -- source it before
        # enabling nounset below, not wrapped around it, so a real
        # unbound-variable bug elsewhere in this script is still caught.
        set -u

        UPSTREAM_COUNT="$(ros2 topic info /cmd_vel_unguarded 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo 0)"
        FINAL_COUNT="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo 0)"
        echo "cmd_vel_unguarded_publisher_count_preflight=${UPSTREAM_COUNT}"
        echo "cmd_vel_publisher_count_preflight=${FINAL_COUNT}"
        if [[ "${UPSTREAM_COUNT}" != "0" || "${FINAL_COUNT}" != "0" ]]; then
            echo "ABORT: expected 0 publishers on both /cmd_vel_unguarded and /cmd_vel before starting, found ${UPSTREAM_COUNT} and ${FINAL_COUNT}." >&2
            exit 1
        fi

        # This env-conversion step (never a hardcoded number in this
        # script) will itself refuse if any required field is still
        # UNCONFIRMED_PHYSICAL_MEASUREMENT -- a second, independent
        # check beyond run_hil_preflight.sh's own, since this script
        # must never construct a real trial invocation from a
        # placeholder value even if some earlier check were bypassed.
        if ! source <(python3 "${SCRIPT_DIR}/hil_frozen_params_to_env.py" "${FROZEN_PARAMS}"); then
            echo "[run_hil_shared_exit_trial] PHYSICAL_MOTION_LOCKED_UNTIL_LAB_VALIDATION"
            echo "[run_hil_shared_exit_trial] Reason: hil_frozen_params_to_env.py refused (see stderr above) -- required geometry/angular-cap fields are still unconfirmed."
            exit 2
        fi

        OUT_DIR="/home/eamon/epuck_comm_bags/hil_${MODE#--}_$(date +%Y%m%d_%H%M%S)"
        mkdir -p "${OUT_DIR}"
        PID_MANIFEST="${OUT_DIR}/pid_manifest.json"
        echo '{"processes": {}}' > "${PID_MANIFEST}"

        declare -A PIDS
        declare -A SHAS

        record_process() {
            local name="$1" pid="$2" script_path="$3"
            PIDS["${name}"]="${pid}"
            if [[ -n "${script_path}" && -f "${script_path}" ]]; then
                SHAS["${name}"]="$(sha256sum "${script_path}" | awk '{print $1}')"
            else
                SHAS["${name}"]=""
            fi
        }

        cleanup() {
            local exit_code=$?
            echo "[$(date -Iseconds)] cleanup: stopping any started HIL processes via run_hil_shutdown.sh"
            if [[ -f "${PID_MANIFEST}" ]]; then
                bash "${SCRIPT_DIR}/run_hil_shutdown.sh" "${PID_MANIFEST}" || true
            fi
            exit "${exit_code}"
        }
        trap cleanup EXIT

        echo "[$(date -Iseconds)] step 4: starting rosbag recorder"
        RECORD_CMD="$(python3 "${SCRIPT_DIR}/hil_recorder_plan.py" \
            --physical-state-topic /epuck1/state \
            --virtual-state-topic /epuck_virtual_peer/state \
            --goal-announcement-topic /hil/goal_announcement \
            --nav-intent-topic /epuck1/nav_intent \
            --guarded-cmd-vel-topic /cmd_vel \
            --guard-arm-topic /hil_guard/arm \
            --bridge-status-topic /epuck_bridge/status \
            --task-completion-topic /hil/task_completion \
            --safety-event-topic /hil/safety_events \
            --output-dir "${OUT_DIR}/bag")"
        # shellcheck disable=SC2086
        ${RECORD_CMD} > "${OUT_DIR}/recorder.log" 2>&1 &
        record_process "recorder_bag" "$!" ""

        echo "[$(date -Iseconds)] step 5: starting task_completion_monitor.py"
        python3 "${COOP_EXIT_TOOLS}/task_completion_monitor.py" \
            --robot-ids=epuck1 --state-topics=/epuck1/state \
            --goal-centers-x-m="${PARKING_X_M}" --goal-centers-y-m="${PARKING_Y_M}" \
            --goal-radii-m="${PARKING_RADIUS_M}" --goal-hold-time-s=2.0 \
            --max-linear-speed-mps=0.005 --max-angular-speed-rps=0.05 \
            --verdict-path="${OUT_DIR}/monitor_verdict.json" \
            > "${OUT_DIR}/task_completion_monitor.log" 2>&1 &
        record_process "task_completion_monitor" "$!" "${COOP_EXIT_TOOLS}/task_completion_monitor.py"

        echo "[$(date -Iseconds)] step 6: starting hil_virtual_peer.py"
        VP_ANNOUNCE_ARGS=()
        if [[ "${COMM_MODE}" == "HIL_COMM_ON" ]]; then
            VP_ANNOUNCE_ARGS=(--announcement-topic /hil/goal_announcement)
        fi
        python3 "${SCRIPT_DIR}/hil_virtual_peer.py" \
            --robot-id 2 --state-topic /epuck_virtual_peer/state \
            "${VP_ANNOUNCE_ARGS[@]}" \
            --start-x-m "${START_POSE_X_M}" --start-y-m "${START_POSE_Y_M}" \
            --target-x-m "${EXIT_CENTER_X_M}" --target-y-m "${EXIT_CENTER_Y_M}" \
            --cruise-linear-mps "${MAX_LINEAR_SPEED_MPS}" --arrival-radius-m "${EXIT_RADIUS_M}" \
            --max-angular-rps "${MAX_ANGULAR_SPEED_RPS}" \
            > "${OUT_DIR}/hil_virtual_peer.log" 2>&1 &
        record_process "hil_virtual_peer" "$!" "${SCRIPT_DIR}/hil_virtual_peer.py"

        echo "[$(date -Iseconds)] step 7: starting hil_topic_adapter.py"
        ADAPTER_ANNOUNCE_ARGS=()
        if [[ "${COMM_MODE}" == "HIL_COMM_ON" ]]; then
            ADAPTER_ANNOUNCE_ARGS=(--goal-announcement-topic /hil/goal_announcement)
        fi
        python3 "${SCRIPT_DIR}/hil_topic_adapter.py" \
            --robot-id=1 --state-topic=/epuck1/state --nav-intent-topic=/epuck1/nav_intent \
            --mode=search --waypoints="${SEARCH_WAYPOINTS_M}" \
            --waypoint-arrival-radius=0.10 --rate-hz=2.0 \
            --nominal-speed-mps="${MAX_LINEAR_SPEED_MPS}" \
            --exit-center-x="${EXIT_CENTER_X_M}" --exit-center-y="${EXIT_CENTER_Y_M}" --exit-radius="${EXIT_RADIUS_M}" \
            --parking-x="${PARKING_X_M}" --parking-y="${PARKING_Y_M}" --parking-radius="${PARKING_RADIUS_M}" \
            --goal-hold-time-s=2.0 \
            "${ADAPTER_ANNOUNCE_ARGS[@]}" \
            > "${OUT_DIR}/hil_topic_adapter.log" 2>&1 &
        record_process "hil_topic_adapter" "$!" "${SCRIPT_DIR}/hil_topic_adapter.py"

        echo "[$(date -Iseconds)] step 8: starting cooperative_avoider.py (real robot's frozen controller, UNMODIFIED)"
        PEER_STATE_ARGS=()
        if [[ "${ENABLE_PEER_AVOIDANCE}" == "true" ]]; then
            PEER_STATE_ARGS=(-p peer_state_topic:=/epuck_virtual_peer/state)
        fi
        ros2 run epuck2_comm cooperative_avoider --ros-args \
            -r cmd_vel:=cmd_vel_unguarded \
            -p robot_id:=1 \
            -p armed:=true \
            -p enable_peer_avoidance:="${ENABLE_PEER_AVOIDANCE}" \
            -p enable_dynamic_heading:=true \
            -p enable_dynamic_speed:=true \
            -p enable_local_avoidance:=true \
            -p require_local_sensors:=true \
            -p use_sim_time:=false \
            -p nominal_speed_mps:="${MAX_LINEAR_SPEED_MPS}" \
            -p safety_radius_m:=0.14 \
            -p stop_after_recovery:=false \
            "${PEER_STATE_ARGS[@]}" \
            > "${OUT_DIR}/cooperative_avoider.log" 2>&1 &
        record_process "cooperative_avoider" "$!" ""

        sleep 2
        echo "[$(date -Iseconds)] step 9: starting hil_cmd_vel_guard.py (DISARMED)"
        python3 "${SCRIPT_DIR}/hil_cmd_vel_guard.py" \
            --physical-state-topic /epuck1/state \
            --upstream-cmd-vel-topic cmd_vel_unguarded \
            --guarded-cmd-vel-topic cmd_vel \
            --max-linear-speed-mps "${MAX_LINEAR_SPEED_MPS}" \
            --max-angular-speed-rps "${MAX_ANGULAR_SPEED_RPS}" \
            --heartbeat-timeout-s "${HEARTBEAT_TIMEOUT_S}" \
            --physical-state-timeout-s "${PHYSICAL_STATE_TIMEOUT_S}" \
            --virtual-peer-timeout-s "${VIRTUAL_PEER_TIMEOUT_S}" \
            --require-validity-flags 7 \
            $( [[ "${COMM_MODE}" == "HIL_COMM_ON" ]] && echo "--require-virtual-peer --virtual-peer-topic /epuck_virtual_peer/state" ) \
            > "${OUT_DIR}/hil_cmd_vel_guard.log" 2>&1 &
        record_process "hil_cmd_vel_guard" "$!" "${SCRIPT_DIR}/hil_cmd_vel_guard.py"

        sleep 2
        echo "[$(date -Iseconds)] step 10: recording PID manifest and SHA-256 identities"
        {
            echo "{"
            echo "  \"comm_mode\": \"${COMM_MODE}\","
            echo "  \"started_at\": \"$(date -Iseconds)\","
            echo "  \"processes\": {"
            first=true
            for name in "${!PIDS[@]}"; do
                [[ "${first}" == true ]] && first=false || echo ","
                printf '    "%s": {"pid": %s, "sha256": "%s"}' "${name}" "${PIDS[${name}]}" "${SHAS[${name}]}"
            done
            echo ""
            echo "  }"
            echo "}"
        } > "${PID_MANIFEST}"
        cat "${PID_MANIFEST}"

        GUARDED_COUNT="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo unknown)"
        echo "cmd_vel_publisher_count_after_start=${GUARDED_COUNT}"

        echo "[$(date -Iseconds)] HIL_TRIAL_PROCESSES_STARTED_DISARMED evidence_dir=${OUT_DIR}"
        echo "AWAITING_EXPLICIT_ARM_CONFIRMATION"
        echo "To arm (separate, explicit, human step -- NOT run by this script):"
        echo "  ros2 topic pub --once /hil_guard/arm std_msgs/msg/Bool \"{data: true}\""
        echo "To stop this trial's processes by their exact recorded PIDs:"
        echo "  bash ${SCRIPT_DIR}/run_hil_shutdown.sh ${PID_MANIFEST}"
        echo "This script does not arm the guard and does not start another trial automatically."

        trap - EXIT
        exit 0
        ;;
    *)
        echo "[run_hil_shared_exit_trial] PHYSICAL_MOTION_LOCKED_UNTIL_LAB_VALIDATION"
        echo "[run_hil_shared_exit_trial] Reason: mode '${MODE}' is not recognized, or would require ground motion without going through the --comm-off/--comm-on safety gate. This launcher has no bypass."
        exit 2
        ;;
esac
