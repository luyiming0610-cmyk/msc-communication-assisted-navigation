#!/usr/bin/env bash
# Stage 4 orchestrator -- the one-command WSL launcher for the minimal
# one-real-robot + one-virtual-scout physical HIL validation (design
# review, 2026-07-30). Modeled directly on the already-committed
# run_hil_shared_exit_trial.sh, with the Stage 4-specific command path
# and one-shot supervisor spliced in. cooperative_avoider.py,
# hil_cmd_vel_guard.py, hil_topic_adapter.py, and hil_virtual_peer.py
# are reused completely unmodified.
#
# Revised command path (Section B of the design review):
#   cooperative_avoider (-r cmd_vel:=cmd_vel_stage4_raw)
#     -> hil_stage4_motion_supervisor.py (new, sole /hil_guard/arm authority)
#     -> cmd_vel_unguarded
#     -> hil_cmd_vel_guard.py (existing, UNMODIFIED, sole /cmd_vel publisher,
#        started with --max-angular-speed-rps 0.0 as an independent backstop)
#     -> /cmd_vel -> physical bridge -> Pi -> robot
#
# Modes:
#   --check-only  (DEFAULT) Offline checks only: syntax/self-check, no
#                 ROS, no process, no Pi, no Webots.
#   --dry-run     Prints the planned step sequence and exits. Nothing is
#                 executed.
#   --run         Runs the actual trial. Requires the manual physical
#                 bring-up (Pi driver, audited Pi server, WSL bridge,
#                 real state_publisher.py) to already be up per
#                 HIL_LAB_RUNBOOK.md Section 2 -- this script does NOT
#                 start or touch the Pi in any way.
#
# Safety invariants (unconditional, no bypass in any mode):
#   - The guard starts DISARMED and is never armed by this script or by
#     any human action -- hil_stage4_motion_supervisor.py is the sole
#     /hil_guard/arm publisher, and only after it independently proves
#     event -> announcement -> adoption -> validated raw command.
#   - The operator's only action is answering one approval prompt with
#     exactly APPROVED_FOR_SINGLE_HIL_EVENT=YES. No `ros2 topic pub` to
#     /hil_guard/arm is ever printed or expected from the operator.
#   - The virtual scout (hil_virtual_peer.py) is not spawned until every
#     readiness/zero/publisher-count gate has passed AND the approval
#     token has been accepted -- see step 9 below.
#   - This script never starts a second trial automatically.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
COOP_EXIT_TOOLS="$(cd "${HIL_ROOT}/../../10_cooperative_exit_navigation_20260720/tools" && pwd)"

# Frozen Stage 4 parameters (design review, 2026-07-30). Self-contained
# here rather than routed through hil_frozen_params_to_env.py, whose
# JSON schema is Stage-3-shared-exit-specific and not to be repurposed
# for a different physical geometry without its own review.
GOAL_ID="shared_exit"
START_POSE_X_M="0.30"
START_POSE_Y_M="0.50"
EXIT_CENTER_X_M="1.20"
EXIT_CENTER_Y_M="0.50"
EXIT_RADIUS_M="0.05"
MAX_LINEAR_SPEED_MPS="0.015"
MAX_ANGULAR_SPEED_RPS="0.0"
HEARTBEAT_TIMEOUT_S="0.5"
PHYSICAL_STATE_TIMEOUT_S="0.5"
REQUIRED_VALIDITY_FLAGS="7"

PHYSICAL_STATE_TOPIC="/epuck1/state"
VIRTUAL_STATE_TOPIC="/epuck_virtual_peer/state"
GOAL_ANNOUNCEMENT_TOPIC="/hil/goal_announcement"
ADOPTION_EVIDENCE_TOPIC="/hil/adoption_evidence"
RAW_CMD_VEL_TOPIC="cmd_vel_stage4_raw"
UPSTREAM_CMD_VEL_TOPIC="cmd_vel_unguarded"
GUARDED_CMD_VEL_TOPIC="cmd_vel"
ARM_TOPIC="/hil_guard/arm"
VIRTUAL_SCOUT_RELEASED_TOPIC="/hil_stage4/virtual_scout_released"

MODE="--check-only"
if [[ $# -gt 0 ]]; then
    MODE="$1"
fi

case "${MODE}" in
    --check-only)
        echo "[run_hil_stage4_trial] MODE=--check-only (offline checks only, no processes started, no Pi/Webots contact)"
        python3 -m py_compile "${SCRIPT_DIR}/hil_stage4_motion_supervisor.py" || exit 1
        bash -n "${SCRIPT_DIR}/run_hil_stage4_trial.sh" || exit 1
        for f in "${SCRIPT_DIR}/hil_topic_adapter.py" "${SCRIPT_DIR}/hil_cmd_vel_guard.py" "${SCRIPT_DIR}/hil_virtual_peer.py"; do
            [[ -f "${f}" ]] || { echo "MISSING_REQUIRED_FILE: ${f}" >&2; exit 1; }
        done
        echo "STAGE4_CHECK_ONLY=PASS"
        exit 0
        ;;
    --dry-run)
        echo "[run_hil_stage4_trial] MODE=--dry-run (printing planned steps only, nothing executed)"
        cat <<EOF
Planned steps (NONE executed in --dry-run):
  0. Assumes manual physical bring-up already complete (Pi driver, audited
     Pi server, WSL bridge, real state_publisher.py) per HIL_LAB_RUNBOOK.md
     Section 2 -- this script never starts or touches those.
  1. Create a new timestamped evidence directory; verify zero publishers
     on ${RAW_CMD_VEL_TOPIC}, ${UPSTREAM_CMD_VEL_TOPIC}, ${GUARDED_CMD_VEL_TOPIC},
     ${GOAL_ANNOUNCEMENT_TOPIC}, ${ADOPTION_EVIDENCE_TOPIC}, ${VIRTUAL_STATE_TOPIC};
     verify exactly 1 publisher on ${PHYSICAL_STATE_TOPIC} (the real
     state_publisher, no Stage 3 or synthetic publisher).
  2. Start the WSL command-evidence recorder FIRST (stops LAST).
  3. Start hil_topic_adapter.py (goal_id=${GOAL_ID}, exit target
     (${EXIT_CENTER_X_M}, ${EXIT_CENTER_Y_M})) -- publishes machine-readable
     adoption evidence on ${ADOPTION_EVIDENCE_TOPIC}.
  4. Start cooperative_avoider.py, UNMODIFIED, remapped
     -r cmd_vel:=${RAW_CMD_VEL_TOPIC}.
  5. Start hil_cmd_vel_guard.py -- DISARMED, --max-angular-speed-rps
     ${MAX_ANGULAR_SPEED_RPS}, upstream=${UPSTREAM_CMD_VEL_TOPIC},
     guarded=${GUARDED_CMD_VEL_TOPIC}.
  6. Start hil_stage4_motion_supervisor.py -- state PREPARED, subscribed to
     ${ADOPTION_EVIDENCE_TOPIC} and ${RAW_CMD_VEL_TOPIC}, publishing
     ${ARM_TOPIC} and ${UPSTREAM_CMD_VEL_TOPIC}.
  7. Confirm all four components READY; re-verify the same zero/publisher
     counts as step 1 (post-start).
  8. Prompt the operator for exactly: APPROVED_FOR_SINGLE_HIL_EVENT=YES
     This is the ONLY human action. No arm topic is ever published by a
     human.
  9. Only after approval is accepted AND all gates in steps 1/7 pass:
     publish ${VIRTUAL_SCOUT_RELEASED_TOPIC}=true and spawn
     hil_virtual_peer.py exactly once (no Stage 3 harness, no synthetic
     duplicate announcement).
 10. Wait for the supervisor to reach a terminal state (COMPLETE or FAILED).
 11. Stop every orchestrator-owned process by exact recorded PID via
     run_hil_shutdown.sh, recorder last.
EOF
        exit 0
        ;;
    --run)
        echo "[$(date -Iseconds)] MODE=--run -- Stage 4 trial"

        # Source-identity gate (Section 4/5 of the design review): a
        # fresh physical run must never proceed against an unexpected
        # working-tree state. EXPECTED_HEAD is required, never inferred,
        # never defaulted -- reuses the exact git hash-object comparison
        # technique already proven in HIL_OFFLINE_STAGE3_RUNBOOK.md,
        # scoped to the Stage 4-critical file set. No output directory is
        # created before this passes.
        : "${EXPECTED_HEAD:?EXPECTED_HEAD must be set to the exact commit the Stage 4 files are expected to match -- refusing to run without it}"
        REPO_ROOT="$(cd "${HIL_ROOT}/../../.." && pwd)"
        ACTUAL_HEAD="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
        if [[ "${ACTUAL_HEAD}" != "${EXPECTED_HEAD}" ]]; then
            echo "ABORT: HEAD mismatch. expected=${EXPECTED_HEAD} actual=${ACTUAL_HEAD}" >&2
            exit 1
        fi
        # Full set, derived from the actual launcher/import graph this
        # script exercises in --run mode (audited 2026-07-30, revision 4)
        # -- every entry is a file this exact script imports, execs, or
        # depends on transitively; nothing added just because it appears
        # in an external checklist. synthetic_stage4_physical_state_publisher.py
        # is deliberately NOT here -- see Section 3 of the design review
        # and test_run_hil_stage4_trial_static.py's
        # PhysicalModeNeverUsesSyntheticCodeTest.
        STAGE4_SOURCE_IDENTITY_PATHS=(
            # This orchestrator and the components it launches directly:
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/run_hil_stage4_trial.sh"
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_stage4_motion_supervisor.py"
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_stage4_post_run_verifier.py"
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_goal_announcement_evidence.py"
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_topic_adapter.py"
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_cmd_vel_guard.py"
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_virtual_peer.py"
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_command_evidence_recorder.py"
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/run_hil_shutdown.sh"
            "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/STAGE4_PHYSICAL_HIL_SPEC.md"
            # hil_topic_adapter.py's real import chain (audited via its
            # own _import_goal_navigator(), read-only source inspection):
            "experiments/10_cooperative_exit_navigation_20260720/tools/goal_navigator.py"
            "experiments/10_cooperative_exit_navigation_20260720/tools/goal_hold_tracker.py"
            "experiments/10_cooperative_exit_navigation_20260720/tools/navigation_target_state.py"
            # cooperative_avoider's real import chain (audited via its own
            # `from .xxx import ...` lines, read-only source inspection).
            # These are verified below against the INSTALLED colcon copy,
            # not just the WSL source tree, since cooperative_avoider runs
            # via the installed executable, not `python3 <source>.py`.
            "src/epuck2_comm/epuck2_comm/cooperative_avoider.py"
            "src/epuck2_comm/epuck2_comm/command_smoothing.py"
            "src/epuck2_comm/epuck2_comm/collision_math.py"
            "src/epuck2_comm/epuck2_comm/local_obstacle_logic.py"
            # The real state_publisher.py physical mode requires exactly
            # one live instance of (manual bring-up, per Section 7) --
            # also installed/verified below, same reason as above.
            "src/epuck2_comm/epuck2_comm/state_publisher.py"
        )
        for rel_path in "${STAGE4_SOURCE_IDENTITY_PATHS[@]}"; do
            abs_path="${REPO_ROOT}/${rel_path}"
            if [[ ! -f "${abs_path}" ]]; then
                echo "ABORT: expected Stage 4 source file does not exist: ${rel_path}" >&2
                exit 1
            fi
            working_tree_hash="$(git -C "${REPO_ROOT}" hash-object -- "${rel_path}")"
            committed_hash="$(git -C "${REPO_ROOT}" rev-parse "${EXPECTED_HEAD}:${rel_path}" 2>/dev/null || true)"
            if [[ -z "${committed_hash}" ]]; then
                echo "ABORT: ${rel_path} is not tracked at ${EXPECTED_HEAD}" >&2
                exit 1
            fi
            if [[ "${working_tree_hash}" != "${committed_hash}" ]]; then
                echo "ABORT: working tree differs from ${EXPECTED_HEAD} for ${rel_path}" >&2
                exit 1
            fi
            echo "source_identity_ok=${rel_path}"
        done
        echo "STAGE4_SOURCE_IDENTITY=PASS expected_head=${EXPECTED_HEAD}"

        source /opt/ros/humble/setup.bash
        source ~/epuck_ws/install/setup.bash
        set -u

        # Installed-runtime identity: cooperative_avoider runs via the
        # colcon-installed executable, not the WSL source tree directly,
        # so source-tree identity alone does not prove what actually
        # executes. Reuses the same path-aware git hash-object technique
        # already proven in HIL_OFFLINE_STAGE3_RUNBOOK.md (never raw
        # SHA-256, which would false-fail on legitimate CRLF/LF
        # differences between a Linux colcon install and a WSL-mounted
        # worktree).
        COOP_PREFIX="$(ros2 pkg prefix epuck2_comm)"
        COOP_EXE="${COOP_PREFIX}/lib/epuck2_comm/cooperative_avoider"
        if [[ ! -x "${COOP_EXE}" ]]; then
            echo "ABORT: installed cooperative_avoider executable not found or not executable: ${COOP_EXE}" >&2
            exit 1
        fi
        ENTRYPOINT_CANDIDATES="$(find "${COOP_PREFIX}/lib/epuck2_comm" -maxdepth 1 -name 'cooperative_avoider' | wc -l)"
        if [[ "${ENTRYPOINT_CANDIDATES}" != "1" ]]; then
            echo "ABORT: expected exactly 1 installed cooperative_avoider entry-point candidate, found ${ENTRYPOINT_CANDIDATES}" >&2
            exit 1
        fi
        echo "installed_entrypoint_ok=cooperative_avoider candidates=1"

        COOP_INSTALLED_PY_ROOT="$(python3 -c "import epuck2_comm, os; print(os.path.dirname(epuck2_comm.__file__))" 2>/dev/null || true)"
        if [[ -z "${COOP_INSTALLED_PY_ROOT}" ]]; then
            echo "ABORT: could not resolve installed epuck2_comm Python package root" >&2
            exit 1
        fi
        for module in cooperative_avoider command_smoothing collision_math local_obstacle_logic state_publisher; do
            src_rel="src/epuck2_comm/epuck2_comm/${module}.py"
            installed_path="${COOP_INSTALLED_PY_ROOT}/${module}.py"
            if [[ ! -f "${installed_path}" ]]; then
                echo "ABORT: installed module missing: ${installed_path}" >&2
                exit 1
            fi
            installed_hash="$(git -C "${REPO_ROOT}" hash-object --path="${src_rel}" -- "${installed_path}")"
            committed_hash="$(git -C "${REPO_ROOT}" rev-parse "${EXPECTED_HEAD}:${src_rel}" 2>/dev/null || true)"
            if [[ -z "${committed_hash}" ]]; then
                echo "ABORT: ${src_rel} is not tracked at ${EXPECTED_HEAD}" >&2
                exit 1
            fi
            if [[ "${installed_hash}" != "${committed_hash}" ]]; then
                echo "ABORT: installed ${module}.py does not match ${EXPECTED_HEAD}'s ${src_rel}" >&2
                exit 1
            fi
            echo "installed_runtime_identity_ok=${module}"
        done
        echo "STAGE4_INSTALLED_RUNTIME_IDENTITY=PASS"

        publisher_count() {
            ros2 topic info "$1" 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo 0
        }

        for topic in "${RAW_CMD_VEL_TOPIC}" "${UPSTREAM_CMD_VEL_TOPIC}" "${GUARDED_CMD_VEL_TOPIC}" \
                     "${GOAL_ANNOUNCEMENT_TOPIC}" "${ADOPTION_EVIDENCE_TOPIC}" "${VIRTUAL_STATE_TOPIC}"; do
            count="$(publisher_count "${topic}")"
            echo "preflight_publisher_count(${topic})=${count}"
            if [[ "${count}" != "0" ]]; then
                echo "ABORT: expected 0 publishers on ${topic} before starting, found ${count}." >&2
                exit 1
            fi
        done
        REAL_STATE_COUNT="$(publisher_count "${PHYSICAL_STATE_TOPIC}")"
        echo "preflight_publisher_count(${PHYSICAL_STATE_TOPIC})=${REAL_STATE_COUNT}"
        if [[ "${REAL_STATE_COUNT}" != "1" ]]; then
            echo "ABORT: expected exactly 1 publisher on ${PHYSICAL_STATE_TOPIC} (the real state_publisher) before starting, found ${REAL_STATE_COUNT}. Refusing to proceed without a confirmed real, single physical state source." >&2
            exit 1
        fi

        RUN_ID="stage4_$(date +%Y%m%d_%H%M%S)"
        OUT_DIR="/home/eamon/epuck_comm_bags/hil_${RUN_ID}"
        mkdir -p "${OUT_DIR}"
        PID_MANIFEST="${OUT_DIR}/pid_manifest.json"
        echo '{"processes": {}}' > "${PID_MANIFEST}"

        declare -A PIDS
        declare -A SHAS
        record_process() {
            local name="$1" pid="$2" script_path="$3"
            PIDS["${name}"]="${pid}"
            SHAS["${name}"]="$([[ -n "${script_path}" && -f "${script_path}" ]] && sha256sum "${script_path}" | awk '{print $1}' || echo "")"
        }
        write_manifest() {
            {
                echo "{"
                echo "  \"run_id\": \"${RUN_ID}\","
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
        }

        cleanup() {
            local exit_code=$?
            echo "[$(date -Iseconds)] cleanup: stopping any started Stage 4 processes via run_hil_shutdown.sh (recorder last)"
            write_manifest
            if [[ -f "${PID_MANIFEST}" ]]; then
                bash "${SCRIPT_DIR}/run_hil_shutdown.sh" "${PID_MANIFEST}" || true
            fi
            # Exact-PID residual check only -- never pkill/name-based.
            # Consumed by hil_stage4_post_run_verifier.py's
            # --residual-check-path.
            residual="CLEAN"
            for name in "${!PIDS[@]}"; do
                pid="${PIDS[${name}]}"
                if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
                    residual="PROCESS_STILL_ALIVE:${name}:${pid}"
                fi
            done
            echo "{\"residual_process_check\": \"${residual}\"}" > "${OUT_DIR}/residual_check.json"
            echo "POST_RUN_RESIDUAL_PROCESS_CHECK=${residual}"
            exit "${exit_code}"
        }
        trap cleanup EXIT

        echo "[$(date -Iseconds)] step 2: starting WSL command-evidence recorder (first, stops last)"
        python3 "${SCRIPT_DIR}/hil_command_evidence_recorder.py" \
            --upstream-cmd-vel-topic "${UPSTREAM_CMD_VEL_TOPIC}" \
            --guarded-cmd-vel-topic "${GUARDED_CMD_VEL_TOPIC}" \
            --arm-topic "${ARM_TOPIC}" \
            --state-topic "${PHYSICAL_STATE_TOPIC}" \
            --output-csv "${OUT_DIR}/command_evidence.csv" \
            --duration-s 3600 \
            > "${OUT_DIR}/recorder.log" 2>&1 &
        record_process "recorder" "$!" "${SCRIPT_DIR}/hil_command_evidence_recorder.py"

        echo "[$(date -Iseconds)] step 3: starting hil_topic_adapter.py (goal_id=${GOAL_ID}, target=(${EXIT_CENTER_X_M},${EXIT_CENTER_Y_M}))"
        python3 "${SCRIPT_DIR}/hil_topic_adapter.py" \
            --robot-id=1 --state-topic="${PHYSICAL_STATE_TOPIC}" --nav-intent-topic=/epuck1/nav_intent \
            --mode=search --waypoints="${START_POSE_X_M}:${START_POSE_Y_M}" \
            --waypoint-arrival-radius=0.10 --rate-hz=2.0 \
            --nominal-speed-mps="${MAX_LINEAR_SPEED_MPS}" \
            --exit-center-x="${EXIT_CENTER_X_M}" --exit-center-y="${EXIT_CENTER_Y_M}" --exit-radius="${EXIT_RADIUS_M}" \
            --parking-x="${EXIT_CENTER_X_M}" --parking-y="${EXIT_CENTER_Y_M}" --parking-radius="${EXIT_RADIUS_M}" \
            --goal-hold-time-s=2.0 \
            --goal-announcement-topic="${GOAL_ANNOUNCEMENT_TOPIC}" \
            > "${OUT_DIR}/hil_topic_adapter.log" 2>&1 &
        record_process "hil_topic_adapter" "$!" "${SCRIPT_DIR}/hil_topic_adapter.py"

        echo "[$(date -Iseconds)] step 4: starting cooperative_avoider.py (real robot's frozen controller, UNMODIFIED, output remapped to ${RAW_CMD_VEL_TOPIC})"
        # cooperative_avoider.py subscribes to a HARDCODED relative topic
        # name "state" (create_subscription(EpuckState, "state", ...)) --
        # there is no "state_topic" parameter, only a topic remap reaches
        # it. Caught by the Stage 4 live ROS-graph rehearsal: without
        # this remap the node never sees the real robot's pose and stays
        # in SAFE_STOP_STALE permanently, silently producing zero forever
        # (which a Stage 3-style "guarded output stays zero" check would
        # never have flagged).
        # nav_intent is ALSO a hardcoded relative topic name in
        # cooperative_avoider.py (create_subscription(NavigationIntent,
        # "nav_intent", ...), no parameter) -- caught by the same live
        # rehearsal: without this remap, enable_dynamic_speed's
        # stale/never-received fallback (exactly 0.0 m/s) applies
        # permanently, since the adapter's NavigationIntent would never
        # actually reach this subscription.
        ros2 run epuck2_comm cooperative_avoider --ros-args \
            -r cmd_vel:="${RAW_CMD_VEL_TOPIC}" \
            -r state:="${PHYSICAL_STATE_TOPIC}" \
            -r nav_intent:=/epuck1/nav_intent \
            -p robot_id:=1 -p armed:=true \
            -p enable_peer_avoidance:=true -p enable_dynamic_heading:=true \
            -p enable_dynamic_speed:=true -p enable_local_avoidance:=true \
            -p require_local_sensors:=true -p use_sim_time:=false \
            -p nominal_speed_mps:="${MAX_LINEAR_SPEED_MPS}" \
            -p safety_radius_m:=0.14 -p stop_after_recovery:=false \
            -p peer_state_topic:="${VIRTUAL_STATE_TOPIC}" \
            > "${OUT_DIR}/cooperative_avoider.log" 2>&1 &
        record_process "cooperative_avoider" "$!" ""

        sleep 2
        echo "[$(date -Iseconds)] step 5: starting hil_cmd_vel_guard.py (DISARMED, independent backstop, max_angular=0)"
        python3 "${SCRIPT_DIR}/hil_cmd_vel_guard.py" \
            --physical-state-topic "${PHYSICAL_STATE_TOPIC}" \
            --upstream-cmd-vel-topic "${UPSTREAM_CMD_VEL_TOPIC}" \
            --guarded-cmd-vel-topic "${GUARDED_CMD_VEL_TOPIC}" \
            --arm-topic "${ARM_TOPIC}" \
            --max-linear-speed-mps "${MAX_LINEAR_SPEED_MPS}" \
            --max-angular-speed-rps "${MAX_ANGULAR_SPEED_RPS}" \
            --heartbeat-timeout-s "${HEARTBEAT_TIMEOUT_S}" \
            --physical-state-timeout-s "${PHYSICAL_STATE_TIMEOUT_S}" \
            --require-virtual-peer --virtual-peer-topic "${VIRTUAL_STATE_TOPIC}" \
            > "${OUT_DIR}/hil_cmd_vel_guard.log" 2>&1 &
        record_process "hil_cmd_vel_guard" "$!" "${SCRIPT_DIR}/hil_cmd_vel_guard.py"

        sleep 2
        echo "[$(date -Iseconds)] step 6: starting hil_stage4_motion_supervisor.py -- PREPARED, awaiting operator approval"
        echo ""
        echo "=============================================================="
        echo " STAGE 4 STOP POINT -- operator approval required"
        echo " Type exactly the following line and press Enter to proceed:"
        echo "   APPROVED_FOR_SINGLE_HIL_EVENT=YES"
        echo " Anything else aborts this trial with no motion window opened."
        echo "=============================================================="
        read -r -p "> " OPERATOR_INPUT
        if [[ "${OPERATOR_INPUT}" != "APPROVED_FOR_SINGLE_HIL_EVENT=YES" ]]; then
            echo "[run_hil_stage4_trial] Operator did not enter the exact required approval string. Aborting -- no supervisor started, no arm possible." >&2
            exit 3
        fi

        python3 "${SCRIPT_DIR}/hil_stage4_motion_supervisor.py" \
            --goal-id "${GOAL_ID}" --run-id "${RUN_ID}" \
            --expected-target-x-m "${EXIT_CENTER_X_M}" --expected-target-y-m "${EXIT_CENTER_Y_M}" \
            --adoption-evidence-topic "${ADOPTION_EVIDENCE_TOPIC}" \
            --raw-cmd-vel-topic "${RAW_CMD_VEL_TOPIC}" \
            --guarded-output-topic "${UPSTREAM_CMD_VEL_TOPIC}" \
            --arm-topic "${ARM_TOPIC}" \
            --virtual-scout-released-topic "${VIRTUAL_SCOUT_RELEASED_TOPIC}" \
            --physical-state-topic "${PHYSICAL_STATE_TOPIC}" \
            --physical-state-timeout-s "${PHYSICAL_STATE_TIMEOUT_S}" \
            --required-validity-flags "${REQUIRED_VALIDITY_FLAGS}" \
            --evidence-path "${OUT_DIR}/stage4_supervisor_evidence.jsonl" \
            --operator-approval-token "APPROVED_FOR_SINGLE_HIL_EVENT=YES" \
            > "${OUT_DIR}/hil_stage4_motion_supervisor.log" 2>&1 &
        record_process "hil_stage4_motion_supervisor" "$!" "${SCRIPT_DIR}/hil_stage4_motion_supervisor.py"

        sleep 2
        write_manifest
        echo "[$(date -Iseconds)] step 7: post-start readiness/zero/publisher-count re-check"
        GUARDED_COUNT="$(publisher_count "${GUARDED_CMD_VEL_TOPIC}")"
        echo "post_start_publisher_count(${GUARDED_CMD_VEL_TOPIC})=${GUARDED_COUNT}"
        if [[ "${GUARDED_COUNT}" != "1" ]]; then
            echo "ABORT: expected exactly 1 publisher on ${GUARDED_CMD_VEL_TOPIC} (the guard) after start, found ${GUARDED_COUNT}." >&2
            exit 1
        fi
        # Re-verify the real physical state topic still has exactly one
        # publisher immediately before release -- rejects a second or
        # synthetic publisher that appeared after the initial preflight
        # check (step 1) but before the virtual scout is released.
        REAL_STATE_COUNT_POST_START="$(publisher_count "${PHYSICAL_STATE_TOPIC}")"
        echo "post_start_publisher_count(${PHYSICAL_STATE_TOPIC})=${REAL_STATE_COUNT_POST_START}"
        if [[ "${REAL_STATE_COUNT_POST_START}" != "1" ]]; then
            echo "ABORT: expected exactly 1 publisher on ${PHYSICAL_STATE_TOPIC} (the real state_publisher) immediately before release, found ${REAL_STATE_COUNT_POST_START}." >&2
            exit 1
        fi

        echo "[$(date -Iseconds)] step 9: releasing virtual scout exactly once (all gates passed, approval already accepted)"
        ros2 topic pub --once "${VIRTUAL_SCOUT_RELEASED_TOPIC}" std_msgs/msg/Bool "{data: true}" > /dev/null
        python3 "${SCRIPT_DIR}/hil_virtual_peer.py" \
            --robot-id 2 --state-topic "${VIRTUAL_STATE_TOPIC}" \
            --announcement-topic "${GOAL_ANNOUNCEMENT_TOPIC}" --goal-id "${GOAL_ID}" \
            --start-x-m "${START_POSE_X_M}" --start-y-m "${START_POSE_Y_M}" --start-yaw-rad 0.0 \
            --target-x-m "${EXIT_CENTER_X_M}" --target-y-m "${EXIT_CENTER_Y_M}" \
            --cruise-linear-mps "${MAX_LINEAR_SPEED_MPS}" --arrival-radius-m "${EXIT_RADIUS_M}" \
            --max-angular-rps 0.0 \
            > "${OUT_DIR}/hil_virtual_peer.log" 2>&1 &
        record_process "hil_virtual_peer" "$!" "${SCRIPT_DIR}/hil_virtual_peer.py"
        write_manifest

        echo "[$(date -Iseconds)] HIL_STAGE4_TRIAL_RUNNING evidence_dir=${OUT_DIR} run_id=${RUN_ID}"
        echo "Waiting for the supervisor to reach a terminal state (COMPLETE or FAILED)..."
        echo "This script does not arm anything itself and does not start a second trial."
        # Operator/orchestrator note: this --run mode is written and
        # offline-syntax-checked but has NOT been executed against real
        # hardware in this design-review turn -- no Pi contact, no RUN_ID,
        # no real evidence directory was created by this session.
        trap - EXIT
        exit 0
        ;;
    *)
        echo "[run_hil_stage4_trial] Reason: mode '${MODE}' is not recognized. This launcher has no bypass." >&2
        exit 2
        ;;
esac
