#!/usr/bin/env bash
# Stage 4 orchestrator -- the one-command WSL launcher for the minimal
# one-real-robot + one-virtual-scout physical HIL validation (design
# review, 2026-07-30 through 2026-07-31, revision 5). Modeled directly
# on the already-committed run_hil_shared_exit_trial.sh, with the
# Stage 4-specific command path and one-shot supervisor spliced in.
# cooperative_avoider.py, hil_cmd_vel_guard.py, hil_topic_adapter.py,
# and hil_virtual_peer.py are reused completely unmodified.
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
#                 start or touch the Pi in any way. Atomically creates
#                 source_identity_manifest.json (before any process
#                 starts) and launcher_status.json (updated throughout).
#                 Exits after backgrounding all components (non-blocking
#                 by design -- the operator watches progress via WSL
#                 Window 4, per the command sheet).
#   --finalize <evidence_root>
#                 Read-only, offline, run AFTER the trial has ended and
#                 the operator has confirmed cleanup (per command sheet
#                 steps 12-14: Pi evidence transferred, physical
#                 measurements JSON authored). Produces the deterministic
#                 two-stage hash-verified physical evidence package and
#                 invokes the committed physical verifier. Never starts
#                 ROS, never contacts the Pi, never touches production
#                 topics.
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
        python3 -m py_compile "${SCRIPT_DIR}/hil_stage4_post_run_verifier.py" || exit 1
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
  1. Verify HEAD == EXPECTED_HEAD; verify source-tree and installed-runtime
     identity for every Stage 4-critical file; ATOMICALLY WRITE
     source_identity_manifest.json as the first file in a new evidence
     directory -- created only after identity PASS, before any process
     starts.
  2. Verify zero publishers on ${RAW_CMD_VEL_TOPIC}, ${UPSTREAM_CMD_VEL_TOPIC},
     ${GUARDED_CMD_VEL_TOPIC}, ${GOAL_ANNOUNCEMENT_TOPIC}, ${ADOPTION_EVIDENCE_TOPIC},
     ${VIRTUAL_STATE_TOPIC}; verify exactly 1 publisher on ${PHYSICAL_STATE_TOPIC}
     (the real state_publisher, no Stage 3 or synthetic publisher).
  3. Start the WSL command-evidence recorder FIRST (stops LAST). Update
     launcher_status.json.
  4. Start hil_topic_adapter.py (goal_id=${GOAL_ID}, exit target
     (${EXIT_CENTER_X_M}, ${EXIT_CENTER_Y_M})) -- publishes machine-readable
     adoption evidence on ${ADOPTION_EVIDENCE_TOPIC}. Update launcher_status.json.
  5. Start cooperative_avoider.py, UNMODIFIED, remapped
     -r cmd_vel:=${RAW_CMD_VEL_TOPIC}. Update launcher_status.json.
  6. Start hil_cmd_vel_guard.py -- DISARMED, --max-angular-speed-rps
     ${MAX_ANGULAR_SPEED_RPS}, upstream=${UPSTREAM_CMD_VEL_TOPIC},
     guarded=${GUARDED_CMD_VEL_TOPIC}. Update launcher_status.json.
  7. Start hil_stage4_motion_supervisor.py -- state PREPARED, subscribed to
     ${ADOPTION_EVIDENCE_TOPIC} and ${RAW_CMD_VEL_TOPIC}, publishing
     ${ARM_TOPIC} and ${UPSTREAM_CMD_VEL_TOPIC}. Update launcher_status.json.
  8. Confirm all four components READY; re-verify the same zero/publisher
     counts as step 2 (post-start).
  9. Prompt the operator for exactly: APPROVED_FOR_SINGLE_HIL_EVENT=YES
     This is the ONLY human action. No arm topic is ever published by a
     human. Record operator approval state in launcher_status.json.
 10. Only after approval is accepted AND all gates in steps 2/8 pass:
     publish ${VIRTUAL_SCOUT_RELEASED_TOPIC}=true and spawn
     hil_virtual_peer.py exactly once (no Stage 3 harness, no synthetic
     duplicate announcement).
 11. Exit non-blocking (this script hands off to the operator, per the
     command sheet, for observation via WSL Window 4).
 12. Operator later runs \`run_hil_stage4_trial.sh --finalize <evidence_root>\`
     (a separate, explicit, read-only-except-hash-files step) once the
     supervisor has reached a terminal state, cleanup has run, Pi evidence
     has been transferred in, and physical_measurements.json has been
     authored -- see --finalize's own description.
EOF
        exit 0
        ;;
    --run)
        echo "[$(date -Iseconds)] MODE=--run -- Stage 4 trial"

        : "${EXPECTED_HEAD:?EXPECTED_HEAD must be set to the exact commit the Stage 4 files are expected to match -- refusing to run without it}"
        REPO_ROOT="$(cd "${HIL_ROOT}/../../.." && pwd)"
        ACTUAL_HEAD="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
        if [[ "${ACTUAL_HEAD}" != "${EXPECTED_HEAD}" ]]; then
            echo "ABORT: HEAD mismatch. expected=${EXPECTED_HEAD} actual=${ACTUAL_HEAD}" >&2
            exit 1
        fi

        # RUN_ID is generated here, before any identity check, because
        # source_identity_manifest.json (written only once identity PASSES,
        # inside the RUN_ID-derived evidence directory) must itself record
        # the RUN_ID it belongs to. Generated exactly once; never
        # regenerated later in this invocation.
        RUN_ID="stage4_$(date +%Y%m%d_%H%M%S)"
        echo "RUN_ID=${RUN_ID}"

        # Source-identity gate (Section 4/5 of the design review): a
        # fresh physical run must never proceed against an unexpected
        # working-tree state. EXPECTED_HEAD is required, never inferred,
        # never defaulted -- reuses the exact git hash-object comparison
        # technique already proven in HIL_OFFLINE_STAGE3_RUNBOOK.md,
        # scoped to the Stage 4-critical file set. No output directory is
        # created before this passes. Every individual result is
        # collected into a scratch file (never inline JSON in bash, which
        # would be fragile against arbitrary path content) so the full
        # per-file result list can be written into
        # source_identity_manifest.json once the gate passes.
        IDENTITY_RESULTS_TMP="$(mktemp)"
        INSTALLED_RESULTS_TMP="$(mktemp)"
        trap 'rm -f "${IDENTITY_RESULTS_TMP}" "${INSTALLED_RESULTS_TMP}"' EXIT

        # Full set, derived from the actual launcher/import graph this
        # script exercises in --run mode (audited 2026-07-30, revision 4)
        # -- every entry is a file this exact script imports, execs, or
        # depends on transitively; nothing added just because it appears
        # in an external checklist. synthetic_stage4_physical_state_publisher.py
        # is deliberately NOT here -- see Section 3 of the design review
        # and test_run_hil_stage4_trial_static.py's
        # PhysicalModeNeverUsesSyntheticCodeTest.
        STAGE4_SOURCE_IDENTITY_PATHS=(
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
            "experiments/10_cooperative_exit_navigation_20260720/tools/goal_navigator.py"
            "experiments/10_cooperative_exit_navigation_20260720/tools/goal_hold_tracker.py"
            "experiments/10_cooperative_exit_navigation_20260720/tools/navigation_target_state.py"
            "src/epuck2_comm/epuck2_comm/cooperative_avoider.py"
            "src/epuck2_comm/epuck2_comm/command_smoothing.py"
            "src/epuck2_comm/epuck2_comm/collision_math.py"
            "src/epuck2_comm/epuck2_comm/local_obstacle_logic.py"
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
            match="true"
            if [[ "${working_tree_hash}" != "${committed_hash}" ]]; then
                echo "ABORT: working tree differs from ${EXPECTED_HEAD} for ${rel_path}" >&2
                exit 1
            fi
            echo "source_identity_ok=${rel_path}"
            printf '%s\t%s\t%s\t%s\n' "${rel_path}" "${committed_hash}" "${working_tree_hash}" "${match}" >> "${IDENTITY_RESULTS_TMP}"
        done
        echo "STAGE4_SOURCE_IDENTITY=PASS expected_head=${EXPECTED_HEAD}"

        set +u
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
            match="true"
            if [[ "${installed_hash}" != "${committed_hash}" ]]; then
                echo "ABORT: installed ${module}.py does not match ${EXPECTED_HEAD}'s ${src_rel}" >&2
                exit 1
            fi
            echo "installed_runtime_identity_ok=${module}"
            printf '%s\t%s\t%s\t%s\n' "${module}" "${committed_hash}" "${installed_hash}" "${match}" >> "${INSTALLED_RESULTS_TMP}"
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

        # Identity fully PASSED (source-tree + installed-runtime) -- only
        # now does the output directory get created, and
        # source_identity_manifest.json is the FIRST file written into
        # it, atomically (write to a .tmp sibling, then rename), before
        # any process is launched.
        OUT_DIR="/home/eamon/epuck_comm_bags/hil_${RUN_ID}"
        mkdir -p "${OUT_DIR}"

        SOURCE_IDENTITY_MANIFEST="${OUT_DIR}/source_identity_manifest.json"
        python3 - "${SOURCE_IDENTITY_MANIFEST}" "${RUN_ID}" "${EXPECTED_HEAD}" "${ACTUAL_HEAD}" \
                  "${IDENTITY_RESULTS_TMP}" "${INSTALLED_RESULTS_TMP}" "${ENTRYPOINT_CANDIDATES}" <<'PYEOF'
import json
import sys

out_path, run_id, expected_head, actual_head, identity_tmp, installed_tmp, entrypoint_candidates = sys.argv[1:8]


def _load(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n")
            if not line:
                continue
            parts = line.split("\t")
            rows.append(parts)
    return rows

source_paths = [
    {"path": p, "expected_blob": e, "worktree_blob": w, "match": m == "true"}
    for p, e, w, m in _load(identity_tmp)
]
installed_runtime = [
    {"module": mod, "expected_hash": e, "installed_hash": i, "match": m == "true"}
    for mod, e, i, m in _load(installed_tmp)
]

overall_result = "PASS"
if not source_paths or not all(r["match"] for r in source_paths):
    overall_result = "BLOCKED"
if not installed_runtime or not all(r["match"] for r in installed_runtime):
    overall_result = "BLOCKED"
if int(entrypoint_candidates) != 1:
    overall_result = "BLOCKED"

manifest = {
    "schema_version": "1.0.0",
    "run_id": run_id,
    "expected_head": expected_head,
    "actual_head": actual_head,
    "source_paths": source_paths,
    "installed_runtime": installed_runtime,
    "entrypoint_check": {"component": "cooperative_avoider", "candidates": int(entrypoint_candidates), "result": "PASS" if int(entrypoint_candidates) == 1 else "BLOCKED"},
    "overall_result": overall_result,
}

tmp_path = out_path + ".tmp"
with open(tmp_path, "w", encoding="utf-8") as f:
    json.dump(manifest, f, indent=2)
import os
os.replace(tmp_path, out_path)
print(f"source_identity_manifest_written={out_path} overall_result={overall_result}")
PYEOF
        rm -f "${IDENTITY_RESULTS_TMP}" "${INSTALLED_RESULTS_TMP}"
        trap - EXIT

        LAUNCHER_STATUS="${OUT_DIR}/launcher_status.json"
        PID_MANIFEST="${OUT_DIR}/pid_manifest.json"
        echo '{"processes": {}}' > "${PID_MANIFEST}"

        declare -A PIDS
        declare -A SHAS
        declare -A COMPONENT_STATE
        OPERATOR_APPROVAL_STATE="PENDING"
        SUPERVISOR_TERMINAL_STATE="null"
        CLEANUP_RESULT="NOT_YET_RUN"
        RESIDUAL_RESULT="NOT_YET_CHECKED"
        FINAL_CLASSIFICATION="RUNNING"

        record_process() {
            local name="$1" pid="$2" script_path="$3"
            PIDS["${name}"]="${pid}"
            SHAS["${name}"]="$([[ -n "${script_path}" && -f "${script_path}" ]] && sha256sum "${script_path}" | awk '{print $1}' || echo "")"
            COMPONENT_STATE["${name}"]="STARTED"
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

        write_launcher_status() {
            local component_tmp
            component_tmp="$(mktemp)"
            for name in "${!COMPONENT_STATE[@]}"; do
                printf '%s\t%s\t%s\n' "${name}" "${COMPONENT_STATE[${name}]}" "${PIDS[${name}]:-}" >> "${component_tmp}"
            done
            python3 - "${LAUNCHER_STATUS}" "${RUN_ID}" "${ACTUAL_HEAD}" "${component_tmp}" \
                      "${OPERATOR_APPROVAL_STATE}" "${SUPERVISOR_TERMINAL_STATE}" \
                      "${CLEANUP_RESULT}" "${RESIDUAL_RESULT}" "${FINAL_CLASSIFICATION}" <<'PYEOF'
import json
import os
import sys

(out_path, run_id, execution_head, component_tmp, operator_approval_state,
 supervisor_terminal_state, cleanup_result, residual_result, final_classification) = sys.argv[1:10]

components = {}
with open(component_tmp, encoding="utf-8") as f:
    for line in f:
        line = line.rstrip("\n")
        if not line:
            continue
        name, state, pid = line.split("\t")
        components[name] = {"launch_state": state, "pid": (int(pid) if pid else None), "exit_status": None}

status = {
    "run_id": run_id,
    "execution_head": execution_head,
    "components": components,
    "operator_approval_state": operator_approval_state,
    "supervisor_terminal_state": None if supervisor_terminal_state == "null" else supervisor_terminal_state,
    "cleanup": {"result": cleanup_result},
    "recorder_stopped_last": None,
    "residual_process_result": residual_result,
    "final_launcher_classification": final_classification,
}

tmp_path = out_path + ".tmp"
with open(tmp_path, "w", encoding="utf-8") as f:
    json.dump(status, f, indent=2)
os.replace(tmp_path, out_path)
PYEOF
            rm -f "${component_tmp}"
        }
        write_launcher_status

        cleanup() {
            local exit_code=$?
            echo "[$(date -Iseconds)] cleanup: stopping any started Stage 4 processes via run_hil_shutdown.sh (recorder last)"
            write_manifest
            if [[ -f "${PID_MANIFEST}" ]]; then
                bash "${SCRIPT_DIR}/run_hil_shutdown.sh" "${PID_MANIFEST}" || true
            fi
            # Exact-PID residual check only -- never pkill/name-based.
            residual="CLEAN"
            for name in "${!PIDS[@]}"; do
                pid="${PIDS[${name}]}"
                if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
                    residual="PROCESS_STILL_ALIVE:${name}:${pid}"
                fi
            done
            echo "{\"residual_process_check\": \"${residual}\"}" > "${OUT_DIR}/residual_check.json"
            echo "POST_RUN_RESIDUAL_PROCESS_CHECK=${residual}"
            RESIDUAL_RESULT="${residual}"
            CLEANUP_RESULT="RAN"
            FINAL_CLASSIFICATION="ABORTED"
            write_launcher_status
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
        write_launcher_status

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
        write_launcher_status

        echo "[$(date -Iseconds)] step 4: starting cooperative_avoider.py (real robot's frozen controller, UNMODIFIED, output remapped to ${RAW_CMD_VEL_TOPIC})"
        # cooperative_avoider.py subscribes to two HARDCODED relative
        # topic names -- "state" and "nav_intent" (no parameters exist
        # for either) -- caught by the Stage 4 live ROS-graph rehearsal:
        # without these remaps the node never sees the real robot's pose
        # or NavigationIntent, and stays at zero forever, silently.
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
        write_launcher_status

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
        write_launcher_status

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
            OPERATOR_APPROVAL_STATE="REJECTED"
            write_launcher_status
            echo "[run_hil_stage4_trial] Operator did not enter the exact required approval string. Aborting -- no supervisor started, no arm possible." >&2
            exit 3
        fi
        OPERATOR_APPROVAL_STATE="ACCEPTED"
        write_launcher_status

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
        write_launcher_status
        echo "[$(date -Iseconds)] step 7: post-start readiness/zero/publisher-count re-check"
        GUARDED_COUNT="$(publisher_count "${GUARDED_CMD_VEL_TOPIC}")"
        echo "post_start_publisher_count(${GUARDED_CMD_VEL_TOPIC})=${GUARDED_COUNT}"
        if [[ "${GUARDED_COUNT}" != "1" ]]; then
            echo "ABORT: expected exactly 1 publisher on ${GUARDED_CMD_VEL_TOPIC} (the guard) after start, found ${GUARDED_COUNT}." >&2
            exit 1
        fi
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
        FINAL_CLASSIFICATION="RUNNING_DETACHED"
        write_launcher_status

        echo "[$(date -Iseconds)] HIL_STAGE4_TRIAL_RUNNING evidence_dir=${OUT_DIR} run_id=${RUN_ID}"
        echo "Waiting for the supervisor to reach a terminal state (COMPLETE or FAILED)..."
        echo "This script does not arm anything itself and does not start a second trial."
        echo "Next step (after cleanup/Pi-evidence-transfer/measurements are done):"
        echo "  bash ${SCRIPT_DIR}/run_hil_stage4_trial.sh --finalize ${OUT_DIR}"
        trap - EXIT
        exit 0
        ;;
    --finalize)
        EVIDENCE_ROOT="${2:-}"
        if [[ -z "${EVIDENCE_ROOT}" ]]; then
            echo "ABORT: --finalize requires an evidence-root path argument" >&2
            echo "STAGE4_FINALIZE=INVALID_EVIDENCE" >&2
            exit 1
        fi
        if [[ ! -d "${EVIDENCE_ROOT}" ]]; then
            echo "ABORT: evidence root does not exist: ${EVIDENCE_ROOT}" >&2
            echo "STAGE4_FINALIZE=INVALID_EVIDENCE" >&2
            exit 1
        fi
        echo "[$(date -Iseconds)] MODE=--finalize evidence_root=${EVIDENCE_ROOT}"
        echo "Read-only except for hash-manifest/report files written inside evidence_root. No ROS, no Pi, no process started."

        # Fixed filenames within evidence_root -- the operator must have
        # already placed the Pi-side evidence and authored the physical
        # measurements JSON here (command sheet steps 14-15) before
        # calling --finalize.
        SUPERVISOR_EVIDENCE="${EVIDENCE_ROOT}/stage4_supervisor_evidence.jsonl"
        WSL_COMMAND_EVIDENCE="${EVIDENCE_ROOT}/command_evidence.csv"
        PID_MANIFEST="${EVIDENCE_ROOT}/pid_manifest.json"
        LAUNCHER_STATUS="${EVIDENCE_ROOT}/launcher_status.json"
        SOURCE_IDENTITY_MANIFEST="${EVIDENCE_ROOT}/source_identity_manifest.json"
        RESIDUAL_CHECK="${EVIDENCE_ROOT}/residual_check.json"
        PI_COMMAND_AUDIT="${EVIDENCE_ROOT}/pi_command_audit.jsonl"
        PI_VERIFIER_VERDICT="${EVIDENCE_ROOT}/pi_verifier_verdict.json"
        PHYSICAL_MEASUREMENTS="${EVIDENCE_ROOT}/physical_measurements.json"
        ADOPTION_EVIDENCE="${EVIDENCE_ROOT}/adoption_evidence.jsonl"
        HASH_MANIFEST="${EVIDENCE_ROOT}/SHA256SUMS.txt"
        POST_RUN_VERIFICATION="${EVIDENCE_ROOT}/post_run_verification.json"
        FINAL_HASH_MANIFEST="${EVIDENCE_ROOT}/FINAL_SHA256SUMS.txt"

        REQUIRED_FILES=(
            "${SUPERVISOR_EVIDENCE}" "${WSL_COMMAND_EVIDENCE}" "${PID_MANIFEST}"
            "${LAUNCHER_STATUS}" "${SOURCE_IDENTITY_MANIFEST}" "${RESIDUAL_CHECK}"
            "${PI_COMMAND_AUDIT}" "${PI_VERIFIER_VERDICT}" "${PHYSICAL_MEASUREMENTS}"
        )
        MISSING=0
        for f in "${REQUIRED_FILES[@]}"; do
            if [[ ! -s "${f}" ]]; then
                echo "MISSING_OR_EMPTY_REQUIRED_FILE: ${f}" >&2
                MISSING=1
            fi
        done
        if [[ "${MISSING}" -ne 0 ]]; then
            echo "STAGE4_FINALIZE=INVALID_EVIDENCE" >&2
            exit 1
        fi
        echo "all_required_files_present=true"

        # Extract adoption evidence from the supervisor's own evidence
        # JSONL (no separate recorder exists for /hil/adoption_evidence
        # itself -- the supervisor already records the exact adoption
        # fact, with its own raw payload, in its ADOPTION_CONFIRMED
        # record). Read-only extraction, never regenerated from a live
        # topic.
        python3 -c "
import json
with open('${SUPERVISOR_EVIDENCE}', encoding='utf-8') as f_in, open('${ADOPTION_EVIDENCE}', 'w', encoding='utf-8') as f_out:
    for line in f_in:
        line = line.strip()
        if not line:
            continue
        rec = json.loads(line)
        if rec.get('event') == 'ADOPTION_CONFIRMED':
            f_out.write(json.dumps(rec) + '\n')
"
        if [[ ! -s "${ADOPTION_EVIDENCE}" ]]; then
            echo "MISSING_OR_EMPTY_REQUIRED_FILE: ${ADOPTION_EVIDENCE} (no ADOPTION_CONFIRMED record found in supervisor evidence)" >&2
            echo "STAGE4_FINALIZE=INVALID_EVIDENCE" >&2
            exit 1
        fi
        echo "adoption_evidence_extracted=${ADOPTION_EVIDENCE}"

        echo "[$(date -Iseconds)] building SHA256SUMS.txt (excludes itself, post_run_verification.json, FINAL_SHA256SUMS.txt, temp files)"
        (
            cd "${EVIDENCE_ROOT}" && \
            find . -maxdepth 1 -type f \
                ! -name "SHA256SUMS.txt" \
                ! -name "FINAL_SHA256SUMS.txt" \
                ! -name "post_run_verification.json" \
                ! -name "*.tmp" ! -name "*.tmp.*" ! -name "*.new" -print0 \
                | sort -z | xargs -0 sha256sum > "SHA256SUMS.txt.new"
            mv -f "SHA256SUMS.txt.new" "SHA256SUMS.txt"
        )
        echo "[$(date -Iseconds)] verifying SHA256SUMS.txt"
        if ! (cd "${EVIDENCE_ROOT}" && sha256sum -c "SHA256SUMS.txt" > /dev/null); then
            echo "HASH_VERIFICATION_FAILED: SHA256SUMS.txt" >&2
            echo "STAGE4_FINALIZE=INVALID_EVIDENCE" >&2
            exit 1
        fi
        echo "STAGE4_HASH_STAGE_1=PASS"

        echo "[$(date -Iseconds)] invoking committed physical verifier"
        python3 "${SCRIPT_DIR}/hil_stage4_post_run_verifier.py" \
            --mode physical \
            --supervisor-evidence-path "${SUPERVISOR_EVIDENCE}" \
            --pid-manifest-path "${PID_MANIFEST}" \
            --hash-manifest-path "${HASH_MANIFEST}" \
            --evidence-dir "${EVIDENCE_ROOT}" \
            --residual-check-path "${RESIDUAL_CHECK}" \
            --physical-measurements-path "${PHYSICAL_MEASUREMENTS}" \
            --adoption-evidence-path "${ADOPTION_EVIDENCE}" \
            --wsl-command-evidence-path "${WSL_COMMAND_EVIDENCE}" \
            --pi-command-audit-path "${PI_COMMAND_AUDIT}" \
            --pi-verifier-verdict-path "${PI_VERIFIER_VERDICT}" \
            --source-identity-manifest-path "${SOURCE_IDENTITY_MANIFEST}" \
            --launcher-status-path "${LAUNCHER_STATUS}" \
            --bridge-status-path "${WSL_COMMAND_EVIDENCE}" \
            --report-path "${POST_RUN_VERIFICATION}"
        VERIFIER_EXIT=$?

        if [[ ! -s "${POST_RUN_VERIFICATION}" ]]; then
            echo "MISSING_OR_EMPTY_REQUIRED_FILE: ${POST_RUN_VERIFICATION}" >&2
            echo "STAGE4_FINALIZE=INVALID_EVIDENCE" >&2
            exit 1
        fi
        VERIFIER_CLASSIFICATION="$(python3 -c "import json; print(json.load(open('${POST_RUN_VERIFICATION}'))['classification'])" 2>/dev/null || echo "")"
        if [[ -z "${VERIFIER_CLASSIFICATION}" ]]; then
            echo "POST_RUN_VERIFICATION_UNPARSEABLE_OR_MISSING_CLASSIFICATION" >&2
            echo "STAGE4_FINALIZE=INVALID_EVIDENCE" >&2
            exit 1
        fi
        echo "post_run_verification_classification=${VERIFIER_CLASSIFICATION}"

        echo "[$(date -Iseconds)] building FINAL_SHA256SUMS.txt (all final evidence, including SHA256SUMS.txt and post_run_verification.json, excluding itself)"
        (
            cd "${EVIDENCE_ROOT}" && \
            find . -maxdepth 1 -type f \
                ! -name "FINAL_SHA256SUMS.txt" \
                ! -name "*.tmp" ! -name "*.tmp.*" ! -name "*.new" -print0 \
                | sort -z | xargs -0 sha256sum > "FINAL_SHA256SUMS.txt.new"
            mv -f "FINAL_SHA256SUMS.txt.new" "FINAL_SHA256SUMS.txt"
        )
        echo "[$(date -Iseconds)] verifying FINAL_SHA256SUMS.txt"
        if ! (cd "${EVIDENCE_ROOT}" && sha256sum -c "FINAL_SHA256SUMS.txt" > /dev/null); then
            echo "HASH_VERIFICATION_FAILED: FINAL_SHA256SUMS.txt" >&2
            echo "STAGE4_FINALIZE=INVALID_EVIDENCE" >&2
            exit 1
        fi
        echo "STAGE4_HASH_STAGE_2=PASS"

        echo "STAGE4_FINALIZE=${VERIFIER_CLASSIFICATION}"
        [[ "${VERIFIER_CLASSIFICATION}" == "PASS" ]] && exit 0 || exit "${VERIFIER_EXIT}"
        ;;
    *)
        echo "[run_hil_stage4_trial] Reason: mode '${MODE}' is not recognized. This launcher has no bypass." >&2
        exit 2
        ;;
esac
