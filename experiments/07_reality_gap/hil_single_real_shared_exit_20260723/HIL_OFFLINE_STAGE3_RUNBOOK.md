# HIL Offline Stage 3 Runbook

**Classification: `OFFLINE_INTEGRATION_VALIDATION`.** This is a hardware-free,
mixed-node ROS 2 validation of the message chain virtual scout ->
GoalAnnouncement -> adapter-hosted GoalNavigator reception/adoption ->
NavigationIntent -> requested cmd_vel -> guard evaluation -> guarded
(test-only) command -> evidence capture. It is explicitly **not**: a
physical HIL trial, a Webots navigation experiment, a physical
navigation trial, angular calibration, a dual-physical-robot
experiment, formal A-G or N2/N3 data, or evidence that physical
steering is safe. A successful run proves the software chain is wired
and behaves as designed in software only.

## Authoritative Git HEAD requirement

Before starting any Stage 3 run, confirm the working tree's HEAD is the
commit this runbook and its supporting tools were reviewed against
(`10ad3a9768369bb5163d778d095416dd9a3b4362` at the time this runbook was
first written, or a later HEAD that has been independently re-verified
to contain the same, unmodified `hil_virtual_peer.py`,
`goal_navigator.py`, `cooperative_avoider.py`, `hil_cmd_vel_guard.py`,
and `hil_command_evidence_recorder.py`). Do not proceed if any of those
five files differ from what this runbook assumes.

## Isolated topic table

All Stage 3 topics live under `/hil_offline_stage3/...`. None reuse a
production topic name.

| Purpose | Isolated topic | Message type |
|---|---|---|
| Own-robot test state | `/hil_offline_stage3/epuck1/state` | `EpuckState` |
| Virtual peer source state (published by `hil_virtual_peer.py`, unmodified) | `/hil_offline_stage3/virtual_peer/source_state` | `EpuckState` |
| Virtual peer state after the test gate (consumed by the guard) | `/hil_offline_stage3/virtual_peer/guard_input_state` | `EpuckState` |
| GoalAnnouncement | `/hil_offline_stage3/goal_announcement` | `GoalAnnouncement` |
| NavigationIntent | `/hil_offline_stage3/epuck1/nav_intent` | `NavigationIntent` |
| Requested (pre-guard) cmd_vel | `/hil_offline_stage3/cmd_vel_unguarded_test_only` | `Twist` |
| Guarded (post-guard) cmd_vel | `/hil_offline_stage3/cmd_vel_guarded_test_only` | `Twist` |
| Guard arm state | `/hil_offline_stage3/guard_arm_test_only` | `Bool` |
| Bridge-status substitute | `/hil_offline_stage3/bridge_status_test_only` | `std_msgs/String` (JSON `{"connected": bool, "rx_count": int}`) |
| Phase/gate/adoption/duplicate evidence events | `/hil_offline_stage3/phase_event_test_only` | `std_msgs/String` (JSON; keys among `phase`, `gate_open`, `gate_epoch`, `adoption_confirmed`, `duplicate_sent`, `duplicate_rejected`, `guard_blocked_reasons`) -- recorded in the CSV under the fixed pseudo-topic name `PHASE_EVENT`, not the real topic string, so the verifier can find every such event unambiguously regardless of the actual topic name used for a given run |
| Gate-owned structured forward-decision evidence | `/hil_offline_stage3/gate_decision_test_only` | `std_msgs/String` (JSON; one event per source message processed by the gate, emitted synchronously at the gate's own decision point -- keys: `event_type` (always `"GATE_DECISION"`), `gate_epoch`, `gate_state`, `source_protocol_version`, `source_robot_id`, `source_sequence`, `source_production_stamp_s`, `decision` (`"FORWARDED"` or `"REJECTED_GATE_CLOSED"`), `decision_timestamp_s`, `first_source_after_reopen`, `forwarded_destination_topic`) -- recorded in the CSV under the fixed pseudo-topic name `GATE_DECISION_EVENT` |
| Guarded (post-guard) cmd_vel, as observed by the harness's automatic runner for its own zero/bounded phase checks | `/hil_offline_stage3/cmd_vel_guarded_test_only` (same topic as above; the harness subscribes to it read-only, in addition to the guard publishing it) | `Twist` |

## ROS_DOMAIN_ID

**`ROS_DOMAIN_ID=91`** for any real Stage 3 run. Confirmed distinct from:
- `0` (ROS default / any physical process);
- `77` (Stage 2 hardware-free verification domain);
- `89` (the project's standing pytest-isolation domain, `src/epuck2_comm/test/conftest.py`'s `TEST_ROS_DOMAIN_ID`).

During Stage 3 *preparation* (writing/testing the tooling below, not a
real Stage 3 run), focused harness/gate tests use **`ROS_DOMAIN_ID=92`**,
and the limited recorder-verifier integration test (see below) uses
**`ROS_DOMAIN_ID=93`** -- both distinct from 0/77/89/91 and from each
other, so preparation-time test runs can never collide with a real
Stage 3 run's domain, or with each other, even if several existed at
once. The pre-existing Stage 2 adapter/adoption live test
(`test_hil_goal_announcement_adoption.py`) uses its own dedicated
**`ROS_DOMAIN_ID=90`**.

## ROS_LOCALHOST_ONLY

**Every ROS-based test in this directory (new Stage 3 preparation tests
and pre-existing Stage 1/Stage 2 live tests alike) must set
`ROS_LOCALHOST_ONLY=1` alongside its assigned domain.** Each live test's
`setUp()` asserts this explicitly and fails closed if it is unset --
this is not merely a documentation convention, it is enforced in code.
Example invocation form:

```bash
ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_offline_stage3_harness_live.py -v
ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_offline_stage3_recorder_verifier_integration.py -v
ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_goal_announcement_adoption.py -v
```

## Environment setup (per terminal session used for the real run)

```bash
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
for f in /opt/ros/*/setup.bash; do [ -f "$f" ] && source "$f" && break; done
source "$HOME/epuck_ws/install/setup.bash"
```

## Test-only angular bound disclaimer

Any angular-speed bound passed to `hil_cmd_vel_guard.py --max-angular-speed-rps`
for a Stage 3 run is a **`TEST_ONLY_SOFTWARE_BOUND_NOT_A_PHYSICAL_LIMIT`**
(see `hil_offline_stage3_harness.py`'s constant of that name). It must
never be copied into `hil_frozen_params.json` and must never be cited
as a ground-contact angular calibration result.

## Evidence directory convention

One directory per run, named `hil_offline_stage3_<RUN_ID>/`, containing
`evidence.csv` (written by `hil_offline_stage3_evidence_recorder.py`),
`summary.json` (same tool), and `post_run_verification.json` (written
by `hil_offline_stage3_post_run_verifier.py`). No raw evidence from this
directory is ever committed to the repository, matching the existing
project convention for native/raw evidence.

## Synthetic clear-sensor fixture (own-state, harness-generated only)

`hil_offline_stage3_harness.py`'s own-state publish sets
`validity_flags=7` (`FLAG_ODOM_VALID|FLAG_IR_VALID|FLAG_TOF_VALID`).
Every `EpuckState` field `cooperative_avoider.py`/`local_obstacle_logic.py`
read under that flag combination (`front_distance_m`, `left_distance_m`,
`right_distance_m`, `left_front_m`, `left_mid_m`, `left_rear_m`,
`right_front_m`, `right_mid_m`, `right_rear_m`) is therefore explicitly
set to positive infinity (`EpuckState.msg`'s own documented "+Inf means
no valid return within range" convention), never left at the ROS
float32 implicit default of `0.0` -- which `decide_local_obstacle()`
would otherwise read as a genuine obstacle at zero distance, locking
`cooperative_avoider` permanently into `LOCAL_FRONT_DANGER` and
preventing it from ever reaching its NavigationIntent-driven cruise/CPA
command. This is `SYNTHETIC_CLEAR_SENSOR_FIXTURE` / `TEST_ONLY` /
`NOT_A_PHYSICAL_MEASUREMENT` -- never a physical sensor reading, never
citable as one. See `apply_synthetic_clear_sensor_fixture()` in
`hil_offline_stage3_harness.py`.

## Exact executable launch/shutdown/verification script

Every process is launched in its own background job within the one
script below (equivalently, in its own terminal window, still capturing
`$!` immediately after each launch). PIDs are captured **only** via
direct `$!` immediately after backgrounding each process -- never via
`pgrep -f`. `pgrep -af` (always the full-command form) appears only in
two places: the pre-run forbidden-process check and the post-run
residual-process check, both strictly **read-only diagnostics** -- their
output is never used to choose what gets signalled. `cooperative_avoider`
is launched by directly invoking its resolved, installed executable
(`$(ros2 pkg prefix epuck2_comm)/lib/epuck2_comm/cooperative_avoider`,
confirmed to be the `epuck2_comm.cooperative_avoider:main` entry point
declared in `src/epuck2_comm/setup.py:29`, and confirmed live to run as
exactly one OS process for its entire lifetime) rather than via `ros2
run`, specifically to avoid the wrapper/child ambiguity `ros2 run`
introduces in this installation. Its cleanup identity (PID, PGID,
`/proc/<pid>/stat` start time, `/proc/<pid>/exe`) is captured immediately
after launch and re-verified before every signal -- cleanup NEVER
signals a PID or process group discovered only by name matching, and
never kills an unrelated process merely because it shares the
`cooperative_avoider` executable name.

`RUN_ID` and `OUT_DIR` are deliberately **not** generated by this
document -- they are supplied by a separate, later, explicitly
authorised execution-preparation step. Every other control parameter
below is already exact; nothing else may remain a placeholder.

```bash
#!/usr/bin/env bash
# HIL Stage 3 OFFLINE_INTEGRATION_VALIDATION launcher.
# Deliberately NOT `set -e`: the verifier's own nonzero exit (a real,
# valid TASK_OUTCOME!=SUCCESS result) must never abort this script
# before its own exit code is captured and evaluated -- see the
# "Verifier execution" step below, which relies on plain `>`
# redirection followed immediately by `$?`, both of which work exactly
# as expected without `-e` in the way.
set -uo pipefail

# --- RUN_ID / OUT_DIR: supplied by a separate, later-authorised step ---
# RUN_ID="<supplied by a separate, explicitly authorised execution-preparation step>"
# OUT_DIR="<HIL_ROOT>/hil_offline_stage3_${RUN_ID}"
# if [[ -e "${OUT_DIR}" ]]; then
#     echo "ABORT: ${OUT_DIR} already exists -- refusing to reuse an evidence directory." >&2
#     exit 1
# fi
# mkdir -p "${OUT_DIR}"

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/tools"
cd "${TOOLS_DIR}"

for f in /opt/ros/*/setup.bash; do [ -f "${f}" ] && source "${f}" && break; done
source "${HOME}/epuck_ws/install/setup.bash"

export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1

# --- Resolve the real, installed cooperative_avoider executable via the
# standard, committed, reproducible ROS 2 package-prefix mechanism
# (matches `ament_index_python.packages.get_package_prefix`, confirmed
# live to return the identical path). Never `ros2 run` for this
# process: `ros2 run epuck2_comm cooperative_avoider` was live-tested
# (this same authorised test suite) to fork a separate child process
# for the real node while the `ros2 run` CLI wrapper can exit on its
# own -- an unowned-child ambiguity a direct executable invocation does
# not have (confirmed live: exactly one OS process for its entire
# lifetime). ---
COOP_PREFIX="$(ros2 pkg prefix epuck2_comm)"
COOP_EXE="${COOP_PREFIX}/lib/epuck2_comm/cooperative_avoider"
if [[ ! -x "${COOP_EXE}" ]]; then
    echo "ABORT: resolved cooperative_avoider executable is missing or not executable: ${COOP_EXE}" >&2
    exit 1
fi

NS="/hil_offline_stage3"
OWN_STATE_TOPIC="${NS}/epuck1/state"
VP_SOURCE_TOPIC="${NS}/virtual_peer/source_state"
VP_GATE_INPUT_TOPIC="${NS}/virtual_peer/guard_input_state"
GOAL_ANNOUNCEMENT_TOPIC="${NS}/goal_announcement"
NAV_INTENT_TOPIC="${NS}/epuck1/nav_intent"
REQUESTED_CMD_VEL_TOPIC="${NS}/cmd_vel_unguarded_test_only"
GUARDED_CMD_VEL_TOPIC="${NS}/cmd_vel_guarded_test_only"
ARM_TOPIC="${NS}/guard_arm_test_only"
BRIDGE_STATUS_TOPIC="${NS}/bridge_status_test_only"
PHASE_EVENT_TOPIC="${NS}/phase_event_test_only"
GATE_DECISION_TOPIC="${NS}/gate_decision_test_only"

TEST_ONLY_ANGULAR_BOUND_RPS="3.0"   # TEST_ONLY_SOFTWARE_BOUND_NOT_A_PHYSICAL_LIMIT -- copied from the already-reviewed, committed, executed FullBehaviouralIntegrationTest / hil_offline_stage3_harness.py's own --runner-angular-bound-rps default (3.0), never from hil_frozen_params.json (whose hil_guard_limits.max_angular_speed_rps remains UNCONFIRMED_PHYSICAL_MEASUREMENT)
TEST_ONLY_LINEAR_BOUND_MPS="0.02"   # matches hil_cmd_vel_guard.py's own --max-linear-speed-mps default AND hil_frozen_params.json's confirmed (non-UNCONFIRMED) hil_guard_limits.max_linear_speed_mps

RECORDER_LOG="${OUT_DIR}/recorder.log"
GUARD_LOG="${OUT_DIR}/guard.log"
ADAPTER_LOG="${OUT_DIR}/adapter.log"
COOP_LOG="${OUT_DIR}/cooperative_avoider.log"
HARNESS_LOG="${OUT_DIR}/harness.log"
PEER_LOG="${OUT_DIR}/virtual_peer.log"
EVIDENCE_CSV="${OUT_DIR}/evidence.csv"
SUMMARY_JSON="${OUT_DIR}/summary.json"
VERIFIER_JSON="${OUT_DIR}/post_run_verification.json"
VERIFIER_EXIT_FILE="${OUT_DIR}/verifier_exit_status.txt"
PID_MANIFEST="${OUT_DIR}/pid_manifest.json"
SHA256SUMS_FILE="${OUT_DIR}/SHA256SUMS.txt"

RECORDER_PID=""
GUARD_PID=""
ADAPTER_PID=""
COOP_PID=""
COOP_PGID=""
COOP_START_TIME=""
COOP_EXE_PATH=""
COOP_CLEANUP_CLASSIFICATION="normal"
HARNESS_PID=""
PEER_PID=""

wait_for_log_pattern() {
    local logfile="$1" pattern="$2" timeout_s="$3"
    local start="${SECONDS}"
    while true; do
        if grep -qF -- "${pattern}" "${logfile}" 2>/dev/null; then
            return 0
        fi
        if (( SECONDS - start >= timeout_s )); then
            echo "READINESS_TIMEOUT(${timeout_s}s): pattern not found in ${logfile}" >&2
            return 1
        fi
        sleep 0.2
    done
}

# --- Exact owned-process identity helpers for cooperative_avoider only.
# Never used for any other process in this script (the other five are
# launched directly via `python3 ...&`, so their $! already identifies
# the single real process with no wrapper/child ambiguity, and their
# cleanup already uses that exact PID -- see cleanup() below). ---
_proc_start_time() {
    local pid="$1" stat_content after_comm
    stat_content="$(cat "/proc/${pid}/stat" 2>/dev/null)" || { echo ""; return 1; }
    after_comm="${stat_content##*)}"
    local fields
    read -r -a fields <<< "${after_comm}"
    # fields[0] is process state (overall field 3); start time is
    # overall field 22, i.e. index 19 in this post-comm split.
    echo "${fields[19]:-}"
}

_proc_exe_path() {
    readlink -f "/proc/$1/exe" 2>/dev/null
}

_owned_identity_still_matches() {
    local pid="$1" pgid="$2" start_time="$3" exe_path="$4"
    [[ -d "/proc/${pid}" ]] || return 1
    local cur_start cur_exe cur_pgid
    cur_start="$(_proc_start_time "${pid}")"
    cur_exe="$(_proc_exe_path "${pid}")"
    cur_pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
    [[ -n "${cur_start}" && "${cur_start}" == "${start_time}" ]] || return 1
    [[ -n "${cur_exe}" && "${cur_exe}" == "${exe_path}" ]] || return 1
    [[ -n "${cur_pgid}" && "${cur_pgid}" == "${pgid}" ]] || return 1
    return 0
}

# Signals ONLY the exact owned process group identified by
# (pid, pgid, start_time, exe_path) -- after re-verifying, immediately
# before signalling, that this is still the same process this script
# itself started (guards against PID reuse). Never uses `pgrep`/`pkill`
# or any other name-based discovery to pick a signal target. If the
# identity no longer matches (already exited, or -- vanishingly
# unlikely -- reused), this does nothing rather than risk touching an
# unrelated process. Sets COOP_CLEANUP_CLASSIFICATION=abnormal if
# SIGKILL escalation was required. Returns nonzero only if the owned
# process could not be confirmed stopped after the full sequence.
terminate_owned_process_group() {
    local pid="$1" pgid="$2" start_time="$3" exe_path="$4" label="$5"
    local sigint_wait_s="${6:-10}" sigkill_wait_s="${7:-5}"
    if [[ -z "${pid}" ]]; then
        return 0
    fi
    if ! _owned_identity_still_matches "${pid}" "${pgid}" "${start_time}" "${exe_path}"; then
        return 0
    fi
    kill -INT -- "-${pgid}" 2>/dev/null || true
    local waited=0
    while _owned_identity_still_matches "${pid}" "${pgid}" "${start_time}" "${exe_path}"; do
        if (( waited >= sigint_wait_s )); then
            break
        fi
        sleep 1
        waited=$(( waited + 1 ))
    done
    if _owned_identity_still_matches "${pid}" "${pgid}" "${start_time}" "${exe_path}"; then
        echo "CLEANUP_ESCALATION: ${label} pid=${pid} pgid=${pgid} did not exit after SIGINT -- escalating to SIGKILL on the same verified owned process group (ABNORMAL cleanup)" >&2
        COOP_CLEANUP_CLASSIFICATION="abnormal"
        kill -KILL -- "-${pgid}" 2>/dev/null || true
        sleep "${sigkill_wait_s}"
    fi
    if _owned_identity_still_matches "${pid}" "${pgid}" "${start_time}" "${exe_path}"; then
        echo "CLEANUP_FAILED_TO_EXIT: ${label} pid=${pid} pgid=${pgid} (verified owned process group, after SIGINT+SIGKILL)" >&2
        return 1
    fi
    return 0
}

# --- Step 0: forbidden-process check (pattern excludes this script's
# own invocation by matching only the target executables, never a
# shell/pytest/grep/pgrep invocation string) ---
if pgrep -af 'webots-bin|[c]ooperative_avoider|state_publisher|hil_cmd_vel_guard\.py|hil_topic_adapter\.py|hil_offline_stage3_|hil_virtual_peer\.py|ros2 bag record|piserver|pi_driver|epuck_bridge'; then
    echo "ABORT: a forbidden process is already running." >&2
    exit 1
fi
echo "PRE_RUN_FORBIDDEN_PROCESS_CHECK=CLEAN"

# --- Forbidden-topic check ---
for t in /cmd_vel /cmd_vel_unguarded /epuck1/state /epuck_bridge/status /hil_guard/arm; do
    if ros2 topic list 2>/dev/null | grep -qx "${t}"; then
        echo "ABORT: forbidden production topic ${t} already exists on this domain." >&2
        exit 1
    fi
done
echo "PRE_RUN_FORBIDDEN_TOPIC_CHECK=CLEAN"

# --- Abort-safe cleanup, installed BEFORE the first long-running
# process starts. Stops only PIDs this script itself captured via $!
# (each variable starts empty; a still-empty variable is simply
# skipped, so a startup failure before every process exists never
# targets an unrelated, accidentally-reused PID). Reverse launch order.
# Recorder always stops last, even if some later process never
# started, because RECORDER_PID is checked and killed in its own final
# step regardless of how far launch progressed. Runs on normal
# completion (EXIT) and on interrupt/termination/error.
#
# COOP_PID/COOP_PGID/COOP_START_TIME/COOP_EXE_PATH are the exact owned
# identity captured immediately after cooperative_avoider's direct
# executable launch (Step 4 below) -- never a name-discovered PID.
# cleanup() re-verifies that identity (PID exists, start time matches,
# resolved exe matches, PGID matches) immediately before signalling, via
# terminate_owned_process_group(); it never uses `pgrep`/`pkill` to pick
# a signal target, and never touches an unrelated process even if it
# happens to share the same executable name.
cleanup() {
    echo "[cleanup] stopping any processes this run actually started, reverse order"
    local failed=0
    for name_pid in "PEER_PID:virtual_peer" "HARNESS_PID:harness"; do
        local var="${name_pid%%:*}" label="${name_pid##*:}"
        local pid="${!var}"
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            kill -INT "${pid}"
            local waited=0
            while kill -0 "${pid}" 2>/dev/null; do
                if (( waited >= 10 )); then
                    echo "CLEANUP_FAILED_TO_EXIT: ${label} pid=${pid}" >&2
                    failed=1
                    break
                fi
                sleep 1
                waited=$(( waited + 1 ))
            done
        fi
    done
    # cooperative_avoider: exact owned-process-group cleanup only --
    # never name-based -- placed here to preserve reverse launch order
    # (peer -> harness -> cooperative_avoider -> adapter -> guard ->
    # recorder). See terminate_owned_process_group() above.
    if ! terminate_owned_process_group \
        "${COOP_PID}" "${COOP_PGID}" "${COOP_START_TIME}" "${COOP_EXE_PATH}" "cooperative_avoider"
    then
        failed=1
    fi
    for name_pid in "ADAPTER_PID:adapter" "GUARD_PID:guard"; do
        local var="${name_pid%%:*}" label="${name_pid##*:}"
        local pid="${!var}"
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            kill -INT "${pid}"
            local waited=0
            while kill -0 "${pid}" 2>/dev/null; do
                if (( waited >= 10 )); then
                    echo "CLEANUP_FAILED_TO_EXIT: ${label} pid=${pid}" >&2
                    failed=1
                    break
                fi
                sleep 1
                waited=$(( waited + 1 ))
            done
        fi
    done
    if [[ -n "${RECORDER_PID}" ]] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
        kill -INT "${RECORDER_PID}"
        local waited=0
        while kill -0 "${RECORDER_PID}" 2>/dev/null; do
            if (( waited >= 10 )); then
                echo "CLEANUP_FAILED_TO_EXIT: recorder pid=${RECORDER_PID}" >&2
                failed=1
                break
            fi
            sleep 1
            waited=$(( waited + 1 ))
        done
    fi
    echo "COOP_CLEANUP_CLASSIFICATION=${COOP_CLEANUP_CLASSIFICATION}"
    if pgrep -af 'webots-bin|[c]ooperative_avoider|state_publisher|hil_cmd_vel_guard\.py|hil_topic_adapter\.py|hil_offline_stage3_|hil_virtual_peer\.py|ros2 bag record|piserver|pi_driver|epuck_bridge'; then
        echo "POST_RUN_RESIDUAL_PROCESS_CHECK=FAIL" >&2
        failed=1
    else
        echo "POST_RUN_RESIDUAL_PROCESS_CHECK=CLEAN"
    fi
    return "${failed}"
}
trap cleanup EXIT INT TERM

# --- Step 1: evidence recorder (started first) ---
python3 hil_offline_stage3_evidence_recorder.py \
    --own-state-topic "${OWN_STATE_TOPIC}" \
    --virtual-peer-source-topic "${VP_SOURCE_TOPIC}" \
    --virtual-peer-guard-input-topic "${VP_GATE_INPUT_TOPIC}" \
    --goal-announcement-topic "${GOAL_ANNOUNCEMENT_TOPIC}" \
    --nav-intent-topic "${NAV_INTENT_TOPIC}" \
    --requested-cmd-vel-topic "${REQUESTED_CMD_VEL_TOPIC}" \
    --guarded-cmd-vel-topic "${GUARDED_CMD_VEL_TOPIC}" \
    --arm-topic "${ARM_TOPIC}" \
    --bridge-status-topic "${BRIDGE_STATUS_TOPIC}" \
    --phase-event-topic "${PHASE_EVENT_TOPIC}" \
    --gate-decision-topic "${GATE_DECISION_TOPIC}" \
    --output-csv "${EVIDENCE_CSV}" \
    --output-summary-json "${SUMMARY_JSON}" \
    > "${RECORDER_LOG}" 2>&1 &
RECORDER_PID="$!"
wait_for_log_pattern "${RECORDER_LOG}" "HIL_OFFLINE_STAGE3_EVIDENCE_RECORDER_READY output_csv=${EVIDENCE_CSV}" 15

# --- Step 2: guard (DISARMED by default) ---
python3 hil_cmd_vel_guard.py \
    --upstream-cmd-vel-topic "${REQUESTED_CMD_VEL_TOPIC}" \
    --guarded-cmd-vel-topic "${GUARDED_CMD_VEL_TOPIC}" \
    --arm-topic "${ARM_TOPIC}" \
    --physical-state-topic "${OWN_STATE_TOPIC}" \
    --virtual-peer-topic "${VP_GATE_INPUT_TOPIC}" \
    --require-virtual-peer \
    --max-linear-speed-mps "${TEST_ONLY_LINEAR_BOUND_MPS}" \
    --max-angular-speed-rps "${TEST_ONLY_ANGULAR_BOUND_RPS}" \
    --heartbeat-timeout-s 0.5 \
    --physical-state-timeout-s 0.5 \
    --virtual-peer-timeout-s 1.0 \
    --required-validity-flags 7 \
    > "${GUARD_LOG}" 2>&1 &
GUARD_PID="$!"
wait_for_log_pattern "${GUARD_LOG}" "HIL_CMD_VEL_GUARD_READY armed=False (DISARMED by default) max_linear_speed_mps=${TEST_ONLY_LINEAR_BOUND_MPS} max_angular_speed_rps=${TEST_ONLY_ANGULAR_BOUND_RPS}" 15

# --- Step 3: adapter-hosted navigator ---
python3 hil_topic_adapter.py \
    --robot-id 1 \
    --state-topic "${OWN_STATE_TOPIC}" \
    --nav-intent-topic "${NAV_INTENT_TOPIC}" \
    --mode search \
    --waypoints "0.0:0.0,1.0:1.0" \
    --waypoint-arrival-radius 0.10 \
    --rate-hz 2.0 \
    --goal-announcement-topic "${GOAL_ANNOUNCEMENT_TOPIC}" \
    --nominal-speed-mps 0.05 \
    --exit-center-x 5.0 --exit-center-y 5.0 --exit-radius 0.1 \
    --parking-x 9.0 --parking-y 9.0 --parking-radius 0.1 \
    --goal-hold-time-s 1.0 \
    > "${ADAPTER_LOG}" 2>&1 &
ADAPTER_PID="$!"
wait_for_log_pattern "${ADAPTER_LOG}" "goal_navigator READY robot_id=1 mode=search target=(0.0, 0.0) announce=False listens_for_announcement=True exit_region=(5.0000,5.0000,r=0.1000) parking_region=(9.0000,9.0000,r=0.1000) goal_hold_time_s=1.000" 15

# --- Step 4: cooperative_avoider (real, unmodified). Launched by
# directly invoking the resolved installed executable (COOP_EXE, never
# `ros2 run`), with `set -m` enabled ONLY for this one launch so bash
# job control assigns the background job its own process group
# (PGID == its own PID -- confirmed live) instead of inheriting this
# script's own process group; `set +m` immediately restores normal
# (non-job-control) semantics for the rest of the script. Its exact
# owned identity (PID, PGID, /proc start time, /proc/<pid>/exe) is
# captured immediately afterward, for terminate_owned_process_group() to
# re-verify before ever signalling it. ---
set -m
"${COOP_EXE}" --ros-args \
    -r "state:=${OWN_STATE_TOPIC}" \
    -r "cmd_vel:=${REQUESTED_CMD_VEL_TOPIC}" \
    -r "nav_intent:=${NAV_INTENT_TOPIC}" \
    -p "peer_state_topic:=${VP_GATE_INPUT_TOPIC}" \
    -p robot_id:=1 \
    -p armed:=true \
    -p enable_peer_avoidance:=true \
    -p enable_dynamic_heading:=true \
    -p enable_dynamic_speed:=true \
    -p enable_local_avoidance:=true \
    -p require_local_sensors:=true \
    -p use_sim_time:=false \
    -p nominal_speed_mps:=0.05 \
    -p safety_radius_m:=0.14 \
    -p stop_after_recovery:=false \
    > "${COOP_LOG}" 2>&1 &
COOP_PID="$!"
set +m
COOP_PGID="$(ps -o pgid= -p "${COOP_PID}" | tr -d ' ')"
COOP_START_TIME="$(_proc_start_time "${COOP_PID}")"
COOP_EXE_PATH="$(_proc_exe_path "${COOP_PID}")"
wait_for_log_pattern "${COOP_LOG}" "robot=1 peer=${VP_GATE_INPUT_TOPIC} armed=True heading=0.000rad" 15
ros2 node list 2>/dev/null | grep -qx '/cooperative_avoider'

# --- Step 5: Stage 3 harness (--auto-run is mandatory; the committed
# Stage3AutomaticRunner design is the only authorised orchestration
# mechanism for a real run). Duplicate overrides match the accepted
# duplicate-identity contract exactly: same source robot (2), same
# goal_id ("shared_exit"), same coordinates (2.0,3.0) as the virtual
# peer's own original announcement. ---
python3 hil_offline_stage3_harness.py \
    --own-state-topic "${OWN_STATE_TOPIC}" \
    --bridge-status-topic "${BRIDGE_STATUS_TOPIC}" \
    --arm-topic "${ARM_TOPIC}" \
    --phase-event-topic "${PHASE_EVENT_TOPIC}" \
    --goal-announcement-topic "${GOAL_ANNOUNCEMENT_TOPIC}" \
    --virtual-peer-source-topic "${VP_SOURCE_TOPIC}" \
    --virtual-peer-guard-input-topic "${VP_GATE_INPUT_TOPIC}" \
    --gate-decision-topic "${GATE_DECISION_TOPIC}" \
    --nav-intent-topic "${NAV_INTENT_TOPIC}" \
    --guarded-cmd-vel-topic "${GUARDED_CMD_VEL_TOPIC}" \
    --own-robot-id 1 --own-x-m 0.0 --own-y-m 0.0 --own-yaw-rad 0.0 \
    --max-runtime-s 60.0 \
    --auto-run \
    --runner-per-phase-timeout-s 20.0 \
    --runner-overall-timeout-s 55.0 \
    --runner-linear-bound-mps "${TEST_ONLY_LINEAR_BOUND_MPS}" \
    --runner-angular-bound-rps "${TEST_ONLY_ANGULAR_BOUND_RPS}" \
    --runner-peer-timeout-s 1.2 \
    --runner-duplicate-source-robot-id 2 \
    --runner-duplicate-goal-x-m 2.0 \
    --runner-duplicate-goal-y-m 3.0 \
    --runner-duplicate-goal-id shared_exit \
    > "${HARNESS_LOG}" 2>&1 &
HARNESS_PID="$!"
wait_for_log_pattern "${HARNESS_LOG}" "HIL_OFFLINE_STAGE3_HARNESS_READY phase=INITIALISING own_state_topic=${OWN_STATE_TOPIC} virtual_peer_guard_input_topic=${VP_GATE_INPUT_TOPIC} gate_decision_topic=${GATE_DECISION_TOPIC}" 15

# --- Step 6: virtual peer (target == start position) ---
python3 hil_virtual_peer.py \
    --robot-id 2 \
    --state-topic "${VP_SOURCE_TOPIC}" \
    --announcement-topic "${GOAL_ANNOUNCEMENT_TOPIC}" \
    --goal-id shared_exit \
    --start-x-m 2.0 --start-y-m 3.0 --start-yaw-rad 0.0 \
    --target-x-m 2.0 --target-y-m 3.0 \
    --cruise-linear-mps 0.05 --arrival-radius-m 0.5 \
    --max-angular-rps 0.2 --rate-hz 20 \
    > "${PEER_LOG}" 2>&1 &
PEER_PID="$!"
wait_for_log_pattern "${PEER_LOG}" "HIL_VIRTUAL_PEER_READY namespace=epuck_virtual_peer announce_enabled=True" 15

echo "{\"recorder\": ${RECORDER_PID}, \"guard\": ${GUARD_PID}, \"adapter\": ${ADAPTER_PID}, \"cooperative_avoider\": {\"pid\": ${COOP_PID}, \"pgid\": ${COOP_PGID}, \"start_time\": \"${COOP_START_TIME}\", \"exe_path\": \"${COOP_EXE_PATH}\"}, \"harness\": ${HARNESS_PID}, \"virtual_peer\": ${PEER_PID}}" > "${PID_MANIFEST}"

# --- Wait for the harness's own Stage3AutomaticRunner to reach COMPLETE
# (its own max-runtime-s watchdog aborts the process on timeout; this
# loop simply waits for the harness process to exit on its own). ---
wait "${HARNESS_PID}"
HARNESS_EXIT=$?
echo "HARNESS_EXIT=${HARNESS_EXIT}"

# --- Verifier execution. Plain `>` redirection: `$?` immediately
# afterward is this command's own exit code, unaffected by `pipefail`
# (no pipe is used) and never skipped, because this script does not use
# `set -e`. ---
python3 hil_offline_stage3_post_run_verifier.py \
    --csv "${EVIDENCE_CSV}" --summary-json "${SUMMARY_JSON}" \
    --test-only-angular-bound-rps "${TEST_ONLY_ANGULAR_BOUND_RPS}" \
    --test-only-linear-bound-mps "${TEST_ONLY_LINEAR_BOUND_MPS}" \
    > "${VERIFIER_JSON}"
VERIFIER_EXIT=$?
echo "${VERIFIER_EXIT}" > "${VERIFIER_EXIT_FILE}"
echo "VERIFIER_EXIT=${VERIFIER_EXIT}"

# --- JSON output validation (never converts a verifier failure into
# shell success: VERIFIER_EXIT is preserved and inspected separately
# below, DATA_VALIDITY/TASK_OUTCOME are read from the file regardless
# of VERIFIER_EXIT). ---
if [[ ! -s "${VERIFIER_JSON}" ]]; then
    echo "ABORT: ${VERIFIER_JSON} missing or empty." >&2
    exit 1
fi
python3 -c "
import json, sys
with open('${VERIFIER_JSON}', encoding='utf-8') as f:
    result = json.load(f)
print('DATA_VALIDITY=' + str(result['DATA_VALIDITY']))
print('TASK_OUTCOME=' + str(result['TASK_OUTCOME']))
"

# --- Evidence hashing: only after every writer (recorder, guard,
# adapter, cooperative_avoider, harness, virtual peer, verifier) has
# already stopped and every file is closed. SHA256SUMS.txt does NOT
# hash itself -- it is generated last, from the files that exist at
# that point, and is never included in its own input list. ---
( cd "${OUT_DIR}" && sha256sum \
    "$(basename "${EVIDENCE_CSV}")" \
    "$(basename "${SUMMARY_JSON}")" \
    "$(basename "${VERIFIER_JSON}")" \
    "$(basename "${VERIFIER_EXIT_FILE}")" \
    "$(basename "${PID_MANIFEST}")" \
    "$(basename "${RECORDER_LOG}")" "$(basename "${GUARD_LOG}")" \
    "$(basename "${ADAPTER_LOG}")" "$(basename "${COOP_LOG}")" \
    "$(basename "${HARNESS_LOG}")" "$(basename "${PEER_LOG}")" \
    > "$(basename "${SHA256SUMS_FILE}")" )

exit "${VERIFIER_EXIT}"
```

## Forbidden-process and forbidden-topic checks

- **Pre-run**: the exact `pgrep -af` pattern in the script above must report nothing (it matches only the target executables -- `[c]ooperative_avoider` and the other bracket-first-letter forms are the standard shell idiom that prevents the `pgrep`/`grep` invocation itself from matching its own search pattern).
- **Forbidden-topic check**: before step 1, the script confirms via `ros2 topic list` that none of `/cmd_vel`, `/cmd_vel_unguarded`, `/epuck1/state`, `/epuck_bridge/status`, `/hil_guard/arm` already exist on the bus under this domain.
- **Post-run**: the same `pgrep -af` pattern (inside `cleanup()`) must report nothing; confirm via `git status`/`git diff` that no repository file changed during the run.
- **Operational-topic isolation**: every topic variable in the script above (`OWN_STATE_TOPIC` through `GATE_DECISION_TOPIC`) is built from `NS="/hil_offline_stage3"`; no launch argument or remap destination anywhere in the script uses `/cmd_vel`, `/cmd_vel_unguarded`, `/epuck1/state`, `/epuck_bridge/status`, or `/hil_guard/arm` -- those five strings appear in this document only in this prohibition section and the forbidden-topic/forbidden-process checks, never as an executable argument.

## Abort conditions

Any physical/Pi/bridge/Webots process detected at any point; any
publisher/subscriber on a real production topic; any node reporting
`ROS_DOMAIN_ID` other than `91`; any topic/type mismatch against the
isolated topic table; guard output outside the declared test-only
bound; more than one adoption event; missing expected evidence row/log
line; residual process after shutdown; unexpected repository
modification; any readiness `wait_for_log_pattern` timeout (15s per
process); any `CLEANUP_FAILED_TO_EXIT` report.

## Automatic orchestration runner

`hil_offline_stage3_harness.py`'s `Stage3AutomaticRunner` drives the
harness through all 11 `Stage3Phase` values end-to-end automatically,
using only the harness's own observable evidence (adoption confirmation
from the real `NavigationIntent` stream, the guarded-cmd-vel topic's own
zero/bounded values, the gate's own forwarding evidence) -- it contains
no navigation, GoalAnnouncement-acceptance, guard-decision, or
virtual-peer motion logic of its own. It enforces a per-phase timeout
and an overall run timeout (`RunnerTimeoutError`, fail-closed), never
skips or repeats a phase (delegated entirely to the existing
`PhaseMachine`), and never permits any action after `COMPLETE`
(delegated to the existing `DuplicateAnnouncementController`). Invoke it
by passing `--auto-run` (plus `--runner-*` tuning flags) to
`hil_offline_stage3_harness.py`; without `--auto-run`, the harness
behaves exactly as before, waiting for an external caller to drive
`advance_phase()`/`close_gate()`/etc.

## Duplicate-announcement ordering (enforced internally, not left to the caller)

`hil_offline_stage3_harness.py`'s `DuplicateAnnouncementController` enforces,
inside the harness itself: the duplicate `GoalAnnouncement` may only be
published after exactly one adoption rising-edge has been observed on
the `NavigationIntent` stream; a second call after the first successful
publication always fails closed (`DuplicateOrderingError`); a call
before adoption or after the run reaches `COMPLETE` always fails closed;
and a second adoption rising-edge (which the frozen navigation logic's
own idempotent latch should never actually produce) aborts the run
(`AdoptionCountExceededError`). No external orchestration script is
relied upon to get this order right.

## DATA_VALIDITY / TASK_OUTCOME separation

`DATA_VALIDITY` (infrastructure/measurement question, computed first,
independently) covers: file existence/non-emptiness, topic/type
contract, sanctioned `ROS_DOMAIN_ID`, presence of every required
evidence stream (including the gate-decision topic), own-state
`validity_flags==7` throughout, monotonic timestamps, no production
topic in the contract, no residual process, the `[PEER_GATE_CLOSED,
PEER_GATE_REOPENED)` boundary-event proof (both events exist exactly
once, close strictly precedes reopen, the source topic continued
publishing during the interval, at least one gate-input row exists
before closure), and gate-decision-event **structural** well-formedness
(every event has a matched `event_type`/`decision`/`gate_epoch`/finite
`decision_timestamp_s`, every `FORWARDED` event's
`forwarded_destination_topic` matches the real gate-input topic, every
`REJECTED_GATE_CLOSED` event carries none).

`TASK_OUTCOME` (only meaningful when `DATA_VALIDITY=VALID`) is one of a
**non-physical** taxonomy -- `SUCCESS`, `GUARD_BOUND_VIOLATION`,
`STALE_ZERO_FAILURE`, `RECOVERY_FAILURE`, `ADOPTION_FAILURE`,
`DUPLICATE_HANDLING_FAILURE`, `GATE_FORWARDING_FAILURE`,
`BACKLOG_REPLAY_DETECTED`, `NOT_EVALUABLE` -- never a physical-safety
word like "UNSAFE_FAILURE". Every verifier result additionally carries
`result_type="OFFLINE_SOFTWARE_CONTRACT_RESULT"` and an explicit
`physical_claim` disclaimer stating this is not evidence of physical
collision or physical unsafe motion. Checks: exactly one announcement
accepted/adopted; exactly one duplicate successfully sent (not merely
attempted); all guarded commands within the declared test-only bound;
`STALE_ZERO_CONFIRMED` timed strictly after the peer timeout has
elapsed past closure and strictly before reopening; `RECOVERY_CONFIRMED`
timed strictly after the first fresh, gate-decision-proven post-reopen
forward; clean completion within the bounded runtime; and the **strict
first-post-reopen forwarding contract**, proven exclusively from the
gate's own `GATE_DECISION_EVENT` rows (`evaluate_gate_forwarding_outcome()`
in `hil_offline_stage3_post_run_verifier.py`) -- never by comparing this
recorder's rows for the gate-input topic against its rows for the
source topic, since those are two independently-scheduled subscriber
callbacks with no guaranteed relative ordering:
- the events span at least two gate epochs (a reopen actually occurred);
- every event recorded while the gate was `CLOSED` has
  `decision=REJECTED_GATE_CLOSED` (none `FORWARDED`), and at least one
  such event exists (the source kept being processed while closed);
- exactly one event in the final (post-reopen) epoch is marked
  `first_source_after_reopen=true`, and its decision is `FORWARDED`;
- no `FORWARDED` event in the post-reopen epoch carries a
  `source_sequence` less than or equal to the highest `source_sequence`
  already seen while the gate was `CLOSED` -- a violation of this rule
  is reported as its own, more specific `BACKLOG_REPLAY_DETECTED`
  outcome (using the message's own sequence number, never local receipt
  time, which can never distinguish a replayed old message from a fresh
  one); any other contract violation is `GATE_FORWARDING_FAILURE`.

A valid data chain with a failed task outcome (e.g. a guarded command
exceeding the test-only bound, reported as `GUARD_BOUND_VIOLATION`) is a
real, valid failure and must be reported as such -- never hidden,
retried automatically, or reclassified.

## Recorder-verifier integration test (preparation-time only, not the final graph)

`test_hil_offline_stage3_recorder_verifier_integration.py` starts only
the evidence recorder plus plain stimulus publishers (never
`cooperative_avoider`, never the real adapter/virtual-peer/guard graph
together, never a bridge) under topics namespaced
`/hil_offline_stage3_preparation_test/...` and
`ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1`, writes evidence to a
`tempfile.TemporaryDirectory` (deleted automatically), and confirms the
post-run verifier reads the produced files and reports
`DATA_VALIDITY=VALID`/`TASK_OUTCOME=SUCCESS` for a synthetic,
textbook-correct scenario that includes: one original `GoalAnnouncement`
and exactly one adoption event; one duplicate `GoalAnnouncement`
publication and one recorded duplicate-rejection event, correctly
ordered (duplicate-sent strictly after adoption, duplicate-rejected
strictly after duplicate-sent); gate-decision events proving the source
continued to be processed and rejected while closed and the first
post-reopen message was forwarded, with no backlog replay. This proves
the recorder and verifier work together end-to-end; it is not, and must
never be cited as, a real Stage 3 run.

## No automatic progression

A completed Stage 3 run does not authorize, and must never be cited
as, Stage 4, any physical work, angular calibration, or HIL field
geometry population. Each of those requires its own separate,
explicit operator authorization.
