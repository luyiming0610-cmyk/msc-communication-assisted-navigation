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
source /opt/ros/humble/setup.bash
source /home/eamon/epuck_ws/install/setup.bash
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
residual-process check (both inside `_forbidden_process_scan()`, called
once before launch and once inside `cleanup()`), both strictly
**read-only diagnostics** -- their output is never used to choose what
gets signalled, and `_forbidden_process_scan()` additionally excludes
this script's own PID from the match set before the emptiness test, so
it cannot ever match its own invocation regardless of where this script
happens to be staged. `cooperative_avoider` is launched by directly
invoking its resolved, installed executable
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
`cooperative_avoider` executable name. The other five processes are
launched directly via `python3 ...&`, so their `$!` is already the
exact, unambiguous owned process; their cleanup uses that exact PID
directly (`terminate_owned_pid()`), with a bounded SIGINT wait and
SIGKILL escalation limited to that same exact PID, never a
name-discovered one.

The script uses **`set -Eeuo pipefail`** (fail-fast): a readiness-gate
timeout, a node-graph mismatch, a source/HEAD/hash-identity failure, a
forbidden-process/topic detection, or an executable-resolution failure
stops startup immediately -- it can never fall through to a later
launch step. Every command whose nonzero status is meaningful and must
be recorded (the harness wait, cleanup, verifier execution, JSON
validation, hashing) is wrapped in an explicit `if cmd; then ... else
... fi` (or `cmd || true`) specifically so `-e` never discards that
status before it is captured; a bare `cmd; STATUS=$?` would abort at
`cmd` itself; the following line would never run.

Signal/exit handling is separated and idempotent: `on_int()` and
`on_term()` each disable all three traps first (preventing recursion),
run cleanup exactly once (`run_cleanup_once()`'s own `CLEANUP_DONE`
guard), and end with an explicit `exit 130` / `exit 143` respectively,
so the shell never resumes normal script execution after an
interrupt/termination signal. `on_exit()` likewise disables the traps,
runs cleanup at most once, and preserves the script's own exit status
unless cleanup itself introduced a new failure into an otherwise
successful run (in which case the cleanup failure is what gets
reported, never silently swallowed as a false success).

`RUN_ID` and `EXPECTED_HEAD` are **execution identity values, not
experiment-control parameters** -- both are REQUIRED at runtime from the
environment (`EXPECTED_HEAD=<40-hex-char-commit> RUN_ID=<run-id> bash
execution_script.sh`), supplied by a separate, later, explicitly
authorised execution-preparation step, and never embedded as a literal
inside this file. A tracked file cannot correctly predict its own
post-commit identity -- a hardware-free execution attempt against this
runbook's own previous revision exposed exactly this defect (the
script's own embedded `EXPECTED_HEAD` necessarily referred to the
*parent* of the commit that introduced it, so it could never pass on
its own introducing commit). Every required source/tool/message path
Stage 3 depends on -- including this runbook's own path -- is instead
verified through a real git blob comparison, computed fresh at run time
against the externally-supplied `EXPECTED_HEAD`, never a SHA-256 frozen
inside the file. Once frozen for a specific execution attempt, neither
value is ever regenerated or reused for a different run. `REPO_ROOT` and
`TOOLS_DIR` remain frozen absolute paths, never derived from wherever
this script happens to be staged when it runs (`BASH_SOURCE[0]` is used
only once, to copy the script itself into the evidence root for
preservation -- never to locate the repository or its tools). Every
other control parameter below is already exact; nothing else may remain
a placeholder.

```bash
#!/usr/bin/env bash
# HIL Stage 3 OFFLINE_INTEGRATION_VALIDATION launcher.
# Fail-fast: `set -Eeuo pipefail`. A readiness-gate failure, a
# source/hash/HEAD mismatch, a forbidden-process/topic detection, or an
# executable-resolution failure must stop startup immediately, never
# fall through to the next launch step. Every command whose nonzero
# status is meaningful and must be recorded (harness wait, cleanup,
# verifier execution, JSON validation, hashing) is wrapped in an
# explicit `if cmd; then ... else ... fi` (or `cmd || true`) so `-e`
# never discards that status before it is captured -- a bare
# `cmd; STATUS=$?` would abort at `cmd` itself before the second line
# ever ran.
set -Eeuo pipefail

# --- Repository identity: a frozen absolute path, never derived from
# the location of an extracted/staged copy of this script
# (BASH_SOURCE[0] here is used ONLY to preserve a copy of the script
# itself further below, never to locate the repo or its tools). ---
REPO_ROOT="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm"
TOOLS_DIR="${REPO_ROOT}/experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools"
HIL_ROOT="/home/eamon/epuck_comm_bags"

# ============================================================
# Step 1a: source-identity verification. No literal 40-character
# commit id or SHA-256 of any tracked file (least of all this
# runbook itself) is embedded anywhere in this script -- a tracked
# file cannot correctly predict its own post-commit identity, so any
# such embedded value would necessarily go stale the instant this
# file is next edited (this is exactly the defect a real, hardware-free
# execution attempt exposed). EXPECTED_HEAD and RUN_ID are instead
# REQUIRED as externally-supplied values, frozen by a separate, later,
# explicitly authorised execution-preparation step -- never generated,
# guessed, or silently substituted by this script.
# ============================================================

: "${EXPECTED_HEAD:?EXPECTED_HEAD must be supplied externally, e.g. EXPECTED_HEAD=<40-hex-char-commit> RUN_ID=<run-id> bash execution_script.sh -- refusing to embed or guess one}"

if [[ ! "${EXPECTED_HEAD}" =~ ^[0-9a-f]{40}$ ]]; then
    echo "ABORT: EXPECTED_HEAD is not a well-formed 40-character lowercase-hex commit id: '${EXPECTED_HEAD}'" >&2
    exit 1
fi
if [[ ! -d "${REPO_ROOT}" ]]; then
    echo "ABORT: REPO_ROOT does not exist: ${REPO_ROOT}" >&2
    exit 1
fi
if ! git -C "${REPO_ROOT}" cat-file -e "${EXPECTED_HEAD}^{commit}" 2>/dev/null; then
    echo "ABORT: EXPECTED_HEAD does not exist as a commit in this repository: ${EXPECTED_HEAD}" >&2
    exit 1
fi
ACTUAL_HEAD="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
if [[ "${ACTUAL_HEAD}" != "${EXPECTED_HEAD}" ]]; then
    echo "ABORT: HEAD mismatch (expected ${EXPECTED_HEAD}, got ${ACTUAL_HEAD})" >&2
    exit 1
fi
GIT_VERSION="$(git --version | awk '{print $3}')"

# --- Step A: Git source-path identity (proven complete dependency closure,
# 23 paths -- see dependency-closure audit accompanying this revision;
# setup.py added because it is the tracked file that DEFINES the
# cooperative_avoider console_scripts mapping -- deployment/entry-point
# metadata, not a Python runtime import). ---
REQUIRED_SOURCE_PATHS=(
    "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/HIL_OFFLINE_STAGE3_RUNBOOK.md"
    "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_offline_stage3_harness.py"
    "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_offline_stage3_evidence_recorder.py"
    "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_offline_stage3_post_run_verifier.py"
    "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_cmd_vel_guard.py"
    "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_topic_adapter.py"
    "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_virtual_peer.py"
    "experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/hil_goal_announcement_evidence.py"
    "experiments/10_cooperative_exit_navigation_20260720/tools/goal_navigator.py"
    "experiments/10_cooperative_exit_navigation_20260720/tools/goal_hold_tracker.py"
    "experiments/10_cooperative_exit_navigation_20260720/tools/navigation_target_state.py"
    "src/epuck2_comm/epuck2_comm/__init__.py"
    "src/epuck2_comm/epuck2_comm/cooperative_avoider.py"
    "src/epuck2_comm/epuck2_comm/command_smoothing.py"
    "src/epuck2_comm/epuck2_comm/collision_math.py"
    "src/epuck2_comm/epuck2_comm/local_obstacle_logic.py"
    "src/epuck2_comm/epuck2_comm/models.py"
    "src/epuck2_comm/epuck2_comm/neighbor_cache.py"
    "src/epuck2_comm/epuck2_comm/transmission_policy.py"
    "src/epuck2_comm/setup.py"
    "src/epuck2_comm_interfaces/msg/EpuckState.msg"
    "src/epuck2_comm_interfaces/msg/GoalAnnouncement.msg"
    "src/epuck2_comm_interfaces/msg/NavigationIntent.msg"
)

declare -A EXPECTED_BLOB_BY_PATH=()
SOURCE_IDENTITY_ROWS=()
SOURCE_IDENTITY_OK=1
for src_path in "${REQUIRED_SOURCE_PATHS[@]}"; do
    full_path="${REPO_ROOT}/${src_path}"
    if [[ ! -e "${full_path}" ]]; then
        echo "ABORT: required source path missing from working tree: ${src_path}" >&2
        SOURCE_IDENTITY_OK=0
        SOURCE_IDENTITY_ROWS+=("${src_path}"$'\t'"MISSING"$'\t'"MISSING"$'\t'"MISSING"$'\t'"FAIL")
        continue
    fi
    if ! expected_blob="$(git -C "${REPO_ROOT}" rev-parse --verify -q "${EXPECTED_HEAD}:${src_path}")"; then
        echo "ABORT: required path does not exist in EXPECTED_HEAD's tree: ${src_path}" >&2
        SOURCE_IDENTITY_OK=0
        SOURCE_IDENTITY_ROWS+=("${src_path}"$'\t'"MISSING_IN_COMMIT"$'\t'"-"$'\t'"-"$'\t'"FAIL")
        continue
    fi
    EXPECTED_BLOB_BY_PATH["${src_path}"]="${expected_blob}"
    # Path-aware: attribute lookup (.gitattributes eol/text rules) is keyed
    # off the explicit --path value, never inferred from cwd or the literal
    # argv path alone -- correct even if REPO_ROOT is reached via a
    # different-cased or symlinked route than the one .gitattributes was
    # authored against.
    worktree_blob="$(git -C "${REPO_ROOT}" hash-object --path="${src_path}" -- "${full_path}")"
    raw_sha256="$(sha256sum -- "${full_path}" | awk '{print $1}')"
    if [[ "${expected_blob}" != "${worktree_blob}" ]]; then
        echo "ABORT: source-identity mismatch for ${src_path} (expected blob ${expected_blob}, worktree blob ${worktree_blob})" >&2
        SOURCE_IDENTITY_OK=0
        SOURCE_IDENTITY_ROWS+=("${src_path}"$'\t'"${expected_blob}"$'\t'"${worktree_blob}"$'\t'"${raw_sha256}"$'\t'"FAIL")
        continue
    fi
    SOURCE_IDENTITY_ROWS+=("${src_path}"$'\t'"${expected_blob}"$'\t'"${worktree_blob}"$'\t'"${raw_sha256}"$'\t'"PASS")
done
if [[ "${SOURCE_IDENTITY_OK}" -ne 1 ]]; then
    echo "ABORT: one or more required source paths failed identity verification." >&2
    exit 1
fi
for src_path in "${REQUIRED_SOURCE_PATHS[@]}"; do
    if ! git -C "${REPO_ROOT}" diff --cached --quiet -- "${src_path}"; then
        echo "ABORT: staged content change detected for required path: ${src_path}" >&2
        exit 1
    fi
done
echo "GIT_SOURCE_IDENTITY_CHECK=CLEAN"

# --- Step B: installed-runtime identity (ties the Git-verified source above
# to what ~/epuck_ws/install actually contains and what the ROS executable
# actually loads -- Git source verification alone proves nothing about a
# stale colcon build). Read-only: no rclpy.init, no node creation, no
# process launch, no rebuild/reinstall. ---
: "${INSTALL_ROOT:?INSTALL_ROOT must be supplied externally, e.g. INSTALL_ROOT=/home/eamon/epuck_ws/install}"
if [[ ! -d "${INSTALL_ROOT}" ]]; then
    echo "ABORT: INSTALL_ROOT does not exist: ${INSTALL_ROOT}" >&2
    exit 1
fi

PY_SITE="${INSTALL_ROOT}/epuck2_comm/lib/python3.10/site-packages/epuck2_comm"
MSG_SHARE="${INSTALL_ROOT}/epuck2_comm_interfaces/share/epuck2_comm_interfaces/msg"
LAUNCHER="${INSTALL_ROOT}/epuck2_comm/lib/epuck2_comm/cooperative_avoider"

# Installed Python modules that are byte-for-byte tied to a specific
# REQUIRED_SOURCE_PATHS entry via the SAME path-aware git-blob identity
# used in Step A, never raw byte/sha256 equality -- a colcon-built install
# tree on Linux and a WSL/Windows-mounted worktree can legitimately differ
# only by CRLF/LF representation while being logically identical. Raw
# sha256 is recorded for audit only, never used for PASS/FAIL.
declare -A INSTALLED_PY_TO_SOURCE=(
    ["${PY_SITE}/__init__.py"]="src/epuck2_comm/epuck2_comm/__init__.py"
    ["${PY_SITE}/cooperative_avoider.py"]="src/epuck2_comm/epuck2_comm/cooperative_avoider.py"
    ["${PY_SITE}/command_smoothing.py"]="src/epuck2_comm/epuck2_comm/command_smoothing.py"
    ["${PY_SITE}/collision_math.py"]="src/epuck2_comm/epuck2_comm/collision_math.py"
    ["${PY_SITE}/local_obstacle_logic.py"]="src/epuck2_comm/epuck2_comm/local_obstacle_logic.py"
    ["${PY_SITE}/models.py"]="src/epuck2_comm/epuck2_comm/models.py"
    ["${PY_SITE}/neighbor_cache.py"]="src/epuck2_comm/epuck2_comm/neighbor_cache.py"
    ["${PY_SITE}/transmission_policy.py"]="src/epuck2_comm/epuck2_comm/transmission_policy.py"
    ["${MSG_SHARE}/EpuckState.msg"]="src/epuck2_comm_interfaces/msg/EpuckState.msg"
    ["${MSG_SHARE}/GoalAnnouncement.msg"]="src/epuck2_comm_interfaces/msg/GoalAnnouncement.msg"
    ["${MSG_SHARE}/NavigationIntent.msg"]="src/epuck2_comm_interfaces/msg/NavigationIntent.msg"
)

INSTALLED_RUNTIME_ROWS=()
INSTALLED_RUNTIME_OK=1

for installed_path in "${!INSTALLED_PY_TO_SOURCE[@]}"; do
    src_path="${INSTALLED_PY_TO_SOURCE[${installed_path}]}"
    if [[ ! -e "${installed_path}" ]]; then
        echo "ABORT: installed runtime artifact missing: ${installed_path}" >&2
        INSTALLED_RUNTIME_OK=0
        INSTALLED_RUNTIME_ROWS+=("${installed_path}"$'\t'"MISSING"$'\t'"copy_or_symlink"$'\t'"MISSING"$'\t'"blob_vs_${src_path}"$'\t'"FAIL")
        continue
    fi
    resolved_type="copy"
    resolved_target="-"
    if [[ -L "${installed_path}" ]]; then
        resolved_type="symlink"
        resolved_target="$(readlink -f "${installed_path}")"
    fi
    installed_sha256="$(sha256sum -- "${installed_path}" | awk '{print $1}')"
    installed_blob="$(git -C "${REPO_ROOT}" hash-object --path="${src_path}" -- "${installed_path}")"
    expected_blob="${EXPECTED_BLOB_BY_PATH[${src_path}]}"
    if [[ "${installed_blob}" != "${expected_blob}" ]]; then
        echo "ABORT: installed runtime artifact does not match reviewed source: ${installed_path} (tied to ${src_path})" >&2
        INSTALLED_RUNTIME_OK=0
        INSTALLED_RUNTIME_ROWS+=("${installed_path}"$'\t'"${resolved_target}"$'\t'"${resolved_type}"$'\t'"${installed_sha256}"$'\t'"blob_vs_${src_path}"$'\t'"FAIL")
        continue
    fi
    INSTALLED_RUNTIME_ROWS+=("${installed_path}"$'\t'"${resolved_target}"$'\t'"${resolved_type}"$'\t'"${installed_sha256}"$'\t'"blob_vs_${src_path}"$'\t'"PASS")
done

if [[ "${INSTALLED_RUNTIME_OK}" -ne 1 ]]; then
    echo "INSTALLED_RUNTIME_IDENTITY=BLOCKED" >&2
    exit 1
fi
echo "INSTALLED_RUNTIME_MODULES_CHECK=CLEAN"

# --- Step B2: installed launcher structural identity (fail-closed AST
# comparison against a single, narrowly-defined accepted structural
# template -- never mere presence of a load_entry_point(...) substring).
# No launcher execution occurs. ---
EXPECTED_DIST="epuck2-comm==0.1.0"
EXPECTED_GROUP="console_scripts"
EXPECTED_NAME="cooperative_avoider"

if [[ ! -e "${LAUNCHER}" ]]; then
    echo "ABORT: installed cooperative_avoider launcher missing: ${LAUNCHER}" >&2
    echo "INSTALLED_LAUNCHER_IDENTITY=BLOCKED" >&2
    exit 1
fi
LAUNCHER_SHA256="$(sha256sum -- "${LAUNCHER}" | awk '{print $1}')"

LAUNCHER_AST_RESULT="$(python3 - "${LAUNCHER}" "${EXPECTED_DIST}" "${EXPECTED_GROUP}" "${EXPECTED_NAME}" <<'PYEOF'
import ast
import sys

launcher_path, dist, group, name = sys.argv[1:5]

GOLDEN_TEMPLATE = '''
import re
import sys

__requires__ = {dist!r}

try:
    from importlib.metadata import distribution
except ImportError:
    try:
        from importlib_metadata import distribution
    except ImportError:
        from pkg_resources import load_entry_point


def importlib_load_entry_point(spec, group, name):
    dist_name, _, _ = spec.partition('==')
    matches = (
        entry_point
        for entry_point in distribution(dist_name).entry_points
        if entry_point.group == group and entry_point.name == name
    )
    return next(matches).load()


globals().setdefault('load_entry_point', importlib_load_entry_point)


if __name__ == '__main__':
    sys.argv[0] = re.sub(r'(-script\\.pyw?|\\.exe)?$', '', sys.argv[0])
    sys.exit(load_entry_point({dist!r}, {group!r}, {name!r})())
'''.format(dist=dist, group=group, name=name)


def normalize(source):
    return ast.dump(ast.parse(source), annotate_fields=False)


with open(launcher_path, "r", encoding="utf-8") as f:
    installed_source = f.read()

try:
    installed_norm = normalize(installed_source)
except SyntaxError as e:
    print(f"BLOCKED:not_valid_python:{e}")
    sys.exit(1)

golden_norm = normalize(GOLDEN_TEMPLATE)
if installed_norm != golden_norm:
    print("BLOCKED:structural_mismatch")
    sys.exit(1)

print("PASS")
sys.exit(0)
PYEOF
)"
LAUNCHER_AST_EXIT=$?
if [[ "${LAUNCHER_AST_EXIT}" -ne 0 ]]; then
    echo "ABORT: installed launcher failed structural identity check: ${LAUNCHER_AST_RESULT}" >&2
    echo "INSTALLED_LAUNCHER_IDENTITY=BLOCKED" >&2
    exit 1
fi
echo "INSTALLED_LAUNCHER_IDENTITY_CHECK=CLEAN"

# --- Step B3: installed entry-point metadata agreement. Requires all
# three of (a) the committed setup.py at EXPECTED_HEAD, (b) the installed
# distribution's own entry_points.txt (or equivalent), and (c) the
# structurally-verified launcher above, to agree on the SAME mapping. ---
if ! grep -q 'cooperative_avoider = epuck2_comm.cooperative_avoider:main' "${REPO_ROOT}/src/epuck2_comm/setup.py"; then
    echo "ABORT: committed setup.py no longer declares the expected cooperative_avoider console_scripts entry point" >&2
    echo "INSTALLED_LAUNCHER_IDENTITY=BLOCKED" >&2
    exit 1
fi

# Null-delimited, path-safe discovery -- requires EXACTLY one candidate.
# Never selects the first glob result: a zero-candidate or
# multiple-candidate outcome is itself a fail-closed identity failure,
# not merely a "pick one and hope" situation.
ENTRY_POINTS_CANDIDATES=()
while IFS= read -r -d '' candidate; do
    ENTRY_POINTS_CANDIDATES+=("${candidate}")
done < <(
    {
        find "${INSTALL_ROOT}/epuck2_comm/lib/python3.10/site-packages" -maxdepth 2 -iname 'entry_points.txt' -path '*epuck2_comm-*.egg-info/*' -print0 2>/dev/null
        find "${INSTALL_ROOT}/epuck2_comm/lib/python3.10/site-packages" -maxdepth 2 -iname 'entry_points.txt' -path '*epuck2_comm-*.dist-info/*' -print0 2>/dev/null
    }
)
ENTRY_POINTS_CANDIDATE_COUNT="${#ENTRY_POINTS_CANDIDATES[@]}"
if [[ "${ENTRY_POINTS_CANDIDATE_COUNT}" -eq 0 ]]; then
    echo "ABORT: INSTALLED_ENTRYPOINT_METADATA_NOT_FOUND" >&2
    echo "INSTALLED_LAUNCHER_IDENTITY=BLOCKED" >&2
    exit 1
fi
if [[ "${ENTRY_POINTS_CANDIDATE_COUNT}" -gt 1 ]]; then
    echo "ABORT: INSTALLED_ENTRYPOINT_METADATA_AMBIGUOUS(count=${ENTRY_POINTS_CANDIDATE_COUNT})" >&2
    echo "INSTALLED_LAUNCHER_IDENTITY=BLOCKED" >&2
    exit 1
fi
ENTRY_POINTS_METADATA="${ENTRY_POINTS_CANDIDATES[0]}"
ENTRY_POINTS_SHA256="$(sha256sum -- "${ENTRY_POINTS_METADATA}" | awk '{print $1}')"
if ! grep -q '^cooperative_avoider = epuck2_comm.cooperative_avoider:main$' "${ENTRY_POINTS_METADATA}"; then
    echo "ABORT: installed entry_points.txt does not map cooperative_avoider to epuck2_comm.cooperative_avoider:main: ${ENTRY_POINTS_METADATA}" >&2
    echo "INSTALLED_LAUNCHER_IDENTITY=BLOCKED" >&2
    exit 1
fi
echo "INSTALLED_ENTRYPOINT_METADATA_CHECK=CLEAN"

# --- Step C: runtime message-schema verification (read-only class
# introspection via get_fields_and_field_types() -- no rclpy.init(), no
# node, no process). ROS environment must be sourced narrowly for this
# import to succeed; `set -u` is lifted only for the two `source` calls,
# exactly as already accepted for the later real launch. ---
set +u
source /opt/ros/humble/setup.bash
source /home/eamon/epuck_ws/install/setup.bash
set -u

MSG_SCHEMA_RESULT="$(INSTALL_ROOT="${INSTALL_ROOT}" python3 - \
    "${REPO_ROOT}/src/epuck2_comm_interfaces/msg/EpuckState.msg" \
    "${REPO_ROOT}/src/epuck2_comm_interfaces/msg/GoalAnnouncement.msg" \
    "${REPO_ROOT}/src/epuck2_comm_interfaces/msg/NavigationIntent.msg" <<'PYEOF'
import json
import os
import re
import sys

TYPE_MAP = {
    "float32": "float",
    "float64": "double",
    "bool": "boolean",
}


def parse_msg_file(path, msg_name):
    fields = []
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            m = re.match(r"^([A-Za-z0-9_/]+)\s+([A-Za-z0-9_]+)\s*(=.*)?$", line)
            if not m:
                continue
            ftype, fname, is_const = m.group(1), m.group(2), m.group(3)
            if is_const:
                continue  # constants are not fields
            fields.append((fname, TYPE_MAP.get(ftype, ftype)))
    return fields


MSG_PATHS = {
    "EpuckState": sys.argv[1],
    "GoalAnnouncement": sys.argv[2],
    "NavigationIntent": sys.argv[3],
}
INSTALL_ROOT = os.environ["INSTALL_ROOT"]

results = {}
overall_ok = True
for msg_name, msg_path in MSG_PATHS.items():
    expected_fields = parse_msg_file(msg_path, msg_name)
    try:
        module = __import__("epuck2_comm_interfaces.msg", fromlist=[msg_name])
        cls = getattr(module, msg_name)
    except Exception as e:
        results[msg_name] = {"result": "FAIL", "reason": f"IMPORT_FAILED:{e}"}
        overall_ok = False
        continue

    resolved_file = getattr(sys.modules.get(cls.__module__), "__file__", None)
    if resolved_file is None or not os.path.realpath(resolved_file).startswith(os.path.realpath(INSTALL_ROOT)):
        results[msg_name] = {
            "result": "FAIL",
            "reason": f"RESOLVED_OUTSIDE_INSTALL_ROOT:{resolved_file}",
            "resolved_module_file": resolved_file,
        }
        overall_ok = False
        continue

    actual_fields_dict = cls.get_fields_and_field_types()
    actual_fields = list(actual_fields_dict.items())

    if actual_fields == expected_fields:
        results[msg_name] = {
            "result": "PASS",
            "resolved_module_file": resolved_file,
            "expected_fields": expected_fields,
            "actual_fields": actual_fields,
        }
        continue

    expected_names = [n for n, _ in expected_fields]
    actual_names = [n for n, _ in actual_fields]
    missing = [n for n in expected_names if n not in actual_names]
    extra = [n for n in actual_names if n not in expected_names]
    reason = "FIELD_MISMATCH"
    if missing:
        reason = f"MISSING_FIELDS:{missing}"
    elif extra:
        reason = f"EXTRA_FIELDS:{extra}"
    elif actual_names != expected_names:
        reason = "FIELD_ORDER_MISMATCH"
    else:
        reason = "FIELD_TYPE_MISMATCH"
    results[msg_name] = {
        "result": "FAIL",
        "reason": reason,
        "resolved_module_file": resolved_file,
        "expected_fields": expected_fields,
        "actual_fields": actual_fields,
    }
    overall_ok = False

print(json.dumps({"overall_result": "PASS" if overall_ok else "FAIL", "messages": results}))
sys.exit(0 if overall_ok else 1)
PYEOF
)"
MSG_SCHEMA_EXIT=$?
if [[ "${MSG_SCHEMA_EXIT}" -ne 0 ]]; then
    echo "${MSG_SCHEMA_RESULT}" >&2
    echo "ABORT: runtime message-schema verification failed for one or more messages." >&2
    echo "MESSAGE_SCHEMA_IDENTITY=BLOCKED" >&2
    exit 1
fi
echo "MESSAGE_SCHEMA_IDENTITY_CHECK=CLEAN"
echo "PRE_RUN_SOURCE_IDENTITY_CHECK=CLEAN"

# RUN_ID is required and validated only AFTER all four identity
# layers (Git source paths, installed runtime modules, installed
# launcher/entry-point metadata, runtime message-schema) have already
# passed -- a failed identity check must never consume, cancel, or
# even generate a RUN_ID, and must never create OUT_DIR.
: "${RUN_ID:?RUN_ID must be supplied externally by the same execution-preparation step that supplies EXPECTED_HEAD, only after all source/installed identity checks have already passed}"

OUT_DIR="${HIL_ROOT}/hil_offline_stage3_${RUN_ID}"

cd "${REPO_ROOT}"
cd "${TOOLS_DIR}"

# `set -u` is lifted only for these two `source` calls: ROS 2's own
# setup scripts reference internal variables (e.g.
# AMENT_TRACE_SETUP_FILES) without a safe default and abort under
# nounset -- confirmed live. Restored immediately afterward.
# ROS was already sourced earlier (Step C, message-schema verification)
# -- not re-sourced here.

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

# --- Self-match-safe forbidden-process scan. `_forbidden_process_scan`
# excludes this script's own PID ($$) from the match set BEFORE the
# emptiness test, by construction -- not merely by the bracket-quoting
# idiom (`[c]ooperative_avoider` etc., which only protects the pgrep
# invocation's own argv from matching its own search pattern, not this
# script's separate invocation path). With `pipefail`, the pipeline's
# exit status is 0 only when pgrep itself found at least one process;
# when pgrep finds nothing at all, the pipeline itself is nonzero, so
# the `VAR="$(...)" && [[ -n "${VAR}" ]]` guard below is never even
# reached in the common "nothing running" case, and in the case where
# the only raw match is this script's own PID, awk filters it to an
# empty string, so the emptiness test correctly finds nothing to abort
# on. pgrep is used here ONLY as a read-only detector -- its output
# never selects a signal target anywhere in this script. ---
SELF_PID=$$
FORBIDDEN_PROCESS_PATTERN='webots-bin|[c]ooperative_avoider|state_publisher|hil_cmd_vel_guard\.py|hil_topic_adapter\.py|hil_offline_stage3_|hil_virtual_peer\.py|ros2 bag record|piserver|pi_driver|epuck_bridge'
_forbidden_process_scan() {
    pgrep -af "${FORBIDDEN_PROCESS_PATTERN}" 2>/dev/null | awk -v self="${SELF_PID}" '$1 != self'
}

if FORBIDDEN_MATCHES="$(_forbidden_process_scan)" && [[ -n "${FORBIDDEN_MATCHES}" ]]; then
    echo "${FORBIDDEN_MATCHES}"
    echo "ABORT: a forbidden process is already running." >&2
    exit 1
fi
echo "PRE_RUN_FORBIDDEN_PROCESS_CHECK=CLEAN"

for t in /cmd_vel /cmd_vel_unguarded /epuck1/state /epuck_bridge/status /hil_guard/arm; do
    if ros2 topic list 2>/dev/null | grep -qx "${t}"; then
        echo "ABORT: forbidden production topic ${t} already exists on this domain." >&2
        exit 1
    fi
done
echo "PRE_RUN_FORBIDDEN_TOPIC_CHECK=CLEAN"

# ============================================================
# Step 2: create the evidence root (only after preflight passes).
# ============================================================

if [[ -e "${OUT_DIR}" ]]; then
    echo "ABORT: ${OUT_DIR} already exists -- refusing to reuse an evidence directory." >&2
    exit 1
fi
mkdir -p "${OUT_DIR}"

EXECUTION_SCRIPT_COPY="${OUT_DIR}/execution_script.sh"
cp -- "${BASH_SOURCE[0]}" "${EXECUTION_SCRIPT_COPY}"

# --- Source-identity manifest: persists the exact two-tier verification
# computed during Step 1 (before OUT_DIR existed) -- Git source-path
# records and installed-runtime records are kept in separate arrays and
# distinct manifest sections; neither is used to infer the other. Never
# re-verified here, only written now that an authorised evidence root
# exists. ---
SOURCE_IDENTITY_MANIFEST_JSON="${OUT_DIR}/source_identity_manifest.json"
SOURCE_IDENTITY_ROWS_TSV="$(printf '%s\n' "${SOURCE_IDENTITY_ROWS[@]}")"
INSTALLED_RUNTIME_ROWS_TSV="$(printf '%s\n' "${INSTALLED_RUNTIME_ROWS[@]}")"
DEPENDENCY_CLOSURE_TSV="$(printf '%s\n' "${REQUIRED_SOURCE_PATHS[@]}")"
{
printf '%s\n' "${SOURCE_IDENTITY_ROWS_TSV}"
printf '===INSTALLED===\n'
printf '%s\n' "${INSTALLED_RUNTIME_ROWS_TSV}"
printf '===CLOSURE===\n'
printf '%s\n' "${DEPENDENCY_CLOSURE_TSV}"
} | python3 -c "
import json, sys

section = 'git'
git_paths = []
installed = []
closure = []
for line in sys.stdin:
    line = line.rstrip('\n')
    if not line:
        continue
    if line == '===INSTALLED===':
        section = 'installed'
        continue
    if line == '===CLOSURE===':
        section = 'closure'
        continue
    if section == 'git':
        path, expected_blob, worktree_blob, raw_sha256, result = line.split('\t')
        git_paths.append({
            'path': path,
            'expected_commit_blob': expected_blob,
            'worktree_filtered_blob': worktree_blob,
            'raw_working_tree_sha256': raw_sha256,
            'result': result,
        })
    elif section == 'installed':
        installed_path, resolved_target, resolved_type, sha256, comparison_basis, result = line.split('\t')
        installed.append({
            'installed_path': installed_path,
            'resolved_target': resolved_target,
            'resolved_type': resolved_type,
            'sha256': sha256,
            'comparison_basis': comparison_basis,
            'result': result,
        })
    else:
        closure.append(line)

git_ok = all(p['result'] == 'PASS' for p in git_paths)
installed_ok = all(p['result'] == 'PASS' for p in installed)

launcher_result = 'PASS'
msg_schema_payload = json.loads('''${MSG_SCHEMA_RESULT}''')
msg_schema_ok = msg_schema_payload['overall_result'] == 'PASS'

manifest = {
    'schema_version': '3.0.0',
    'expected_head': '${EXPECTED_HEAD}',
    'actual_head': '${ACTUAL_HEAD}',
    'git_version': '${GIT_VERSION}',
    'git_source_paths': git_paths,
    'installed_runtime_artifacts': installed,
    'installed_launcher': {
        'path': '${LAUNCHER}',
        'raw_sha256': '${LAUNCHER_SHA256}',
        'structural_check_result': launcher_result,
        'expected_dist': '${EXPECTED_DIST}',
        'expected_group': '${EXPECTED_GROUP}',
        'expected_name': '${EXPECTED_NAME}',
    },
    'installed_entrypoint_metadata': {
        'candidate_count': ${ENTRY_POINTS_CANDIDATE_COUNT},
        'path': '${ENTRY_POINTS_METADATA}',
        'raw_sha256': '${ENTRY_POINTS_SHA256}',
        'resolved_mapping': 'cooperative_avoider = epuck2_comm.cooperative_avoider:main',
        'result': 'PASS',
    },
    'message_schema_verification': msg_schema_payload,
    'dependency_closure': closure,
    'unavailable_identity_items': [],
    'git_source_identity_result': 'PASS' if git_ok else 'FAIL',
    'installed_runtime_identity_result': 'PASS' if installed_ok else 'FAIL',
    'installed_launcher_identity_result': launcher_result,
    'message_schema_identity_result': 'PASS' if msg_schema_ok else 'FAIL',
    'overall_result': 'PASS' if (git_ok and installed_ok and launcher_result == 'PASS' and msg_schema_ok) else 'FAIL',
}
with open('${SOURCE_IDENTITY_MANIFEST_JSON}', 'w', encoding='utf-8') as f:
    json.dump(manifest, f, indent=2)
"

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
STATUS_RECORD_JSON="${OUT_DIR}/launcher_status.json"
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
HARNESS_EXIT=""
CLEANUP_DONE=0
CLEANUP_EXIT=0
POST_RUN_RESIDUAL_PROCESS_CHECK="UNKNOWN"
VERIFIER_EXIT=""
VERIFIER_JSON_VALID=0
DATA_VALIDITY_VALUE=""
TASK_OUTCOME_VALUE=""
HASHING_EXIT=""
HASH_VERIFY_STATUS=""

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

_wait_for_file_ready() {
    local path="$1" timeout_s="$2"
    local start="${SECONDS}"
    while true; do
        if [[ -s "${path}" ]]; then
            return 0
        fi
        if (( SECONDS - start >= timeout_s )); then
            return 1
        fi
        sleep 0.2
    done
}

# --- Exact owned-process identity helpers for cooperative_avoider only.
# Never used for any other process in this script (the other five are
# launched directly via `python3 ...&`, so their $! already identifies
# the single real process with no wrapper/child ambiguity). ---
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
# itself started. Never uses `pgrep`/`pkill` or any other name-based
# discovery to pick a signal target. Sets
# COOP_CLEANUP_CLASSIFICATION=abnormal if SIGKILL escalation was
# required. Returns nonzero only if the owned process could not be
# confirmed stopped after the full sequence.
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

# Bounded owned-PID cleanup for the five plain `python3 ...&` processes
# (their $! is already the exact, unambiguous owned process -- no
# wrapper/child split to worry about). Escalates to SIGKILL on that
# same exact PID only, records failure rather than silently claiming a
# clean shutdown if even SIGKILL does not work.
terminate_owned_pid() {
    local pid="$1" label="$2"
    local sigint_wait_s="${3:-10}" sigkill_wait_s="${4:-5}"
    if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
        return 0
    fi
    kill -INT "${pid}" 2>/dev/null || true
    local waited=0
    while kill -0 "${pid}" 2>/dev/null; do
        if (( waited >= sigint_wait_s )); then
            echo "CLEANUP_ESCALATION: ${label} pid=${pid} did not exit after SIGINT -- escalating to SIGKILL on this exact owned PID (ABNORMAL cleanup)" >&2
            kill -KILL "${pid}" 2>/dev/null || true
            sleep "${sigkill_wait_s}"
            break
        fi
        sleep 1
        waited=$(( waited + 1 ))
    done
    if kill -0 "${pid}" 2>/dev/null; then
        echo "CLEANUP_FAILED_TO_EXIT: ${label} pid=${pid}" >&2
        return 1
    fi
    return 0
}

# --- Idempotent cleanup + separated INT/TERM/EXIT handling. Stops
# every process this run actually started (still-empty PID variables
# are simply skipped, so cleanup is safe even if only a prefix of the
# stack ever launched), reverse launch order, recorder always last.
# Runs at most once (CLEANUP_DONE guard) regardless of how many of
# INT/TERM/EXIT eventually fire, and never lets the shell resume normal
# script execution after an INT/TERM -- each handler ends with an
# explicit `exit`. ---
cleanup() {
    echo "[cleanup] stopping any processes this run actually started, reverse order"
    local failed=0
    terminate_owned_pid "${PEER_PID}" "virtual_peer" || failed=1
    terminate_owned_pid "${HARNESS_PID}" "harness" || failed=1
    terminate_owned_process_group \
        "${COOP_PID}" "${COOP_PGID}" "${COOP_START_TIME}" "${COOP_EXE_PATH}" "cooperative_avoider" || failed=1
    terminate_owned_pid "${ADAPTER_PID}" "adapter" || failed=1
    terminate_owned_pid "${GUARD_PID}" "guard" || failed=1
    terminate_owned_pid "${RECORDER_PID}" "recorder" || failed=1
    echo "COOP_CLEANUP_CLASSIFICATION=${COOP_CLEANUP_CLASSIFICATION}"
    if RESIDUAL_MATCHES="$(_forbidden_process_scan)" && [[ -n "${RESIDUAL_MATCHES}" ]]; then
        echo "${RESIDUAL_MATCHES}"
        POST_RUN_RESIDUAL_PROCESS_CHECK="FAIL"
        echo "POST_RUN_RESIDUAL_PROCESS_CHECK=FAIL" >&2
        failed=1
    else
        POST_RUN_RESIDUAL_PROCESS_CHECK="CLEAN"
        echo "POST_RUN_RESIDUAL_PROCESS_CHECK=CLEAN"
    fi
    return "${failed}"
}

run_cleanup_once() {
    if [[ "${CLEANUP_DONE}" -eq 1 ]]; then
        return 0
    fi
    CLEANUP_DONE=1
    if cleanup; then
        CLEANUP_EXIT=0
    else
        CLEANUP_EXIT=$?
    fi
    return "${CLEANUP_EXIT}"
}

on_int() {
    trap - INT TERM EXIT
    echo "[signal] SIGINT received -- running cleanup once, then exiting 130" >&2
    run_cleanup_once || true
    exit 130
}

on_term() {
    trap - INT TERM EXIT
    echo "[signal] SIGTERM received -- running cleanup once, then exiting 143" >&2
    run_cleanup_once || true
    exit 143
}

on_exit() {
    local exit_code=$?
    trap - INT TERM EXIT
    run_cleanup_once || true
    # Preserve the script's own exit status unless cleanup itself
    # introduced a NEW failure into an otherwise-successful run.
    if [[ "${exit_code}" -eq 0 && "${CLEANUP_EXIT}" -ne 0 ]]; then
        exit "${CLEANUP_EXIT}"
    fi
    exit "${exit_code}"
}

trap on_int INT
trap on_term TERM
trap on_exit EXIT

# ============================================================
# Steps 3-8: launch recorder, guard, adapter, cooperative_avoider,
# harness, virtual peer. Each readiness gate is a bare statement --
# under `set -e` its nonzero return immediately triggers the EXIT
# trap, running cleanup and stopping before any subsequent step, so a
# readiness timeout, node-graph mismatch, or executable failure can
# never be silently skipped past.
# ============================================================

# --- Step 3: evidence recorder (started first) ---
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

# --- Step 4: guard (DISARMED by default) ---
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

# --- Step 5: adapter-hosted navigator ---
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

# --- Step 6: cooperative_avoider (real, unmodified). Direct executable
# (COOP_EXE, never `ros2 run`), with `set -m` enabled ONLY for this one
# launch so bash job control assigns the background job its own
# process group (PGID == its own PID -- confirmed live). ---
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

# --- Step 7: Stage 3 harness (--auto-run is mandatory) ---
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

# --- Step 8: virtual peer (target == start position) ---
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

# ============================================================
# Step 9: wait for harness completion, capture HARNESS_EXIT (protected
# from errexit by the explicit if/else -- a nonzero harness exit is
# recorded data, not itself a reason to skip cleanup/verification).
# ============================================================
if wait "${HARNESS_PID}"; then
    HARNESS_EXIT=0
else
    HARNESS_EXIT=$?
fi
echo "HARNESS_EXIT=${HARNESS_EXIT}"

# ============================================================
# Step 10: stop the complete process stack in reverse order, recorder
# last (explicit call -- not deferred to the EXIT trap, so the
# verifier below never runs while any producer is still active).
# ============================================================
run_cleanup_once || true
echo "CLEANUP_EXIT=${CLEANUP_EXIT}"

# ============================================================
# Step 11: wait until every writer is closed. The recorder's own
# main() catches (KeyboardInterrupt, ExternalShutdownException) around
# rclpy.spin() and, in its `finally` block, calls node.write_summary()
# (finalizes summary.json) then node.writer.close() (flushes+closes
# evidence.csv) before the process exits -- so by the time
# terminate_owned_pid() above has confirmed the recorder PID no longer
# exists, both files are already complete. The bounded checks here are
# a defensive second proof, not a race.
# ============================================================
if ! _wait_for_file_ready "${EVIDENCE_CSV}" 5; then
    echo "ABORT: ${EVIDENCE_CSV} not finalized after recorder shutdown." >&2
    exit 1
fi
if ! _wait_for_file_ready "${SUMMARY_JSON}" 5; then
    echo "ABORT: ${SUMMARY_JSON} not finalized after recorder shutdown." >&2
    exit 1
fi

# ============================================================
# Step 12: residual-process check already performed inside cleanup()
# above (POST_RUN_RESIDUAL_PROCESS_CHECK is set there).
# ============================================================
echo "POST_RUN_RESIDUAL_PROCESS_CHECK=${POST_RUN_RESIDUAL_PROCESS_CHECK}"

# ============================================================
# Step 13: verifier execution against immutable evidence (only after
# the full stack has stopped). If cleanup or the residual check
# reported a problem, this is passed to the verifier's own
# --residual-process-detected flag rather than silently omitted.
# ============================================================
VERIFIER_ARGS=(
    --csv "${EVIDENCE_CSV}" --summary-json "${SUMMARY_JSON}"
    --test-only-angular-bound-rps "${TEST_ONLY_ANGULAR_BOUND_RPS}"
    --test-only-linear-bound-mps "${TEST_ONLY_LINEAR_BOUND_MPS}"
)
if [[ "${CLEANUP_EXIT}" -ne 0 || "${POST_RUN_RESIDUAL_PROCESS_CHECK}" == "FAIL" ]]; then
    VERIFIER_ARGS+=(--residual-process-detected)
fi
if python3 hil_offline_stage3_post_run_verifier.py "${VERIFIER_ARGS[@]}" > "${VERIFIER_JSON}"; then
    VERIFIER_EXIT=0
else
    VERIFIER_EXIT=$?
fi
echo "${VERIFIER_EXIT}" > "${VERIFIER_EXIT_FILE}"
echo "VERIFIER_EXIT=${VERIFIER_EXIT}"

# ============================================================
# Step 14: validate verifier JSON (existence, non-emptiness, strict
# parse, required keys) -- protected from errexit by the explicit
# if/else, so a validation failure is recorded, not silently skipped.
# ============================================================
if [[ ! -s "${VERIFIER_JSON}" ]]; then
    echo "ABORT: ${VERIFIER_JSON} missing or empty." >&2
    VERIFIER_JSON_VALID=0
else
    if VERIFIER_JSON_PARSED="$(python3 -c "
import json, sys
with open('${VERIFIER_JSON}', encoding='utf-8') as f:
    result = json.load(f)
if 'DATA_VALIDITY' not in result or 'TASK_OUTCOME' not in result:
    print('MISSING_REQUIRED_KEY', file=sys.stderr)
    sys.exit(1)
print('DATA_VALIDITY=' + str(result['DATA_VALIDITY']))
print('TASK_OUTCOME=' + str(result['TASK_OUTCOME']))
")"; then
        VERIFIER_JSON_VALID=1
        echo "${VERIFIER_JSON_PARSED}"
        DATA_VALIDITY_VALUE="$(echo "${VERIFIER_JSON_PARSED}" | sed -n 's/^DATA_VALIDITY=//p')"
        TASK_OUTCOME_VALUE="$(echo "${VERIFIER_JSON_PARSED}" | sed -n 's/^TASK_OUTCOME=//p')"
    else
        VERIFIER_JSON_VALID=0
        echo "ABORT: verifier JSON failed strict validation (parse error or missing required key)." >&2
    fi
fi

# ============================================================
# Step 15: record harness/cleanup/verifier status, source identity,
# and PID/PGID/start-time identities -- BEFORE hashing.
# ============================================================
python3 -c "
import json
status = {
    'harness_exit': ${HARNESS_EXIT},
    'cleanup_exit': ${CLEANUP_EXIT},
    'post_run_residual_process_check': '${POST_RUN_RESIDUAL_PROCESS_CHECK}',
    'coop_cleanup_classification': '${COOP_CLEANUP_CLASSIFICATION}',
    'verifier_exit': ${VERIFIER_EXIT},
    'verifier_json_valid': ${VERIFIER_JSON_VALID},
    'data_validity': '${DATA_VALIDITY_VALUE}',
    'task_outcome': '${TASK_OUTCOME_VALUE}',
}
with open('${STATUS_RECORD_JSON}', 'w', encoding='utf-8') as f:
    json.dump(status, f, indent=2)
"

python3 -c "
import json
manifest = {
    'run_id': '${RUN_ID}',
    'source_head': '${ACTUAL_HEAD}',
    'source_identity_manifest': '$(basename "${SOURCE_IDENTITY_MANIFEST_JSON}")',
    'resolved_cooperative_avoider_executable': '${COOP_EXE}',
    'ros_domain_id': ${ROS_DOMAIN_ID},
    'ros_localhost_only': ${ROS_LOCALHOST_ONLY},
    'pids': {
        'recorder': ${RECORDER_PID},
        'guard': ${GUARD_PID},
        'adapter': ${ADAPTER_PID},
        'cooperative_avoider': {
            'pid': ${COOP_PID}, 'pgid': ${COOP_PGID},
            'start_time': '${COOP_START_TIME}', 'exe_path': '${COOP_EXE_PATH}',
        },
        'harness': ${HARNESS_PID},
        'virtual_peer': ${PEER_PID},
    },
}
with open('${PID_MANIFEST}', 'w', encoding='utf-8') as f:
    json.dump(manifest, f, indent=2)
"

# ============================================================
# Step 16: evidence hashing -- only after every writer has stopped,
# evidence.csv/summary.json are finalized, the verifier JSON is
# validated, and status records are written. SHA256SUMS.txt does NOT
# hash itself. Immediately verified with `sha256sum -c`.
# ============================================================
if (
    cd "${OUT_DIR}" && sha256sum \
        "$(basename "${EVIDENCE_CSV}")" \
        "$(basename "${SUMMARY_JSON}")" \
        "$(basename "${VERIFIER_JSON}")" \
        "$(basename "${VERIFIER_EXIT_FILE}")" \
        "$(basename "${STATUS_RECORD_JSON}")" \
        "$(basename "${PID_MANIFEST}")" \
        "$(basename "${SOURCE_IDENTITY_MANIFEST_JSON}")" \
        "$(basename "${EXECUTION_SCRIPT_COPY}")" \
        "$(basename "${RECORDER_LOG}")" "$(basename "${GUARD_LOG}")" \
        "$(basename "${ADAPTER_LOG}")" "$(basename "${COOP_LOG}")" \
        "$(basename "${HARNESS_LOG}")" "$(basename "${PEER_LOG}")" \
        > "$(basename "${SHA256SUMS_FILE}")"
); then
    HASHING_EXIT=0
else
    HASHING_EXIT=$?
fi

if HASH_VERIFY_OUTPUT="$(cd "${OUT_DIR}" && sha256sum -c "$(basename "${SHA256SUMS_FILE}")" 2>&1)"; then
    HASH_VERIFY_STATUS=0
else
    HASH_VERIFY_STATUS=$?
fi
echo "${HASH_VERIFY_OUTPUT}"
echo "HASHING_EXIT=${HASHING_EXIT}"
echo "HASH_VERIFY_STATUS=${HASH_VERIFY_STATUS}"

# ============================================================
# Step 17: final launcher exit status. Never reports success when the
# harness failed, cleanup failed, a residual process was detected, the
# verifier's JSON was missing/invalid, the verifier itself returned
# nonzero, or hashing/hash-verification failed.
# ============================================================
FAILURE_REASONS=()
if [[ "${HARNESS_EXIT}" -ne 0 ]]; then FAILURE_REASONS+=("HARNESS_EXIT=${HARNESS_EXIT}"); fi
if [[ "${CLEANUP_EXIT}" -ne 0 ]]; then FAILURE_REASONS+=("CLEANUP_EXIT=${CLEANUP_EXIT}"); fi
if [[ "${POST_RUN_RESIDUAL_PROCESS_CHECK}" == "FAIL" ]]; then FAILURE_REASONS+=("RESIDUAL_PROCESS_DETECTED"); fi
if [[ "${VERIFIER_JSON_VALID}" -ne 1 ]]; then FAILURE_REASONS+=("VERIFIER_JSON_INVALID"); fi
if [[ "${VERIFIER_EXIT}" -ne 0 ]]; then FAILURE_REASONS+=("VERIFIER_EXIT=${VERIFIER_EXIT}"); fi
if [[ "${HASHING_EXIT}" -ne 0 ]]; then FAILURE_REASONS+=("HASHING_FAILED"); fi
if [[ "${HASH_VERIFY_STATUS}" -ne 0 ]]; then FAILURE_REASONS+=("HASH_VERIFICATION_FAILED"); fi

if [[ ${#FAILURE_REASONS[@]} -gt 0 ]]; then
    echo "FINAL_LAUNCHER_STATUS=FAIL (${FAILURE_REASONS[*]})" >&2
    if [[ "${VERIFIER_EXIT}" -ne 0 ]]; then
        FINAL_EXIT="${VERIFIER_EXIT}"
    else
        FINAL_EXIT=1
    fi
else
    echo "FINAL_LAUNCHER_STATUS=PASS"
    FINAL_EXIT=0
fi

exit "${FINAL_EXIT}"
```

## Forbidden-process and forbidden-topic checks

- **Pre-run**: `_forbidden_process_scan()` (self-PID-excluding, read-only) must report nothing.
- **Forbidden-topic check**: before step 3 (recorder launch), the script confirms via `ros2 topic list` that none of `/cmd_vel`, `/cmd_vel_unguarded`, `/epuck1/state`, `/epuck_bridge/status`, `/hil_guard/arm` already exist on the bus under this domain.
- **Post-run**: `_forbidden_process_scan()` (the same self-PID-excluding function, called again inside `cleanup()`) must report nothing; confirm via `git status`/`git diff` that no repository file changed during the run.
- **Operational-topic isolation**: every topic variable in the script above (`OWN_STATE_TOPIC` through `GATE_DECISION_TOPIC`) is built from `NS="/hil_offline_stage3"`; no launch argument or remap destination anywhere in the script uses `/cmd_vel`, `/cmd_vel_unguarded`, `/epuck1/state`, `/epuck_bridge/status`, or `/hil_guard/arm` -- those five strings appear in this document only in this prohibition section and the forbidden-topic/forbidden-process checks, never as an executable argument.

## Abort conditions

Any physical/Pi/bridge/Webots process detected at any point; any
publisher/subscriber on a real production topic; any node reporting
`ROS_DOMAIN_ID` other than `91`; any topic/type mismatch against the
isolated topic table; guard output outside the declared test-only
bound; more than one adoption event; missing expected evidence row/log
line; residual process after shutdown; unexpected repository
modification; any readiness `wait_for_log_pattern` timeout (15s per
process); any `CLEANUP_FAILED_TO_EXIT` report; a repository `HEAD` or
committed Stage 3 file hash that does not match what this execution
attempt was resolved against; an unstaged content diff in any committed
Stage 3 file; a missing or non-executable resolved `cooperative_avoider`
executable; `evidence.csv`/`summary.json` not finalised within 5s of
recorder shutdown; a missing, empty, or strictly-invalid verifier JSON
(parse error or missing `DATA_VALIDITY`/`TASK_OUTCOME` key); a nonzero
evidence-hashing or `sha256sum -c` verification result. The final
launcher exit status never reports success under any of these
conditions, nor when the harness itself exited nonzero or cleanup
failed, even if the verifier's own result happened to read `SUCCESS`.

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
