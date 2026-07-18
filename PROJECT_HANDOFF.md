# PROJECT_HANDOFF.md — start here

Single entry point for any AI (Claude, Codex, or otherwise) or human
picking up this project. Last updated: 2026-07-18. This document was
current as of index-files commit `026223b` (source state at scan time:
`2558216`) — **do not trust a hardcoded commit hash in any document,
including this one, as "the current commit."** Always run
`git rev-parse HEAD` / `git log -1` for the true current state; a
document's own commit field necessarily lags the commit that contains it.

## What this project is

**Official title** (COMP5200M Project Specification, `LU26-Spec.pdf`):
"Developing a Lightweight Communication Library for Collaborative
Navigation in e-puck2 Robot Swarms." Supervisor: Dr. Arshad Jhumka.

**Official five Objectives** (from the Spec and Scoping/Planning Document):
1. **Environment Setup** — ROS2/Gazebo simulation + physical e-puck2/Pi-puck hardware interface.
2. **Protocol Design** — a streamlined ROS2 Custom Msg / Protobuf message format.
3. **Library Implementation** — a P2P abstraction, neighbor "subscription" model (`e-puck2-Comm`).
4. **Task-Specific Validation** — a navigation/avoidance task measuring how communication latency affects collision avoidance.
5. **Performance Analysis** — packet delivery rate + coordination efficiency, virtual vs. real-world (reality gap).

**Deliverables**: the `e-puck2-Comm` library, the Simulation Package, an Evaluation Report.

**The avoidance controller (v1-v4) is Objective 4's task-specific
validation vehicle, not the project's primary contribution.** It is now
frozen (see below). Current project effort is on Objectives 2/3/5
(protocol/library formal metrics) and, next, Objectives 1/6/7 (physical
Pi-puck, reality gap).

## Current overall status

- **Objective 1** (Environment Setup): simulation side done (ROS2 Humble + Webots, not Gazebo — see the deviation note below). Physical Pi-puck side: not started.
- **Objective 2** (Protocol Design): `EpuckState.msg` implemented and **frozen as PROTOCOL_VERSION=1** (commit `b5a0351`).
- **Objective 3** (Library Implementation): `epuck2_comm` library implemented — `state_publisher`, `cooperative_avoider`, `network_impairment_relay`, analyzers. 126/126 tests passing.
- **Objective 4** (Task-Specific Validation): controller v1→v4 defect chain resolved; **Phase 4 formal batch SEALED, 5/5 PASS** (commit `e32560e`). Avoidance-scenario scope is now intentionally frozen.
- **Objective 5** (Performance Analysis): analyzer + impairment relay implemented and unit-tested; **zero-impairment baseline is still diagnostic only** — root cause of an earlier ~40-55% message-loss anomaly has been traced to writing rosbag directly to a `/mnt/c` (Windows-mounted) path from WSL2, confirmed via two independent native-WSL-path diagnostic trials (PDR=1.0 both times), but a full formal baseline (with the actual task controller running) has not yet been completed under that fixed workflow. **No formal PDR/latency figures exist yet.**
- **Objective 6/7** (physical validation, reality gap): not started.

## Key paths

| What | Path |
|---|---|
| Windows repo root | `C:\Users\路一鸣\Desktop\硬件实验毕设\e-puck2-Comm` |
| WSL repo root (same repo, mounted) | `/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm` |
| ROS2 workspace (built package lives here, synced from repo `src/`) | `~/epuck_ws` (i.e. `/home/eamon/epuck_ws`) |
| Webots world/launch working directory (**outside** the git repo) | `/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/simulation_comm_experiment_v1/working` |
| Native WSL scratch path for rosbag recording (see Objective 5 finding below) | `/home/eamon/epuck_comm_bags/` |

**Workflow note**: the repo's `src/epuck2_comm` package is edited in the
Windows-visible path, then **synced into `~/epuck_ws/src/epuck2_comm`**
(`rsync`) and rebuilt (`colcon build --packages-select epuck2_comm
--symlink-install`) before any `ros2 run`/pilot script will see the
change. Every pilot `.sh` script sources `/opt/ros/humble/setup.bash` then
`~/epuck_ws/install/setup.bash`.

## Build / test commands

```bash
source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash
cd ~/epuck_ws
colcon build --packages-select epuck2_comm --symlink-install
colcon test --packages-select epuck2_comm
colcon test-result --verbose
```

Fast unit-test-only loop (no ROS workspace rebuild needed for pure-Python logic changes, but the workspace copy must still be synced before any `ros2 run`):
```bash
cd "/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/src/epuck2_comm"
python3 -m pytest test/ -q
```

## Protocol and message definition

- File: `src/epuck2_comm_interfaces/msg/EpuckState.msg`
- **Frozen as `PROTOCOL_VERSION=1`** at commit `b5a0351`. SHA-256:
  `a7ec4184dec52b157a87beea20b44fb2dff5c6dee199d0c76b7c347c26abe15b`.
- Full rules and rationale: `src/epuck2_comm_interfaces/PROTOCOL_FREEZE_20260717.md`.
- **Do not add, remove, or reorder fields.** A genuine structural need requires a new, distinct `PROTOCOL_VERSION=2` message, never mutating v1.

## Frozen controller

- Version: `controller_v4_timebase_fix_20260717`
- Key commits: `980e7d0` (timebase fix), `06e0f0f` (post-fix regression evidence), `f1830c5` (docs)
- Files: `src/epuck2_comm/epuck2_comm/cooperative_avoider.py`, `local_obstacle_logic.py`
- **Frozen. Do not modify the avoidance algorithm, safety thresholds, CPA parameters, or the world/geometry it was validated against unless a new blocking safety defect is found and proven with log evidence.** Analysis-only tooling (new analyzer scripts, offline classifiers) may be added freely — see `analyze_trigger_reason.py`, `analyze_comm_performance.py` as examples of the accepted pattern (new read-only script, never touches the controller).

## Experiment categories (full detail: `experiments/EXPERIMENT_INDEX.md`)

01 protocol/unit tests · 02 controller regression (v1-v4 dev evidence, NOT formal comm stats) · 03 Phase 4 task validation (Phases 1-4, formal vs. pilot/diagnostic clearly separated) · 04 Objective 5 comm baseline (currently diagnostic only) · 05 Objective 5 impairment matrix (not started) · 06 physical Pi-puck (not started) · 07 reality gap (not started) · 08 paper-ready outputs (currently empty) · 09 legacy/excluded (old protocol-format bags, failed pilots — never deleted, always indexed with an exclusion reason).

## Formal vs. diagnostic — the distinction that matters most

**Formal** (`evidence_level=FORMAL_SIM` or `FORMAL_PHYSICAL` in the
registry): counted toward dissertation statistics, run under a frozen,
documented configuration, typically part of an n≥5 batch.

**Diagnostic/pilot** (`evidence_level=PILOT`): exploratory, debugging,
measurement-chain-isolation, or pre-validation runs. **Never pooled with
formal statistics, regardless of PASS/FAIL.** Most of the Objective 5
comm-baseline work is still diagnostic-only; the exception is
`objective5_comm_baseline_zero_impairment_formal_trial01`
(`evidence_level=FORMAL_SIM`, PASS) — a genuine task-level run (real
`cooperative_avoider` task completion, native WSL bag path, PDR=1.0,
zero gaps/duplicates/out-of-order) that is now formal Objective 5
evidence for PDR/rate/bandwidth. Message age/latency from that trial is
**N/A**, not a real number — see known limitation 3 below.

Full machine-readable record: `experiments/experiment_registry.csv` (one
row per experiment/batch — `status`, `evidence_level`,
`formal_or_diagnostic`, `included_in_paper` columns are authoritative).

## Known limitations (see `experiments/project_status.json` for the full list)

1. Phase 4's combined scenario (5/5 formal) triggers via `PROXIMITY_FALLBACK` in every trial, never `PREDICTED_CPA` — must be named "staged local-obstacle avoidance followed by communication-assisted proximity/cooperative avoidance," never described as preventing a certain collision.
2. 5 pre-protocol-freeze bags cannot be reprocessed by current analyzers (old `EpuckState` wire shape) — original historical analysis remains valid, re-analysis does not work.
3. `/mnt/c` rosbag-write message loss (confirmed via two native-path diagnostic trials) is fully resolved for formal work by using the native WSL ext4 bag path — `objective5_comm_baseline_zero_impairment_formal_trial01` validated this at the task level (PASS). Separately, `analyze_comm_performance.py`'s `mean_message_age_s`/`p95_message_age_s` are **not reliable**: `EpuckState.stamp` is never populated by `state_publisher.py`, so the "age" reduces to absolute epoch bag-receive time, not a true latency delta — treat message age/latency as N/A until `state_publisher` sets `stamp`. PDR/rate/bandwidth are unaffected.
4. `sequence_counter` was rewritten this session (periodic atomic checkpoints + a `finally`-block final write, no custom SIGINT handler) and verified working in the formal baseline PASS (`complete=true` both robots). A related orchestration-script bug — the shell script signaling only the relay+counter launch-service's own PID, not its child processes — caused the first 3 attempts at the formal baseline to fail with `complete=false`; fixed by running that process tree under `setsid` and signaling the whole process group on shutdown (`run_objective5_comm_baseline_formal_trial.sh`'s `stop_pid_group`).
5. Webots R2025a is used, not Gazebo as the Spec names — this is a disclosed, deliberate deviation (protocol/library are simulator-agnostic; see `HANDOFF_20260717.md` for the full risk note and the recommendation to confirm with the supervisor). Do not silently redo the platform in Gazebo.
6. No CPU/memory overhead measurement exists yet (would need a live psutil-style sampling companion).
7. No physical-hardware clock-sync procedure exists yet (`verify_clock_sync()` intentionally raises `NotImplementedError` until Objective 6 begins).

## Current single next step

The formal zero-impairment Objective 5 baseline
(`objective5_comm_baseline_zero_impairment_formal_trial01`) has PASSED.
Next: design the delay/loss impairment matrix (A-F conditions: no-peer /
baseline / medium-delay / high-delay / medium-loss / high-loss) and
submit it for explicit user confirmation before running any of it — do
not auto-run the matrix after a baseline PASS.

## What to read first, in order

1. **This file.**
2. `experiments/EXPERIMENT_INDEX.md` — the full experiment taxonomy and where everything is.
3. `experiments/project_status.json` — machine-readable current state, blocked items, known limitations.

Then, depending on what you're doing: `experiments/experiment_registry.csv`
(per-experiment detail), `experiments/path_manifest.csv` (exact paths,
Windows+WSL, existence/size), or
`experiments/controller_v4_full_sensor_bypass_20260717/HANDOFF_20260717.md`
(the detailed controller v1→v4 technical narrative).

## Do not

- Do not overwrite, delete, move, or rename any existing bag/log/analysis/CSV/JSON/report. If you must copy, keep the original and record the copy's path + SHA-256 in `path_manifest.csv`.
- Do not mix formal and diagnostic/pilot data into the same statistical batch.
- Do not modify the frozen controller (`cooperative_avoider.py`, `local_obstacle_logic.py`) or the frozen `PROTOCOL_VERSION=1` message without a proven, logged, blocking defect.
- Do not re-open or rerun the SEALED Phase 4 formal batch (Trials 01-05, commit `e32560e`).
- Do not claim a cross-device (physical hardware) latency measurement without first running and passing a real clock-sync verification.
- Do not silently redo the simulation platform in Gazebo — the Webots deviation is disclosed and accepted pending supervisor confirmation, not a defect to quietly fix.

## Maintenance rule

After every experiment run, update (in this order): that experiment's own
evidence (already handled by its own pilot script), then
`experiments/experiment_registry.csv`, `experiments/path_manifest.csv`,
`experiments/project_status.json`, `experiments/EXPERIMENT_INDEX.md`, and
— only once a new formal gate is reached — this file and the paper-output
directory (`08_paper_ready_outputs` in the index).
