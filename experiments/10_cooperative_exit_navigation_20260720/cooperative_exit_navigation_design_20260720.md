# Cooperative Exit Navigation Study (new task-level extension): design document (2026-07-20)

Prepared following the supervisor's new task-level research question. Does
NOT modify, delete, rename, or reinterpret any existing Objective 5
impairment-matrix evidence (Conditions A-D remain frozen and valid — see
`experiments/05_objective5_impairment_matrix/`). This is an ADDITIONAL
experiment category, `10_cooperative_exit_navigation_20260720`.

## 1. Research question and relationship to A-D

**New task-level question** (directly answers the supervisor's request):

> How does inter-robot communication affect the safety and efficiency of
> cooperative navigation as the number of robots increases?

**Existing A-D question** (unchanged, still valid, still cited):

> How does communication impairment (delay, jitter/reordering) affect the
> safety of pairwise CPA-based collision avoidance between two robots?

A-D established the underlying mechanism-level evidence: communication
delay/jitter measurably changes the CPA avoidance behavior of a 2-robot
pair (Condition C's repeated S-curve oscillation and 1/5 unsafe trial;
Condition D's genuine reordering with 5/5 valid, safe trials at a smaller
delay). This study builds on that mechanism by asking a task-level
question: with that same communication substrate, does HAVING
communication at all (vs none), and does robot COUNT, change whether and
how safely/efficiently a group of robots can complete a shared task
(reach a common exit)?

## 2. Task definition

All robots start from a frozen set of start poses in an obstacle-populated
Webots world. Task succeeds (`TASK_OUTCOME=SUCCESS`) only if EVERY robot
individually enters a shared, explicitly-marked exit/goal region and
remains inside it continuously for a pre-registered hold time (anti
single-frame-trigger requirement), with zero collisions and
`minimum_pairwise_distance_m >= safety_radius_m` maintained throughout,
and the trial did not end via `max_runtime_s`. See
`experiments/10_cooperative_exit_navigation_20260720/tools/task_completion_analyzer.py`
for the exact, unit-tested implementation (`build_task_verdict`).

Supervisor (Webots) ground truth is used ONLY for offline measurement and
success judging, never fed back into the controller in real time — the
controller only ever sees its own odometry/IR-ToF sensors and (in
COMM_ON) the peer `EpuckState` topic(s), identical to Conditions A-D.

## 3. Conditions (6, this design; only N2 executable this round)

| condition_id | robots (N) | communication | peers per robot | executable_now |
|---|---:|---|---:|---|
| N2_COMM_OFF | 2 | none | 0 | YES (pilot this round) |
| N2_COMM_ON | 2 | full EpuckState + CPA | 1 | YES (pilot this round) |
| N3_COMM_OFF | 3 | none | 0 | NOT YET (world/poses not built) |
| N3_COMM_ON | 3 | full EpuckState + CPA | 2 | NOT YET (needs multi-peer extension, see below) |
| N4_COMM_OFF | 4 | none | 0 | NOT YET (world/poses not built) |
| N4_COMM_ON | 4 | full EpuckState + CPA | 3 | NOT YET (needs multi-peer extension, see below) |

See `cooperative_exit_navigation_conditions.csv` for the
machine-readable frozen parameter table (same pattern as
`objective5_impairment_matrix_conditions.csv`).

### COMM_OFF — strict definition

- Own odometry (`EpuckState` published for measurement/logging purposes
  only) and own local IR/ToF avoidance remain fully enabled.
- `enable_peer_avoidance=false` (an EXISTING `cooperative_avoider.py`
  parameter, already used and validated in the frozen Phase 4 ablation
  design — no new parameter needed).
- No peer `EpuckState` subscription is created at all
  (`cooperative_avoider.py:258-261`'s subscription is gated on
  `enable_peer_avoidance`, so it is genuinely absent, not just ignored).
- No Supervisor ground truth, no centralized controller, no hidden shared
  state substitutes for the disabled communication channel.

### COMM_ON — strict definition

- Full `EpuckState` publish/subscribe, using the identical
  `state_publisher.py`, neighbor-subscription model, and
  `collision_math.py` CPA avoidance already frozen from A-D.
- Local IR/ToF safety layer remains enabled (identical to COMM_OFF —
  local avoidance is NEVER the manipulated variable in this comparison).
- **First-phase impairment level: ZERO** (`delay_s=0, jitter_s=0,
  drop_probability=0`, outage disabled) — a fair, uncontaminated
  communication-vs-no-communication comparison. Combining WITH-impairment
  COMM_ON conditions is a natural future extension, explicitly out of
  scope this round (would conflate "does communication help" with "does
  impaired communication help less," two different questions).

### Fairness requirement (binding)

Within one N-group, COMM_OFF and COMM_ON share, byte-for-byte identical:
world file, obstacle placement, start poses, goal region, speed limits,
control period, local sensor logic/thresholds, `max_runtime_s`. The ONLY
difference is `enable_peer_avoidance` and whether the peer-state
relay/subscription chain is launched at all. This is enforced structurally
by the frozen CSV (one row per N-group's shared geometry parameters,
referenced by both the OFF and ON condition rows) — never achieved by
hand-editing two separate world files that could silently drift apart.

## 4. Why N2 needs ZERO controller code changes (see architecture audit)

`cooperative_avoider.py` already handles exactly 2 robots (1 peer each) —
this is precisely Conditions A-D's setup. `enable_peer_avoidance=false`
already exists and is already validated for COMM_OFF. The only genuinely
NEW code this round is the goal/exit-region task-completion analyzer
(section 6) — no frozen controller, relay, `state_publisher`, or
`sequence_counter` code is touched.

**Goal-directed navigation without a controller change** (see architecture
audit section 9 for the full reasoning): `cooperative_avoider.py`'s CRUISE
mode holds a FIXED `desired_heading_rad`, not a recomputed bearing-to-goal.
This round's scenario geometry is deliberately designed so each robot's
straight-line path at its fixed heading already points at a sufficiently
WIDE shared exit zone, wide enough to absorb the bounded lateral drift the
existing avoidance layers can introduce (empirically bounded at roughly
0.1-0.4m in prior A-D trials). This is a genuine geometry constraint on
this round's scenario design, stated explicitly here so it is never
silently forgotten — it is NOT claimed to generalize to arbitrary start
poses, and N3/N4 (with more robots needing more varied start positions)
may require true closed-loop goal-seeking (a real controller change,
proposed but NOT implemented in `architecture_audit_multi_robot_20260720.md`
section 9's approach (b)) before they can be fairly designed.

## 5. N3/N4 — explicitly NOT ready this round

Confirmed by the architecture audit: multi-peer subscription/freshness/
risk-ranking/conflict-handling does not exist in `cooperative_avoider.py`
today. A minimal-change design for it is written and design-reviewed in
`multi_peer_extension_design_20260720.md`, with its core ranking rule
already implemented as a standalone, unit-tested pure function
(`tools/multi_peer_risk.py`, 9/9 tests passing) — but NOT wired into the
frozen controller. N3/N4 pilots require: (a) this round's N2 pilots to
pass, (b) explicit user authorization, (c) the multi-peer controller
change actually implemented and tested, (d) new world files/start poses
for 3 and 4 robots.

## 6. New analyzer: `tools/task_completion_analyzer.py`

Pure Python, ROS/rosbag-independent (operates on plain
`(t_s, x_m, y_m, yaw_rad, linear_velocity_mps)` per-robot sample lists,
extractable from a bag by a thin wrapper reusing the existing
`rosbag2_py` read pattern from `analyze_objective5_trajectory.py`).
20/20 unit tests passing (see
`tools/test_task_completion_analyzer.py`), covering: single-frame
anti-false-trigger, hold-timer gap reset, all-robots-required (not any
one), pairwise distance for N=3 and N=4 (all `C(N,2)` combinations, not a
single hardcoded pair), `DATA_VALIDITY`/`TASK_OUTCOME` field separation,
max-runtime never read as success, latched-FAILSAFE forcing failure,
collision vs below-safety-radius-but-no-contact distinction, path length,
heading-change unwrapping, stop-duration integration.

### Pre-registered metrics (all fields this analyzer or its companion
communication/trajectory analyzers must emit)

**Task**: `all_robots_reached_goal`, `completed_robot_count`,
`individual_completion_time_s` (per robot), `makespan_s`,
`task_success_rate` (computed at batch level: successes / n_trials),
`timeout_count` (batch level: count of `ended_by_max_runtime` trials).

**Safety**: `collision_count`, `minimum_pairwise_distance_m` (over ALL
pairwise combinations), `safety_margin_m = minimum_pairwise_distance_m -
0.14`, `local_safety_intervention_count` (from controller.log transition
counts, reusing the existing edge-based mode-counting method from
`analyze_objective5_trajectory.py` — never raw grep line counts, per the
D-condition lesson), `stale_state_stop_count`, `failsafe_count`.

**Efficiency**: per-robot `path_length_m`, `total_path_length_m` (sum over
robots), `cumulative_absolute_heading_change_rad`, `turn_reversal_count`,
`stop_duration_s`, `recovery_time_s` (time from an avoidance-mode entry to
the next CRUISE entry, from `controller.log` TRANSITION lines).

**Communication** (COMM_ON only; COMM_OFF reports every field below as the
literal string `NOT_APPLICABLE`, never a numeric 0, per instruction): all
fields already implemented and proven correct in `reorder_safe_delivery_analyzer.py`
(message count, missing/duplicate/out_of_order, capture ratio, message-age
mean/median/p95/p99/max, throughput) plus a new `peer_freshness_violation_count`
(count of ticks where `_fresh()` would have returned False, derivable from
message-age vs `peer_timeout_s` without any controller change).

`safety_radius_m` remains **0.14m**, unchanged from A-D, for this round. Per
instruction, it will not be altered after seeing results; any future
change requires a separate, pre-registered design review specific to the
N-robot task, not a post-hoc adjustment.

## 7. `safety_radius_m` and world/task geometry decisions

- `safety_radius_m = 0.14` (unchanged).
- `collision_contact_distance_m = 0.07` (new parameter, this experiment
  category only): a distinct, smaller threshold representing genuine
  physical contact risk (roughly twice the e-puck2's ~3.5cm body radius),
  used only to distinguish `UNSAFE_FAILURE` (safety-radius breach, no
  contact) from a even-more-severe near-contact event
  (`collision_count>0`). Neither threshold is tuned post-hoc from pilot
  results — both are fixed before Phase 3 begins.
- Goal region: circular, pre-registered center + radius per N-group,
  written into the frozen CSV — never adjusted after observing pilot
  behavior.
- Hold time: pre-registered per N-group (this round: 2.0s for N2),
  written into the frozen CSV.

## 8. Phased execution plan (binding — do not skip ahead)

1. **Phase 1 — read-only audit.** DONE:
   `architecture_audit_multi_robot_20260720.md`.
2. **Phase 2 — design + minimal implementation + tests.** DONE for N2:
   this document, the frozen CSV, `task_completion_analyzer.py` (20/20
   tests), `multi_peer_risk.py` design-only module (9/9 tests, not wired
   into the controller). NOT done: the N2 Webots world file and
   orchestrator/launch scripts — **BLOCKED**, see section 9.
3. **Phase 3 — two exclusionary pilots only** (`N2_COMM_OFF pilot01`,
   `N2_COMM_ON pilot01`). NOT started — blocked by section 9, and in any
   case requires the user to manually launch and observe Trial 01 of
   each formal condition per the separate manual-observation requirement
   below; pilots themselves may run automatically once the Webots
   environment is restored, since pilots are diagnostic/exclusionary, not
   formal Trial01.
4. **Phase 4 — compare the two pilots**, confirm task/record/success/
   fairness criteria before any formal trial.
5. **Phase 5 — wait for explicit user confirmation** before running formal
   n=5 or any N3/N4 work.

## 9. BLOCKING ENVIRONMENT FINDING (reported separately in this session's
   final summary, repeated here for the permanent record)

The Webots simulation working directory referenced by every prior formal
trial's orchestrator (`simulation_comm_experiment_v1/working`, documented
in `PROJECT_HANDOFF.md` as living OUTSIDE the git repo by design) does not
exist in this environment as of 2026-07-20. Webots itself remains
installed (`C:\Program Files\Webots`), and all A-D raw evidence remains
intact at the native WSL bag path (`/home/eamon/epuck_comm_bags/`) — only
the ephemeral launch/world-file working directory is missing. No Webots
trial (old or new) can be launched until this is resolved. This blocks
Phase 3 entirely; it does not block Phases 1-2 (this document, the CSV,
and the two new analyzer modules), which are complete and tested.

## 10. Paper-scope wording (binding)

- No claim of inventing ROS2, Webots, TCP/IP, or CPA collision math.
- Contribution framing: (1) a unified `EpuckState` communication interface
  reused unchanged across every condition and every robot count; (2) a
  reproducible communication/no-communication contrast platform, with
  fairness enforced structurally (shared CSV-driven geometry, not
  hand-duplicated world files); (3) a quantitative link between
  communication quality/presence and cooperative-task safety and
  efficiency, building on A-D's pairwise-mechanism evidence; (4) an
  automated, auditable experiment/evidence pipeline (frozen SHA-256
  code identity, DATA_VALIDITY/TASK_OUTCOME separation, unique
  never-overwritten trial directories, SHA256SUMS-verified raw-to-copy
  integrity) extended from A-D's proven pattern to a new task and
  variable robot count.
- A-D remain cited as: "evidence for the effect of communication
  impairment on pairwise CPA collision-avoidance safety" — never
  described as having tested task-level cooperative navigation.
- This study is described as: "evidence for the effect of communication
  presence and robot count on cooperative-task safety and efficiency" —
  never described as having tested communication IMPAIRMENT (that
  remains A-D's contribution; this study's first phase uses zero
  impairment by design, per section 3).
- Not numbered as "Objective 7": that label is already informally in use
  in `PROJECT_HANDOFF.md` for the physical Pi-puck reality-gap work
  (Objectives 1/6/7). This study is referred to by name (cooperative exit
  navigation), not by an objective number, to avoid collision with the
  official Spec numbering (Objectives 1-5) and the existing informal 6/7
  usage.

## 11. Manual-observation requirement (binding, all N-groups)

Per instruction, EVERY condition's first FORMAL Trial 01 (not the
exclusionary pilots) — `N2_COMM_OFF Trial 01`, `N2_COMM_ON Trial 01`,
`N3_COMM_OFF Trial 01`, `N3_COMM_ON Trial 01`, `N4_COMM_OFF Trial 01`,
`N4_COMM_ON Trial 01` — must be manually launched and observed by the
user. No condition's formal Trial 01 will be auto-started. After each
pilot batch passes, this session's deliverable is limited to: pilot
analysis, confirmed process cleanup, a ready permanent launch script, and
one copy-pasteable PowerShell command for the user to run Trial 01
themselves. Trial 02-05 for a condition are only auto-run after that
condition's Trial 01 is confirmed PASS via cross-checked manual
observation AND automated evidence, with explicit user authorization.
Manual observation vs automated evidence conflicts are marked
`PENDING_REVIEW` (read-only), never silently resolved to PASS.
