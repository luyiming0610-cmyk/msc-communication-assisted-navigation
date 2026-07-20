# Cooperative exit-navigation study -- stage classification (2026-07-20)

This document is the evidence-chain index for `10_cooperative_exit_navigation_20260720/`.
It exists so that any future AI or human reviewer can immediately tell
which data is preparatory system-validation evidence and which is the
actual research result, without re-reading the full session history.

## Why this document exists

The first working scenario used a central circular rendezvous point
(goal region at the world origin) as the shared "exit". After it was
built, tested, and run, the supervisor's explicit direction (2026-07-20)
was that the *research question* requires a real edge/corner exit and an
asymmetric-information communication design (Robot A discovers the exit,
Robot B does not, and COMM_ON is the only condition in which that
information reaches Robot B) -- a central meeting point cannot
distinguish "communication helped find the exit" from "communication
only affected collision avoidance." The central-rendezvous work is
**not deleted, not rerun, and not described as failed or useless** --
it is reclassified as **Stage 0: preparatory shared-goal mechanism
validation**, per explicit instruction.

## Stage 0 -- Preparatory shared-goal mechanism validation (COMPLETE, frozen)

**Status: `PREPARATORY / EXCLUSIONARY / NOT_INCLUDED_IN_FORMAL_STATISTICS`.**

Not a formal experiment. Not evidence for or against "does communication
improve navigation efficiency." Not part of any n=5 statistic. Its sole
purpose was to validate, end-to-end, the pieces of infrastructure that
the real Stage 1 edge-exit study reuses unmodified:

1. **`task_completion_monitor.py`** (independent, read-only rclpy node)
   correctly detects when both robots have entered and continuously held
   a shared goal region -- proven by `test_goal_hold_tracker.py`'s
   batch-vs-stream cross-check AND by live pilot runs.
2. **A start-pose-inside-goal-region defect** was found and fixed: with
   `goal_radius_m=0.5`, both robots' static start poses (0.35m from
   origin) were already "in the goal," so `TASK_COMPLETE_GOAL` fired
   with `path_length_m=0.0` -- a false premature success. Caught by the
   pilot process exactly as it is designed to catch such things.
   (`PILOT04`, commit `9795145`.)
3. **`goal_radius_m` corrected 0.5 -> 0.20** forces genuine motion before
   the hold timer can start.
4. **`TASK_COMPLETE_GOAL` can genuinely replace `max_runtime_s`** as the
   trial-ending condition -- proven by `PILOT06` (OFF) and the ON-side
   `PILOT02`, both of which stopped via real goal-region convergence,
   not timeout.
5. **A cmd_vel-verification race was found and fixed**: the frozen
   `cooperative_avoider.py.stop()` publishes zero velocity 3x then
   immediately `destroy_node()`s; those publishes can be lost to a
   DDS-teardown race before the bag captures them (the controller's own
   code comment already acknowledges this). Checking the raw
   `/epuckN/cmd_vel` topic's last sample is unreliable for this reason;
   replaced with `verify_state_velocity_settled.py`, checking the
   robot's actual physical velocity from `/epuckN/state`
   (`PILOT05` finding -> fix, commit `9795145`).
6. **Both OFF and ON data-recording/goal-hold-judgment/rosbag-close/
   auto-analysis chains run correctly** -- proven by `PILOT06` (local
   IR/ToF avoidance path) and `PILOT02` (communication CPA avoidance
   path) both reaching genuine `TASK_OUTCOME=SUCCESS`.

### Stage 0 pilot inventory (all preserved, none deleted, none rerun)

| Trial | Condition | Disposition | Evidence |
|---|---|---|---|
| `EXCLUSIONARY_PILOT01` | N2_COMM_OFF | Failed: orchestrator env-var mismatch | `cooperative_exit_n2_n2_comm_off_EXCLUSIONARY_PILOT01_analysis/NOTE.md` |
| `EXCLUSIONARY_PILOT02` | N2_COMM_OFF | Failed: own-state relay-skip bug (robots never moved) | `cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT02_analysis/NOTE.md` |
| `EXCLUSIONARY_PILOT03` | N2_COMM_OFF | Structurally clean but ended via `max_runtime_s` (pre-monitor) | `cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT03_analysis/task_completion_report.json` |
| `EXCLUSIONARY_PILOT04` | N2_COMM_OFF | Failed: goal region contained static start pose (finding #2 above) | `cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT04_analysis/NOTE.md` |
| `EXCLUSIONARY_PILOT05` | N2_COMM_OFF | Failed: cmd_vel-verification race (finding #5 above) | `cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT05_analysis/NOTE.md` |
| **`EXCLUSIONARY_PILOT06`** | **N2_COMM_OFF** | **PASS -- Stage 0 mechanism validated** | `cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT06_analysis/task_completion_report.json` |
| `EXCLUSIONARY_PILOT01` | N2_COMM_ON | Structurally clean but ended via `max_runtime_s` (pre-monitor) | `cooperative_exit_n2_comm_on_EXCLUSIONARY_PILOT01_analysis/task_completion_report.json` |
| **`EXCLUSIONARY_PILOT02`** | **N2_COMM_ON** | **PASS -- Stage 0 mechanism validated** | `cooperative_exit_n2_comm_on_EXCLUSIONARY_PILOT02_analysis/task_completion_report.json` |

Relevant commits: `dc1a81e` (initial design/tooling), `b93a42e`
(Webots-path correction), `04c9c1f` (orchestrator + first 3 pilots),
`1c99aa6` (WSL-mirror test-sync fix), `67ade64` (task-completion monitor
+ visual marker), `9795145` (goal-radius + cmd_vel-verification fixes,
both PASS pilots). Native bag paths under
`/home/eamon/epuck_comm_bags/cooperative_exit_n2_comm_{off,on}_EXCLUSIONARY_PILOT*`;
Windows SHA-256-verified copies under this directory's `bags/`
(gitignored).

Central-goal scene files (`two_epuck_cooperative_exit_n2_world.wbt`,
`run_dual_head_on_clean_n2_exit.py`, in the external Webots working
directory) are **retained, unmodified** -- they are not deleted, and Stage
1 does not reuse them for the actual exit scene (a real edge/corner exit
requires new scene geometry), but they remain as the Stage 0 evidence
record.

## Stage 1 -- Two-robot shared exit pilot (IN PROGRESS)

Real edge/corner exit, asymmetric exit-discovery information,
`GoalAnnouncement`/`ExitAnnouncement` message, minimal goal-directed
navigation layer, deterministic Robot-B search strategy. `N2_EXIT_COMM_OFF`
vs `N2_EXIT_COMM_ON`, exactly 2 exclusionary pilots only. See
`edge_exit_design_20260720.md` in this directory for the frozen scene/
message/navigation design.

## Stage 2 -- Formal N2 trials (NOT STARTED)

Each condition's Trial 01 manually launched and observed by the user;
Trial 02-05 only after explicit authorization.

## Stage 3 -- Multi-robot extension (NOT STARTED)

N3/N4. Not begun, not scheduled until Stage 2 is complete and authorized.
