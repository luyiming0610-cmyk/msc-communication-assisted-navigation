# SINGLE_ROBOT_GROUND_REPEATABILITY_BASELINE -- specification (DRAFT, not yet approved)

**Status: offline design only. No physical process, no batch `RUN_ID`,
and no evidence collection has occurred as part of preparing this
document or its supporting tooling.** Prepared 2026-07-27, revised the
same day after a correction pass, following the accepted
`EXCLUSIONARY_GROUND_DIAGNOSTIC_PASS` record for RUN_ID
`20260727_102033` (closed at commit `39c99bc`). This document does not
authorize starting any process -- it requires separate, explicit human
approval before Trial 1, and again before each subsequent trial (see
"Safety stop points" below).

**Classification: `EXCLUSIONARY_REPEATABILITY_BASELINE`.** This is
**not** cooperative navigation, **not** a shared-exit trial, and
**not** a formal HIL task trial. It is a bounded, single-robot,
single-condition, straight-line, low-speed repeatability check -- it
asks only whether the already-validated command path produces similar
outcomes across repeated trials from the same start pose, before a
virtual peer or any cooperative behaviour is introduced.

## 1. Experiment objective

Evaluate whether the physical command path validated in
`first_ground_diagnostic_20260727_102033/` (guard -> bridge -> driver
-> wheels, observed via the Pi audit + WSL command-evidence recorder)
produces **repeatable** low-speed straight-ground motion across `n=5`
independent trials from the same marked start pose -- not whether it
works at all (already shown), and not whether it works *well enough*
for any formal task (no such claim is made here). Output is
descriptive: a small sample of trial outcomes and their spread, not a
statistical generalization and not a pass/fail hypothesis test on the
robot's behavior.

## 2. Frozen parameters (reused unchanged, none modified)

All values below are copied from already-confirmed sources, never
re-derived or re-measured for this experiment:

| Parameter | Value | Source |
|---|---|---|
| Test area | 0.65 m (length) x 0.25 m (width) | `ground_diagnostic_params.json.measured_geometry` |
| Start pose | `x=0.25 m, y=0.125 m, yaw=0.0 rad` | `ground_diagnostic_params.json.measured_geometry` |
| Travel direction | forward along length | `ground_diagnostic_params.json.measured_geometry.travel_direction` |
| Stop-line distance | 0.10 m | `ground_diagnostic_params.json.measured_geometry.stop_line_distance_m` |
| Min. boundary clearance | 0.10 m | `ground_diagnostic_params.json.measured_geometry.min_boundary_clearance_m` |
| Requested linear speed | 0.015 m/s | `ground_diagnostic_params.json.diagnostic_command_limits`, already physically validated |
| Guard hard linear cap | 0.02 m/s | `hil_frozen_params.json.hil_guard_limits.max_linear_speed_mps` (unchanged) |
| Angular speed | exactly `0.0` rad/s, prohibited otherwise | unchanged; no separate safety review has approved otherwise |
| Zero-hold before pulse | 1.0 s | `hil_wheel_suspension_test.py --zero-hold-s`, reused unchanged |
| Pulse duration | 2.0 s | `hil_wheel_suspension_test.py --pulse-s`, reused unchanged |
| Zero-hold after pulse | 1.0 s | `hil_wheel_suspension_test.py --post-hold-s`, reused unchanged |
| Nominal expected travel | requested speed x pulse duration = 0.015 m/s x 2.0 s = **0.03 m nominal** -- a planning estimate only, **not a displacement acceptance tolerance** (see section 9) | derived, not separately measured |
| Planned sample size | `5` valid trials (see section 6 for the completion rule) | this document |

Per requirement 6: linear speed stays at or below 0.015 m/s and
`angular.z` stays exactly `0.0` for every trial. No trial may request a
higher speed or any angular value without a separately justified,
separately approved safety review -- out of scope for this document.

## 3. Trial protocol

- **Planned sample: 5 valid trials** (see section 6's completion
  rule), run **sequentially, one at a time**, never batched or
  automated. Trial `k+1` does not begin until trial `k`'s full
  shutdown, evidence closeout, and manual return-to-start are complete
  and a fresh human approval has been given for that specific next
  attempt.
- Each attempt repeats the **entire** `GROUND_DIAGNOSTIC_RUNBOOK.md`
  sequence unchanged (steps -1 through 17: session init, pre-stack,
  physical confirmations, full bring-up, live-zero-state gate, ground
  placement, second zero recheck, explicit per-trial
  `APPROVED_FOR_SINGLE_PULSE=YES`, one pulse, immediate zero, human
  observation, exact-PID shutdown, then the offline post-run verifier
  with `--require-motion-metrics true`) -- with a fresh `RUN_ID` and
  fresh evidence paths every time. No step is skipped or assumed still
  valid from a previous attempt (e.g., a previous attempt's
  `LIVE_ZERO_STATE_CHECK_PASS` does not carry over).
- Between attempts: the robot is moved back to the exact marked start
  pose **manually, by hand**, only while every motion command is
  confirmed zero and the guard is confirmed `DISARMED`. This is a
  physical action, not a terminal command, and is itself a stop point
  (see section 8) -- the operator must confirm the guard's disarmed
  state via the same read-only checks used in step 10 of the runbook
  before touching the robot.
- This is a **single-condition** batch: no virtual peer, no paired
  condition, no comparison arm. See section 10 for why a paired
  structure is deliberately deferred rather than designed in here.

## 4. Attempts, trial numbers, and retries (defined before data
collection, per requirement 7)

- Every **attempt** -- successful, invalid, or excluded -- is
  preserved. An attempt's evidence and directory are never overwritten
  and never silently discarded, regardless of its outcome.
- Identification is **trial number + attempt number**, not trial
  number alone: `trial<N>_attempt<A>_<RUN_ID>`, e.g.
  `trial3_attempt1_20260728_101500`. `N` (1-5) is the trial *slot* this
  attempt is trying to fill; `A` (1, 2, 3, ...) counts every attempt
  made at that slot, starting at 1, incrementing on every retry
  regardless of why the previous attempt didn't count.
- **Proposed bounded retry policy (for approval, not yet in effect):**
  - A trial slot may be retried **at most 2 times** (i.e. up to 3
    total attempts: `attempt1`, `attempt2`, `attempt3`) if and only if
    the failing attempt was classified `INVALID` for a reason that is
    clearly a **procedural/instrumentation** issue, not a safety event
    and not a motion anomaly -- e.g. `NO_POSE_SAMPLE_BEFORE_PULSE_START`
    from a state-topic hiccup, or a `PI_JSONL_PATH_MISMATCH` from a
    transcription slip while copying the verdict file.
  - **Any attempt classified `EXCLUDED`** (per section 5's
    safety-abort criteria -- unexpected motion, any bridge
    disconnection, any boundary approach, any parameter deviation)
    **ends the retry policy immediately for the whole batch**, not
    just that slot: no further attempts at any trial slot are made
    without a fresh, separate approval cycle treated as a new
    incident review, exactly as `GROUND_DIAGNOSTIC_RUNBOOK.md`'s
    existing emergency procedure already requires.
  - A trial slot that exhausts its retry budget without producing a
    `VALID` attempt is reported as `UNFILLED`, counted against the
    batch's completion rule (section 6), never silently substituted
    with a different slot's data.
  - This policy is a proposal only -- it is not authorized for use
    until approved separately from the rest of this document, since it
    is the one place this specification makes a judgment call about
    when to keep trying versus when to stop.

## 5. Acceptance criteria, defined before any data collection

### Valid-attempt criteria (an attempt fills its trial slot only if
ALL of these hold; if any fails, the attempt is recorded as `INVALID`,
never silently discarded and never averaged in)

- `evaluate_verdict()` (unchanged, reused exactly) returns `PASS` for
  the attempt's own evidence, via the packaged
  `ground_diagnostic_post_run_verifier.py --require-motion-metrics true`.
- `validity_flags == 7` throughout (zero dropouts).
- Bridge never disconnected during the attempt.
- Exactly one pulse recorded on the guarded topic (no missed pulse, no
  double pulse) -- `MULTIPLE_PULSES_DETECTED` or `NO_PULSE_DETECTED`
  both make the attempt `INVALID`.
- Final command confirmed zero.
- No nonzero command before arm.
- **Motion metrics available** (`motion_metrics.available == True`,
  i.e. `motion_metrics_ok == True` with
  `--require-motion-metrics true`) -- an attempt whose pose data is
  missing, stale, or otherwise `NOT_AVAILABLE` per
  `hil_motion_repeatability_metrics.py` does not fill the slot, even if
  every other criterion above passed. This is the one criterion this
  correction pass made **mandatory** (see requirement 1) -- it was
  previously written as optional.

### Safety-abort criteria (immediately end the attempt and the whole
remaining batch; classify `EXCLUDED`, per the existing emergency
procedure in `GROUND_DIAGNOSTIC_RUNBOOK.md`)

- Any unexpected sound, drift, direction, or failure to stop
  completely (per the same operator-observation fields used in
  `SUMMARY.md` for RUN_ID `20260727_102033`).
- Any `LIVE_ZERO_STATE_CHECK_BLOCKED`, bridge disconnection, or
  recorder exit before the pulse step completes.
- Robot approaches the measured boundary clearance (0.10 m) at any
  point, or crosses the stop line.
- Any deviation from this document's frozen parameters (e.g., a
  different requested speed) without a completed, separate safety
  review.

### Displacement/drift/yaw are measured outcomes, never a pass/fail
gate (per requirement 9)

No displacement, lateral-drift, or yaw-error tolerance is invented
from the 0.03 m nominal figure in section 2. Safety/task PASS for an
attempt requires only the criteria listed above -- exactly one
authorized pulse, final command zero, no pre-arm nonzero, no validity
dropout, bridge connected throughout, no unexpected motion/direction,
and the robot remaining within the measured safe area without crossing
the stop line. `longitudinal_displacement_m`, `lateral_displacement_m`,
and `final_yaw_error_rad` are reported as measured facts in every valid
attempt's evidence and in the batch summary, never compared against an
invented number. A future tolerance could be proposed, separately
justified, and frozen before a *different* experiment relies on one --
not silently introduced here.

### Batch completion rule (corrected per requirement 6)

- Planned sample: **5 valid trials.**
- `BATCH_COMPLETE` only when **all 5 trial slots are filled** with a
  `VALID` attempt (per the retry policy in section 4).
- Fewer than 5 valid trials at the time the batch is reported is
  `INCOMPLETE_BATCH` -- never rounded up, never described as
  "complete enough."
- Descriptive values (min/max/mean/stddev per metric) may be shown for
  however many valid trials are actually available, but the batch
  summary must label them clearly as incomplete
  (`INCOMPLETE_BATCH, n_valid=<k>/5`) whenever `k<5` -- never presented
  with the same heading as a genuine 5/5 result.
- No inferential statistical generalisation is made at any sample size
  -- 5/5 valid trials still produces descriptive statistics only (this
  was already the intent; restated here because "batch complete" must
  not be read as "conclusions are now supported beyond this batch").

## 6. Evidence paths / naming convention (single condition only)

Batch identifier: `SRGRB_<YYYYMMDD>` (e.g. `SRGRB_20260728`), assigned
once at the start of the batch, distinct from each attempt's own
`RUN_ID`.

Every attempt gets its own fresh `RUN_ID` (`<YYYYMMDD>_<HHMMSS>`, same
convention as every prior run this session) and fresh, never-reused
evidence paths, exactly as `GROUND_DIAGNOSTIC_RUNBOOK.md` step -1
already requires:

```
Pi command-audit JSONL:      /home/pi/real_robot_avoidance_v1/command_audit_<RUN_ID>.jsonl
Pi verifier verdict JSON:    /home/pi/real_robot_avoidance_v1/pi_audit_verdict_<RUN_ID>.json
WSL evidence root:           /home/eamon/epuck_comm_bags/srgrb_<BATCH_ID>_trial<N>_attempt<A>_<RUN_ID>/
  command_evidence.csv
  recorder.log
  manifest.json
  pi_audit_verdict_<RUN_ID>.json          (copied)
  command_audit_<RUN_ID>.jsonl            (copied, for offline analysis only)
  post_run_verification.json              (from ground_diagnostic_post_run_verifier.py,
                                            run with --require-motion-metrics true)
```

Tracked (committed), gitignored raw evidence excluded exactly as
before. Every attempt directory is committed, including `INVALID` and
`EXCLUDED` ones -- per requirement 7, nothing is overwritten or
silently dropped:

```
experiments/07_reality_gap/hil_single_real_shared_exit_20260723/
  single_robot_ground_repeatability_baseline/
    SINGLE_ROBOT_GROUND_REPEATABILITY_BASELINE_SPEC.md   (this file)
    <BATCH_ID>/
      trial1_attempt1_<RUN_ID>/
        SUMMARY.md
        post_run_verification.json
      trial2_attempt1_<RUN_ID>/
        ...
      trial3_attempt1_<RUN_ID>/            (e.g. INVALID)
      trial3_attempt2_<RUN_ID>/            (e.g. VALID -- fills slot 3)
      ...
      trial5_attempt1_<RUN_ID>/
      BATCH_SUMMARY.md
      batch_summary.json
```

`N` in `trial<N>` is the trial slot (1-5); `A` in `attempt<A>` is the
attempt count at that slot. This is a **single-condition** naming
scheme -- there is no paired/comparison-condition slot in this
document (see section 10, "Explicitly deferred, not designed here").

### Batch-summary format (proposed)

`BATCH_SUMMARY.md` (human-readable, mirrors `SUMMARY.md`'s structure)
plus `batch_summary.json` (machine-readable) both list, per attempt:
its `RUN_ID`, trial/attempt numbers, `VALID`/`INVALID`/`EXCLUDED`
classification (and reason, for `INVALID`/`EXCLUDED`), and every
metric in section 7; plus, once reported, `BATCH_COMPLETE` or
`INCOMPLETE_BATCH, n_valid=<k>/5`, and -- only if `k>=1` -- the
descriptive statistics (min/max/mean/stddev per metric) across valid
attempts only, clearly labelled with `k` and whether the batch is
complete.

## 7. Metrics: source field -> computation -> unit -> validity rule ->
batch statistic (per requirement 10)

| Metric | Source field(s) | Computation | Unit | Validity rule | Batch statistic |
|---|---|---|---|---|---|
| Commanded max linear/angular | `linear_x`/`angular_z` on the upstream/guarded CSV rows | `compute_speed_summary()` | m/s, rad/s | Always available if the topic has any rows | min/max/mean/stddev across valid attempts |
| Pi-applied max linear/angular | Pi JSONL `command_received`/`tick_applied` records | `compute_pi_command_maxima()` | m/s, rad/s | Always available if the Pi JSONL parsed | min/max/mean/stddev across valid attempts |
| Pulse count / duration | Guarded-topic CSV rows | `count_nonzero_pulses()` (in `analyze_ground_diagnostic.py`, reused by both verifiers) | count, seconds | Exactly 1 pulse required for the attempt to be `VALID` (see section 5) | pulse count reported per attempt (always 1 for valid attempts); duration min/max/mean/stddev |
| Final-zero confirmation | Last guarded-topic CSV row | `find_nonzero_command_window().final_is_zero` | boolean | Required `True` for `VALID` | count of attempts with `True` (should equal `n_valid`) |
| `validity_flags` dropouts | State-topic `validity_flags` column | `compute_validity_flags_dropouts()` | count, seconds | Required `dropout_count == 0` for `VALID` | count of attempts with 0 dropouts (should equal `n_valid`) |
| Bridge disconnections | Bridge-status CSV rows | `compute_bridge_summary()` | count, boolean | Required `ever_disconnected == False` for `VALID` | count of attempts with no disconnection (should equal `n_valid`) |
| Command mismatch diagnostic | Guarded CSV rows + Pi `tick_applied` records | `compute_guarded_vs_pi_applied_mismatch()` | matched/mismatched pair counts | Diagnostic only, never gates `VALID`/`INVALID` | mismatch count min/max/mean/stddev across valid attempts, reported alongside, not as an acceptance input |
| Task success/abort | All of the above + `ExternalConfirmations` | `evaluate_verdict()`, unchanged | `PASS`/`FAIL`/`EXCLUDED` | The attempt is `VALID` only if this is `PASS` | count of `PASS`/`FAIL`/`EXCLUDED` per batch |
| **Longitudinal displacement** | `state_x_m`/`state_y_m` (new columns) at the pulse's start/end samples | `hil_motion_repeatability_metrics.compute_motion_metrics()` -- rotates `(dx, dy)` into the frozen `start_yaw_rad=0.0` frame | metres | Requires exactly one pulse, a start sample within `max_sample_staleness_s` (default 1.0 s) before pulse start, and an end sample at/after pulse end; else `NOT_AVAILABLE` with a specific reason -- required `available=True` for `VALID` per section 5 | min/max/mean/stddev across valid attempts (measured outcome, no tolerance) |
| **Lateral displacement/drift** | Same pose samples | Same function, orthogonal (rotated) component | metres | Same as above | min/max/mean/stddev across valid attempts |
| **Final yaw error** | `state_yaw_rad` at start/end samples | Same function, `wrap_to_pi(yaw_end - yaw_start)` | radians, wrapped to `[-pi, pi]` | Same as above | min/max/mean/stddev across valid attempts |
| Centre-to-stop-line clearance | Longitudinal displacement + frozen `stop_line_distance_m` (0.10 m) | `stop_line_distance_m - longitudinal_displacement_m`, same function | metres | Same availability rule as longitudinal displacement | min/max/mean/stddev across valid attempts; negative values (line crossed) are also an automatic safety-abort per section 5, not merely a low value |
| Manual centre-to-stop-line clearance | Tape-measure reading, as in RUN_ID `20260727_102033`'s `MEASURED_STOPPING_CLEARANCE_M` | Manual field measurement | metres | Always collected per attempt, independent of pose instrumentation | min/max/mean/stddev across valid attempts; also cross-checked against the computed clearance above where both exist |

## 8. Analysis plan

1. Run `ground_diagnostic_post_run_verifier.py --require-motion-metrics true`
   against each attempt's evidence immediately after that attempt's
   shutdown -- never batched at the end, so an evidence problem in one
   attempt is caught before the next is attempted.
2. Classify each attempt `VALID` / `INVALID` / `EXCLUDED` per section 5,
   and update the retry-policy bookkeeping in section 4.
3. When all 5 trial slots are filled (`BATCH_COMPLETE`) or the batch
   is otherwise ended (retry budget exhausted on an unfilled slot, or
   an `EXCLUDED` attempt per section 4): compute min/max/mean/stddev
   per metric in section 7 across valid attempts only, write
   `BATCH_SUMMARY.md` + `batch_summary.json`, explicitly labelled
   `BATCH_COMPLETE` or `INCOMPLETE_BATCH, n_valid=<k>/5`.
4. This analysis plan produces no new acceptance-rule function and
   does not touch `evaluate_verdict()` -- it only aggregates
   already-computed per-attempt verdicts and metrics.

## 9. Safety stop points (binding, in addition to every stop point
already in `GROUND_DIAGNOSTIC_RUNBOOK.md`, which each attempt repeats
in full)

1. Before the batch starts: explicit approval of this document.
2. Before Trial 1's physical bring-up: the same six verbal
   confirmations already required (`GROUND_DIAGNOSTIC_RUNBOOK.md`
   step 2).
3. Before each attempt's pulse: a **separate, per-attempt**
   `APPROVED_FOR_SINGLE_PULSE=YES`, never inferred from an earlier
   attempt's approval (runbook step 12, repeated in full every
   attempt).
4. After each attempt's shutdown, before manually returning the robot
   to the start marker: explicit confirmation that all motion commands
   are zero and the guard is `DISARMED`.
5. Before starting the *next* attempt's bring-up (whether a new trial
   slot or a retry): explicit approval to proceed with that specific
   attempt.
6. Any safety-abort criterion (section 5) ends the attempt and the
   entire remaining batch immediately, and ends the retry policy for
   the whole batch (section 4) -- remaining trials are not attempted
   "to complete the set."

## 10. Explicitly deferred, not designed here (paired/future work)

A future, **separate** experiment could compare this single-condition
baseline against a paired condition (a virtual peer present but not
interacting, a different surface, etc.). This document deliberately
does **not** reserve a naming slot, directory layout field, or schema
element for that comparison -- doing so inside a single-condition
batch's own schema was flagged as premature scope creep in the
previous draft and has been removed. If and when a paired experiment
is proposed, it will define its own naming convention (e.g. a
`condition` field distinguishing it from `SRGRB_<YYYYMMDD>`) in its
own specification document, without requiring any change to this
one's evidence layout.

## 11. Minimal new offline tooling (implemented and tested this
correction pass; mandatory per requirement 1, not optional)

**Reusable as-is, no change:** the measured area, coordinate
convention, marked start point, `hil_cmd_vel_guard.py`,
`wsl_epuck_tcp_bridge_sensors.py`, `pi_epuck_tcp_server_sensors_audited.py`,
`state_publisher.py`, the Pi audit server and
`pi_ground_diagnostic_audit_verifier.py`, `hil_wheel_suspension_test.py`,
`hil_ground_diagnostic_session.py`, `hil_frozen_params.json`,
`ground_diagnostic_params.json`'s measured geometry, and the entire
`GROUND_DIAGNOSTIC_RUNBOOK.md` procedure.

**Implemented (additive only, no protocol/controller/guard/bridge
change):**

- `hil_command_evidence_recorder.py`: three additive CSV columns,
  `state_x_m`/`state_y_m`/`state_yaw_rad`, appended to `CSV_FIELDS`
  and populated in `_on_state()` from the SAME `/epuck1/state`
  subscription the recorder already held (`msg.x_m`/`msg.y_m`/
  `msg.yaw_rad` -- `state_publisher.py` has always computed these from
  its own real `/odom` subscription; this recorder simply did not
  record them before). No new subscription, no `EpuckState` change, no
  `state_publisher.py` change. Every existing column and its semantics
  are unchanged; a CSV produced before this change (missing all three
  columns) still parses.
- `analyze_ground_diagnostic.py`: `load_wsl_csv_rows()` additionally
  converts `state_x_m`/`state_y_m`/`state_yaw_rad` to float when the
  columns are present (`.get()`-based, so their absence in an
  old-format CSV is not an error). `count_nonzero_pulses()`/
  `PulseWindow` moved here (from `ground_diagnostic_post_run_verifier.py`,
  unchanged behavior) so the new motion-metrics module can reuse pulse
  detection without a circular import.
- `hil_motion_repeatability_metrics.py` (new, pure, no ROS): computes
  longitudinal/lateral displacement (rotated into the frozen
  `start_yaw_rad` frame), final yaw error (correctly wrapped to
  `[-pi, pi]`), and stop-line clearance, from the three new columns
  and the existing single-pulse detection. Explicit, documented rules
  for start/end sample selection and staleness -- see the module's
  docstring and section 7's table. Deliberately separate from
  `analyze_ground_diagnostic.compute_odometry_displacement()`, whose
  `NotAvailable` behavior remains asserted by an existing test and
  stays true for evidence that genuinely lacks pose data.
- `ground_diagnostic_post_run_verifier.py`: computes motion metrics
  for every run (always reported when available), and adds
  `--require-motion-metrics` (default `false`) plus
  `motion_metrics_ok`/`motion_metrics_required` as fields orthogonal
  to `integrity_ok` and `diagnostic_verdict` -- motion metrics never
  feed `evaluate_verdict()` and never change `diagnostic_verdict`,
  satisfying requirement 9's prohibition on inventing a PASS tolerance.

**Backward compatibility, verified:** re-running
`ground_diagnostic_post_run_verifier.py` (with `--require-motion-metrics`
left at its default `false`) against the already-accepted RUN_ID
`20260727_102033` evidence (which has no pose columns at all)
reproduces `INTEGRITY_OK=true`, `VERDICT=PASS`, `REASONS=[]`, and the
already-reconciled `68/12112` mismatch count unchanged;
`motion_metrics.available=False` (`reason=NO_POSE_SAMPLES_AVAILABLE`),
`motion_metrics_ok=true` (not required for historical evidence).

**Not yet implemented, deliberately out of scope for this document:**
`BATCH_SUMMARY.md`/`batch_summary.json` generation tooling (section 6's
proposed format) -- to be built, if approved, before the batch starts,
as its own small aggregation script over already-produced
`post_run_verification.json` files; produces no new acceptance logic.

## 12. Explicitly not authorized by this document

- No physical process may be started from this document alone.
- No batch `RUN_ID` or attempt may begin without the separate
  approvals in section 9, including approval of the bounded retry
  policy proposed in section 4.
- Passing (or failing) this baseline does not authorize introducing a
  virtual peer, cooperative behaviour, a higher speed, any nonzero
  angular value, a paired condition, or a formal navigation/shared-exit
  trial -- those remain governed entirely by `../HIL_SAFETY_CHECKLIST.md`
  and `../HIL_KNOWN_LIMITATIONS_AND_READINESS_20260723.md`, unchanged
  by this document.
