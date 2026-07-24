# HIL known limitation and readiness status (2026-07-23)

This document consolidates evidence from four separate stationary
diagnostics into one accurate finding, defines how any future ground
pilot must handle it, and states current HIL readiness. It does not
change any code, threshold, or timing.

## Evidence consolidated

1. **Excluded overrun stationary diagnostic**
   (`stationary_physical_diagnostic_20260723/SUMMARY.md`, marked
   `INCOMPLETE_DIAGNOSTIC`/`EXCLUDED` due to an orchestration bug, not a
   physical finding) -- first observed 15 `validity_flags=0` samples in
   ~300s of real `/epuck1/state` traffic, spaced ~32.4-33.0s apart.
2. **Correctly bounded 300s stationary diagnostic**
   (`stationary_physical_diagnostic_20260723_attempt02/SUMMARY.md`,
   `STATIONARY_PHYSICAL_DIAGNOSTIC` PASS) -- independently reproduced
   11 `validity_flags=0` samples at the same ~32-33s spacing.
3. **120s targeted topic diagnostic**
   (`targeted_stationary_diagnostic_20260723/SUMMARY.md`,
   `TARGETED_STATIONARY_DIAGNOSTIC`) -- millisecond-precision capture
   of 4 events at 16.138s/49.285s/82.346s/115.459s (33.15s/33.06s/33.11s
   apart). Every event showed a **common ~0.917-1.033s gap across every
   physical-input topic simultaneously** (`/odom`, `/tof`, all six
   `FLAG_IR_VALID` sensors) -- not a `state_publisher`-only artifact.
4. **130s instrumented bridge substitution**
   (`bridge_instrumentation_substitution_20260723/SUMMARY.md`, bounded,
   exact-PID, rolled back) -- reproduced the same ~32-33s period a
   fourth time, this occurrence manifesting as a much smaller ~30-100ms
   1Hz-timer-ordering wobble rather than a full topic freeze. 13 real
   GC cycles captured, all sub-millisecond (30-650µs).

## Accurate finding

- **A repeatable ~32-33s disturbance exists**, confirmed independently
  four times across three different diagnostic tools and two different
  bridge processes (original and instrumented).
- **Its severity varies** from a ~30-100ms single-timer-ordering wobble
  (mildest observed) to a ~0.9-1.0s common freeze across every physical
  input topic simultaneously (most severe observed). Both magnitudes
  share the same ~32-33s period.
- **Python garbage collection is ruled out** for the instrumented run:
  every real GC cycle captured was sub-millisecond, far too short to
  explain any observed gap.
- **rclpy single-threaded-executor timer-ordering jitter, WSL2/host OS
  scheduling contention, and upstream (Wi-Fi/TCP) transport delay
  remain hypotheses, not proven root causes.** No instrumentation run
  has isolated which of these (or some other, uninstrumented factor) is
  the actual trigger, or why the same trigger produces such different
  magnitudes on different occurrences.

**This is not solved. The stack is not being claimed fault-free.**

## `validity_flags=0` is real, and the guard already fails closed

`validity_flags=0` occurs in the genuine `/epuck1/state` stream itself
(confirmed via direct subscription, not a recording-pipeline artifact).
`hil_cmd_vel_guard.py`'s existing, already-unit-tested
`PHYSICAL_STATE_INVALID_FLAGS` check (requiring
`FLAG_ODOM_VALID | FLAG_IR_VALID | FLAG_TOF_VALID`, value 7) forces a
zero-velocity command for the duration of any such event, and resumes
only once `validity_flags` returns to 7 and every other guard condition
is satisfied. This is existing, already-verified behavior -- not a new
mitigation invented for this finding, and not itself a fix for the
underlying disturbance.

## Dissertation discussion point

This is recorded as a known HIL/physical-reality-gap limitation: a
real, repeatable, moderate-to-severe intermittent data-freshness gap of
unresolved origin exists in the physical bridge path, at a consistent
~32-33s period, with variable severity. Any HIL ground result must be
read against this backdrop -- the guard's fail-closed handling of it is
part of the system's designed safety behavior, but repeated
interruptions of this kind could still affect task-completion outcomes
independent of whether communication helps or hurts (see acceptance
handling below).

## Ground-pilot acceptance handling (binding for any future ground pilot)

1. Any `validity_flags != 7` sample from the real robot's `EpuckState`
   must command zero velocity -- already guaranteed by the existing
   guard, and must not be worked around.
2. Every occurrence and its stop duration must be recorded in the
   trial's evidence (not summarized away) -- this is a normal expected
   category of event given the finding above, not something to hide or
   average out.
3. **No automatic PASS** if repeated interruptions from this
   disturbance prevent the task from actually completing within the
   trial's runtime -- a trial that fails to complete because of this
   must be marked `FAIL`/`EXCLUDED` with the reason stated, exactly
   like any other genuine failure mode, never silently reclassified.
4. No threshold relaxation (e.g., loosening `required_validity_flags`,
   `physical_state_timeout_s`, or `sensor_timeout_s`) and no hidden or
   dropped samples, ever, to make a trial look cleaner than it was.

## HIL readiness status (2026-07-23)

| Item | Status |
|---|---|
| Suspended-wheel **linear** diagnostic | **PASS** (`suspended_wheel_diagnostic_20260723/SUMMARY.md`) |
| Suspended-wheel **angular** diagnostic | **PASS**, at a temporary, test-scoped ±0.1 rad/s (`angular_suspension_diagnostic_20260723/SUMMARY.md`) -- **not** a measured ground-motion turning-rate limit |
| Stationary communication/health diagnostics | **Complete** (this document's 4 consolidated sources) |
| Field geometry | **Not frozen** -- `hil_frozen_params.json`'s `field_geometry.*` remain `UNCONFIRMED_PHYSICAL_MEASUREMENT` |
| Ground angular cap | **Not confirmed** -- `max_angular_speed_rps` remains `UNCONFIRMED_PHYSICAL_MEASUREMENT`; the two suspended-wheel values above were explicitly test-scoped, never adopted as this |
| Ground motion / first ground-motion pilot | **Not yet authorized** -- field geometry and the first ground pilot remain the two outstanding items before any ground trial; the command-evidence chain being active is a precondition for that pilot, not a substitute for it |
| `UNEXPECTED_PHYSICAL_MOTION` safety incident (2026-07-23) | **Audited, root cause NOT_MEASURABLE, not solved** -- see `safety_incident_unexpected_motion_20260723/SUMMARY.md`. Added binding preconditions to `HIL_SAFETY_CHECKLIST.md` for any future powered session |
| Command-evidence chain (Pi audit + WSL recorder) | **PASS** -- `command_evidence_activation_pass_20260724/SUMMARY.md`. Suspended, zero-motion validation only, not a ground trial |
| Computer-to-e-puck zero-command delivery and auditing | **VERIFIED** -- guard confirmed sole `/cmd_vel` publisher and `armed=False` throughout; Pi JSONL (34,458 records) and WSL CSV (22,812 rows) both independently confirmed zero nonzero commands for the entire session |
| First ground diagnostic (bounded, low-speed, straight-line only) | **PREPARED_OFFLINE / NOT_RUN** -- see `first_ground_diagnostic/FIRST_GROUND_DIAGNOSTIC_SPEC.md`. Spec, field-measurement form, parameter template, preflight, runbook, offline analysis, and acceptance-rule tests are all in place and pass the isolated test suite; the diagnostic itself requires the field measurements below plus supervised execution, neither of which has happened |
| First ground diagnostic -- required measurements | **Outstanding** -- `first_ground_diagnostic/FIELD_MEASUREMENT_FORM.md` and `tools/ground_diagnostic_params.json`'s `measured_geometry` remain `UNCONFIRMED_PHYSICAL_MEASUREMENT`; this diagnostic's angular motion is prohibited by design (fixed `0.0`), independent of the still-unconfirmed `hil_frozen_params.json` ground angular cap |
| Ground navigation / formal HIL trial | **Not started** -- unaffected by the first ground diagnostic's preparation or (once run) its outcome; still gated by field geometry and a separately measured/justified ground angular cap |
| First ground-motion pilot | **Still outstanding** -- not yet authorized; command-evidence chain being active is a precondition for it, not a substitute |

## Topic-contract accuracy check (finding only -- not implemented)

`HIL_TOPIC_CONTRACT.md` names the physical robot's state topic as
`/epuck5809/state` and several related topics as `/epuck5809/...`
(`nav_intent`, `cmd_vel_unguarded`, `cmd_vel`). Every live diagnostic
this session instead used, and confirmed working end-to-end, the real
topic names: `/epuck1/state` (matches `state_publisher.py`'s actual
launch remap `-r state:=/epuck1/state`, used in every bring-up this
session) and un-namespaced `/cmd_vel_unguarded` / `/cmd_vel` (matching
`hil_cmd_vel_guard.py`'s actual CLI defaults, exercised live in the
suspended-wheel and angular diagnostics). `HIL_TOPIC_CONTRACT.md` also
still states the guard's validity requirement as `FLAG_ODOM_VALID`
alone -- stale since the guard was hardened to require
`FLAG_ODOM_VALID | FLAG_IR_VALID | FLAG_TOF_VALID` (commit
`9e2b586`, "fix: harden hardware-in-loop physical preflight").

**Proposed smallest correction** (not implemented, awaiting approval):
replace `/epuck5809/state`, `/epuck5809/nav_intent`,
`/epuck5809/cmd_vel_unguarded`, `/epuck5809/cmd_vel` with `/epuck1/state`,
`/epuck1/nav_intent`, `/cmd_vel_unguarded`, `/cmd_vel` respectively in
`HIL_TOPIC_CONTRACT.md`'s table, and update its "Validity and freshness
rules" section to state the full `FLAG_ODOM_VALID | FLAG_IR_VALID |
FLAG_TOF_VALID` requirement instead of `FLAG_ODOM_VALID` alone. No
other file needs this correction: `HIL_SAFETY_CHECKLIST.md` and
`hil_frozen_params.json` were already updated for the validity
requirement in the earlier hardening commit; only `HIL_TOPIC_CONTRACT.md`
was missed.
