# NEW_FIELD_SINGLE_PULSE_REVALIDATION -- specification (DRAFT, offline preparation only)

**Status: offline preparation only. No physical process, no RUN_ID, no
Pi contact, and no evidence collection has occurred as part of
preparing this document or its supporting tooling.** Prepared
2026-07-28, following `SRGRB_20260727_02` Trial 1 Attempt 2's exclusion
(`MIN_BOUNDARY_CLEARANCE_VIOLATION`, measured 0.05 m clearance against
the old field's 0.10 m minimum) in the old 0.65 m x 0.25 m field.

**Classification: `EXCLUSIONARY_NEW_FIELD_SINGLE_PULSE_REVALIDATION`.**
This is a **single bounded pulse, one attempt only** -- explicitly
**not** a repeatability batch (no `n=5`), **not** a formal
navigation/shared-exit trial, and **not** cooperative navigation. Its
sole purpose is to confirm that the new, larger, marked physical field
and the unchanged command parameters (0.015 m/s, ~6.67 s, angular
0.0) produce a single pulse that stays safely within the new field's
clearances, before any repeatability batch or exit-navigation work is
attempted there.

`SRGRB_20260727`, `SRGRB_20260727_02`, and the old field's own
`ground_diagnostic_params.json`/`FIELD_MEASUREMENT_FORM.md` are
historical records, unchanged and unread by this experiment.

## Frozen geometry (see `new_field_geometry_params.json`,
`NEW_FIELD_MEASUREMENT_FORM.md`)

| Parameter | Value |
|---|---|
| Test area | 1.40 m (length) x 1.00 m (width) |
| Coordinate origin | rear-left physical corner |
| Start pose (robot body centre) | x=0.30 m, y=0.50 m, yaw=0.0 rad |
| Travel direction | forward along the 1.40 m length |
| Stop line (absolute x) | 1.20 m |
| `stop_line_distance_m` (from start) | 0.90 m (= 1.20 - 0.30) |
| Physical front boundary (absolute x) | 1.40 m |
| Central corridor | y = 0.30 m to y = 0.70 m -- human-visible tape reference only |
| Manual reference line (absolute x) | 0.40 m -- manual distance-measurement aid only, never a gate |
| Minimum real-boundary body-edge clearance | 0.10 m |
| Requested linear speed | 0.015 m/s (unchanged from every prior run) |
| Guard hard linear cap | 0.02 m/s (`hil_frozen_params.json`, unchanged, untouched) |
| Angular speed | exactly 0.0 rad/s, prohibited otherwise |
| Target commanded travel | 0.10 m (nominal, a planning value -- not a claim about actual physical displacement) |
| Pulse duration | 6.67 s (= 0.10 / 0.015) |
| Zero-hold before/after pulse | 1.0 s each |

## Configuration selection (binding)

This experiment's geometry is selected via the `GROUND_DIAGNOSTIC_PARAMS`
environment variable, **never** by editing `ground_diagnostic_params.json`
(the old field's own file, left completely unchanged):

```bash
GROUND_DIAGNOSTIC_PARAMS=<repo path>/experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/new_field_geometry_params.json \
  bash run_ground_diagnostic_preflight.sh pre-stack
```

`run_ground_diagnostic_preflight.sh`'s existing, unmodified
`PARAMS_FILE="${GROUND_DIAGNOSTIC_PARAMS:-...}"` line and
`hil_preflight.check_required_fields_ready()` are entirely
schema-driven (no hardcoded field names) -- this new file's own,
extended `required_before_ground_motion` (12 paths, including the two
new corridor bounds) is honored with zero code changes. This is
verified end-to-end, not assumed, by
`test_run_ground_diagnostic_preflight_new_field_override_e2e.sh`,
which proves: (1) a deliberately-broken copy of this file blocks on
exactly the broken field; (2) a fully-confirmed copy passes; (3) the
shipped file (not yet confirmed) blocks on exactly the two stable-venue
booleans; (4) the default invocation (no override) never mentions this
field's own paths at all.

The post-run verifier is likewise invoked with this field's own value,
never the old field's default:

```bash
python3 ground_diagnostic_post_run_verifier.py ... --stop-line-distance-m 0.90 --require-motion-metrics true ...
```

## Ground pulse tool (binding)

`hil_ground_single_pulse_test.py` -- **not**
`hil_wheel_suspension_test.py`, whose name and established safety
guarantee are for wheels-suspended diagnostics only. The new tool
reuses (imports, does not duplicate) that tool's own `compute_phase()`
state machine, wrapped in a clearly ground-named node
(`hil_ground_single_pulse_test`, log prefixes
`HIL_GROUND_SINGLE_PULSE_TEST_START/PHASE/DONE`). Publishes only to
`cmd_vel_unguarded`, no CLI angular override exists, `--pulse-linear-mps`
and `--pulse-s` are both required (no silent default), publishes zero
three times and exits on its own -- no auto-repeat, no second pulse
from one invocation.

**This specification does not itself state an abbreviated executable
pulse or recorder command.** The exact, frozen, per-session invocation
-- including the frozen RUN_ID and evidence paths -- is recorded in
exactly one place: the dedicated command sheet
[`NEW_FIELD_SINGLE_PULSE_COMMAND_SHEET_<RUN_ID>.md`](./NEW_FIELD_SINGLE_PULSE_COMMAND_SHEET_20260728_143937.md)
for the session in question. This avoids a second, potentially
drifting copy of the executable command living alongside the design
rationale here.

## Emergency power-off arrangement (confirmed 2026-07-28)

Manually confirmed, `safety.emergency_stop_position_confirmed=true` in
`new_field_geometry_params.json`:

- Operator position: beside and slightly behind the robot, outside the
  intended forward travel path.
- The operator has an unobstructed view of the robot and the complete
  marked field.
- The robot's physical power button remains within the operator's
  immediate arm's reach throughout the pulse.
- Immediate power-removal method: press the robot's physical power
  button immediately at the first sign of unexpected wheel motion,
  abnormal sound, rotation, sudden acceleration, unknown command
  source, or evidence-chain failure.
- The operator remains at this position for the complete live session
  and will not move away from the physical power button while the
  robot is powered.

This is a stable fact about the venue/operator arrangement, confirmed
once for this field -- it is separate from, and does not substitute
for, the four genuinely per-session confirmations (floor condition,
travel path clear, operator present, Wi-Fi checked) tracked in
`hil_ground_diagnostic_session.py`, which must still be freshly
reconfirmed for each live session.

## Acceptance and exclusion rules

**Required gates, in order** (all reused unchanged from the existing
runbook/tooling): `PRE_STACK_CHECK PASS` (against this experiment's own
geometry file) -> bring-up -> `LIVE_ZERO_STATE_CHECK PASS` ->
`REPEATABILITY_POSE_READINESS_PASS` -> ground placement ->
post-placement zero recheck -> explicit, separate
`APPROVED_FOR_SINGLE_PULSE=YES` -> arm -> one pulse -> confirm zero ->
disarm -> human observation -> exact-PID shutdown, recorder last ->
post-run verifier.

**Exclusion (any one -> `EXCLUDED`, no silent retry):**
- Body-edge clearance to the nearest **real physical field boundary**
  (the true 1.40 m / 1.00 m edges) measured below 0.10 m.
- **Post-pulse human observation** that any part of the robot body
  crossed either corridor tape edge (y=0.30 m or y=0.70 m). The
  corridor tape is not sensed by the robot in any way and is never a
  runtime gate -- this is a retrospective, human-only exclusion
  criterion, decided after the pulse completes and the robot has
  stopped.
- The stop line (absolute x=1.20 m) is crossed.
- Any `BLOCKED` gate result, recorder exit, connection loss, power
  instability, nonzero pre-arm command, unexpected movement/sound, or
  unexpected direction.

**No invented tolerance:** displacement, lateral drift, and yaw are
reported as measured facts only (odometry-derived, per
`hil_motion_repeatability_metrics.py`) and compared honestly against
manual measurements (per `SRGRB_20260727_02` Attempt 2's own
established "two distinct verdicts, never conflated" pattern) --
never averaged away, never silently resolved.

## Required manual observations after the pulse

1. Measured body-centre longitudinal position/displacement using the
   x=0.40 m manual reference line.
2. Lateral body-centre offset from the marked central line.
3. Final yaw observation.
4. Front-edge-to-forward-boundary clearance.
5. Left/right body-edge clearance to the real field boundaries.
6. Whether any body part crossed either corridor tape edge.
7. Whether the stop line was crossed.
8. Unexpected sound, drift, or direction.

Odometry-derived displacement/lateral displacement/yaw (from
`post_run_verification.json`'s `motion_metrics`) are compared against
1-3 above and reported honestly, including any disagreement -- not
resolved or averaged away.

## What this revalidation does not authorize

Passing this single-pulse revalidation authorizes nothing beyond
itself: not a resumed `SRGRB_20260727_02`, not an automatic new `n=5`
repeatability batch under the new field, not any parameter change, and
not any part of the `COOPERATIVE_EXIT_NAVIGATION_UNDER_COMMUNICATION_IMPAIRMENT`
design (frozen separately, Stage 1 simulation study first, per the
approved implementation order). A fresh `n=5` batch under this field
would require its own explicit approval and its own `BATCH_ID`.
