# NEW_FIELD_SINGLE_PULSE_REVALIDATION -- RUN_ID 20260728_143937

**Classification: `NEW_FIELD_SINGLE_PULSE_REVALIDATION` / `PHYSICAL_REVALIDATION_PASS`.**

This is a single bounded pulse, one attempt only, in the new 1.40 m x
1.00 m field -- **not** a main navigation experiment, **not** a
repeatability batch, **not** an `n=5` result, **not** any part of
`COOPERATIVE_EXIT_NAVIGATION_UNDER_COMMUNICATION_IMPAIRMENT`, **not** a
cooperative multi-robot validation, and **not** evidence of physical
communication-impairment effects. Passing this revalidation authorizes
nothing beyond itself, per
`NEW_FIELD_SINGLE_PULSE_REVALIDATION_SPEC.md`'s own "What this
revalidation does not authorize" section.

## Frozen identity

- `RUN_ID`: `20260728_143937`
- Command sheet: `NEW_FIELD_SINGLE_PULSE_COMMAND_SHEET_20260728_143937.md`
- Repository HEAD used throughout the live session: `8c458dd8ec6c97e41a56daa2eda1faec696ffa9b`
- Geometry file consumed by `PRE_STACK_CHECK`: `tools/new_field_geometry_params.json` (confirmed via `GROUND_DIAGNOSTIC_PARAMS` override, verified against the historical file, not the default)

## What happened

1. Release-gate `PRE_STACK_CHECK` passed against the new-field geometry file (`TRACKED_FIELDS_OK=true`, `SOURCE_IDENTITY=CLEAN`).
2. Physical bring-up, in order: Pi driver, Pi audited command server (new JSONL path), WSL TCP bridge, `state_publisher`, WSL command-evidence recorder (`--output-root` = frozen WSL evidence root, `--flush-interval-s 1`, `--duration-s 3600`), `hil_cmd_vel_guard.py` (started `DISARMED`).
3. `GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_PASS` achieved with wheels suspended on the stand (Phase A), using a freshly generated Pi audit verdict copied and SHA-256-verified.
4. **Documented decision:** `REPEATABILITY_POSE_READINESS_PASS` was **not** invoked for this run. This run is explicitly not a repeatability baseline, and `hil_repeatability_pose_readiness.py`'s own docstring states it is only required for the repeatability baseline and that "a historical or future non-repeatability ground diagnostic never calls this module at all." Its mention in this spec's "Required gates, in order" list is treated as inherited wording from the repeatability-baseline spec, not a live blocking gate for this run -- see the separate documentation-inconsistency note below.
5. Robot placed at the marked ground start pose (body-centre `x=0.30 m, y=0.50 m, yaw=0`), wheels now bearing weight (Phase B). Post-placement `GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_PASS` re-run and passed (first attempt blocked on `PI_VERDICT_STALE`, expected and resolved by regenerating and re-copying a fresh Pi verdict).
6. Explicit `APPROVED_FOR_SINGLE_PULSE=YES` given. Armed (guard's `DISARMED` reason cleared). One bounded pulse issued via `hil_ground_single_pulse_test.py` -- phases `ZERO_HOLD -> PULSE_FORWARD -> POST_HOLD -> DONE`, self-terminated, no auto-repeat. Final command confirmed zero. Disarmed (guard's `DISARMED` reason reappeared).
7. **Operator observation:**
   ```
   MEASURED_BODY_CENTRE_X_M=approximately 0.40
   MEASURED_LONGITUDINAL_DISPLACEMENT_M=approximately 0.10
   Slight rightward drift was visually observed; the lateral
   body-centre offset was not directly measured.
   FINAL_YAW_OBSERVATION=slightly rotated / unclear from visual observation
   CORRIDOR_TAPE_CROSSED=NO (visually observed)
   STOP_LINE_X_1_20_CROSSED=NO
   UNEXPECTED_SOUND=NO
   UNEXPECTED_DIRECTION=NO
   STOPPED_COMPLETELY=YES
   All real field-boundary clearances appeared greater than 0.10 m.
   ```
   No observed exclusion criterion was triggered. The operator
   reported that all visible field-boundary clearances remained
   greater than 0.10 m, with no corridor-tape crossing, stop-line
   crossing, unexpected sound, or unexpected direction.
8. Exact-PID reverse shutdown, recorder last (see table below).
9. Offline `ground_diagnostic_post_run_verifier.py` run, entirely
   read-only against the preserved evidence: `INTEGRITY_OK=true`,
   `VERDICT=PASS`, `REASONS=[]`.

## Two distinct verdicts -- deliberately not conflated

- **Attempt-level classification (this document):** `PHYSICAL_REVALIDATION_PASS`, based on the operator's own manual observations above (no exclusion criterion triggered).
- **`ground_diagnostic_post_run_verifier.py`'s own binding technical verdict**, computed from this run's actual evidence with all five operator attestations (`--geometry-confirmed`, `--guard-sole-publisher-confirmed`, `--no-unexpected-motion-observed`, `--robot-stayed-within-measured-area`, `--run-not-interrupted`) set `true`: `VERDICT=PASS`, `REASONS=[]`. Both verdicts describe the same clean outcome; neither was invented or adjusted to fit the other.

## Motion metrics (measured outcomes, no tolerance applied)

| Metric | Value |
|---|---|
| Longitudinal displacement (odometry) | 0.09299115836620331 m |
| Lateral displacement (odometry) | 0.0 m |
| Final yaw error (odometry) | 0.0 rad |
| Stop-line clearance (computed, relative to the frozen `stop_line_distance_m=0.90 m`) | 0.8070088416337967 m |
| Manually observed longitudinal displacement (operator, x=0.40 m reference line) | approximately 0.10 m |
| Manually observed lateral body-centre offset | not directly measured (slight rightward drift visually observed) |
| Manually observed final yaw | slightly rotated / unclear from visual observation |

**Discrepancy noted, not resolved here -- preserved exactly as
reported, per this project's own established discipline:** the
operator visually observed slight rightward drift and a possibly
rotated final heading, but the odometry-derived
`lateral_displacement_m` and `final_yaw_error_rad` are both exactly
`0.0`. The odometry values are not an independent ground-truth
measurement of the robot's physical path. Therefore, the recorded zero
lateral displacement and zero yaw error do not by themselves confirm
or refute the operator's qualitative observation of slight drift.
Manual observations and odometry-derived values remain separate and
are reported without averaging or reconciliation.

## Command/evidence-chain facts (from `post_run_verification.json`)

| Fact | Value |
|---|---|
| Requested / guarded / Pi-applied max linear speed | 0.015 / 0.015 / 0.015 m/s |
| Max absolute angular speed | 0.0 rad/s |
| Guarded pulse count | 1 |
| Guarded pulse duration | 6.650453674 s |
| `validity_flags` dropout count | 0 |
| Bridge ever disconnected (during tracked pulse window) | False |
| Pre-arm nonzero command found | False |
| Guarded-vs-Pi command mismatch | 3 / 40531 checked pairs |

The 3 mismatches among 40531 checked pairs occurred at
command-transition edges and are consistent with the previously
documented asynchronous WSL/Pi pulse-edge sampling effect. This run
does not independently prove that root cause. The verifier reported
the count transparently and did not classify it as blocking, with
`REASONS=[]`.

## Shutdown

Exact-PID `kill -INT` only, never `pkill`, recorder last. One
correction made live: `state_publisher` was recorded with only one PID
initially (`678`); it was discovered mid-shutdown, via `ps --ppid`,
that -- like the driver -- `ros2 run` had forked a separate child
process (`679`, the actual node) that needed its own `kill -INT`.

| Process | PID(s) |
|---|---|
| Guard | 853 |
| state_publisher | 678 (`ros2 run` wrapper) / 679 (actual node) |
| WSL bridge | 468 |
| Audited Pi server | 697 |
| Pi driver | 618 (wrapper, already exited on its own) / 619 (node) |
| WSL command-evidence recorder | 765 (last) |

Confirmed afterward (read-only, this closeout): no matching HIL/physical process remains on the WSL side. The Pi side was confirmed gone via `pgrep` immediately after each `kill -INT` during the live session itself.

## Evidence

| File | Path | SHA-256 |
|---|---|---|
| Pi command-audit JSONL | `/home/pi/real_robot_avoidance_v1/command_audit_20260728_143937.jsonl` | `0c9a3390b45b21ebe7128038820bd87abbf68261d889ae48c52108d6d5957bf8` |
| Pi verifier verdict JSON | `/home/pi/real_robot_avoidance_v1/pi_audit_verdict_20260728_143937.json` | `c2d9172bd81eae9458230a087d403384b69bea9f661f9013550d3c02d103a227` |
| WSL command-evidence CSV | `/home/eamon/epuck_comm_bags/new_field_single_pulse_revalidation_20260728_143937/command_evidence.csv` | `f72a38737b75a37fd18083701cde1208a251f2f2153810774f1ea7676d444194` |
| WSL copy of Pi JSONL | `/home/eamon/epuck_comm_bags/new_field_single_pulse_revalidation_20260728_143937/command_audit_20260728_143937.jsonl` | `0c9a3390b45b21ebe7128038820bd87abbf68261d889ae48c52108d6d5957bf8` (identical) |
| WSL copy of Pi verdict | `/home/eamon/epuck_comm_bags/new_field_single_pulse_revalidation_20260728_143937/pi_audit_verdict_20260728_143937.json` | `c2d9172bd81eae9458230a087d403384b69bea9f661f9013550d3c02d103a227` (identical) |
| `post_run_verification.json` (derived, WSL filesystem, not copied into this repo) | `/home/eamon/epuck_comm_bags/new_field_single_pulse_revalidation_20260728_143937/post_run_verification.json` | `10c6b02bbf2e559b535f1e777eeee042d47fb5f24d67d77c3fc2c2de07a78916` |

The original Pi-side files remained in place. Verified copies were
created in the frozen WSL evidence root. None of the raw evidence
files was committed to the repository or deleted.

## Known documentation inconsistency (reported, not corrected in this closeout)

`NEW_FIELD_SINGLE_PULSE_REVALIDATION_SPEC.md`'s "Required gates, in
order" lists `REPEATABILITY_POSE_READINESS_PASS` between
`LIVE_ZERO_STATE_CHECK PASS` and ground placement. `tools/hil_repeatability_pose_readiness.py`'s
own docstring states this gate is "a SEPARATE, stricter check required
only for the repeatability baseline" and that "a historical or future
non-repeatability ground diagnostic never calls this module at all."
This spec's own status line states this run is explicitly "not a
repeatability batch." The operator decided, live, not to invoke this
gate for this run, treating its mention in the gate list as inherited
wording from the repeatability-baseline spec rather than a live
requirement. This inconsistency should be corrected in
`NEW_FIELD_SINGLE_PULSE_REVALIDATION_SPEC.md` in a future, separate,
offline documentation-only change -- not made as part of this closeout.
