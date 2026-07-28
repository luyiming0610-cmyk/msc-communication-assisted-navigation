# SRGRB_20260727_02 -- Trial 1, Attempt 2 -- RUN_ID 20260727_153600

**Classification: `EXCLUDED`.** Reason: `MIN_BOUNDARY_CLEARANCE_VIOLATION`.

**This is a safety-margin finding, not an instrumentation failure.**
Attempt 1's `POSE_READINESS_FLUSH_FRESHNESS_INCOMPATIBLE` issue is
**confirmed fixed** by this attempt: `REPEATABILITY_POSE_READINESS_PASS`
was achieved cleanly (`REASONS=[]`) using the frozen
`--flush-interval-s 0.2`/`--output-root` recorder invocation, and the
motion metrics this whole batch exists to collect were successfully
captured end-to-end for the first time. The exclusion here is
independent and physical: the operator's post-pulse measurement found
the robot's clearance to the forward test-area boundary to be **0.05 m**,
**below** the frozen `min_boundary_clearance_m=0.10 m` in
`ground_diagnostic_params.json`.

## Frozen identity

- `spec_commit`: `4d777590d7189fedaffb105eddfd5003ea1cb40e`
- `execution_code_commit`: `8515dbb0cc6dba30dfc342bb215d453ce3b6286c`
- Repository HEAD at start of this attempt: `760c74061fa8df0a0f294560f74dd55b8f21f694`

## What happened

1. Bring-up proceeded through the full stack -- Pi driver, audited Pi
   command server, WSL TCP bridge, `state_publisher` -- all four
   started and confirmed running.
2. The WSL command-evidence recorder was started with the frozen,
   corrected invocation: `--output-root
   /home/eamon/epuck_comm_bags/srgrb_20260727_02_trial1_attempt2_20260727_153600
   --flush-interval-s 0.2` (never `--output-csv`, never `1.0`). Process
   survived, manifest/log/CSV paths all agreed exactly with the
   frozen root, and the CSV was observed growing.
3. Guard started `DISARMED`. `GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_PASS`
   achieved. `REPEATABILITY_POSE_READINESS_PASS` achieved with
   `REASONS=[]` -- the flush-interval fix resolved the prior spurious
   `BLOCKED` result.
4. Robot placed at the marked ground start pose. Post-placement
   zero-output recheck passed (`validity_flags=7`, sole publisher,
   zero output).
5. Explicit `APPROVED_FOR_SINGLE_PULSE=YES` given. Armed. One bounded
   pulse issued: requested/guarded/Pi-applied linear all `0.015 m/s`,
   angular `0.0`, guarded pulse duration `2.000460022 s`. Final command
   confirmed zero. Disarmed.
6. **Operator observation:**
   ```
   PULSE_VISUAL_OBSERVATION=EXPECTED_WITH_MINOR_DRIFT
   MOTION=Robot moved forward a short distance at low speed and drifted slightly to the right.
   STOPPED_COMPLETELY=YES
   UNEXPECTED_SOUND=NO
   VISIBLE_DRIFT=YES
   UNEXPECTED_DIRECTION=NO
   MEASURED_STOPPING_CLEARANCE_M=0.05
   MEASUREMENT_REFERENCE=Distance from the robot front edge to the forward test-area boundary after the pulse.
   ```
   `0.05 m` is below the frozen `min_boundary_clearance_m=0.10 m` --
   this is the safety-abort criterion this attempt's `EXCLUDED`
   classification is based on. The run was stopped cleanly with no
   further motion, arm, or pulse commands issued after this
   observation was reported.

## Two distinct verdicts -- deliberately not conflated

- **Attempt-level classification (this document, the manifest, and
  the batch record):** `EXCLUDED`, reason
  `MIN_BOUNDARY_CLEARANCE_VIOLATION` -- the operator's own physical
  measurement against the frozen safety parameter.
- **`ground_diagnostic_post_run_verifier.py`'s own binding technical
  verdict** (`evaluate_verdict()`, unchanged, reused exactly), computed
  from this run's actual evidence with
  `--robot-stayed-within-measured-area false` (reflecting the
  measured clearance violation above): `VERDICT=FAIL`,
  `REASONS=['ROBOT_LEFT_MEASURED_SAFE_AREA']`. `evaluate_verdict()`'s
  own vocabulary is `PASS`/`FAIL`/`EXCLUDED` and does not have a
  `MIN_BOUNDARY_CLEARANCE_VIOLATION` reason string of its own --
  `ROBOT_LEFT_MEASURED_SAFE_AREA` is its existing, correct
  representation of exactly this fact. Both verdicts describe the same
  underlying safety-margin finding; neither was invented or adjusted
  to fit the other.

## Motion metrics (measured outcomes, no tolerance applied -- per this
batch's own specification, section 5)

| Metric | Value |
|---|---|
| Longitudinal displacement | 0.0292 m |
| Lateral displacement (odometry) | 0.0 m |
| Final yaw error (odometry) | 0.0 rad |
| Stop-line clearance (computed, relative to the frozen 0.10 m stop-line-distance parameter) | 0.0708 m |
| Manually measured boundary clearance (operator, front edge to forward test-area boundary) | 0.05 m |

**Discrepancy noted, not resolved here:** the operator visually
observed rightward drift (`VISIBLE_DRIFT=YES`), but the odometry-derived
`lateral_displacement_m` and `final_yaw_error_rad` are both exactly
`0.0`. Wheel-odometry dead reckoning may simply lack the resolution to
register a small drift the human eye can see, or the visual
observation may include subjective/parallax error -- this document
does not assert either explanation, only that the two measurements
disagree and both are reported as-is.

**The computed `stop_line_clearance_m` (0.0708 m) and the manually
measured boundary clearance (0.05 m) are not the same quantity and are
not in conflict:** the former is computed relative to the frozen
`stop_line_distance_m=0.10 m` parameter (an early, conservative marked
line closer to the start pose), while the latter is the operator's
direct measurement to the actual forward test-area boundary (a
farther, physically real edge). The stop line was not crossed; the
boundary clearance margin was.

## Shutdown

Exact-PID `kill -INT` only, never `pkill`, recorder last:

| Process | PID(s) |
|---|---|
| Guard | 1016 |
| state_publisher | 844 (wrapper) / 845 (node) |
| WSL bridge | 765 |
| Audited Pi server | 862 |
| Pi driver | 785 (wrapper) / 786 (node) |
| WSL command-evidence recorder | 929 (last) |

Confirmed afterward: no related process remains on either machine;
`/cmd_vel` absent from `ros2 topic list`.

## Evidence

| File | Path | SHA-256 |
|---|---|---|
| Pi command-audit JSONL | `/home/pi/real_robot_avoidance_v1/command_audit_20260727_153600.jsonl` | `06e144069d2b7870ebecfd18ddfaa5861ce8a7a55d880a4afe7c9c24f6002e7c` |
| Pi verifier verdict JSON | `/home/pi/real_robot_avoidance_v1/pi_audit_verdict_20260727_153600.json` | `944ed647a406443fa62bc43dc77ded747c526cded3129ab284bcca782e5d54b0` |
| WSL command-evidence CSV | `/home/eamon/epuck_comm_bags/srgrb_20260727_02_trial1_attempt2_20260727_153600/command_evidence.csv` | `f7c02e9231b0c5bb1d1b8f8742fe15bd3475fdf4caef3bc1d714dff6ceb72318` |
| WSL copy of Pi JSONL | `.../command_audit_20260727_153600.jsonl` | `06e144069d2b7870ebecfd18ddfaa5861ce8a7a55d880a4afe7c9c24f6002e7c` (identical) |
| WSL copy of Pi verdict | `.../pi_audit_verdict_20260727_153600.json` | `944ed647a406443fa62bc43dc77ded747c526cded3129ab284bcca782e5d54b0` (identical) |
| `post_run_verification.json` (tracked, derived) | `trial1_attempt2_20260727_153600/post_run_verification.json` | `2d1d3d8cd0094fd4e37c79710d8bd0acfc6c029932462ebb7c886dcc6f5a619a` |

Raw CSV/JSONL remain local to the WSL/Pi filesystems and gitignored --
never committed.

## Effect on the batch

Per the approved retry policy, this `EXCLUDED` attempt **immediately
ends the entire `SRGRB_20260727_02` batch**
(`BATCH_ABORTED_EXCLUDED`) and requires a fresh approval cycle before
any further attempt -- see `BATCH_SUMMARY.md` in this batch's
directory. Trial slot 1 remains unfilled; slots 2-5 were never
attempted.
