# Head-on Lateral-offset 0.040 m CPA-only Formal Trial 01

Date: 2026-07-16

## Classification

**PASS — include as lateral-offset formal repetition 1/5.**

## Geometry clarification

This scenario is still head-on in heading: `epuck1` points at 0 rad and `epuck2`
at pi rad. The experimental factor is a 0.040 m offset between the two parallel
nominal path centre-lines, not an angular crossing.

- `epuck1 (-0.35, -0.02, 0)`.
- `epuck2 (0.35, +0.02, pi)`.
- Expected initial centre distance:
  `sqrt(0.70^2 + 0.04^2) = 0.701141 m`.
- Logged initial distance: 0.701 m, confirming the offset world was loaded.
- The 0.040 m no-avoidance path separation is below the 0.070 m geometric
  collision diameter.

Because 0.040 m is small relative to the robot body and arena floor pattern, the
offset is difficult to distinguish visually from the centred head-on condition.
The later 90-degree crossing scenario is the test in which headings are visibly
not face-to-face.

## Locked condition

- Clean arena with no wooden box.
- Periodic communicated state.
- Communication/CPA-only reciprocal pass-right control.
- Local obstacle avoidance disabled and local sensors not required.
- Same controller parameters as the centred head-on batch and offset pilot.
- Automatic stop after recovery with a 0.5 s hold.

## User observation

All behavioural checks were normal. The user noted correctly that the robot
headings still appeared face-to-face; no failure was reported.

## Controller and timing evidence

- `epuck1` entered `AVOID_TURN` at a logged centre distance of 0.328 m; the other
  robot entered the same reciprocal manoeuvre between its 0.5 s log samples.
- Avoidance-command onset skew: 0.000121 s.
- `epuck1` initial clockwise-turn duration: 0.440677 s.
- `epuck2` initial clockwise-turn duration: 0.476001 s.
- Turn-duration difference: 0.035324 s.
- Clockwise peak angular command: 0.650000 rad/s for both robots.
- Controller completion timestamps differed by approximately 0.018234 s.

## Automated rosbag results

- Bag duration: 33.382888 s, including pre-controller recording time.
- Valid paired state samples: 602.
- Invalid state messages: 0 for both robots.
- Minimum centre separation: 0.181022 m.
- Minimum safety margin above the 0.070 m threshold: 0.111022 m.
- Robot-to-robot geometric collision detected: false.
- Final centre separation: 0.331967 m.
- Motion-start skew: 0.115440 s.
- Last-motion-command skew: 0.018464 s.
- Peak absolute linear command: 0.025 m/s for both robots.
- Peak absolute angular command: 0.650 rad/s for both robots.
- Significant angular-command sign changes: 3 per robot.

## Decision

The trial is a valid 0.040 m lateral-offset repetition and is retained as formal
sample 1/5. No world or controller parameter changes are made within the batch.

## Evidence

- Log: `logs/head_on_offset_040_formal_trial_01.log`
- Bag: `bags/head_on_offset_040_formal_trial_01/`
- Automated metrics:
  `bags/head_on_offset_040_formal_trial_01/analysis/summary.json`
- Frozen configuration: `config/head_on_lateral_offset_040/`
