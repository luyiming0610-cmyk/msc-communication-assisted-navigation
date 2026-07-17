# Head-on Lateral-offset 0.040 m CPA-only Pilot Trial 01

Date: 2026-07-16

## Classification

**PASS — scenario and parameters frozen; exclude this pilot from the subsequent
five-repetition formal batch.**

## Geometry and controller condition

- `epuck1 (-0.35, -0.02, 0)`.
- `epuck2 (0.35, +0.02, pi)`.
- Nominal path offset: 0.040 m, below the 0.070 m geometric collision diameter.
- Clean arena with no wooden box.
- Periodic state communication and communication/CPA-only reciprocal pass-right
  control.
- Local obstacle avoidance disabled and local sensors not required.
- Controller parameters identical to the completed centred head-on batch.
- Automatic stop after recovery with a 0.5 s hold.

## User observation

All requested checks were normal: both robots avoided, no repeated rotation,
collision, visible oscillation or wall impact occurred, recovery and automatic
stop completed, and the run was then closed normally.

## Controller and high-rate timing evidence

- Both controllers entered avoidance at a logged centre distance of approximately
  0.313 m.
- Avoidance-command onset skew: 0.007228 s.
- `epuck1` initial clockwise-turn duration: 0.455780 s.
- `epuck2` initial clockwise-turn duration: 0.469635 s.
- Turn-duration difference: 0.013855 s.
- Clockwise peak angular command: 0.650000 rad/s for both robots.
- Controller completion timestamps differed by approximately 0.016862 s.

## Automated rosbag results

- Bag duration: 51.592755 s, including pre-controller recording time.
- Valid paired state samples: 876.
- State samples: 459 for `epuck1` and 441 for `epuck2`.
- Invalid state messages: 0 for both robots.
- Minimum centre separation: 0.195333 m.
- Minimum safety margin above the 0.070 m threshold: 0.125333 m.
- Robot-to-robot geometric collision detected: false.
- Final centre separation: 0.327323 m.
- Motion-start skew: 0.057405 s.
- Last-motion-command skew: 0.019491 s.
- Peak absolute linear command: 0.025 m/s for both robots.
- Peak absolute angular command: 0.650 rad/s for both robots.
- Significant angular-command sign changes: 3 per robot.

## Decision

The pilot satisfies the scenario gate. Freeze the world, origins and controller
parameters and begin a separate formal batch of five repetitions. The pilot is
retained as setup-validation evidence and is not pooled with the formal batch.

## Evidence

- Log: `logs/head_on_offset_040_pilot_trial_01.log`
- Bag: `bags/head_on_offset_040_pilot_trial_01/`
- Automated metrics:
  `bags/head_on_offset_040_pilot_trial_01/analysis/summary.json`
- Frozen configuration: `config/head_on_lateral_offset_040/`
