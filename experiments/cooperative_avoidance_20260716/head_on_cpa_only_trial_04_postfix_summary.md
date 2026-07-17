# CPA-only Head-on Trial 04 — Post-fix Formal Summary

Date: 2026-07-16

## Classification

**PASS — include as post-fix CPA-only formal repetition 3/5.**

## Experimental condition

- Clean Webots arena with no wooden box.
- Initial shared poses: `epuck1 (-0.35, 0, 0)` and
  `epuck2 (0.35, 0, pi)`.
- Periodic communicated `EpuckState` messages.
- Communication/CPA-only reciprocal pass-right control.
- Local obstacle avoidance disabled and local sensors not required.
- `stop_after_recovery:=true` with `post_recovery_hold_s:=0.5`.
- Maximum runtime safety limit: 30 s.
- Controller parameters unchanged from formal Trials 02 and 03.

## User observation

All requested behavioural checks were normal: simultaneous avoidance, no repeated
rotation, no collision, no visible oscillation, automatic stop, no wall impact
and no obvious final-position asymmetry. All interfaces were then stopped.

## Controller-log evidence

- Both robots showed the same reciprocal avoidance-pass and recovery behaviour.
- Completion timestamps were 1784216520.471539 for `epuck2` and
  1784216520.472877 for `epuck1`, a difference of approximately 0.001339 s.
- Both completion messages state
  `cooperative recovery completed; commanding zero`.

## Automated rosbag results

- Bag duration: 48.360781 s, including pre-controller recording time.
- Valid paired state samples: 897.
- State samples: 448 for `epuck1` and 450 for `epuck2`.
- Invalid state messages: 0 for both robots.
- Minimum centre separation: 0.131077 m.
- Minimum geometric safety margin above the 0.070 m threshold: 0.061077 m.
- Robot-to-robot geometric collision detected: false.
- Final centre separation: 0.383661 m.
- Motion-start skew: 0.061326 s.
- Last-motion-command skew: 0.000335 s.
- Peak absolute linear command: 0.025 m/s for both robots.
- Peak absolute angular command: 0.635426 rad/s for `epuck1` and 0.650 rad/s
  for `epuck2`.
- Significant angular-command sign changes: 3 per robot.

The minimum separation is lower than in formal Trials 02 and 03 but remains
0.061077 m above the locked geometric collision threshold. It is retained as
normal trial-to-trial variation; no parameter is changed within the batch.

## Decision

All behavioural and automated acceptance checks passed. Trial 04 is retained as
formal repetition 3/5. Continue the identical locked condition for Trial 05.

## Evidence

- Log: `logs/head_on_cpa_only_trial_04_postfix.log`
- Bag: `bags/head_on_cpa_only_trial_04_postfix/`
- Automated metrics:
  `bags/head_on_cpa_only_trial_04_postfix/analysis/summary.json`

## Limitation

Clearance and collision values use paired odometry rather than Webots Supervisor
ground truth/contact events. Supervisor evidence remains required before final
geometric claims.
