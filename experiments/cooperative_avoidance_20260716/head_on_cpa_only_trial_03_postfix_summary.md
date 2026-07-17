# CPA-only Head-on Trial 03 — Post-fix Formal Summary

Date: 2026-07-16

## Classification

**PASS — include as post-fix CPA-only formal repetition 2/5.**

## Experimental condition

- Clean Webots arena with no wooden box.
- Initial shared poses: `epuck1 (-0.35, 0, 0)` and
  `epuck2 (0.35, 0, pi)`.
- Periodic communicated `EpuckState` messages.
- Communication/CPA-only reciprocal pass-right control.
- Local obstacle avoidance disabled and local sensors not required.
- `stop_after_recovery:=true` with `post_recovery_hold_s:=0.5`.
- Maximum runtime safety limit: 30 s.
- Controller parameters unchanged from formal Trial 02.

## User observation

- Both robots avoided simultaneously to their own right.
- No repeated in-place rotation occurred.
- No robot-to-robot collision occurred.
- No obvious oscillation was visible.
- Both controllers stopped automatically after recovery.
- Neither robot collided with a wall.
- No obvious final-position asymmetry was visible.

## Controller-log evidence

- Both robots entered the reciprocal avoidance manoeuvre at a logged centre
  distance of approximately 0.301 m.
- Both completed the avoidance-pass and heading-recovery sequence.
- Completion timestamps were 1784215522.011960 for `epuck1` and
  1784215522.014416 for `epuck2`, a difference of approximately 0.002456 s.
- Both completion messages state
  `cooperative recovery completed; commanding zero`.

## Automated rosbag results

- Bag duration: 64.157258 s, including pre-controller recording time.
- Valid paired state samples: 1173.
- State samples: 586 for `epuck1` and 588 for `epuck2`.
- Invalid state messages: 0 for both robots.
- Minimum centre separation: 0.159625 m.
- Minimum geometric safety margin above the 0.070 m threshold: 0.089625 m.
- Robot-to-robot geometric collision detected: false.
- Final centre separation: 0.364550 m.
- Motion-start skew: 0.122349 s.
- Last-motion-command skew: 0.000015 s.
- Peak absolute linear command: 0.025 m/s for both robots.
- Peak absolute angular command: 0.650 rad/s for both robots.
- Significant angular-command sign changes: 3 per robot.

The symmetric three sign changes agree with the planned turn, pass-heading
correction and recovery phases. No visible oscillation was observed.

## Decision

All behavioural and automated checks passed. Trial 03 is retained as formal
repetition 2/5. No controller parameter is changed before Trial 04.

## Evidence

- Log: `logs/head_on_cpa_only_trial_03_postfix.log`
- Bag: `bags/head_on_cpa_only_trial_03_postfix/`
- Automated metrics:
  `bags/head_on_cpa_only_trial_03_postfix/analysis/summary.json`

## Limitation

Clearance and collision values use paired odometry rather than Webots Supervisor
ground truth/contact events. Supervisor evidence remains required before final
geometric claims.
