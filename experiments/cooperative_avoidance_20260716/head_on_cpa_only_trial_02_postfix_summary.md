# CPA-only Head-on Trial 02 — Post-fix Formal Summary

Date: 2026-07-16

## Classification

**PASS — include as post-fix CPA-only formal repetition 1/5.**

This run validates both corrective actions introduced after diagnostic Trial 01:
the skipped-heading-target fix removed the prolonged in-place rotation, and the
post-recovery completion mode stopped both robots before either reached a wall.

## Experimental condition

- Clean Webots arena with no wooden box.
- Initial shared poses: `epuck1 (-0.35, 0, 0)` and
  `epuck2 (0.35, 0, pi)`.
- Periodic communicated `EpuckState` messages.
- Communication/CPA-only reciprocal pass-right control.
- Local obstacle avoidance disabled and local sensors not required.
- `stop_after_recovery:=true` with `post_recovery_hold_s:=0.5`.
- Maximum runtime safety limit: 30 s.

## User observation

- Both robots turned to their own right at nearly the same time.
- `epuck1` no longer rotated repeatedly in place.
- No robot-to-robot collision occurred.
- No obvious oscillation was visible.
- Both robots completed heading recovery.
- Both controllers stopped automatically after `recovery completed`.
- Neither robot collided with the arena wall; both stopped before reaching it.

## Controller-log evidence

- Both robots entered `AVOID_TURN` at the same logged centre distance of
  0.305 m.
- Both then followed the same sequence:
  `AVOID_TURN -> AVOID_PASS -> RECOVER -> CRUISE -> COMPLETE`.
- Completion timestamps differed by approximately 0.082 s:
  `epuck2` at 1784214842.444 and `epuck1` at 1784214842.526.
- Both completion messages explicitly state
  `cooperative recovery completed; commanding zero`.

## Automated rosbag results

- Bag duration: 55.853044 s, including pre-controller recording time.
- Valid paired state samples: 998.
- State samples: 500 for `epuck1` and 499 for `epuck2`.
- Invalid state messages: 0 for both robots.
- Minimum centre separation: 0.154557 m.
- Minimum geometric safety margin above the 0.070 m threshold: 0.084557 m.
- Robot-to-robot geometric collision detected: false.
- Final centre separation: 0.351715 m.
- Motion-start skew: 0.151325 s.
- Last-motion-command skew: 0.087782 s.
- Peak absolute linear command: 0.025 m/s for both robots.
- Peak absolute angular command: 0.650 rad/s for both robots.
- Significant angular-command sign changes: 3 per robot.

The three angular-command sign changes are retained as a quantitative smoothness
metric. In this run they are symmetric and align with the planned avoidance and
recovery phase changes; the user observed no visible oscillation.

## Decision

All predeclared behavioural acceptance checks passed. No controller parameter is
changed before the remaining four repetitions. The same clean world, initial
poses, commands and recording topics must be reused for Trials 03–06.

## Evidence

- Log: `logs/head_on_cpa_only_trial_02_postfix.log`
- Bag: `bags/head_on_cpa_only_trial_02_postfix/`
- Automated metrics:
  `bags/head_on_cpa_only_trial_02_postfix/analysis/summary.json`

## Limitation

Clearance and collision values currently use paired odometry rather than Webots
Supervisor ground truth/contact events. They are adequate for this controlled
repeat comparison but must be supplemented before final geometric claims.
