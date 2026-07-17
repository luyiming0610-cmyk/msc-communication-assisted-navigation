# Cooperative avoidance diagnostic: head_on_trial_02_recovery

Date: 2026-07-16  
Classification: diagnostic run; exclude from successful-trial statistics

## Observed behaviour

- The two robots did not begin cruising at the same time.
- Avoidance began earlier than in trial 01.
- Both robots avoided collision and used their body-frame right side.
- During recovery, both robots oscillated left and right while advancing.

## Log-based root cause

At approximately simulation log time `1784197983.05`, both controllers received
a transient state pair whose positions were represented as zero while odometry
was invalid. Because the first controller revision did not gate CPA computation
on `FLAG_ODOM_VALID`, both computed `distance=0.000 m` and entered
`AVOID_TURN` at a true separation near `0.58 m`.

Later, after `AVOID_PASS` entered `RECOVER`, separation remained below the
unlatched `0.34 m` proximity trigger. The controllers consequently alternated
between `RECOVER`, `CRUISE`, and a new `AVOID_TURN`. The alternating angular
commands (`+0.45` and `-0.65 rad/s`) explain the observed weave.

## Corrective action

- Reject CPA inputs unless both messages carry `FLAG_ODOM_VALID` and the
  expected protocol version.
- Increase startup discovery hold from `1.5 s` to `5.0 s`.
- Gate proximity risk on positive closing speed.
- Latch encounter completion through recovery, with re-arm only after `0.45 m`.
- Reduce recovery heading gain and angular saturation.

The corrected implementation passed 17 unit tests, including a regression test
that a separating pair inside the proximity threshold must not retrigger
avoidance.

## Evidence files

- `bags/head_on_trial_02_recovery/head_on_trial_02_recovery_0.db3`
- `bags/head_on_trial_02_recovery/metadata.yaml`
- `logs/head_on_trial_02_recovery.log`
