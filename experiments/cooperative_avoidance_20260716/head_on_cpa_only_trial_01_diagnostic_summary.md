# CPA-only Head-on Trial 01 — Diagnostic Summary

Date: 2026-07-16

## Classification

**Diagnostic failure — exclude from the post-fix CPA-only statistical batch.**

The robots avoided a mutual geometric collision, but the run failed the complete
task criteria because the reciprocal turn was asymmetric and both robots later
collided with the arena wall.

## Experimental condition

- Clean Webots arena with no wooden boxes.
- Initial shared poses: `epuck1 (-0.35, 0, 0)` and
  `epuck2 (0.35, 0, pi)`.
- Periodic communicated `EpuckState` messages.
- Local obstacle avoidance explicitly disabled.
- CPA-only reciprocal pass-right controller.
- Configured maximum runtime: 45 s.

## User observation

- Near the mutual encounter, the lower-left robot rotated excessively in place.
- The upper-right robot executed a right-side avoidance manoeuvre.
- After separation, the lower-left robot recovered and continued forward.
- Both robots eventually collided with the arena wall instead of stopping before
  the boundary.

## Automated rosbag results

- Valid paired state samples: 1420.
- Invalid state messages: 0 for both robots.
- Minimum centre separation: 0.128917 m.
- Minimum geometric safety margin above the 0.070 m threshold: 0.058917 m.
- Robot-to-robot geometric collision detected: false.
- Motion-start skew: 0.034895 s.
- Peak absolute linear command: 0.025 m/s.
- Peak absolute angular command: 0.650 rad/s for `epuck1` and 0.636537 rad/s for
  `epuck2`.
- Significant angular sign changes: 3 per robot.
- Final centre separation: 4.076901 m; this is invalid as a task metric because
  post-encounter boundary collisions contaminated the trajectory.

## Root cause 1 — skipped turn-completion tolerance

Both robots began significant turn commands at effectively the same bag time.
The CPA trigger was therefore not the source of the observed asymmetry.

The controller ended `AVOID_TURN` only when a discrete heading sample landed
within ±0.08 rad of the pass heading. For `epuck1`, the sampled pass-heading error
jumped from -0.115 rad to +0.285 rad, skipping the entire tolerance band. The
controller did not recognize that the target had been crossed and continued the
clockwise turn through multiple rotations. It reached the tolerance only 3.654 s
after turn-command onset. `epuck2` happened to sample -0.079 rad and ended its
turn after 0.197 s.

## Root cause 2 — unsuitable fixed experiment termination

After mutual avoidance and recovery, both controllers returned to `CRUISE`. With
local avoidance disabled, wall range measurements were intentionally ignored.
The 45 s runtime therefore allowed both robots to continue into the arena walls.
This is a task-termination design error, not a failure of communicated CPA to
separate the robots.

## Corrective action

1. Detect a clockwise pass-heading sign crossing in addition to the tolerance
   check, so discrete feedback cannot skip the target.
2. Add an optional `stop_after_recovery` mode with a short post-recovery hold.
3. Keep both features disabled by default to preserve previous experiment
   behaviour; enable automatic completion explicitly for the CPA-only batch.
4. Repeat a diagnostic validation before starting the post-fix statistical batch.

## Evidence

- Log: `logs/head_on_cpa_only_trial_01.log`
- Bag: `bags/head_on_cpa_only_trial_01/`
- Automated metrics: `bags/head_on_cpa_only_trial_01/analysis/summary.json`
