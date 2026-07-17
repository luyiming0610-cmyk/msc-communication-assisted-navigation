# Local Static Obstacle Long-Course Trial 06 — Formal Summary

Date: 2026-07-16

## Experimental condition

Trial 06 was the fourth run in the locked short-window batch: identical initial
poses, a centred 0.06 m square wooden box, periodic neighbour-state publication,
calibrated local avoidance, stationary `epuck2`, and a 14 s `epuck1` controller
runtime.

## User observation

- `epuck1` passed the wooden box and avoided toward its own right-hand side.
- No obvious oscillation or collision was observed.
- `epuck2` remained stationary.
- The final stop was complete.
- The final stopping position appeared slightly farther forward than Trial 05.

## Controller evidence

The single-obstacle sequence was:

`STARTUP_HOLD -> CRUISE -> LOCAL_CLEARANCE -> LOCAL_BYPASS -> LOCAL_RECOVER -> CRUISE -> COMPLETE`

## Quantitative results

- Bag duration: 64.343777 s.
- Valid `epuck1` state samples: 582.
- Invalid robot-state messages: 0.
- Motion start in bag: 45.277595 s.
- Last non-zero motion command: 56.889099 s.
- Path length: 0.774210 m.
- Final and maximum forward progress: 0.550873 m.
- Path efficiency: 0.711530.
- Maximum absolute lateral deviation: 0.187775 m.
- Minimum measured front range: 0.176315 m.
- Minimum odometry-derived surface clearance: 0.122733 m.
- Geometric collision detected by the analyzer: false.
- Wooden obstacle passed: true.
- `epuck2` displacement: 0.000000 m.
- Final velocity command zero: true.
- Peak absolute linear command: 0.015 m/s.
- Peak absolute angular command: 0.450 rad/s.
- Angular sign changes: 1.

## Repeatability note

Trial 06 stopped 0.037102 m farther forward and accumulated 0.030650 m more path
length than Trial 05. In contrast, maximum lateral deviation differed by only
0.000222 m and minimum surface clearance by only 0.000223 m. The observed change
is therefore recorded as longitudinal stopping-position variability rather than
an unstable avoidance path. It will be included in the five-run batch statistics.

## Acceptance status

**PASS — short-window repeat 4 of 5 for the locked central-obstacle condition.**
