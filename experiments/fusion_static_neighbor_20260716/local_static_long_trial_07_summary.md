# Local Static Obstacle Long-Course Trial 07 — Formal Summary

Date: 2026-07-16

## Experimental condition

Trial 07 was the fifth and final run in the locked short-window central-obstacle
batch. Initial poses, obstacle geometry, communication mode, controller
parameters and the 14 s runtime were unchanged from Trials 03–06.

## User observation

- `epuck1` passed the wooden box and avoided toward its own right-hand side.
- No obvious oscillation or collision was observed.
- `epuck2` remained stationary.
- The final stop was complete.
- The final stopping position appeared farther forward than the earlier runs.

## Controller evidence

The single-obstacle state sequence was:

`STARTUP_HOLD -> CRUISE -> LOCAL_CLEARANCE -> LOCAL_BYPASS -> LOCAL_RECOVER -> CRUISE -> COMPLETE`

## Quantitative results

- Bag duration: 55.980961 s.
- Valid `epuck1` state samples: 506.
- Invalid robot-state messages: 0.
- Motion start in bag: 36.871609 s.
- Last non-zero motion command: 48.853529 s.
- Path length: 0.801345 m.
- Final and maximum forward progress: 0.592686 m.
- Path efficiency: 0.739614.
- Maximum absolute lateral deviation: 0.185573 m.
- Minimum measured front range: 0.169214 m.
- Minimum odometry-derived surface clearance: 0.120547 m.
- Geometric collision detected by the analyzer: false.
- Wooden obstacle passed: true.
- `epuck2` displacement: 0.000000 m.
- Final velocity command zero: true.
- Peak absolute linear command: 0.015 m/s.
- Peak absolute angular command: 0.450 rad/s.
- Angular sign changes: 1.

## Acceptance status

**PASS — short-window repeat 5 of 5 for the locked central-obstacle condition.**
