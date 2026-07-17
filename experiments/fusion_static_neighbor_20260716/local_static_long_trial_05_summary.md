# Local Static Obstacle Long-Course Trial 05 — Formal Summary

Date: 2026-07-16

## Experimental condition

Trial 05 repeated the locked short-window condition used in Trials 03 and 04:
a centred 0.06 m square wooden box, identical initial poses, periodic state
publication, calibrated local avoidance, a stationary communicated neighbour and
a 14 s `epuck1` controller runtime.

## User observation

- `epuck1` passed the wooden box.
- The robot avoided toward its own right-hand side.
- No obvious oscillation was observed.
- No collision was observed.
- `epuck2` remained stationary.
- The final stop was complete.

## Controller evidence

The recorded state sequence was:

`STARTUP_HOLD -> CRUISE -> LOCAL_CLEARANCE -> LOCAL_BYPASS -> LOCAL_RECOVER -> CRUISE -> COMPLETE`

Only one local-obstacle encounter occurred in the motion window.

## Quantitative results

- Bag duration: 70.700999 s.
- Valid `epuck1` state samples: 648.
- Invalid robot-state messages: 0.
- Motion start in bag: 40.898099 s.
- Last non-zero motion command: 52.854986 s.
- Path length: 0.743559 m.
- Final and maximum forward progress: 0.513771 m.
- Path efficiency: 0.690962.
- Maximum absolute lateral deviation: 0.187553 m.
- Minimum measured front range: 0.169308 m.
- Minimum odometry-derived surface clearance: 0.122510 m.
- Geometric collision detected by the analyzer: false.
- Wooden obstacle passed: true.
- `epuck2` displacement: 0.000000 m.
- Final velocity command zero: true.
- Peak absolute linear command: 0.015 m/s.
- Peak absolute angular command: 0.450 rad/s.
- Angular sign changes: 1.

## Interpretation and limitation

Trial 05 passed all locked-condition acceptance criteria and is the third directly
comparable successful short-window repeat. The geometric metrics remain derived
from transformed wheel odometry; final thesis batches should additionally record
Webots Supervisor pose and contact-event ground truth.

## Acceptance status

**PASS — short-window repeat 3 of 5 for the locked central-obstacle condition.**
