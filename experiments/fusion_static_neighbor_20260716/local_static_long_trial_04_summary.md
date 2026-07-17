# Local Static Obstacle Long-Course Trial 04 — Formal Summary

Date: 2026-07-16

## Experimental purpose

Trial 04 repeated the fixed-parameter, short-window wooden-obstacle scenario used
in Trial 03. `epuck1` moved from the same initial pose, while `epuck2` remained
stationary and published a valid periodic neighbour state. The controller runtime
was limited to 14 s to capture approach, avoidance, passage and initial recovery
without the later arena-boundary encounter.

## User observation

- `epuck1` passed the wooden box.
- The robot avoided toward its own right-hand side.
- No obvious oscillation was observed.
- No collision was observed.
- `epuck2` remained stationary.
- The final stop was complete.

## Controller evidence

The intended single-obstacle state sequence was recorded:

`STARTUP_HOLD -> CRUISE -> LOCAL_CLEARANCE -> LOCAL_BYPASS -> LOCAL_RECOVER -> CRUISE -> COMPLETE`

No second local-avoidance sequence occurred within the motion window.

## Quantitative results

- Bag duration: 87.960634 s.
- Valid `epuck1` state samples: 786.
- Invalid robot-state messages: 0.
- Motion start in bag: 64.656232 s.
- Last non-zero motion command: 76.639277 s.
- Path length: 0.814779 m.
- Final and maximum forward progress: 0.598130 m.
- Path efficiency: 0.734100.
- Maximum absolute lateral deviation: 0.185119 m.
- Minimum measured front range: 0.175209 m.
- Minimum odometry-derived robot-to-box surface clearance: 0.120091 m.
- Geometric collision detected by the analyzer: false.
- Wooden obstacle passed: true.
- `epuck2` displacement: 0.000000 m.
- Final velocity command zero: true.
- Peak absolute linear command: 0.015 m/s.
- Peak absolute angular command: 0.450 rad/s.
- Angular sign changes: 1.

## Interpretation and limitation

Trial 04 passed all functional acceptance criteria and is directly comparable to
Trial 03. Both use the same initial geometry, controller parameters and short
runtime. Trial 02 remains useful functional evidence but is excluded from this
short-window statistical batch because its later arena-boundary encounter changes
the path-efficiency and lateral-deviation measurements.

The clearance estimate is derived from wheel odometry transformed into the shared
frame; no independent Webots Supervisor pose or contact event was recorded. It
therefore agrees with, but does not independently replace, the visual collision
observation. Thesis-grade batches should add Supervisor ground truth.

## Acceptance status

**PASS — short-window repeat 2 of 5 for the locked central-obstacle condition.**
