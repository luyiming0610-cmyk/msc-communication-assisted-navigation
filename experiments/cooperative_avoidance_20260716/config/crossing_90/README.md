# Ninety-degree crossing-path controlled-realtime CPA-only protocol

Date frozen: 2026-07-16

## Geometry

- `epuck1`: starts at `(-0.35, 0.0)`, heading `0` rad, moving east.
- `epuck2`: starts at `(0.0, -0.35)`, heading `pi/2` rad, moving north.
- Equal nominal speeds give simultaneous arrival at the centre intersection if
  no avoidance occurs.
- Both robots use the same deterministic pass-right rule: epuck1 initially bends
  toward negative y; epuck2 initially bends toward positive x.

## Locked controller condition

- Periodic communicated state.
- Local avoidance disabled; local sensors not required.
- Controller parameters unchanged from the accepted centred and offset batches.
- `max_runtime_s=60.0` is a safety ceiling.
- `stop_after_recovery=true`; `post_recovery_hold_s=0.5`.
- Pre-load and full-load simulation/wall-time gates: 0.8–1.2.

## Trial 01 acceptance observations

- Both robots initiate a clear collision-avoidance response before the crossing.
- epuck1 passes toward its own right (negative y) and epuck2 toward its own right
  (positive x).
- No robot-to-robot collision, repeated spin or visible oscillation.
- Both robots recover toward their original headings.
- Both stop automatically after recovery and before the arena wall.
