# Head-on Lateral-offset 0.040 m CPA-only Configuration

Date frozen: 2026-07-16

## Purpose

Test CPA prediction and reciprocal pass-right avoidance when the encounter is not
perfectly centred. This is Phase 2 scenario 2 after the centred head-on batch.

## Geometry

- `epuck1`: `(-0.35, -0.02, 0)`.
- `epuck2`: `(0.35, +0.02, pi)`.
- Parallel opposing nominal paths are separated by 0.040 m.
- Two-robot geometric collision diameter used by the analysis: 0.070 m.
- Therefore the nominal no-avoidance path is a geometric collision case.
- Both robots are initially displaced toward their own pass-right side, so the
  fixed reciprocal pass-right action increases lateral separation.

## Locked controller condition

- Same controller parameters as centred head-on formal Trials 02–06.
- Periodic communicated state.
- Local avoidance disabled; local sensors not required.
- `max_runtime_s:=30.0`.
- `stop_after_recovery:=true`.
- `post_recovery_hold_s:=0.5`.

## Experimental sequence

Run one pilot before freezing a five-repetition formal batch. The pilot must show
CPA activation, no collision, no repeated rotation, recovery and automatic stop.
Do not change controller parameters if the pilot passes. If it fails, retain the
pilot as diagnostic evidence before any correction.
