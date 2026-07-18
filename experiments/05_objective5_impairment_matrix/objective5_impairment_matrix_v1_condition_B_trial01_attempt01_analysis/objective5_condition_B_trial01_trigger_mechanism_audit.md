# Condition B Trial 01 trigger-mechanism audit (offline, no Webots run)

Same offline method used for the Condition A batch: read
`controller.log` and the preserved bag for this one trial, no
Webots run, no controller/relay/matrix-parameter/world change.

## Frozen config

`enable_peer_avoidance=True`, `enable_local_avoidance=True`,
`require_local_sensors=True` -- identical to Condition A (the
controller launch config and `cooperative_avoider.py` were not touched
between conditions). Condition B's own impairment: `delay_s=0.20`,
`jitter_s=0.0`, `drop_probability=0.0`, outage disabled.

## Event counts

| AVOID_TURN | AVOID_PASS | RECOVER | LOCAL_* (all types) | SAFE_STOP_STALE | first trigger_reason |
|---|---|---|---|---|---|
| 8 | 86 | 12 | **0** | 4 (startup, see below) | PREDICTED_CPA |

Every `LOCAL_*` event type (`LOCAL_FRONT_DANGER`/`_WARN`,
`LOCAL_LEFT_SIDE`/`_RIGHT_SIDE`, `LOCAL_NARROW`, `LOCAL_RECOVER`,
`LOCAL_RECOVERY_READY`, `SENSOR_INVALID`, `SAFE_STOP_LOCAL_SENSORS`,
plus the v4-specific side-track/clear variants) is **0**.
`LOCAL_CLEARANCE` and `LOCAL_BYPASS` remain `NOT_APPLICABLE` -- neither
exists in the current controller_v4 code (confirmed during the
Condition A audit; the controller was not modified since).

## Timeline (ros_time, from controller.log and the bag-based recompute)

- `21.609s` -- geometric CPA/proximity trigger condition first true (raw position/velocity recompute across the whole bag, `analyze_trigger_reason.py`).
- `22.960s`-`23.200s` -- epuck1: `WAITING->SAFE_STOP_STALE->STARTUP_HOLD`.
- `25.600s`-`25.700s` -- epuck2: `WAITING->SAFE_STOP_STALE->STARTUP_HOLD`.
- `36.660s` -- epuck1's first committed `CRUISE->AVOID_TURN` transition.
- `37.600s` -- epuck2's first `AVOID_TURN->AVOID_PASS` transition.

**These are not contradictory.** The `21.609s` figure is a pure
geometric recomputation across every row in the bag -- it marks the
earliest instant the raw CPA/proximity math would classify as a
conflict, independent of what the controller's own state machine was
doing at that exact instant (still `WAITING`/`STARTUP_HOLD` here). The
controller's actual committed avoidance response (`AVOID_TURN`) does
not begin until `36.660s`, well after both the geometric trigger point
and the `SAFE_STOP_STALE`/`STARTUP_HOLD` transitions. `SAFE_STOP_STALE`
is confined entirely to the pre-encounter startup window and has no
overlap with the actual encounter.

## First-trigger evidence

`trigger_reason=PREDICTED_CPA`, `trigger_distance_m=0.3636`,
`tcpa_s=4.0` (at horizon), `dcpa_m=0.1325`,
`closing_speed_mps=0.0578`, `minimum_center_separation_m=0.147767`
(matches the orchestrator's own `min_interrobot_distance_m
=0.14777153762172363` to within ~4e-6m -- cross-validated, consistent
with the same small methodology gap seen on Condition A Trial 04).

## Interpretation

**PURE_COMMUNICATION_CPA_AVOIDANCE.** Communication CPA
(`PREDICTED_CPA`) triggered the encounter; every `LOCAL_*` counter is
0. Not `LOCAL_FALLBACK`, not `SAFE_DEGRADATION`. Communication CPA
remained the sole avoidance mechanism under a 0.20s fixed relay delay
-- the expected result for a single-factor delay condition.

## `NOT_APPLICABLE`/`NOT_MEASURABLE` items (not guessed)

- `LOCAL_CLEARANCE`, `LOCAL_BYPASS` -- `NOT_APPLICABLE`, confirmed absent from the current controller_v4 code.
- `ps1`-`ps7` individual topic existence -- `NOT_MEASURABLE` beyond `ps0`/`tof` (only those two are named in the orchestrator's startup gate); not re-audited here since the launch config is byte-identical to the already-audited Condition A config.
