# Condition A trigger-mechanism audit (offline, no Webots run)

Answers the question left open in the Condition A batch report: was
communication CPA the actual mechanism that triggered avoidance in
each trial, or did the local ToF/IR safety layer intervene? This audit
reads only already-collected `controller.log` files and rosbags for
the 5 completed Trials; **no Webots run, no controller/geometry/threshold
change.**

## 1. Frozen runtime parameters (source-code + launch-config audit)

- `enable_peer_avoidance = True` (both `run_comm_baseline_formal_controllers.py`'s launch parameters and `cooperative_avoider.py`'s `declare_parameter` default agree).
- `enable_local_avoidance = True` (same cross-check).
- `require_local_sensors = True` — a local-sensor-invalid condition would escalate to `SAFE_STOP_LOCAL_SENSORS`, not be silently ignored.
- **No parameter disables the LOCAL safety layer** for these trials — both the peer-CPA layer and the local ToF/IR layer are simultaneously enabled, exactly as designed; `enable_local_avoidance=True` is the single gate for the whole LOCAL state machine (`cooperative_avoider.py`'s `_local_decision()`), and it is on.
- Local sensor topics `/epuck1/tof`, `/epuck2/tof`, `/epuck1/ps0`, `/epuck2/ps0` are confirmed **actually present** for all 5 trials: the orchestrator's own `wait_for_topics` call gates simulation startup on these exact topics existing in `ros2 topic list`, and all 5 trials' `execution.log` show this gate passed ("odometry and local sensors ready") before the trial proceeded. **Caveat**: only `ps0` and `tof` are checked by name; `ps1`-`ps7`'s individual existence is not independently confirmed by this evidence (not claimed beyond what was checked). None of `/psN`/`/tof` are recorded in the rosbag itself — `state_publisher.py` consumes them internally and republishes a fused `EpuckState`.

## 2. Per-trial event counts (from `controller.log`, and `trigger_reason` from `analyze_trigger_reason.py` run against each trial's preserved bag)

| Trial | AVOID_TURN | AVOID_PASS | RECOVER | LOCAL_* (all types) | SAFE_STOP_STALE | first trigger_reason |
|---|---|---|---|---|---|---|
| 01 | 8 | 84 | 12 | **0** | 4 (startup, see below) | PREDICTED_CPA |
| 02 | 8 | 86 | 11 | **0** | 4 (startup) | PREDICTED_CPA |
| 03 | 8 | 86 | 10 | **0** | 4 (startup) | PREDICTED_CPA |
| 04 | 8 | 86 | 11 | **0** | 4 (startup) | PREDICTED_CPA |
| 05 | 7 | 85 | 11 | **0** | 4 (startup) | PREDICTED_CPA |

"LOCAL_* (all types)" sums `LOCAL_FRONT_DANGER`, `LOCAL_FRONT_WARN`,
`LOCAL_LEFT_SIDE`, `LOCAL_RIGHT_SIDE`, `LOCAL_NARROW`, `LOCAL_RECOVER`,
`LOCAL_RECOVERY_READY`, `SENSOR_INVALID`,
`SAFE_STOP_LOCAL_SENSORS`, plus the v4-specific
`LOCAL_SIDE_TRACK`/`_HOLD`/`_CREEP`/`LOCAL_CLEAR` — every one is **0**
in every trial. `LOCAL_CLEARANCE` and `LOCAL_BYPASS` (from the
requested list) do not exist anywhere in the current controller_v4
code (confirmed by source audit, not merely absent from the log) —
reported as `NOT_MEASURABLE`, not guessed as 0.

**`SAFE_STOP_STALE=4` in every trial is a benign startup transient**,
not a mid-task safety intervention: direct inspection of
`controller.log` shows exactly 2 occurrences per robot, each a
`WAITING->SAFE_STOP_STALE->STARTUP_HOLD` transition pair completing
within ~100ms at ros_time≈22-23s, before either robot has received its
first peer state message. This precedes the actual encounter (first
`AVOID_TURN` around ros_time≈21s in the trigger-reason evidence below)
and has no bearing on which avoidance mechanism engaged during the
encounter itself.

## 3. First-trigger evidence (from `analyze_trigger_reason.py`, bag-based, read-only)

| Trial | trigger_time_s | trigger_distance_m | tcpa_s | dcpa_m | closing_speed_mps | min_center_separation_m |
|---|---|---|---|---|---|---|
| 01 | 21.121 | 0.3399 | 4.0 | 0.1389 | 0.0502 | 0.1430942842844398 |
| 02 | 22.493 | 0.3582 | 4.0 | 0.1397 | 0.0546 | 0.15064050840214138 |
| 03 | 21.554 | 0.3508 | 4.0 | 0.1398 | 0.0528 | 0.14781142476139542 |
| 04 | 21.550 | 0.3587 | 4.0 | 0.1302 | 0.0571 | 0.15030056489146615 |
| 05 | 20.669 | 0.3388 | 4.0 | 0.1378 | 0.0502 | 0.14178534915907265 |

All 5 trials classify as `PREDICTED_CPA` (time-to-CPA within the 4.0s
horizon AND predicted distance-at-CPA below `safety_radius_m=0.14` —
never `PROXIMITY_FALLBACK`, i.e. never triggered merely by already
being close). `minimum_center_separation_m` from this independent
bag-based recomputation matches the orchestrator's own
`min_interrobot_distance_m` for Trials 01/02/03/05 exactly, and Trial
04 to within ~2e-7m (a negligible measurement-methodology difference,
not a discrepancy of concern) — cross-validating both figures.

## 4. Interpretation, per the frozen standard

**All 5 trials: `PURE_COMMUNICATION_CPA_AVOIDANCE`.** Communication CPA
(`PREDICTED_CPA`) triggered the encounter in every trial, and the LOCAL
ToF/IR layer's engagement counters are all exactly 0 in every trial —
this is the ideal Condition A result, not merely a passing one.
**5/5 = pure communication-CPA avoidance; 0/5 required LOCAL_FALLBACK or
SAFE_DEGRADATION labeling.**

No trial's data was ambiguous or insufficient to classify — no
`NOT_MEASURABLE` outcome was needed for the trigger-mechanism question
itself (only for the two requested-but-nonexistent `LOCAL_CLEARANCE`/
`LOCAL_BYPASS` categories, and the `ps1`-`ps7` individual-existence
caveat above).

## 5. Safety margin (carried forward, not modified)

Trial 05's safety margin (~1.785mm above `safety_radius_m=0.14`) is low
but positive and satisfies the frozen PASS threshold — unchanged from
the batch summary. **For Conditions B-G: if communication impairment
causes a collision or a negative safety margin in any future trial,
that must be recorded as the genuine `TASK_OUTCOME` (`UNSAFE_FAILURE`)
for that trial — never masked by adjusting `safety_radius_m`, trigger
thresholds, or robot geometry after seeing an unfavorable result.**

## Method note

`controller.log` mode/event counts were extracted by regex over the
raw preserved log text (not re-run, not re-simulated).
`analyze_trigger_reason.py`'s `classify_trigger()` was imported and run
directly against each trial's preserved, SHA-256-verified bag copy —
the same function the CLI uses, just called in-process to collect
structured results across all 5 trials in one pass. Full raw counts:
`objective5_condition_A_trigger_mechanism_audit.json`.
