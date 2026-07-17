# Cooperative Avoidance Experiment Index — 2026-07-16

| Trial | Classification | Collision | Main finding | Use in analysis |
|---|---|---:|---|---|
| `head_on_trial_01` | Successful feasibility run | No | Reciprocal avoidance worked; run ended while still in avoidance-pass phase | Preliminary/reference |
| `head_on_trial_02_diagnostic` | Invalid diagnostic run | No | Invalid zero odometry caused mode re-entry and visible oscillation | Exclude from performance statistics; retain for fault analysis |
| `head_on_trial_03_corrected` | Valid successful run | No | Synchronized start, one avoidance event, no obvious oscillation, complete stop | Include |
| `head_on_trial_04_smoothed` | Valid smoothing diagnostic | No | Slew limiting removed visible stutter, but 35 s runtime ended before recovery | Include for smoothness comparison; exclude from completed-task rate |
| `head_on_trial_05_smoothed_recovery` | Valid completed success | No | Full avoid-pass-recover-cruise sequence, smooth return to heading | Include as first formal smoothed nominal repetition |
| `head_on_cpa_only_trial_01` | Diagnostic failure | No robot-to-robot collision; wall collisions | Skipped heading-target tolerance caused prolonged rotation; fixed runtime allowed post-recovery wall impact | Exclude from post-fix statistics; retain for root-cause evidence |
| `head_on_cpa_only_trial_02_postfix` | Valid post-fix CPA-only success | No | Symmetric pass-right avoidance, no visible oscillation, complete recovery and automatic stop before walls | Include as CPA-only formal repetition 1/5 |
| `head_on_cpa_only_trial_03_postfix` | Valid post-fix CPA-only success | No | Simultaneous reciprocal avoidance, no visible oscillation, recovery and near-simultaneous automatic stop | Include as CPA-only formal repetition 2/5 |
| `head_on_cpa_only_trial_04_postfix` | Valid post-fix CPA-only success | No | All behavioural checks normal; bag-derived analysis confirmed positive clearance and synchronized final commands | Include as CPA-only formal repetition 3/5 |
| `head_on_cpa_only_trial_05_postfix` | Valid post-fix CPA-only success with minor timing asymmetry | No | Avoidance onset was simultaneous, but `epuck2` completed the initial turn 0.114 s earlier; safety and completion were unaffected | Include as CPA-only formal repetition 4/5 and retain timing flag |
| `head_on_cpa_only_trial_06_postfix` | Valid post-fix CPA-only success | No | No visible speed asymmetry; high-rate commands confirmed a 1.43 ms initial-turn difference | Include as CPA-only formal repetition 5/5 |
| `head_on_offset_040_pilot_trial_01` | Valid lateral-offset setup pilot | No | 0.040 m offset triggered synchronized CPA avoidance and produced positive clearance with automatic completion | Retain as pilot; exclude from the five-repetition offset batch |
| `head_on_offset_040_formal_trial_01` | Valid lateral-offset formal success | No | Logged initial geometry confirmed 0.040 m path offset; synchronized avoidance and automatic completion passed | Include as lateral-offset formal repetition 1/5 |
| `head_on_offset_040_realtime_validation_01` | Valid controlled-realtime protocol validation | No | Dual rate gates passed and both robots completed recovery | Retain as validation; exclude from formal batch |
| `head_on_offset_040_realtime_formal_trial_01`–`05` | Valid controlled-realtime lateral-offset batch | No (0/5) | Fresh sessions, dual rate gates, synchronized avoidance and recovery completion in all five runs | Primary lateral-offset batch; include 5/5 |
| `head_on_centered_realtime_formal_trial_01`–`05` | Valid controlled-realtime centred batch | No (0/5) | Mean minimum separation 0.147039 m; all runs completed recovery with no invalid states | Primary centred baseline; include 5/5 |
| `crossing_90_realtime_formal_trial_01`–`05` | Valid controlled-realtime ninety-degree crossing batch | No (0/5) | Mean minimum separation 0.141695 m; both robots passed right and completed recovery in all runs | Primary crossing baseline; include 5/5 |
| `ablation_local_only_head_on_realtime_formal_trial_01`–`05` | Valid controlled-realtime local-sensing-only ablation batch | No (0/5) | Mean minimum separation 0.096032 m; both robots completed local recovery without peer-state input | Primary Phase 3 local-only condition; include 5/5 |
| `ablation_fused_head_on_realtime_formal_trial_01`–`05` | Valid controlled-realtime fused local plus communication/CPA batch | No (0/5) | Mean minimum separation 0.150203 m; CPA triggered before the armed local-sensor fallback in all five runs | Primary Phase 3 fused condition; include 5/5 |
| Lateral-offset execution diagnostics | Invalid protocol-development runs | No observed robot collision | Process interruption, safety-ceiling expiry and same-session ROS-graph contamination were detected by gates | Exclude; retain bags/logs for audit |

## Current conclusion

The corrected controller demonstrates the intended communication-aware mutual avoidance mechanism. Trial 02 also provides useful evidence that shared-frame pose validation and encounter-state hysteresis are necessary safety mechanisms.

The clean-world CPA-only post-fix batch is complete. Trials 02–06 are valid
repetitions (`5/5`, collision `0/5`); diagnostic Trial 01 is not pooled with them.

Timing audit note: the earlier centred batch used variable accelerated
Webots factors (2.179–4.097). Retain it as functional evidence, but do not use it
for strict wall-time or cross-condition timing comparisons without a new
controlled-realtime repetition batch.

The controlled-realtime 0.040 m lateral-offset batch is complete (`5/5`, collision
`0/5`) with full-load factors 0.947–0.989.

The controlled-realtime centred batch is also complete (`5/5`, collision `0/5`,
invalid states `0`). Mean minimum centre separation was 0.147039 ± 0.001377 m;
the bag-derived state-time factor was 0.959959 ± 0.004386. The apparent Trial 04
startup skew did not persist into avoidance: turn-onset skew was 0.0026 ms.

The controlled-realtime ninety-degree crossing batch is complete (`5/5`,
collision `0/5`, invalid states `0`). Mean minimum centre separation was 0.141695
± 0.001439 m and the mean bag-derived state-time factor was 0.962222 ± 0.003969.
All three locked Phase 2 geometries are now complete without controller retuning.

The controlled-realtime local-sensing-only ablation batch is complete (`5/5`,
collision `0/5`, invalid states `0`). Mean minimum centre separation was 0.096032
± 0.002114 m, which is 0.051007 m (34.7%) below the matched communication/CPA
baseline. This supports the directly observed later, closer reactive avoidance.

The controlled-realtime fused batch is complete (`5/5`, collision `0/5`,
invalid states `0`). Mean minimum centre separation was 0.150203 ± 0.005517 m.
This is 0.054171 m above local-only but only 0.003164 m above communication/CPA.
All five fused runs had local sensing enabled but no `LOCAL_*` control takeover;
CPA acted first in the clean, communication-normal head-on scenario.

## Required next phase

1. Phase 3 is complete: local-only, communication/CPA and fused conditions each have five accepted repetitions.
2. Advance to the wooden-box plus moving communicated-robot scenario.
3. Use the same dual rate gate and fresh-session protocol for the combined scenario.
4. After the combined scenario passes, compare periodic and event-triggered communication under controlled delay/loss conditions.
