# Condition F outage-timing precondition

**Verdict: PASS_WITH_SCOPE_CLARIFICATION.** This was an offline audit; Webots was not run.

The frozen bidirectional outage schedule uses shared absolute simulation time: `[10.0,10.7)`, `[25.0,25.7)`, `[40.0,40.7)`, and `[55.0,55.7)` seconds within the nominal 60 s task window.

| Historical trial | First AVOID_TURN (s) | First RECOVER (s) | 40 s window inside active avoidance | 55 s window inside active avoidance |
|---|---:|---:|---|---|
| objective5_impairment_matrix_v1_condition_B_trial02_attempt01 | 36.46 | 57.76 | YES | YES |
| objective5_impairment_matrix_v1_condition_B_trial03_attempt01 | 36.36 | 57.76 | YES | YES |
| objective5_impairment_matrix_v1_condition_B_trial04_attempt01 | 36.86 | 58.10 | YES | YES |
| objective5_impairment_matrix_v1_condition_B_trial05_attempt01 | 39.86 | 61.06 | YES | YES |
| objective5_impairment_matrix_v1_condition_D_trial01_attempt01 | 36.06 | 57.36 | YES | YES |
| objective5_impairment_matrix_v1_condition_D_trial02_attempt01 | 36.66 | 58.36 | YES | YES |
| objective5_impairment_matrix_v1_condition_D_trial03_attempt01 | 35.00 | 56.10 | YES | YES |
| objective5_impairment_matrix_v1_condition_D_trial04_attempt01 | 36.70 | 57.96 | YES | YES |
| objective5_impairment_matrix_v1_condition_D_trial05_attempt01 | 37.46 | 58.96 | YES | YES |
| objective5_impairment_matrix_v1_condition_D_trial06_attempt01 | 38.66 | 59.80 | YES | YES |
| objective5_impairment_matrix_v1_condition_E_trial01_attempt01 | 37.56 | 58.86 | YES | YES |
| objective5_impairment_matrix_v1_condition_E_trial02_attempt01 | 37.30 | 58.60 | YES | YES |
| objective5_impairment_matrix_v1_condition_E_trial03_attempt01 | 35.06 | 56.50 | YES | YES |
| objective5_impairment_matrix_v1_condition_E_trial04_attempt01 | 35.16 | 56.50 | YES | YES |
| objective5_impairment_matrix_v1_condition_E_trial05_attempt01 | 38.90 | 60.06 | YES | YES |

Across all 15 available B/D/E timing audits, both `[40.0,40.7)` and `[55.0,55.7)` fall fully after the synchronized first `AVOID_TURN` and before the earliest `RECOVER`. No frozen outage window covers the initial `AVOID_TURN` entry itself. The valid scope is therefore a stale-state stop/recovery test during an already-active CPA manoeuvre, not an initial-trigger-delay test.

The preserved Condition F exclusionary pilot independently reconstructed five bidirectional outage windows with zero measured start/end deviation between directions. This supports synchronization, subject to message-period resolution at the boundaries.

Formal F trials must retain per-trial event-to-window evidence and separate startup-only stale transitions from stale transitions occurring during active avoidance.
