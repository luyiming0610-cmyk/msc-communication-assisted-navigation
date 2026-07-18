# physical_single_device_zero_impairment_baseline_v1 -- batch summary

**Batch verdict: FINAL_BATCH_PASS** (5/5 FINAL_PASS)

Same driver/expanded-server/WSL-bridge continuous session across all 5 trials -- 5 SEPARATE measurement windows within one continuous infrastructure session, NOT 5 independent cold-start sessions. `state_publisher` was FRESH-restarted for every trial (fresh `EpuckState.sequence`).

| trial | verdict | overlap(s) | main(s) | buffers(s) | Tier A ratio | EpuckState Hz | Tier B ratio | RTT median/p95(ms) | RTT >50/100/200ms | Pi CPU mean/p95(%) |
|---|---|---|---|---|---|---|---|---|---|---|
| trial01_attempt02 | FINAL_PASS | 309.905 | 240.0 | 34.952/34.952 | 1.0 | 8.9417 | 1.0 | 8.276/117.557 | 25.00%/25.00%/0.00% | 14.835/15.992 |
| trial02_attempt02 | FINAL_PASS | 311.123 | 240.0 | 35.562/35.562 | 1.0 | 8.9417 | 1.0 | 8.092/117.43 | 20.83%/20.83%/0.00% | 13.94/15.35 |
| trial03_attempt01 | FINAL_PASS | 310.921 | 240.0 | 35.46/35.46 | 1.0 | 8.8833 | 1.0 | 8.115/116.285 | 20.83%/20.83%/0.00% | 14.2/15.766 |
| trial04_attempt01 | FINAL_PASS | 310.66 | 240.0 | 35.33/35.33 | 1.0 | 8.9083 | 1.0 | 8.333/117.575 | 22.92%/22.92%/0.00% | 14.53/16.25 |
| trial05_attempt01 | FINAL_PASS | 311.021 | 240.0 | 35.51/35.51 | 1.0 | 8.8958 | 1.0 | 8.384/118.753 | 22.50%/22.50%/0.00% | 14.457/15.931 |

## RTT tail repeatability across the 5 separate measurement windows (no root-cause attribution)

- >50ms percentage across trials: ['25.00', '20.83', '20.83', '22.92', '22.50'] (mean 22.42%, stdev 1.55%)
- >100ms percentage across trials: ['25.00', '20.83', '20.83', '22.92', '22.50'] (mean 22.42%, stdev 1.55%)
- longest consecutive >50ms run across trials: [5, 6, 4, 3, 3] (mean 4.20)
- The bimodal/long-tail RTT pattern (roughly 20-25% of 1Hz status snapshots exceeding 50ms, 0% exceeding 200ms) recurs consistently across all 5 separate measurement windows, plus the 2 excluded SHORT_WINDOW diagnostic attempts (trial01/02_attempt01) and the earlier physical_expanded_bridge_epuckstate_integration_pilot01_attempt01. This is reported as an observed repeatable pattern only -- no root cause is attributed.

## Explicit scope notes
- Tier A (`APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO`) is Pi application-level state-sequence receipt completeness at the WSL bridge, computed as a trial-start-vs-trial-end snapshot delta of the bridge's own cumulative counters -- NOT IP/TCP packet loss.
- `duplicate_count` is NOT_MEASURABLE for every trial (the bridge's own code does not separately track it) -- never reported as 0.
- One-way Pi-to-WSL latency is NOT reported for any trial -- no NTP/chrony clock-sync procedure has been verified.
- `trial01_attempt01_short_window` and `trial02_attempt01_short_window` remain excluded diagnostic evidence, not counted toward this n=5 batch.
- All Pi/WSL resource figures are computed only from each trial's own 240.000s main window, never from data outside that window.
