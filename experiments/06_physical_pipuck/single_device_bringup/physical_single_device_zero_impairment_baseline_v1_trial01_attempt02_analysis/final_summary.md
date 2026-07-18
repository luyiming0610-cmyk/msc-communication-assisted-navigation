# trial01_attempt02 -- final_summary

**Verdict: FINAL_PASS**

Part of physical_single_device_zero_impairment_baseline_v1 (n=5). Same driver/expanded-server/WSL-bridge continuous session as the other 4 trials -- this is one of 5 SEPARATE measurement windows, not an independent cold-start session. `state_publisher` was FRESH for this trial.

## Window audit (4-source: bag/status_csv/system_csv/pi_batch_csv)
- common overlap: 309.905s (required >= 300.000s)
- limiting sources: start=bag, end=bag
- main window: 1784384853.2372022 .. 1784385093.2372022 (240.000s)
- buffers: left=34.952s, right=34.952s

## Tier A (Pi->WSL application-level state delivery, trial-start-vs-trial-end snapshot delta)
- seq_last: 48293 -> 51202 (delta 2909)
- unique_received delta: 2909
- missing delta: 0, out_of_order delta: 0
- APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO (delta-based): 1.0
- duplicate_count: NOT_MEASURABLE (bridge does not separately track it -- never reported as 0)

## Tier B (state_publisher -> bag capture)
- message_count: 2146, actual rate: 8.9417 Hz
- gap/duplicate/out_of_order: 0/0/0
- bag_capture_ratio: 1.0

## Tier C (raw sensor topics, no source-side sequence, no PDR claimed)
- /odom: 9.1981 Hz, max_gap 0.1405s, total_stall 0s
- /scan: 9.1981 Hz, max_gap 0.1406s, total_stall 0s
- /tof: 9.1981 Hz, max_gap 0.1406s, total_stall 0s
- /ps0: 9.1981 Hz, max_gap 0.1406s, total_stall 0s
- /ps1: 9.1981 Hz, max_gap 0.1409s, total_stall 0s
- /ps2: 9.1981 Hz, max_gap 0.1409s, total_stall 0s
- /ps3: 9.1981 Hz, max_gap 0.1408s, total_stall 0s
- /ps4: 9.1981 Hz, max_gap 0.1408s, total_stall 0s
- /ps5: 9.1981 Hz, max_gap 0.1408s, total_stall 0s
- /ps6: 9.1981 Hz, max_gap 0.1407s, total_stall 0s
- /ps7: 9.1981 Hz, max_gap 0.1407s, total_stall 0s

## State quality
- field errors: 0
- validity_flags durations: {'7': 239.89976129700005}
- NaN count: 0, protocol-allowed +Inf: 7015, unexpected Inf: 0
- state_age_s: mean 0.0534s, p95 0.1017s

## RTT (1Hz /epuck_bridge/status snapshot, NOT a full transaction census)
- sample_count: 240
- mean/median/p95/p99/max: 35.502/8.276/117.557/125.091/130.192 ms
- tail: >50ms 25.00%, >100ms 25.00%, >200ms 0.00%, longest run 5
- no root-cause attribution

## Pi/WSL resources (this trial's 240.000s main window only)
- Pi CPU: mean 14.835%, p95 15.992%, max 17.190%
- Pi mem used: mean 203.937 MB
- Pi Wi-Fi link quality: mean 69.996, signal: mean -27.402 dBm
- Pi sample count in window: 239
- WSL CPU: mean 6.082%, WSL mem used: mean 667.171 MB

## Safety checks
- /cmd_vel nonzero in bag: 0
- cmd_vel checkpoints: [('start', 0), ('mid', 0), ('end', 0)]
- crc_errors_delta (main window): 0
- reconnect_count: 0
- recorder traceback observed: False
- bag_record.log warning/error lines: 0

## Scope notes
- APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO is Pi application-level state-sequence receipt completeness, NOT IP/TCP packet loss.
- Pi-to-WSL one-way latency is NOT reported -- no NTP/chrony clock-sync procedure between Pi and WSL has been verified. RTT and state_age_s remain valid (single WSL clock domain).
