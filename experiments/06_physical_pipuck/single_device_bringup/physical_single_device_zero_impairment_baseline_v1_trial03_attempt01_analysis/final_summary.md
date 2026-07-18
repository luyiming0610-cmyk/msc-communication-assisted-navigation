# trial03_attempt01 -- final_summary

**Verdict: FINAL_PASS**

Part of physical_single_device_zero_impairment_baseline_v1 (n=5). Same driver/expanded-server/WSL-bridge continuous session as the other 4 trials -- this is one of 5 SEPARATE measurement windows, not an independent cold-start session. `state_publisher` was FRESH for this trial.

## Window audit (4-source: bag/status_csv/system_csv/pi_batch_csv)
- common overlap: 310.921s (required >= 300.000s)
- limiting sources: start=bag, end=bag
- main window: 1784386374.9434783 .. 1784386614.9434783 (240.000s)
- buffers: left=35.460s, right=35.460s

## Tier A (Pi->WSL application-level state delivery, trial-start-vs-trial-end snapshot delta)
- seq_last: 62494 -> 65402 (delta 2908)
- unique_received delta: 2908
- missing delta: 0, out_of_order delta: 0
- APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO (delta-based): 1.0
- duplicate_count: NOT_MEASURABLE (bridge does not separately track it -- never reported as 0)

## Tier B (state_publisher -> bag capture)
- message_count: 2132, actual rate: 8.8833 Hz
- gap/duplicate/out_of_order: 0/0/0
- bag_capture_ratio: 1.0

## Tier C (raw sensor topics, no source-side sequence, no PDR claimed)
- /odom: 9.1944 Hz, max_gap 0.1409s, total_stall 0s
- /scan: 9.1944 Hz, max_gap 0.1408s, total_stall 0s
- /tof: 9.1940 Hz, max_gap 0.1409s, total_stall 0s
- /ps0: 9.1944 Hz, max_gap 0.1409s, total_stall 0s
- /ps1: 9.1944 Hz, max_gap 0.1409s, total_stall 0s
- /ps2: 9.1944 Hz, max_gap 0.1409s, total_stall 0s
- /ps3: 9.1944 Hz, max_gap 0.1411s, total_stall 0s
- /ps4: 9.1944 Hz, max_gap 0.1410s, total_stall 0s
- /ps5: 9.1944 Hz, max_gap 0.1411s, total_stall 0s
- /ps6: 9.1940 Hz, max_gap 0.1409s, total_stall 0s
- /ps7: 9.1940 Hz, max_gap 0.1408s, total_stall 0s

## State quality
- field errors: 0
- validity_flags durations: {'7': 239.80019279099994}
- NaN count: 0, protocol-allowed +Inf: 13042, unexpected Inf: 0
- state_age_s: mean 0.0581s, p95 0.1064s

## RTT (1Hz /epuck_bridge/status snapshot, NOT a full transaction census)
- sample_count: 240
- mean/median/p95/p99/max: 31.140/8.115/116.285/121.165/138.504 ms
- tail: >50ms 20.83%, >100ms 20.83%, >200ms 0.00%, longest run 4
- no root-cause attribution

## Pi/WSL resources (this trial's 240.000s main window only)
- Pi CPU: mean 14.200%, p95 15.766%, max 16.670%
- Pi mem used: mean 215.521 MB
- Pi Wi-Fi link quality: mean 70.000, signal: mean -31.900 dBm
- Pi sample count in window: 239
- WSL CPU: mean 6.261%, WSL mem used: mean 667.203 MB

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
