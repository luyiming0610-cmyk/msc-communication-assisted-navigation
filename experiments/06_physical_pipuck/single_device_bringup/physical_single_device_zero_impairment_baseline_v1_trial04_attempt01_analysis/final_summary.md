# trial04_attempt01 -- final_summary

**Verdict: FINAL_PASS**

Part of physical_single_device_zero_impairment_baseline_v1 (n=5). Same driver/expanded-server/WSL-bridge continuous session as the other 4 trials -- this is one of 5 SEPARATE measurement windows, not an independent cold-start session. `state_publisher` was FRESH for this trial.

## Window audit (4-source: bag/status_csv/system_csv/pi_batch_csv)
- common overlap: 310.660s (required >= 300.000s)
- limiting sources: start=bag, end=bag
- main window: 1784386736.793392 .. 1784386976.793392 (240.000s)
- buffers: left=35.330s, right=35.330s

## Tier A (Pi->WSL application-level state delivery, trial-start-vs-trial-end snapshot delta)
- seq_last: 65872 -> 68775 (delta 2903)
- unique_received delta: 2903
- missing delta: 0, out_of_order delta: 0
- APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO (delta-based): 1.0
- duplicate_count: NOT_MEASURABLE (bridge does not separately track it -- never reported as 0)

## Tier B (state_publisher -> bag capture)
- message_count: 2138, actual rate: 8.9083 Hz
- gap/duplicate/out_of_order: 0/0/0
- bag_capture_ratio: 1.0

## Tier C (raw sensor topics, no source-side sequence, no PDR claimed)
- /odom: 9.2004 Hz, max_gap 0.1408s, total_stall 0s
- /scan: 9.2004 Hz, max_gap 0.1409s, total_stall 0s
- /tof: 9.2005 Hz, max_gap 0.1422s, total_stall 0s
- /ps0: 9.2004 Hz, max_gap 0.1413s, total_stall 0s
- /ps1: 9.2004 Hz, max_gap 0.1413s, total_stall 0s
- /ps2: 9.2005 Hz, max_gap 0.1414s, total_stall 0s
- /ps3: 9.2005 Hz, max_gap 0.1415s, total_stall 0s
- /ps4: 9.2005 Hz, max_gap 0.1412s, total_stall 0s
- /ps5: 9.2005 Hz, max_gap 0.1412s, total_stall 0s
- /ps6: 9.2005 Hz, max_gap 0.1412s, total_stall 0s
- /ps7: 9.2005 Hz, max_gap 0.1423s, total_stall 0s

## State quality
- field errors: 0
- validity_flags durations: {'7': 239.90019018400025}
- NaN count: 0, protocol-allowed +Inf: 12750, unexpected Inf: 0
- state_age_s: mean 0.0538s, p95 0.1035s

## RTT (1Hz /epuck_bridge/status snapshot, NOT a full transaction census)
- sample_count: 240
- mean/median/p95/p99/max: 33.429/8.333/117.575/127.536/142.032 ms
- tail: >50ms 22.92%, >100ms 22.92%, >200ms 0.00%, longest run 3
- no root-cause attribution

## Pi/WSL resources (this trial's 240.000s main window only)
- Pi CPU: mean 14.530%, p95 16.250%, max 17.270%
- Pi mem used: mean 218.542 MB
- Pi Wi-Fi link quality: mean 70.000, signal: mean -32.613 dBm
- Pi sample count in window: 238
- WSL CPU: mean 4.976%, WSL mem used: mean 661.205 MB

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
