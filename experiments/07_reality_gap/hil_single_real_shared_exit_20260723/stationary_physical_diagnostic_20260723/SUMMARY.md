# Stationary physical health audit -- epuck5809 (2026-07-23)

**Classification: `INCOMPLETE_DIAGNOSTIC` / `EXCLUDED`.** Not a `PASS`.
Read-only, no publishers, no ground motion, no HIL guard, no
controller, no virtual peer, no goal_navigator, no Webots. Frozen git
commit at test time: `2cac700f5a7a95ce1785dedd2e51a1527a60f548`.

## Why this run is excluded, not passed

The orchestration script used for this run (`run_stationary_audit.sh`,
a scratch script, not committed) started
`wsl_expanded_pilot_recorder.py` (the reused, unmodified bridge-status
recorder) **without ever sending it a stop signal**. Reading the
recorder's own source confirms it has **no internal exit condition by
design** -- its `run()` loop is `while rclpy.ok() and not
stop_requested(): ...`, relying entirely on an external SIGINT (every
prior script that reuses it, e.g. `run_baseline_v1_trial_v2.sh`,
`run_expanded_bridge_epuckstate_pilot.sh`, explicitly sends `kill -INT`
after a fixed `sleep`). This script instead did a bare `wait
"$RECORDER_PID"`, which never returns on its own.

**This was a single missing-termination bug on one process, not a
concurrency/serialization defect.** All eleven `ros2 topic hz`
processes (`/odom`, `/scan`, `/tof`, `/ps0`-`/ps7`) and the
`validity_flags` `ros2 topic echo` stream were each independently
wrapped in their own `timeout 300`, ran fully concurrently, and
self-terminated correctly at ~300s, confirmed by process-list
inspection (only the recorder and the parent script remained alive
after ~300s) and by message-count arithmetic (the validity_flags log's
line count corresponds to almost exactly 300s at the observed ~8.93 Hz
rate).

The recorder instead ran unattended for approximately 26 minutes before
being noticed and manually stopped via `kill -INT` on its exact PID
(4803) -- never `pkill`. The parent script (PID 4776) exited on its own
immediately afterward, as expected (no trap/cleanup logic existed in
that scratch script to require a separate stop).

## Preconditions confirmed

- `/cmd_vel Publisher count: 0` immediately before starting, and again
  at the audit's actual end.
- No `hil_cmd_vel_guard`, `hil_wheel_suspension_test`,
  `hil_angular_suspension_test`, `cooperative_avoider`,
  `goal_navigator`, `hil_virtual_peer`, or Webots process existed at
  any point during or after this run.

## Findings from the data that WAS legitimately collected

**Sensor topic rates, full ~300s window (self-terminated correctly):**
`/odom`, `/scan`, `/tof`, and all of `/ps0`-`/ps7` converged to the
**same average rate, 8.929 Hz**, over the full window. This directly
answers the diagnostic question this test was designed to settle:
the earlier isolated `/ps0 ≈ 5.055 Hz` reading (from a short ~3s/10-sample
window, see the prior session's physical preflight run) **was a
short-window measurement artifact, not a real per-topic rate
difference** -- with a long enough window, `/ps0` matches every other
sensor topic exactly.

**Bridge status, first 300 rows of the recorder's CSV (i.e. the
legitimately-scoped 300s window, sliced out of the longer accidental
recording):** `connected=true` for all 300 rows, `crc_errors=0`
throughout, `last_rtt_ms` ranged 6.10-205.61 ms, `state_missing=0`,
`state_out_of_order=0`, `state_delivery_ratio=1.0` for all 300 rows.
Clean.

**`validity_flags`, full ~300s window (self-terminated correctly):**
2681 of 2696 real samples (excluding `ros2 topic echo`'s own `---`
message separators) read exactly `7`. **15 samples read `0`** -- a
genuine, small, real anomaly, not a parsing artifact (verified against
raw line content, not just a grep count). Their positions are not
random: one early occurrence, a cluster of 4 consecutive `0` readings
around message ~558-562, then a strikingly regular recurrence roughly
every ~32 seconds for the rest of the window (8 further isolated
single-sample `0` readings). Each dip is a single ~112 ms sample
(1/8.929 Hz) that recovers immediately. This is worth follow-up in a
future, deliberately-scoped test -- it is reported here as observed,
without a diagnosed root cause (no correlation against the bridge
CSV's own per-second counters at the same offsets has been done yet).

**Out-of-scope bonus finding (not part of this diagnostic's authorized
scope, from the unintended ~26-minute overrun):** the bridge eventually
lost connection (`connected=false`) at roughly the 17-18 minute mark
and did not recover for the remainder of the accidental extended run
(`rx_count` frozen, `last_state_age_s` growing to 559s by the final
row). This is reported for awareness only -- it says nothing about
short-duration (5-minute) stability, which the legitimately-scoped data
above shows was clean, and it must not be treated as a formal
long-duration stability result without a deliberately designed and
authorized test.

## Fix applied

`tools/run_stationary_physical_diagnostic.sh` (new, committed):
explicitly stops the recorder with `kill -INT` on its exact PID after
the script's own `sleep "$DURATION_S"`, inside a `trap ... EXIT`
cleanup, exactly matching the established pattern from
`run_baseline_v1_trial_v2.sh`. Tested offline (`bash -n`) only --
**not re-run against the physical robot**, per instruction not to
auto-start another physical audit.

## Evidence

Raw logs preserved on the native WSL filesystem
(`/home/eamon/epuck_comm_bags/stationary_physical_diagnostic_20260723/`),
SHA-256-verified against the local (gitignored) Windows copy in
`raw_logs/` (manifest: `raw_logs/SHA256SUMS.txt`), copies never
overwritten.
