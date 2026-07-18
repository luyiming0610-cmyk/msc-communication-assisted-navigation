# physical_expanded_bridge_epuckstate_integration_pilot01_attempt01

`DIAGNOSTIC_PHYSICAL` — expanded Pi-TCP-WSL bridge + `state_publisher` +
current `epuck2_comm_interfaces/msg/EpuckState` protocol, stationary
integration check. **Not a formal baseline. No ground motion, no
controller.**

**Verdict: `PASS_WITH_LIMITATION`** — see `analysis/verdict.json` /
`analysis/summary.md`. Limitations: tier A duplicate_count not separately
measurable (bridge code limitation, not a run defect); one-way Pi→WSL
latency not computed (no clock-sync procedure verified).

## Contents

- `physical_expanded_bridge_epuckstate_integration_pilot01_attempt01_0.db3`
  + `metadata.yaml` — closed rosbag: `/epuck1/state`, `/epuck_bridge/status`,
  `/odom`, `/scan`, `/tof`, `/ps0`–`/ps7`, `/cmd_vel` (0 messages on the
  last topic).
- `analysis/wsl_expanded_status.csv` — 1Hz `/epuck_bridge/status` ticks
  including the expanded bridge's own tier-A sequence stats.
- `analysis/wsl_system_metrics.csv`, `analysis/pi_system_metrics.csv` —
  1Hz local CPU/mem (Pi also Wi-Fi) sampling.
- `analysis/cmd_vel_checkpoints.json` — publisher-count samples at
  start/mid/end, all `0`.
- `analysis/verdict.json`, `analysis/summary.md`, `analysis/metrics.csv`,
  `analysis/runtime_manifest.json` — required analysis artifacts.
- `analysis/sha256_manifest.txt` — cross-checked file hashes (WSL native
  vs. this Windows copy).

## Three-tier sequence results (never conflated)

| tier | what it measures | result |
|---|---|---|
| A: Pi→WSL app transport | expanded bridge's own state-sequence receipt stats | `APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO=1.0`, `state_missing=0`, `state_out_of_order=0` (duplicate not separately trackable by the bridge's own code) |
| B: state_publisher→bag | EpuckState.sequence continuity as captured by the bag | `bag_capture_ratio=1.0`, 0 gap/duplicate/out-of-order |
| C: raw sensor topics | message rate/gaps, no source sequence exists | odom/scan/tof/ps0-7 all ≈9.15Hz, 0 total stall — no PDR claimed |

## Headline (main window, 240.0s, real 4-way-overlap-derived)

- `connected_fraction=1.0`, 0 reconnects, `crc_errors` delta=0.
- RTT status-tick snapshots (240 valid): mean 38.4ms / median 8.4ms /
  p95 119.1ms / max 169.7ms. Tail: 27.1% > 50ms, 27.1% > 100ms, 0% > 200ms.
- `state_age_s`: mean 0.059s / p95 0.107s (WSL single-clock-domain, VALID).
- `version=1`, `source=2` (SOURCE_HARDWARE), `robot_id=1` — 0 field errors.
- `validity_flags` constant 7 (all three sources valid) for the whole
  239.9s main window.
- 0 NaN anywhere; 6377 protocol-allowed `+Inf`; 0 unexpected Inf.
- 0 nonzero `/cmd_vel` — bag (0 messages total), live checkpoints
  (start/mid/end all `publisher_count=0`).

## Known issue, verified not assumed

`wsl_expanded_pilot_recorder.py` hit the same benign trailing
`rcl_shutdown` exception class already fixed elsewhere this session —
**not yet patched in this file**. Its lack of data-integrity impact was
independently verified (row counts, monotonicity, checkpoint presence,
0 analyzer fail_reasons) before being treated as harmless, per
instruction — not presumed in advance.

## Explicitly NOT claimed

- Not a formal EpuckState protocol baseline.
- Not evidence about avoidance behaviour or ground motion.
- Tier A duplicate_count: not separately measurable (bridge code
  limitation).
- One-way Pi→WSL latency: not computed (no clock sync verified).

## Next steps (not started, no automatic follow-on)

- Formal baseline design, ground motion, or any controller: none started,
  none authorized by this pilot.
- `pi_system_sampler.py`'s own SHA-256 was not captured this specific
  run (reused from the prior pilot's deployment) — flagged as a minor
  gap; its output CSV's hash was independently end-to-end verified.
