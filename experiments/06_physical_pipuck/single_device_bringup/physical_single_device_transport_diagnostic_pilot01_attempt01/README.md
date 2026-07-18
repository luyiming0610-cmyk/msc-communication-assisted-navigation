# physical_single_device_transport_diagnostic_pilot01_attempt01

`DIAGNOSTIC_PHYSICAL` — not a formal paper batch, does not validate the
current `EpuckState` protocol, does not validate avoidance behaviour.
Validates ONLY the base Pi-TCP-WSL transport link.

**Verdict: `PASS_WITH_LIMITATION` — BASIC PHYSICAL TRANSPORT STABILITY**
(final for this attempt; not re-run). See `analysis/verdict.json` /
`analysis/summary.md` for the full account, including why this is not an
unqualified PASS (PDR/sequence integrity structurally NOT_MEASURABLE with
the current base bridge; one-way latency NOT_VALID with no clock sync).
**Not a formal EpuckState protocol baseline. Not a PDR experiment.**

`analysis/summary.md` was corrected 2026-07-18 (wording/interpretation
only, no data re-collected) to: name the RTT figures precisely as 1Hz
status-tick snapshots of the most recently completed individual RTT (not
a full per-transaction census); add the RTT tail distribution
(>50/100/200ms counts, longest consecutive high-RTT run) with no
root-cause claim; spell out `state_age_s`'s exact formula/fields/clock
source; reframe the zero-`/cmd_vel` claim as four joint checks, none
alone sufficient.

## Contents

- `physical_single_device_transport_diagnostic_pilot01_attempt01_0.db3` +
  `metadata.yaml` — the closed rosbag (`/scan`, `/odom`,
  `/epuck_bridge/status`, `/cmd_vel` — the last topic recorded 0 messages).
- `analysis/wsl_transport_status.csv` — one row per `/epuck_bridge/status`
  tick (1Hz), WSL-local clock only.
- `analysis/wsl_system_metrics.csv`, `analysis/pi_system_metrics.csv` —
  1Hz local CPU/mem (Pi also Wi-Fi signal) sampling, no SSH-per-sample.
- `analysis/wsl_transport_totals.json` — total scan/odom/nonzero-cmd_vel
  counts observed live on the WSL ROS graph.
- `analysis/bag_info.txt`, `analysis/qos_*.txt`,
  `analysis/bag_record_warnings.txt` — bag readability, QoS, and
  drop/warn/error capture (0 lines).
- `analysis/verdict.json`, `analysis/summary.md`, `analysis/metrics.csv`,
  `analysis/runtime_manifest.json` — the four required analysis artifacts.
- `analysis/sha256_manifest.txt` — SHA-256 of every file copied from the
  WSL native path into this Windows directory, verified to match before
  the WSL native copy is considered safe to eventually clean up (not done
  yet — WSL native copy retained).

## Headline (main window, 240.0s, real overlap-derived)

- `connected_fraction=1.0`, 0 reconnects, `crc_errors` delta = 0.
- RTT status-tick snapshots (240 valid, 1Hz sampling of the most recently
  completed individual command-ack RTT — NOT a full transaction census):
  mean 52.5ms / median 8.5ms / p95 116.6ms / p99 129.6ms / max 355.4ms.
  Tail: 104 (43.3%) > 50ms, 100 (41.7%) > 100ms, 1 (0.4%) > 200ms; longest
  consecutive high-RTT run 6 ticks (~6s). No root cause claimed.
- `state_age_s` (both operands `time.time()` in the same WSL process — same
  clock domain, VALID): mean 0.056 / median 0.052 / p95 0.104 / max 0.485.
- `/scan`, `/odom`: 2227 messages each, ≈9.28Hz.
- 0 nonzero `/cmd_vel` anywhere — confirmed via 4 joint checks (bag
  absence, live WSL observation, Pi driver log, manual wheel observation),
  none alone treated as sufficient.

## Explicitly NOT claimed

- Not a formal communication-performance PASS.
- Not evidence for or against the current `EpuckState` protocol.
- Not evidence about avoidance behaviour.
- PDR / sequence-gap / duplicate / out-of-order: NOT_MEASURABLE (base
  bridge does not expose paired source sequence numbers).
- One-way latency: NOT_VALID (no cross-device clock sync verified).

## Known issue, recorded not hidden

This attempt used the pre-fix version of `wsl_transport_recorder.py`,
which hit a benign trailing shutdown exception after SIGINT (same bug
class as one already fixed in `sequence_counter.py` earlier this
session). All data-critical writes completed before the exception —
verified via row counts and `wsl_transport_totals.json`. Fixed for future
runs; this attempt's data was kept, not discarded.

## Next steps (not started, no automatic follow-on)

- Expanded sensor bridge was explicitly not used this pilot.
- No second diagnostic pilot has been auto-started.
- No motor/ground/avoidance experiment has been run.
- Read-only audit of whether `state_publisher.py` can already consume this
  bridge's topics to publish `EpuckState` is a separate, subsequent step.
