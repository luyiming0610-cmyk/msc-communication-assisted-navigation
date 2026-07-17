# Centred head-on local-sensing-only ablation

Date frozen: 2026-07-16

## Controlled condition

- Geometry, nominal speed and realtime gate match the accepted centred head-on
  communication/CPA batch.
- `epuck1`: `(-0.35, 0.0, 0.0)`.
- `epuck2`: `(0.35, 0.0, pi)`.
- `enable_peer_avoidance=false`.
- `enable_local_avoidance=true`; `require_local_sensors=true`.
- Calibrated IR/ToF thresholds and command smoothing are unchanged.
- `max_runtime_s=60.0`, `stop_after_recovery=true`,
  `post_recovery_hold_s=0.5`.
- Pre-load and full-load simulation/wall-time gates: 0.8–1.2.

Both state topics remain active solely as measurement instrumentation for rosbag
grounding and centre-separation analysis. With peer avoidance disabled, each
controller subscribes only to its own state and does not consume the other
robot's communicated state.

## Trial 01 acceptance observations

- Both robots cruise before local IR/ToF detection.
- Each robot initiates local avoidance before physical contact.
- Both use the deterministic own-right response for an ambiguous centred return.
- No collision, repeated spin or visible oscillation.
- Both complete local bypass and heading recovery, then command zero.
- Webots remains within the 0.8–1.2 realtime gate.
