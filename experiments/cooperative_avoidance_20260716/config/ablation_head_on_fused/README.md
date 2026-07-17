# Centred head-on fused local + communication/CPA ablation

Date frozen: 2026-07-16

## Experimental factor

This is the third Phase 3 ablation condition. Both inputs are enabled explicitly:

- `enable_peer_avoidance=true`
- `enable_local_avoidance=true`
- `require_local_sensors=true`

The initial geometry, desired speed, recovery stop rule and controlled-realtime
gate are identical to the matched local-only and communication/CPA-only batches.
No controller thresholds are retuned for this condition.

## Locked geometry

- `epuck1`: origin `(-0.35, 0.0, 0.0)`, desired heading `0.0`.
- `epuck2`: origin `(0.35, 0.0, pi)`, desired heading `pi`.
- Clean centred head-on Webots world, no wooden obstacle.

## Trial acceptance

- Pre-load and full-load simulation/recorded-time factors both in `0.8–1.2`.
- Both communicated states and all required local sensors are usable.
- Both controllers finish via cooperative or local recovery completion, not the
  60 s safety ceiling.
- Complete rosbag and analysis outputs.
- No collision, no invalid state, positive geometric safety margin.
- Fresh WSL/Webots/ROS session for every repetition.

State topics, local ToF/proximity topics, odometry and command velocity must all
be recorded so that the active trigger and fallback behaviour can be audited.
