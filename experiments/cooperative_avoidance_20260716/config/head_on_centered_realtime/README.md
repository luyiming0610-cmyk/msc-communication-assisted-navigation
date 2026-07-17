# Centred head-on controlled-realtime CPA-only protocol

Date frozen: 2026-07-16

- epuck1 origin: `(-0.35, 0.0, 0.0)`.
- epuck2 origin: `(0.35, 0.0, pi)`.
- Periodic communicated state; local avoidance disabled.
- Controller: `max_runtime_s=60.0`, `stop_after_recovery=true`,
  `post_recovery_hold_s=0.5`.
- Pre-load and full-load simulation/wall-time gates: `0.8–1.2`.
- Each repetition uses a fresh WSL/Webots/ROS session.
- Acceptance requires two recovery-completion messages, a complete rosbag, no
  invalid state messages, no detected collision and positive geometric margin.
- Trial 01 was observed directly; Trials 02–05 use the same frozen scripted
  protocol without controller retuning.
