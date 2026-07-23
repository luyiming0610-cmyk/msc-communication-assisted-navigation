# HIL topic contract (2026-07-23)

All message types below are the existing, unmodified
`epuck2_comm_interfaces` definitions. This document only fixes which
*topics* each HIL node uses -- it defines no new message types.

| Topic (suggested name)              | Type                | Publisher                 | Subscribers                                  | Notes |
|--------------------------------------|---------------------|----------------------------|-----------------------------------------------|-------|
| `/epuck5809/state`                   | `EpuckState`        | `state_publisher.py` (real, unmodified) | `hil_topic_adapter.py`, `hil_cmd_vel_guard.py`, recorder | `source="hardware"`; gated by `WAITING_FOR_CLOCK` until the real-time clock is valid. |
| `/epuck_virtual_peer/state`          | `EpuckState`        | `hil_virtual_peer.py`     | `hil_cmd_vel_guard.py` (if `require_virtual_peer`), recorder | `source="virtual"`. |
| `/hil/goal_announcement`             | `GoalAnnouncement`  | `hil_virtual_peer.py` (HIL_COMM_ON only) | `hil_topic_adapter.py` (HIL_COMM_ON only), recorder | The only cross-agent exit-information channel, per the frozen N2/N3 design. |
| `/epuck5809/nav_intent`              | `NavigationIntent`  | `hil_topic_adapter.py`    | `cooperative_avoider.py` (same robot only), recorder | Never cross-robot -- mirrors `goal_navigator.py`'s existing contract. |
| `/epuck5809/cmd_vel_unguarded`       | `geometry_msgs/Twist` | `cooperative_avoider.py` (unmodified) | `hil_cmd_vel_guard.py` only | NEVER wired directly to the driver. |
| `/hil_guard/arm`                     | `std_msgs/Bool`     | operator tooling (manual, not auto)     | `hil_cmd_vel_guard.py`      | Defaults unset/false; guard never self-arms. |
| `/epuck5809/cmd_vel`                 | `geometry_msgs/Twist` | `hil_cmd_vel_guard.py` (sole publisher) | real driver                | Guard is the ONLY component allowed to publish here. Guard itself checks this topic's own publisher count is exactly 1. |
| `/hil/bridge_status`                 | (bridge-specific, existing) | existing WSL<->Pi bridge (unmodified) | recorder, read-only connectivity check | Reused from `06_physical_pipuck` bridge tooling. |
| `/hil/task_completion`               | (existing `task_completion_monitor.py` output) | `task_completion_monitor.py` (unmodified, real-time clock already) | recorder | Directly reusable, no wrapper needed. |
| `/hil/safety_events`                 | log-derived, not a live topic | guard/adapter loggers (`HIL_GUARD_BLOCKED`, etc.) | recorder (log capture) | Captured via node logs, not a dedicated message type, to avoid inventing a new frozen-protocol message. |

## Validity and freshness rules

- Any `EpuckState` consumed by `hil_cmd_vel_guard.py` must have
  `version == EpuckState.PROTOCOL_VERSION` (currently 1) and
  `validity_flags & FLAG_ODOM_VALID == FLAG_ODOM_VALID`, and must have
  been received within `physical_state_timeout_s` (physical) or
  `virtual_peer_timeout_s` (virtual) of "now" on the RECEIVING node's
  own clock.
- The upstream `cmd_vel_unguarded` stream doubles as the guard's
  heartbeat signal: if `cooperative_avoider.py` stops publishing (crash,
  hang), the heartbeat goes stale and the guard fails closed
  independent of any state-topic freshness.
- Namespace conflicts with the reserved N2/N3 formal-batch namespaces
  (`epuck1`, `epuck2`, `epuck3`) are rejected by
  `hil_preflight.check_namespace_conflicts()`.
