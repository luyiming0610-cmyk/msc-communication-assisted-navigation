# HIL topic contract (2026-07-23)

All message types below are the existing, unmodified
`epuck2_comm_interfaces` definitions. This document only fixes which
*topics* each HIL node uses -- it defines no new message types.

| Topic (suggested name)              | Type                | Publisher                 | Subscribers                                  | Notes |
|--------------------------------------|---------------------|----------------------------|-----------------------------------------------|-------|
| `/epuck1/state`                      | `EpuckState`        | `state_publisher.py` (real, unmodified) | `hil_topic_adapter.py`, `hil_cmd_vel_guard.py`, recorder | `source="hardware"`; gated by `WAITING_FOR_CLOCK` until the real-time clock is valid. Confirmed live this session (matches `state_publisher.py`'s actual launch remap `-r state:=/epuck1/state`, used in every physical bring-up so far) -- not `/epuck5809/state` (#5809 is only the hardware serial/hotspot identifier, never an actual ROS namespace). |
| `/epuck_virtual_peer/state`          | `EpuckState`        | `hil_virtual_peer.py`     | `hil_cmd_vel_guard.py` (if `require_virtual_peer`), recorder | `source="virtual"`. |
| `/hil/goal_announcement`             | `GoalAnnouncement`  | `hil_virtual_peer.py` (HIL_COMM_ON only) | `hil_topic_adapter.py` (HIL_COMM_ON only), recorder | The only cross-agent exit-information channel, per the frozen N2/N3 design. |
| `/epuck1/nav_intent`                 | `NavigationIntent`  | `hil_topic_adapter.py`    | `cooperative_avoider.py` (same robot only), recorder | Never cross-robot -- mirrors `goal_navigator.py`'s existing contract and its own `/epuck1/nav_intent` convention already used in the N2/N3 formal batches. Not yet live-confirmed via an actual `hil_topic_adapter.py` run (no HIL navigation trial has been run yet) -- inferred from the same `/epuck1/` convention `state_publisher.py` already uses live. |
| `/cmd_vel_unguarded`                 | `geometry_msgs/Twist` | `cooperative_avoider.py` (unmodified) | `hil_cmd_vel_guard.py` only | NEVER wired directly to the driver. Confirmed live this session (matches `hil_cmd_vel_guard.py`'s actual `--upstream-cmd-vel-topic` default, un-namespaced) -- not `/epuck5809/cmd_vel_unguarded`. |
| `/hil_guard/arm`                     | `std_msgs/Bool`     | operator tooling (manual, not auto)     | `hil_cmd_vel_guard.py`      | Defaults unset/false; guard never self-arms. |
| `/cmd_vel`                           | `geometry_msgs/Twist` | `hil_cmd_vel_guard.py` (sole publisher) | real driver                | Guard is the ONLY component allowed to publish here. Guard itself checks this topic's own publisher count is exactly 1, and that its sole publisher is itself. Confirmed live this session (matches `hil_cmd_vel_guard.py`'s actual `--guarded-cmd-vel-topic` default, un-namespaced) -- not `/epuck5809/cmd_vel`. |
| `/hil/bridge_status`                 | (bridge-specific, existing) | existing WSL<->Pi bridge (unmodified) | recorder, read-only connectivity check | Reused from `06_physical_pipuck` bridge tooling. |
| `/hil/task_completion`               | (existing `task_completion_monitor.py` output) | `task_completion_monitor.py` (unmodified, real-time clock already) | recorder | Directly reusable, no wrapper needed. |
| `/hil/safety_events`                 | log-derived, not a live topic | guard/adapter loggers (`HIL_GUARD_BLOCKED`, etc.) | recorder (log capture) | Captured via node logs, not a dedicated message type, to avoid inventing a new frozen-protocol message. |

## Validity and freshness rules

- Any `EpuckState` consumed by `hil_cmd_vel_guard.py` from the physical
  robot must have `version == EpuckState.PROTOCOL_VERSION` (currently
  1) and `validity_flags & (FLAG_ODOM_VALID | FLAG_IR_VALID |
  FLAG_TOF_VALID) == (FLAG_ODOM_VALID | FLAG_IR_VALID | FLAG_TOF_VALID)`
  (value 7) -- ODOM alone is insufficient, since
  `cooperative_avoider.py`'s local IR/ToF avoidance path requires both
  (see `hil_cmd_vel_guard.py`'s `--required-validity-flags` default,
  hardened in commit `9e2b586`) -- and must have been received within
  `physical_state_timeout_s` (physical) or `virtual_peer_timeout_s`
  (virtual) of "now" on the RECEIVING node's own clock.
- The upstream `cmd_vel_unguarded` stream doubles as the guard's
  heartbeat signal: if `cooperative_avoider.py` stops publishing (crash,
  hang), the heartbeat goes stale and the guard fails closed
  independent of any state-topic freshness.
- Namespace conflicts with the reserved N2/N3 formal-batch namespaces
  (`epuck1`, `epuck2`, `epuck3`) are rejected by
  `hil_preflight.check_namespace_conflicts()`.
