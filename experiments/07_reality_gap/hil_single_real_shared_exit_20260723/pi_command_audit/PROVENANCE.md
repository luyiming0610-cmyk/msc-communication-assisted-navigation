# Pi command-audit variant -- provenance (2026-07-23)

**Status: proposed change only. NOT deployed to the Pi. The real Pi
still runs the original, unaudited file.** This directory exists to
design, implement, and test the audit logic entirely offline, per
Part 2 of the post-incident command-evidence-chain work (see
`../safety_incident_unexpected_motion_20260723/SUMMARY.md` and
`../safety_incident_unexpected_motion_2_20260723/SUMMARY.md`).

## Files

- `pi_epuck_tcp_server_sensors_original_mirror.py` -- byte-exact copy
  of the file currently running on the Pi (and mirrored at
  `../../../../实体实验交接包_20260715/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py`
  and the WSL native workspace at
  `~/epuck_ws/epuck_comm_project/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py`,
  confirmed byte-identical across all three locations by
  `tools/audit_source_identity.sh` on 2026-07-23).
  SHA-256: `51d3575d64717c3aacac1dcde3300da113a82be5b42980056890bd920a543a16`
- `bridge_protocol.py` -- byte-exact copy of the shared protocol
  module, needed to import/test the audited variant.
  SHA-256: `04ef3cf2faba4b799454b8b9644e44cce23c2e9ad5634ddfc17a6096e102e248`
- `pi_epuck_tcp_server_sensors_audited.py` -- the proposed variant.
  SHA-256: `c14543634629a39bbd2b7d60e79cd5973f6857d3dbc3f5b4b894f8a9cb9ffb33`
- `test_pi_epuck_tcp_server_sensors_audited.py` -- 24 unit tests
  against the audited variant's pure logic (parsing/clamping, zero-reason
  computation, record shapes, the audit sink). All 24 pass. No rclpy
  Node is instantiated by any test -- no socket, no ROS graph.

## What changed vs. the original (diff summary, not a literal diff)

1. Two new, disabled-by-default ROS parameters:
   `command_audit_enabled` (bool, default `False`) and
   `command_audit_path` (string, default `""`).
2. A new `CommandAuditSink` class: an append-only, flush-on-write JSONL
   sink. `enabled=False` (or no path) makes every method a no-op.
3. Five new pure functions extracting existing inline logic so it can
   be unit tested without rclpy: `parse_and_clamp_command` (identical
   parsing/clamping to the original `_handle_command`'s inline code),
   `compute_zero_reason` (an audit ANNOTATION of the original
   `_command_timer`'s existing `fresh` computation -- not a new or
   different gating rule), and four `build_*_record` functions.
4. `_handle_command`, `_command_timer`, and `_network_main` each gained
   one or two `if self._audit.enabled: self._audit.write(...)` calls,
   inserted after all original logic and state mutation, never before
   or in place of it.
5. A `self._connection_id` counter, incremented once per accepted TCP
   connection, included in every audit record so a reconnect's records
   are distinguishable from the previous connection's.
6. `stop()` gained one line: `self._audit.close()`.

**Nothing else changed.** Transport (socket handling, line framing,
CRC-verified protocol), the watchdog timeout computation, the
linear/angular clamping bounds, and the publish-to-`/cmd_vel` behavior
are byte-for-byte the same control flow as the original -- confirmed by
inspection, not merely claimed: every original line of logic is present
unchanged, with audit calls only appended after it.

## Verification performed (all offline, no Pi/robot involved)

- `py_compile` clean on both new files.
- 24/24 unit tests pass (see file above), covering: normal command,
  clamped command (linear and angular), malformed command (wrong type,
  non-numeric, non-finite/NaN, non-dict payload, missing `seq`
  defaulting correctly), force-zero via disconnect
  (`DISCONNECTED`), watchdog-zero via stale timeout
  (`WATCHDOG_STALE_TIMEOUT`), a genuine commanded zero vs. a
  safety zero (`COMMANDED_ZERO` vs `None`), the reconnect sequence
  (`DISCONNECTED` -> `NEVER_RECEIVED`, proving a reconnect can never
  silently replay the previous connection's last command), and that
  the audit sink never mutates or returns what it's given (cannot
  generate or replay a command).

## Explicitly not done

- Not deployed to the Pi. The Pi continues running the original,
  unaudited file.
- Not started against any live socket, ROS graph, or hardware.
- No change to transport, watchdog timeout, or clamping behavior when
  `command_audit_enabled` is left at its default (`False`) -- verified
  by inspection (every audit call site is gated by the same disabled
  flag and placed strictly after the original logic).
