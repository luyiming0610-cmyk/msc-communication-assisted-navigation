# Bridge instrumentation mirror (2026-07-23)

This directory is a git-tracked **mirror** of files whose actual,
executable copies live outside this repository, in the external WSL
working directory `/home/eamon/epuck_ws/epuck_comm_project/real_robot_avoidance_v1/`
(the same convention already used by the pre-existing, unmodified
`wsl_epuck_tcp_bridge_sensors.py` / `pi_epuck_tcp_server_sensors.py` /
`bridge_protocol.py`, none of which are tracked in this repo either --
this mirror exists so the new diagnostic-only instrumentation has
clear source identity and version history, per instruction, rather
than existing only as an untracked WSL workspace file).

## Provenance

| File | External WSL path | SHA-256 (both copies match) |
|---|---|---|
| `bridge_protocol.py` | `.../real_robot_avoidance_v1/bridge_protocol.py` | external/live: `04ef3cf2faba4b799454b8b9644e44cce23c2e9ad5634ddfc17a6096e102e248`; this repo copy: `acb9e624562b5549aee652b1342adbb3aad13515892852588f993acc4f6f0155` (one trailing blank line trimmed for `git diff --check`, confirmed via raw byte inspection to be the only difference -- code content is otherwise identical) |
| `wsl_epuck_tcp_bridge_sensors_instrumented.py` | `.../real_robot_avoidance_v1/wsl_epuck_tcp_bridge_sensors_instrumented.py` | `73678485ca7036c2bd64602003189a0dd26bdc4047f57dfcdc0bd4804419da58` |
| `test_wsl_epuck_tcp_bridge_sensors_instrumented.py` | `.../real_robot_avoidance_v1/test_wsl_epuck_tcp_bridge_sensors_instrumented.py` | `2f39aa8f69721d0a2d390f5db08d5b9d567b6530bd81ec6a8ed8c49407263b6a` |

`bridge_protocol.py` here is copied **unmodified** (a shared, unowned
dependency of the original bridge, needed only so this mirror is
self-contained enough to compile/test independently of the external
workspace) -- its hash matches the Pi-side copy of the same file too
(`04ef3cf2...`, already on record from the
`physical_expanded_bridge_epuckstate_integration_pilot01_attempt01`
runtime manifest), confirming the wire protocol is unchanged.

## What is NOT mirrored here

The currently-running, **unmodified** original bridge,
`wsl_epuck_tcp_bridge_sensors.py` (PID 5477 at the time this
instrumentation was written, SHA-256
`09852be639f6d51a44e240134b8e1a2c7825639315ff5d07e5823c948b876bc0`),
stays exactly where it already was and is not touched, copied, or
duplicated by this commit. This mirror only adds the new, separate,
diagnostic-only instrumented variant and its tests.

## Keeping this mirror in sync

If the external working copies are edited again, re-copy them here and
update the SHA-256 table above in the same commit -- this file is a
record of what was tested and reviewed, not a build artifact that
regenerates itself.
