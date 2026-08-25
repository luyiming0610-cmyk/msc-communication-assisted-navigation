# Pi command-audit deployment verification record (2026-07-23)

**This records a real, user-executed deployment of the audited variant
to the Pi, under a new filename only.** The original,
unaudited server file was never overwritten, was backed up first, and
was never started. No process was started on the Pi as part of this
deployment. The robot remained powered off throughout.

All commands below were run by the operator in a local PowerShell
terminal. The SSH password was entered interactively and was neither
stored in the project nor included in the retained records. Only the
command output was retained for verification.

## Timeline (UTC, approximate, from command sequence this session)

Deployment performed 2026-07-23, ~16:50-17:05 UTC.

## Step-by-step verification

1. **Pre-deployment process check** (read-only):
   ```
   ssh pi@192.168.137.71 "pgrep -af 'epuck_ros2_driver|pi_epuck_tcp_server_sensors\.py' | grep -v '^[0-9]* bash -c' || echo NO_MATCHING_PROCESS"
   ```
   Result: `NO_MATCHING_PROCESS` -- no driver, no server process running.
   (Note: two earlier attempts at this check, without the `grep -v` filter,
   returned a false-positive self-match against the SSH-invoked wrapper
   shell itself, not a real process -- documented here for the record,
   not treated as a driver/server sighting.)

2. **Original file SHA-256, computed directly on the Pi**:
   ```
   ssh pi@192.168.137.71 "sha256sum /home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py"
   ```
   Result: `51d3575d64717c3aacac1dcde3300da113a82be5b42980056890bd920a543a16`
   -- **exact match** to the value recorded in `PROVENANCE.md`.

3. **Timestamped backup created** (original never overwritten):
   ```
   ssh pi@192.168.137.71 "cp /home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py /home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py.backup_20260723_165028"
   ```
   Path: `/home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py.backup_20260723_165028`

4. **Backup SHA-256 verified**:
   ```
   ssh pi@192.168.137.71 "sha256sum /home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py.backup_20260723_165028"
   ```
   Result: `51d3575d64717c3aacac1dcde3300da113a82be5b42980056890bd920a543a16`
   -- **exact match** to the original.

5. **Audited variant copied to a NEW filename only** (never the
   original name):
   ```
   scp "...\pi_command_audit\pi_epuck_tcp_server_sensors_audited.py" pi@192.168.137.71:/home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors_audited.py
   ```
   Destination: `/home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors_audited.py`
   (22KB transferred).

6. **Original file confirmed untouched, permissions set on the
   audited file**:
   ```
   ssh pi@192.168.137.71 "ls -la /home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py && chmod 644 /home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors_audited.py"
   ```
   `ls -la` result: `-rw-r--r-- 1 pi pi 11304 Jul 15 17:44
   .../pi_epuck_tcp_server_sensors.py` -- same size and modification
   date as before deployment; the original was never modified. Audited
   file permissions set to `644` (`-rw-r--r--`).

7. **`py_compile` on the Pi**:
   ```
   ssh pi@192.168.137.71 "cd /home/pi/real_robot_avoidance_v1 && python3 -m py_compile pi_epuck_tcp_server_sensors_audited.py && echo PYCOMPILE_OK"
   ```
   Result: `PYCOMPILE_OK` -- clean syntax check, no execution of the
   server logic itself (py_compile only compiles to bytecode, never
   runs `main()`).

8. **Deployed audited file SHA-256 verified**:
   ```
   ssh pi@192.168.137.71 "sha256sum /home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors_audited.py"
   ```
   Result: `c14543634629a39bbd2b7d60e79cd5973f6857d3dbc3f5b4b894f8a9cb9ffb33`
   -- **exact match** to the value recorded in `PROVENANCE.md` for
   `pi_epuck_tcp_server_sensors_audited.py`.

9. **Post-deployment process check** (read-only):
   ```
   ssh pi@192.168.137.71 "pgrep -af 'epuck_ros2_driver|pi_epuck_tcp_server_sensors\.py' | grep -v '^[0-9]* bash -c' || echo NO_MATCHING_PROCESS"
   ```
   Result: `NO_MATCHING_PROCESS` -- confirms deployment itself started
   no process.

10. **Final read-only permission check** (numeric permission,
    owner/group, path):
    ```
    ssh pi@192.168.137.71 "stat -c '%a %U/%G %n' /home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors_audited.py"
    ```
    Result: `644 pi/pi /home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors_audited.py`
    -- **exact match** to expected permission `644` and expected
    owner/group `pi/pi`.

## Summary table

| Item | Path | SHA-256 | Permissions / Owner |
|---|---|---|---|
| Original | `/home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py` | `51d3575d64717c3aacac1dcde3300da113a82be5b42980056890bd920a543a16` | `644` (unchanged) |
| Backup | `/home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py.backup_20260723_165028` | `51d3575d64717c3aacac1dcde3300da113a82be5b42980056890bd920a543a16` | (inherited from `cp`) |
| Audited (deployed) | `/home/pi/real_robot_avoidance_v1/pi_epuck_tcp_server_sensors_audited.py` | `c14543634629a39bbd2b7d60e79cd5973f6857d3dbc3f5b4b894f8a9cb9ffb33` | `644`, `pi/pi` |

## Explicit confirmations

- **No process was started at any point during this deployment** --
  confirmed read-only before (step 1) and after (step 9).
- **The original server file was never overwritten** -- confirmed
  identical size/mtime before and after (step 6), and its hash was
  never re-checked-and-changed (only the backup and the audited copy
  were newly created).
- **The robot remained powered off throughout.** No driver, bridge,
  state_publisher, guard, or the audited server itself was started.
  Only file-level operations (hash, copy, chmod, py_compile) were
  performed.
- **`command_audit_enabled` was never set to true in this session** --
  the deployed file is present on disk with its documented default
  (`False`) unchanged; enabling it and starting the server are
  separate, not-yet-taken future steps.

## What remains before any powered use of this file

Per `COMMAND_EVIDENCE_ACTIVATION.md` and `HIL_SAFETY_CHECKLIST.md`: the
robot must remain suspended/powered off until the Pi command audit is
actively running (`command_audit_enabled:=true`, not yet done), the
WSL command-evidence recorder is active, the guard is confirmed the
sole `/cmd_vel` publisher, and its output is confirmed continuously
zero -- none of which has been attempted in this session.
