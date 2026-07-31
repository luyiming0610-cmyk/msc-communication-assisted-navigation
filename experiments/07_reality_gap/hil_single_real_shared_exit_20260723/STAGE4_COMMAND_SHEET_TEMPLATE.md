# Stage 4 command-sheet template (fixed window map)

Fill in and follow in order. This template does not itself authorize a
run -- it is a placeholder until a fresh design/implementation review
explicitly authorizes ground execution. **No physical run is
authorised by this document.** `run_hil_stage4_trial.sh --run` requires
an externally-supplied `EXPECTED_HEAD` and verifies Stage 4 source
identity before creating any output directory (see
STAGE4_PHYSICAL_HIL_SPEC.md Section 7).

A rehearsal PASS (`hil_stage4_post_run_verifier.py --mode rehearsal`)
is never a physical PASS. Only `--mode physical`, run against an actual
authorised ground trial's evidence plus an explicit operator-measurements
file, may be cited as a physical result.

## Fixed window map

| Window | Owns |
|---|---|
| Pi Window 1 | Pi driver |
| Pi Window 2 | Audited Pi server |
| Pi Window 3 | Pi-side verification |
| WSL Window 1 | WSL bridge |
| WSL Window 2 | Real `state_publisher.py` |
| WSL Window 3 | Stage 4 orchestrator (`run_hil_stage4_trial.sh --run`) and operator approval |
| WSL Window 4 | Read-only verification (topic echo/info, evidence tail) |
| PowerShell Window 1 | Evidence transfer/hash checks where required |

Manual physical bring-up (Pi Windows 1-3, WSL Windows 1-2) ends before
the orchestrator (WSL Window 3) starts. The orchestrator owns: WSL
command-evidence recorder, `hil_topic_adapter.py`, `cooperative_avoider`,
`hil_cmd_vel_guard.py`, `hil_stage4_motion_supervisor.py`,
`hil_virtual_peer.py`, the PID/status/evidence manifests, and exact
cleanup (recorder started first among orchestrator-owned processes,
stopped last).

## Sequence

1. Pi Window 1: start Pi driver (`ros2 run epuck_ros2_driver driver`).
   Confirm ready via `pgrep -af epuck_ros2_driver` (two PIDs).
2. Pi Window 2: start the **audited** Pi server -- this must be
   `pi_epuck_tcp_server_sensors_audited.py`, never the unaudited
   `pi_epuck_tcp_server_sensors.py`. The physical trial's Pi
   command-audit JSONL only exists if the audited server is running
   with its audit sink enabled:
   ```bash
   cd /home/pi/real_robot_avoidance_v1/ && \
     python3 pi_epuck_tcp_server_sensors_audited.py --ros-args \
     -p command_audit_enabled:=true \
     -p command_audit_path:=/home/pi/real_robot_avoidance_v1/command_audit_<RUN_ID>.jsonl
   ```
   Confirm ready (log line `Pi TCP bridge listening on 0.0.0.0:5809; ...`,
   `ss -ltnp | grep ':5809'` shows one listener).
3. WSL Window 1: start WSL bridge. Confirm connected.
4. WSL Window 2: start real `state_publisher.py`. Confirm `/epuck1/state`
   publishing, `validity_flags=7`.
5. Pi Window 3: run Pi-side read-only verification. Confirm PASS. This
   must explicitly verify, before physical startup:
   - the deployed audited server file (`pi_epuck_tcp_server_sensors_audited.py`)
     against this repo's `pi_command_audit/pi_epuck_tcp_server_sensors_audited.py`
     (`sha256sum` on the Pi side, `git hash-object` on the WSL side --
     compare the two values manually, since the Pi filesystem is not
     WSL-git-reachable);
   - the Pi audit verifier (`tools/pi_ground_diagnostic_audit_verifier.py`)
     and its `analyze_ground_diagnostic.py` dependency (both WSL-tracked
     files -- verify via `git -C <repo> hash-object -- <path>` against
     the expected committed blob, same technique as the Stage 4 source
     identity gate);
   - the actual driver executable/entry point actually invoked in step 1
     (resolve via `ros2 pkg prefix epuck_ros2_driver` on the Pi and
     record its SHA-256 for the run's records).
   None of these checks are performed by this correction turn -- no Pi
   contact was made; only the command sheet was corrected.
6. WSL Window 4: confirm zero publishers on `cmd_vel_stage4_raw`,
   `cmd_vel_unguarded`, `cmd_vel`, `/hil/goal_announcement`,
   `/hil/adoption_evidence`, `/epuck_virtual_peer/state`; confirm exactly
   1 publisher on `/epuck1/state`.
7. Wheel-suspension confirmation and test, per `HIL_LAB_RUNBOOK.md`
   Section 4 (STOP POINT -- separate explicit approval required there
   before proceeding).
8. WSL Window 3: `bash run_hil_stage4_trial.sh --check-only` then
   `--dry-run`; review the printed plan.
9. WSL Window 3: `bash run_hil_stage4_trial.sh --run`. At the stop
   point, type exactly `APPROVED_FOR_SINGLE_HIL_EVENT=YES`.
10. WSL Window 4: monitor `/hil_guard/arm`, `/cmd_vel`,
    `stage4_supervisor_evidence.jsonl` (tail -f) read-only. Do not
    publish anything from this window.
11. Observe the single physical pulse. Record manual measurements
    (body-centre displacement, boundary clearances, any unexpected
    rotation/sound) separately from odometry.
12. Confirm supervisor terminal state (`COMPLETE` or `FAILED`) and guard
    DISARMED before touching anything.
13. WSL Window 3: confirm orchestrator's own cleanup ran (recorder
    stopped last, `residual_check.json` says `CLEAN`); if not, run
    `run_hil_shutdown.sh <pid_manifest.json>` manually.
14. PowerShell Window 1: transfer the Pi command-audit JSONL and Pi
    verifier-verdict JSON into the evidence root as
    `pi_command_audit.jsonl` and `pi_verifier_verdict.json` (fixed
    names `--finalize` expects). Author `physical_measurements.json`
    (manual displacement, corridor/stop-line crossing, boundary
    clearance, unexpected-behaviour flags) in the same directory.
15. WSL Window 3: run
    `bash run_hil_stage4_trial.sh --finalize <evidence_root>`. This
    single, deterministic, read-only-except-hash-files step: validates
    every required file is present; extracts adoption evidence from the
    supervisor's own evidence JSONL; builds and verifies
    `SHA256SUMS.txt`; invokes the committed
    `hil_stage4_post_run_verifier.py --mode physical` against that
    immutable manifest; validates `post_run_verification.json`; builds
    and verifies `FINAL_SHA256SUMS.txt` (covering all final evidence,
    including `SHA256SUMS.txt` and `post_run_verification.json`,
    excluding only itself). Missing Pi evidence, measurements,
    `launcher_status.json`, `source_identity_manifest.json`, or any hash
    mismatch is `INVALID_EVIDENCE`, never silently defaulted. Do not cite
    a PASS/FAIL result from anything other than this step's own
    `STAGE4_FINALIZE=<classification>` output.
16. Manual physical bring-up teardown (reverse of steps 1-4), Pi
    windows last.

## Recording

Do not write a SUMMARY.md before an actual run exists. This template is
reused verbatim as the actual command sheet only once a specific
authorized session begins.
