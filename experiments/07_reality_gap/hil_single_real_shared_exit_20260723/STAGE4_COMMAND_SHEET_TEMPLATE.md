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

1. Pi Window 1: start Pi driver. Confirm ready.
2. Pi Window 2: start audited Pi server. Confirm ready.
3. WSL Window 1: start WSL bridge. Confirm connected.
4. WSL Window 2: start real `state_publisher.py`. Confirm `/epuck1/state`
   publishing, `validity_flags=7`.
5. Pi Window 3: run Pi-side verification. Confirm PASS.
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
14. PowerShell Window 1: transfer evidence off the Pi/WSL machines as
    needed and verify hashes.
15. Run `hil_stage4_post_run_verifier.py --mode physical` with an
    explicit operator-measurements file (manual displacement, corridor/
    stop-line crossing, boundary clearance, unexpected-behaviour flags)
    before citing any PASS/FAIL result.
16. Manual physical bring-up teardown (reverse of steps 1-4), Pi
    windows last.

## Recording

Do not write a SUMMARY.md before an actual run exists. This template is
reused verbatim as the actual command sheet only once a specific
authorized session begins.
