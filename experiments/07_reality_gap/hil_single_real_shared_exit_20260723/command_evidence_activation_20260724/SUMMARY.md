# Command-evidence chain activation attempt (2026-07-24)

**Classification: `INCOMPLETE_DIAGNOSTIC` / `EXCLUDED`.** The HIL guard
was never started; this attempt did not reach or complete the
verification steps (guard sole-publisher check, continuous-zero
check) that would have made it a real activation. Preserved here per
this project's standing rule against silently deleting or
reclassifying an incomplete run.

## What this session was

The first real, powered attempt to activate the Pi-side command audit
(deployed and verified in commit `8b7fc909590a31d088501acade90c13a38a710cc`)
together with the WSL-side continuous command-evidence recorder, per
`COMMAND_EVIDENCE_ACTIVATION.md`. Robot on its stand, wheels suspended,
throughout. All four physical confirmations
(`ROBOT_ON_STAND=YES`/`WHEELS_CLEAR_OF_GROUND=YES`/
`USER_AT_EMERGENCY_STOP=YES`/`TEST_AREA_CLEAR=YES`) were given before
any process started.

## Sequence executed

1. Pi driver started -- confirmed via `/scan` `Publisher count: 1`.
2. Audited Pi server started with `command_audit_enabled:=true`,
   `command_audit_path:=/home/pi/real_robot_avoidance_v1/command_audit_20260724_120000.jsonl`.
   Startup log confirmed `command_audit_enabled=True`, watchdog
   `0.50s` and limits `0.040m/s`/`2.000rad/s` unchanged from the
   original defaults. Audit file confirmed growing within ~1s
   (1945 lines, all `linear=0.0, angular=0.0, zero_reason=DISCONNECTED`).
3. WSL bridge started -- `TCP bridge connected`.
4. state_publisher started -- `/epuck1/state` `validity_flags=7`
   confirmed.
5. WSL command-evidence recorder, first attempt (`hil_command_evidence_20260724_102724`):
   **failed**. `run_hil_command_evidence_recorder.sh start` backgrounded
   the recorder with a plain `&` inside a one-shot
   `wsl.exe ... -- bash -lc "..."` invocation; the recorder process and
   its (empty) log both vanished within about a second of the invoking
   shell session ending. Root cause: a background job started with
   `&` alone remains a member of the invoking shell's session and is
   torn down with it; it was never actually detached. Diagnosed by
   running the recorder directly in the foreground (worked perfectly:
   topic-verify passed, 54 rows in 5s) and confirming the wrapped
   `start` invocation's process/log had disappeared entirely.
6. `run_hil_command_evidence_recorder.sh` fixed (`setsid ... < /dev/null`
   + `disown`) and the fix verified via a synthetic-topic smoke test
   surviving across separate one-shot WSL invocations, before being
   used again live.
7. WSL command-evidence recorder, second attempt (`hil_command_evidence_20260724_103601`):
   started successfully, confirmed alive (PID 1576) and topic-verified
   (`HIL_COMMAND_EVIDENCE_RECORDER_TOPIC_VERIFY ok=True`) in a separate,
   later check.
8. **Design gap found before the guard was started**: the recorder's
   CSV was, at this point in the project, only written once at
   shutdown (in-memory buffering throughout the run) -- checking for
   "the CSV is growing" (as `COMMAND_EVIDENCE_ACTIVATION.md` required)
   found no file at all. This directly violated the activation plan's
   own requirement. Flagged rather than worked around or silently
   waived.
9. User decision: do not start the guard; shut down cleanly instead,
   fix the CSV-flush design gap only after full physical shutdown.

## Shutdown (exact-PID SIGINT only, in order)

state_publisher (PID 473, wrapper 452) -> WSL bridge (PID 303) -> Pi
audited server (PID 900) -> Pi driver (PID 755, wrapper 754) -> WSL
recorder (PID 1576, via its manifest). Every stop confirmed via a
fresh, self-match-safe process check.

## Post-shutdown verification

- No physical/HIL process remained on either side.
- `/cmd_vel`: `Unknown topic '/cmd_vel'` (did not exist at all).
- Pi audit JSONL preserved: `/home/pi/real_robot_avoidance_v1/command_audit_20260724_120000.jsonl`,
  10,401,488 bytes, SHA-256 `084391acb60b8de30470308f69e7b697764c7fd4762463d396d385f8e49c8fc9`.
  Every record inspected showed only `zero_reason` in
  `{DISCONNECTED, COMMANDED_ZERO}` with `linear=0.0, angular=0.0` --
  **no nonzero command ever appeared in the Pi-side trail.**
- WSL CSV preserved (flushed at shutdown, per the pre-fix design):
  `raw_logs/attempt02_incomplete_diagnostic_20260724_103601/command_evidence.csv`,
  4008 rows, SHA-256 `4717b60f7391461a6f1b13735638f85d59ea13ef5e244b75f775332538e2b412`
  (verified identical between the native WSL path and this local copy).

## Evidence preserved (not committed, per this project's raw-evidence convention)

- `raw_logs/attempt01_failed_recorder_20260724_102724/`: the failed
  first recorder attempt's manifest and empty log -- preserved as
  direct evidence of the bug, not deleted because it "didn't work".
- `raw_logs/attempt02_incomplete_diagnostic_20260724_103601/`: the
  second attempt's manifest, log, and the 4008-row CSV.

## Explicitly not done

- The guard was never started. No verification of "guard is sole
  `/cmd_vel` publisher" or "output continuously zero" (with the guard
  actually running) was performed.
- This activation attempt is not a substitute for a completed one --
  a future attempt must repeat the full sequence from the start.
- `command_audit_enabled` and the WSL recorder's periodic-flush fix
  (this document's own trigger) are separate from, and do not
  retroactively validate, anything about the guard itself.
