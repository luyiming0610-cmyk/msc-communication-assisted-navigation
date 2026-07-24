# Command-evidence chain: exact future activation steps (2026-07-23)

Everything in this document is **designed, implemented, and unit-tested
offline only**. Nothing here has been run against the physical stack.
Follow `HIL_SAFETY_CHECKLIST.md`'s "Robot must remain suspended until
ALL of the following are true" section before any ground placement.

## Binding constraints on Pi deployment (restated, 2026-07-23 hardening review)

- **Pi deployment remains entirely manual and has not been performed.**
  No script in this repository copies anything to the Pi, connects to
  it, or modifies it in any way. The steps below are a procedure for a
  human to follow in a future session, not something any tool here
  automates or will ever automate.
- **The audited server must be reviewed and hash-verified immediately
  before deployment, every time** -- never deployed on the strength of
  an earlier review. `pi_command_audit/PROVENANCE.md`'s recorded SHA-256
  values are a snapshot from when they were written, not a permanent
  guarantee; re-verify them against whatever the Pi is actually running
  at deployment time (step 1 below), not against memory of a past
  check.
- **The original Pi server must be backed up before it is ever
  replaced.** Copy the Pi's current, running
  `pi_epuck_tcp_server_sensors.py` (and `bridge_protocol.py`) to a
  timestamped location before overwriting anything -- never rely on the
  git-tracked mirror in this repository as the only copy of what was
  actually running.
- **The robot must remain powered off (or, if powered, wheels
  suspended and the command path fully stopped) throughout Pi
  deployment and the server's subsequent startup.** Deployment and
  startup are themselves a change to the command path; per
  `HIL_SAFETY_CHECKLIST.md`, the robot may not be on the ground during
  any driver/server/bridge startup or reconnect, and swapping the
  server file is exactly that kind of event.

## Part 2 activation: Pi-side command audit

Currently **not deployed**. The Pi still runs the original, unaudited
`pi_epuck_tcp_server_sensors.py`. To activate in a future session:

1. **Robot powered off or wheels suspended and the command path fully
   stopped.** Re-review `pi_command_audit/pi_epuck_tcp_server_sensors_audited.py`
   and `pi_command_audit/PROVENANCE.md` -- confirm the diff summary
   still matches the file's current content (re-run
   `tools/audit_source_identity.sh` first to confirm the Pi's current
   file still matches
   `pi_command_audit/pi_epuck_tcp_server_sensors_original_mirror.py`'s
   recorded SHA-256; if it does not, the audited variant must be
   re-derived from whatever the Pi actually runs now, not assumed
   still valid). Do this review immediately before deployment, not from
   memory of an earlier check.
1a. **Back up the Pi's current, running file** (e.g. `cp
   pi_epuck_tcp_server_sensors.py
   pi_epuck_tcp_server_sensors.py.backup_$(date -u +%Y%m%dT%H%M%SZ)`
   on the Pi itself) before touching anything -- this is the only copy
   of exactly what was running before the change, independent of any
   git history.
2. Copy `pi_epuck_tcp_server_sensors_audited.py` to the Pi, replacing
   the running file, ONLY after that review and backup -- this is a
   deliberate, explicit, separate manual step, never automated by any
   script in this repository, and the robot must still be powered off
   or suspended with the command path stopped while this happens.
3. Start it with `command_audit_enabled:=true` and an explicit
   `command_audit_path` (e.g. a per-session timestamped path on the
   Pi's own filesystem) -- still with the robot powered off or wheels
   suspended and the command path stopped; only once the audit is
   confirmed active (step 4) does the rest of `HIL_SAFETY_CHECKLIST.md`'s
   ground-placement gate begin to apply.
4. Confirm the audit file is growing (`tail -f` or equivalent) before
   relying on it as evidence for that session.
5. After the session, retrieve the audit file, compute its SHA-256, and
   preserve it alongside that session's other evidence.

## Part 3 activation: WSL-side continuous command-evidence recorder

**Status updated 2026-07-24, after a real powered-session activation
attempt.** That attempt found two real bugs in what this section
previously described, both now fixed and covered by new tests (see
`command_evidence_activation_20260724/SUMMARY.md` for the incident):

1. `run_hil_command_evidence_recorder.sh start` backgrounded the
   recorder with a plain `&`; when invoked as a single one-shot shell
   command (exactly how it must be invoked from a separate controlling
   terminal via SSH/WSL), the process did not survive the invoking
   shell exiting -- it and its log both vanished within about a
   second. Fixed with `setsid ... < /dev/null` + `disown`.
2. The CSV was, at that point, only written once at shutdown
   (in-memory buffering for the whole run) -- there was no file at all
   to inspect while the recorder was running, contrary to what this
   document said below. **Fixed**: the CSV file and its header are now
   created immediately at construction, each row is written to disk as
   it arrives, and the file is flushed at a bounded interval (default
   1.0s, `--flush-interval-s`, never `os.fsync`) -- it is now a valid,
   growing, parseable file at essentially any point during a run.

To activate in a future session:

1. Start the recorder BEFORE any guard or controller:
   ```bash
   bash tools/run_hil_command_evidence_recorder.sh start \
       --upstream-cmd-vel-topic cmd_vel_unguarded \
       --guarded-cmd-vel-topic cmd_vel \
       --arm-topic /hil_guard/arm \
       --state-topic /epuck1/state \
       --bridge-status-topic /epuck_bridge/status \
       --flush-interval-s 1 \
       --duration-s 3600
   ```
   This prints `HIL_COMMAND_EVIDENCE_RECORDER_STARTED pid=<PID>
   manifest=<path>` and `output_dir=<path>`. Confirm its log
   (`<output_dir>/recorder.log`) shows
   `HIL_COMMAND_EVIDENCE_RECORDER_TOPIC_VERIFY ok=True` before
   proceeding -- if `ok=False`, STOP: the recorder itself refused to
   start because `/cmd_vel_unguarded` or `/cmd_vel` did not resolve as
   expected (see `hil_command_evidence_recorder.py`'s
   `verify_required_command_topics_present` for exactly what this
   does and does not prove).
2. **Now actually verifiable, unlike before the fix**: confirm the CSV
   is growing during the session --
   `wc -l <output_dir>/command_evidence.csv` should show more than
   just the header line once any subscribed topic has produced a
   message, and its row count should keep increasing on repeated checks.
3. Proceed with the rest of the session (guard, controller, virtual
   peer, etc.) as normal.
4. Stop the recorder LAST, after every other motion process:
   ```bash
   bash tools/run_hil_command_evidence_recorder.sh stop <manifest.json>
   ```
   This sends exactly one SIGINT to the exact recorded PID, waits up
   to 10s, then SHA-256-verifies the produced CSV and records the hash
   and row count back into the manifest.
5. Preserve `<output_dir>/command_evidence.csv`,
   `<output_dir>/recorder.log`, and `<output_dir>/manifest.json`
   (with its `csv_sha256` field) as that session's evidence.

## What this closes, and what it still does not prove

Once both are active in a session, a future safety question about that
session can be answered with continuous, timestamped records of every
command-relevant topic on both sides of the bridge, closing the exact
gap that forced both 2026-07-23 incidents' command origin to be
recorded NOT_MEASURABLE. It does not retroactively apply to either
2026-07-23 incident, and it does not by itself prove the ROOT CAUSE of
either incident -- it only ensures a FUTURE incident would not face the
same evidence gap.
