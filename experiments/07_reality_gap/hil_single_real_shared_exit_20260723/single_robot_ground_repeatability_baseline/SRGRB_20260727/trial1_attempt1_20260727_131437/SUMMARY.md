# SRGRB_20260727 -- Trial 1, Attempt 1 -- RUN_ID 20260727_131437

**Classification: `EXCLUDED`.** Reason: `RECORDER_STARTUP_CRASH_WRONG_OUTPUT_PATH`.

**This is not a motion failure and not a safety failure.** No arm
command was ever issued, the robot was never placed on the ground, no
pulse occurred, and no motion of any kind was observed at any point.
The robot remained on its stand, wheels clear of the ground, for the
entire attempt. The exclusion is purely procedural/instrumentation:
the command-evidence recorder crashed at startup, before any physical
readiness gate (`LIVE_ZERO_STATE_CHECK`, `REPEATABILITY_POSE_READINESS`)
was ever attempted.

## What happened

1. Bring-up proceeded normally through the Pi driver, the audited Pi
   command server, the WSL TCP bridge, and `state_publisher` -- all
   four started, initialized, and were confirmed running exactly as
   expected.
2. The WSL command-evidence recorder was started via
   `run_hil_command_evidence_recorder.sh start`, with an extra
   `--output-csv` argument intended to target this attempt's frozen
   evidence root (`srgrb_20260727_trial1_attempt1_20260727_131437/`).
3. The recorder process (PID `876`) crashed immediately with
   `FileNotFoundError` and exited. Root cause: the wrapper script
   always builds its own `--output-csv` from an auto-generated
   timestamped directory and places it ahead of any caller-supplied
   extra arguments; the caller's own `--output-csv` silently won
   inside the recorder's own argument parsing (last-value-wins), but
   the wrapper's own bash variables -- and therefore the directory it
   actually created via `mkdir -p` -- still referred to its own,
   different, auto-generated path. The frozen evidence root's parent
   directory was never created, so `open()` failed.
4. The crash was noticed immediately (the recorder's reported manifest
   pointed at a directory other than the frozen one) and the attempt
   was stopped before proceeding any further -- the guard was never
   started, no arm command was ever sent.

## Shutdown

Exact-PID `kill -INT` only, never `pkill`. Reverse order, recorder
already dead before shutdown began (no action taken on it):

| Process | PID(s) | Confirmed stopped |
|---|---|---|
| state_publisher | 790 (wrapper) / 791 (node) | Yes -- `pgrep` empty |
| WSL bridge | 712 | Yes -- `pgrep` empty |
| Audited Pi server | 1125 | Yes -- `pgrep` empty |
| Pi driver | 1049 (wrapper) / 1050 (node) | Yes -- `pgrep` empty |
| WSL command-evidence recorder | 876 | Already exited (crash) before shutdown began |

Confirmed afterward: no related process remains on either machine
(`pgrep` empty for `state_publisher`, `wsl_epuck_tcp_bridge`,
`hil_cmd_vel_guard`, `hil_command_evidence_recorder`); `/cmd_vel`
absent from `ros2 topic list` (the guard was never started, so this
topic never existed).

## Evidence

**No raw evidence exists for this attempt.** The recorder crashed
before writing any CSV row; the frozen evidence root
(`/home/eamon/epuck_comm_bags/srgrb_20260727_trial1_attempt1_20260727_131437/`)
was never created and remains absent, preserved exactly as found at
the moment of the crash. The auto-generated directory the wrapper
mistakenly created
(`/home/eamon/epuck_comm_bags/hil_command_evidence_20260727_124840/`)
contains only `manifest.json` (recording PID `876` and the wrong,
never-actually-used `csv_path`) and `recorder.log` (the Python
traceback) -- both preserved unmodified, left in place as the record
of the crash, not deleted or "cleaned up." No Pi JSONL was ever
produced (the audited server never received a single command). No
`post_run_verification.json` exists for this attempt --
`ground_diagnostic_post_run_verifier.py` was never run against it,
since there is no evidence to verify. All of the above remains local
and gitignored, exactly as for every other attempt's raw evidence.

## Fix applied offline, after full shutdown

`run_hil_command_evidence_recorder.sh` now accepts an explicit
`--output-root <dir>` and rejects a bare `--output-csv` outright
(nonzero exit, before any process is spawned) -- see the script's own
module docstring and
`test_run_hil_command_evidence_recorder_output_root_e2e.sh` for the
full fix and its regression coverage. No code was edited while the
physical stack was live; the fix was made only after the shutdown
recorded above was fully confirmed.

## Effect on the batch

Per this repeatability baseline's approved retry policy and
`GROUND_DIAGNOSTIC_RUNBOOK.md`'s existing rule ("if the recorder exits
... before the pulse step completes, the run is EXCLUDED"), this
attempt's `EXCLUDED` classification ends the entire `SRGRB_20260727`
batch (`BATCH_ABORTED_EXCLUDED`) and requires a fresh approval cycle
before any further attempt -- see `BATCH_SUMMARY.md` in this batch's
directory.
