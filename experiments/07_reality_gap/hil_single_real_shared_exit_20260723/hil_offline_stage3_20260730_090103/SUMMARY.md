# Stage 3 offline hardware-free run 20260730_090103 -- FAILED (software bring-up)

**Classification: `FAIL_VALID_EVIDENCE`.**

```
RUN_ID=20260730_090103
FINAL_STAGE3_EXECUTION_STATUS=FAIL_VALID_EVIDENCE
STAGE3_BEHAVIORAL_RESULT=NOT_OBTAINED
FAILURE_PHASE=SOFTWARE_BRINGUP
ROOT_CAUSE=HIL_TOPIC_ADAPTER_GOAL_NAVIGATOR_IMPORT_PATH
```

This run failed during software bring-up, before any Stage 3 behavioural
event occurred. It is **not** evidence about GoalAnnouncement adoption,
recovery behaviour, cooperative-navigation performance, or a
communication/safety-policy failure -- it is valid evidence of a
software bring-up defect only. This RUN_ID is permanently excluded from
reuse; the failed evidence root below is preserved unmodified and must
never be overwritten by a later run.

## Frozen code identity at the time of this run

Git HEAD: `4023178d3014b5d48b67ac2924b41486eb279897` (source/installed-runtime
identity manifest confirmed `overall_result: PASS` before launch -- the
failure occurred entirely after identity verification, during process
bring-up).

## What happened

`hil_topic_adapter.py` crashed at import time:

```
ModuleNotFoundError: No module named 'goal_navigator'
```

Root cause: `_import_goal_navigator()`'s `_TOOLS_DIR` computation used a
three-call `os.path.dirname()` chain that landed one directory level
short of `experiments/`, computing a nonexistent path. Recorder and
guard started and reached READY; the adapter never did. The 15s
readiness-timeout gate correctly aborted the run before
`cooperative_avoider`, the harness, or the virtual peer were ever
launched. Cleanup completed with no residual process.

## Evidence (raw files preserved on disk only, gitignored, never committed here)

Root: `/home/eamon/epuck_comm_bags/hil_offline_stage3_20260730_090103/`

| File | Size (bytes) | SHA-256 |
|---|---|---|
| `adapter.log` | 784 | `4838b32eae05cf8a0debf217420835701ff55df631582a6ed37c3dad2b6fb2d6` |
| `evidence.csv` | 36236 | `dab919cc023e700dc0df62fd78af04a998d97977aa99b8d00bcacc17032d22fb` |
| `execution_script.sh` | 52437 | `4305310ccaaa4123dedcdad7ffa6c535f646e7b627ecd21d9dfebc0db41e0c45` |
| `guard.log` | 5471 | `df5a5716103ea29a32081029accf5b8be11e3b59d621884f5347c81f2ed45806` |
| `recorder.log` | 200 | `699711af9e54e7180ea1f477b6ab7175ee1e04af881c943cc76d15115af563ea` |
| `source_identity_manifest.json` | 22470 | `006ca2de92318ab57be5b2546ac4acde957be878ffb1c055898afa5a2ef74c76` |
| `summary.json` | 1066 | `589d58dcc27dc25ae777569f45293fd9ba83ef398f53fdb762a985bb90e30653` |

No `pid_manifest.json`, `launcher_status.json`, `post_run_verification.json`,
`verifier_exit_status.txt`, `SHA256SUMS.txt`, `cooperative_avoider.log`,
`harness.log`, or `virtual_peer.log` exist for this run -- the script
correctly aborted before reaching those steps; their absence is expected,
not evidence corruption. (A later runbook correction added incremental,
always-run evidence finalization for exactly this class of early abort;
it postdates this run and was not applied retroactively.)

## Confirmed properties

- `evidence.csv`: 291 rows, single topic (guarded `cmd_vel`), zero
  nonzero linear/angular values, zero gate-decision/phase/announcement
  events -- consistent with the adapter never starting.
- Recorder and guard shut down cleanly; no residual process
  (`POST_RUN_RESIDUAL_PROCESS_CHECK=CLEAN`).
- Behavioural verifier never ran (behavioural execution never began).

## Corrective action

Root cause and fix are recorded in the git history: commit
`9483c30fb80d0c2f336ed960c3a4ae29e8cdfb34` ("Fix Stage 3 GoalNavigator
import resolution") and commit `d1fd94b5ca893dc9648253fff400ec8e4074c864`
("Finalize Stage 3 evidence on early failure"). See
[../hil_offline_stage3_20260730_112847/SUMMARY.md](../hil_offline_stage3_20260730_112847/SUMMARY.md)
for the corrected run.

## Limitations -- what this run does and does not establish

- Establishes only that `hil_topic_adapter.py`'s path-resolution logic
  was broken at HEAD `4023178`, and that source/installed-runtime
  identity verification, recorder, and guard bring-up worked correctly
  up to the point of failure.
- Does not establish anything about GoalAnnouncement generation,
  reception, adoption, gate/reopen behaviour, cooperative-avoidance
  behaviour, or final safe-state -- none of that code path executed.
- Not a physical robot experiment; not evidence of physical motion,
  physical communication behaviour, or navigation performance of any
  kind.
