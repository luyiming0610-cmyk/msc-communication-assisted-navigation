# Stage 3 offline hardware-free run 20260730_112847 -- PASS

**Classification: `HARDWARE_FREE_HIL_INTEGRATION_VALIDATION_PASS`.**

```
RUN_ID=20260730_112847
FINAL_STAGE3_EXECUTION_STATUS=PASS
DATA_VALIDITY=VALID
TASK_OUTCOME=SUCCESS
STAGE3_STATUS=COMPLETE
```

## Scope and limitations (read this before citing this result anywhere)

This run is:

- a **hardware-free HIL integration validation** -- every node ran as a
  plain OS process on one WSL machine, on an isolated ROS 2 domain
  (`ROS_DOMAIN_ID=91`, `ROS_LOCALHOST_ONLY=1`), over private
  `/hil_offline_stage3/...` test-only topics;
- evidence that the **automatic virtual-event-to-real-controller-software
  path works**: a scripted virtual peer's GoalAnnouncement is received by
  the real, unmodified `GoalNavigator`/`cooperative_avoider` software
  (the same classes used in production), producing the same
  gate/adoption/recovery decisions those classes make in the field.

This run is **not**:

- a physical robot experiment;
- evidence of physical motion of any kind;
- evidence of physical communication or communication-impairment
  behaviour;
- a navigation-performance experiment;
- an n=5 (or any n>1) statistical result -- this is a single (n=1),
  fully automatic, hardware-free integration trial.

## Frozen code identity

Execution HEAD: `d1fd94b5ca893dc9648253fff400ec8e4074c864` (the corrected
runbook + adapter fix + evidence-finalization commits). Source and
installed-runtime identity verification result: **`overall_result: PASS`**
across all layers -- 23/23 Git source paths, 11/11 installed Python
modules, installed launcher (AST-verified), installed entry-point
metadata (exactly one candidate), and all 3 generated ROS message
schemas (`EpuckState`, `GoalAnnouncement`, `NavigationIntent`).

## Evidence root

`/home/eamon/epuck_comm_bags/hil_offline_stage3_20260730_112847/`
(raw files preserved on disk only, gitignored, never committed here)

| File | Size (bytes) | SHA-256 |
|---|---|---|
| `adapter.log` | 1007 | `1c4b57e84289577e57690761b515b1113f3bcae572630150bdd0877a964fa342` |
| `cooperative_avoider.log` | 4146 | `f10f40a078d32dbbb141a0cc00ecae47b6c60dc6c2f54758c67641d765ac6aee` |
| `evidence.csv` | 46266 | `3cd6a3eca4518fe991358b45325048e0df90f5dc1cf7dd23db3e6238fb8e8ab5` |
| `execution_script.sh` | 60247 | `daf47541c579a7be7b0363054d81503f7f33d3917c368df55c68790bc4be3b7f` |
| `guard.log` | 2138 | `b28f763ca0207e9d14ae214b5e9348cd076b1b459b878039c95816418ffd445c` |
| `harness.log` | 525 | `d4983a5baaeaf44b73f76154aa7317e3f88132f1aa2b8e1063886240795661fc` |
| `launcher_status.json` | 361 | `7ae44c01c62aee59afb4f83a8f8b04600d69a08d2974239700fb2e42d354aea5` |
| `pid_manifest.json` | 1894 | `9722d17b83ab3327461a764c6272d585736f6fed15fc02f50c66318086a82ddd` |
| `post_run_verification.json` | 694 | `f07d24de56faf35b3e0e9e9a0f262de525a03aef9cd13a00ea45cb57a6529f0f` |
| `recorder.log` | 200 | `a890198c0602d75973eacf43d1fe46430293eb798dc5b56f7303b61c72dcf1ba` |
| `source_identity_manifest.json` | 22470 | `5d47d6b8a311449c462006b4c5d2d4c2d99dda41818c16e2a32d59a71e88c7c1` |
| `summary.json` | 1595 | `473db14b967d58d2940666bda4b1dbf8260b18d74c97068f1eb05a6aca5ca58a` |
| `verifier_exit_status.txt` | 2 | `9a271f2a916b0b6ee6cecb2426f0b3206ef074578be55d9bc94f6f3fe3ab86aa` |
| `virtual_peer.log` | 224 | `7b0ab62fa824e9bf130146680f2a33469f3351705f8b0d03576b453de1100b02` |

All 14 files hash-verified against the run's own `SHA256SUMS.txt` via
`sha256sum -c` (all `OK`), and copied above unmodified from that file.

## Component readiness

All six owned processes reached `READY` (exact PID/PGID/executable
captured in `pid_manifest.json`); `cooperative_avoider` correctly holds
its own isolated process group (`PGID=798`, distinct from the shared
`PGID=413` of the other five). Harness exited `0`.

| Component | Readiness |
|---|---|
| Recorder | READY |
| Guard | READY (armed later per phase sequence below) |
| Adapter | READY |
| `cooperative_avoider` | READY |
| Harness | READY (exit 0) |
| Virtual peer | READY |

## Ordered automatic phase sequence

```
ANNOUNCEMENT_ADOPTED -> DISARMED_ZERO_CONFIRMED -> ARMED_BOUNDED_CONFIRMED
-> PEER_GATE_CLOSED -> STALE_ZERO_CONFIRMED -> PEER_GATE_REOPENED
-> RECOVERY_CONFIRMED -> DUPLICATE_REJECTED -> COMPLETE
```

## Automatic GoalAnnouncement generation and adoption

The virtual peer automatically published one GoalAnnouncement
(`HIL_VIRTUAL_PEER_ANNOUNCEMENT_TX target=(2.0,3.0)`); the
adapter-hosted `GoalNavigator` received and adopted it automatically
(`HIL_GOAL_ANNOUNCEMENT_EVIDENCE ... accepted=True duplicate=False`) --
no message was published manually.

## Deliberate duplicate rejection

The harness's scripted duplicate-announcement test (synthetic
`source_sequence=999999`) was correctly rejected:
`accepted=False duplicate=True`, `duplicate_sent=True` recorded in
evidence -- exactly one duplicate-flagged row, as designed.

## Closed-gate rejection counts

32 gate-decision events total: 5 `FORWARDED` (gate OPEN, epoch 0),
26 `REJECTED_GATE_CLOSED` (gate CLOSED, epoch 0), then 1 `FORWARDED`
(epoch 1, `first_after_reopen=True`).

## Reopen/recovery result

Confirmed: phase sequence shows `PEER_GATE_REOPENED` followed by
`RECOVERY_CONFIRMED`, matching the gate-epoch 0->1 transition above.

## Final zero result

Final guarded `cmd_vel` row: `linear_x=0.0 angular_z=0.0`; zero nonzero
rows in the last 10 recorded guarded-command samples.

## Cleanup order

Reverse launch order, recorder last: virtual peer -> harness ->
`cooperative_avoider` -> adapter -> guard -> recorder.
`recorder_confirmed_stopped: true`. No residual process
(`POST_RUN_RESIDUAL_PROCESS_CHECK=CLEAN`, independently re-confirmed
after the run).

## Verifier result

```
DATA_VALIDITY=VALID   data_validity_reasons=[]
TASK_OUTCOME=SUCCESS  task_outcome_reasons=[]
```

Both reason lists empty -- no failure condition recorded.

## Related

See [../hil_offline_stage3_20260730_090103/SUMMARY.md](../hil_offline_stage3_20260730_090103/SUMMARY.md)
for the preceding failed bring-up run and its root cause/fix.
