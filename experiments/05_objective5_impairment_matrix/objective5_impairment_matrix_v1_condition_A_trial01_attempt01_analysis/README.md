# objective5_impairment_matrix_v1_condition_A_trial01_attempt01

**FORMAL_SIM**, Objective 5 impairment matrix, **Condition A** (zero
impairment), **Trial 01**, **attempt 01**. First trial of the formal
n=5 Condition A batch -- **not** the whole batch; Trials 02-05 have not
run.

## Files in this directory (git-tracked)

- `frozen_params.json` -- parameters resolved from the frozen conditions
  CSV before the run started, plus code identity (commit, script
  SHA-256s), written by the orchestrator.
- `matrix_analysis.json` -- the real analyzer's output
  (`matrix_analyzer.py`, strict schema, `legacy_replay=false`):
  per-direction relay/sequence/latency/throughput, queue drain, realtime
  factor.
- `trial_verdict.json` -- the two-dimensional verdict
  (`matrix_verdict.py`): `DATA_VALIDITY`/`TASK_OUTCOME`.
- `final_verdict.json` -- this trial's closeout: verdict + safety margin
  + manual-observation confirmation + process-cleanup/bag-integrity
  checks + raw-data-preservation record, written after the user's
  real-time visual observation was received and cross-checked against
  the automated evidence.
- `runtime_manifest.json` -- launch method, frozen condition params,
  code identity, bag metadata, raw-data paths.
- `summary.md` -- human-readable summary of all of the above.
- `SHA256SUMS` -- SHA-256 of every file in this directory, plus every
  raw bag/diag_logs file preserved outside git (for independent
  verification against the Windows copy and the native WSL source).

## What is NOT here (by design)

The raw rosbag (`.db3`/`metadata.yaml`) and the raw diag logs (relay
CSVs, `sequence_counter.py` JSON, controller/simulation/state/bag_record
logs) are **not committed to git** -- per project policy (`*.db3`,
`*.log` are gitignored globally; this experiment's whole
`bags/` directory is additionally gitignored). They are preserved in
two places, verified identical by SHA-256:

- native WSL: `/home/eamon/epuck_comm_bags/objective5_impairment_matrix_v1_condition_A_trial01_attempt01`
  and the sibling `_diag_logs` directory
- Windows copy (not git-tracked):
  `experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_A_trial01_attempt01/`

## Result (see `summary.md` / `final_verdict.json` for full detail)

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `analyzer_ok=true` under
the strict formal-trial latency schema (p99 is a genuine finite value,
not null, on both directions). Manual observation confirms avoidance
completed with no collision, spinning, or desynchronization.
`min_interrobot_distance_m=0.1430942842844398` exceeds
`safety_radius_m=0.14` by a small margin (~3.09mm) -- reported
explicitly, PASS.

## Batch status

`TRIAL_01_PASS_PENDING_BATCH_COMPLETION`. Condition A's formal batch is
**not** complete. Trial 02 has not been started and will not be started
automatically.
