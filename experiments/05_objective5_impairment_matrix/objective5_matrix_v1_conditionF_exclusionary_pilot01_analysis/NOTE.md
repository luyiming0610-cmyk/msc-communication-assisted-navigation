# EXCLUSIONARY_DIAGNOSTIC -- superseded, not a valid Condition F result

This attempt's `trial_verdict.json` reports `DATA_VALIDITY=INVALID` with
reason "relay pending_queue_depth was not confirmed 0 before shutdown
(queue-drain rule)". That attribution was **false** -- both directions'
`relay_status` payloads, visible directly in this run's own
`execution.log`, showed `pending_queue_depth: 0`. The real cause was an
orchestrator bug: `run_objective5_impairment_matrix_trial.sh` fed one
shared "has anything failed so far" bash flag (`DATA_VALIDITY`) into
`matrix_verdict.py`'s narrowly-named `queue_drained` field, so an
unrelated check tripping later in the pipeline got mislabeled as a
queue-drain failure. Fixed in commit `4928de7` (dedicated
`QUEUE_DRAINED`/`BAG_LOG_CLEAN` flags, `matrix_verdict.py` gained a
`bag_log_clean` field).

This run's process cleanup also left 6 orphaned processes (root-caused
and fixed separately in commit `4928de7`'s companion controller/relay
launch-service fix), which is the reason a second attempt
(`_pilot02`) was needed rather than trusting this one after a post-hoc
relabel.

This directory (bag, logs, verdict) is preserved as diagnostic evidence
of the bug, per this project's never-overwrite/never-delete discipline.
It must **not** be used as Condition F evidence. See
`objective5_matrix_v1_conditionF_exclusionary_pilot03_analysis/` for
the valid exclusionary pilot result (DATA_VALIDITY=VALID,
TASK_OUTCOME=SUCCESS), obtained after all four bugs found across
`_pilot01`/`_pilot02` were fixed.
