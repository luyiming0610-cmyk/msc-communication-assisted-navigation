# EXCLUSIONARY_DIAGNOSTIC -- superseded, not a valid Condition F result

This attempt's `trial_verdict.json` reports `DATA_VALIDITY=INVALID` with
two reasons: a genuine `queue_drained=false` (epuck2's `relay_status
--once` echo returned empty within the then-3s timeout -- a real DDS
discovery/timing race, fixed in commit `e49381d` by widening the
timeout to 6s with one retry) and a **false** `bag_log_clean=false`
("bag_record.log contained drop/warn/error line(s)"), even though this
run's own `execution.log` printed "no drop/warn/error lines in
bag_record.log" immediately above the verdict. The false positive was a
`grep -c` idiom bug: `$(grep -c PATTERN FILE || echo 0)` runs BOTH
branches whenever FILE has zero matches (grep -c still prints "0" but
exits status 1), producing the two-line string "0\n0", which never
equals the single-line "0" comparison downstream. Fixed in commit
`e49381d`.

This directory (bag, logs, verdict) is preserved as diagnostic evidence
of both bugs, per this project's never-overwrite/never-delete
discipline. It must **not** be used as Condition F evidence. See
`objective5_matrix_v1_conditionF_exclusionary_pilot03_analysis/` for
the valid exclusionary pilot result (DATA_VALIDITY=VALID,
TASK_OUTCOME=SUCCESS), obtained after all four bugs found across
`_pilot01`/`_pilot02` were fixed.
