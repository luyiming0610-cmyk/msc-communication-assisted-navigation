# Three-robot shared-exit attempt history

## Formal paired Trial 01, ON attempt01 - excluded failure

- Execution commit: `aa827c8b920b38a5a2ffa28e6d0eb31d053e83d5`.
- Outcome: `VALID` data, `FAIL` task outcome, watchdog termination.
- Raw evidence is preserved under `/home/eamon/epuck_comm_bags/shared_exit_n3_n3_exit_comm_on_trial01_attempt01` and its diagnostic-log sibling.
- Human observation: robots B and C initially stopped in their parking regions, then moved out again.
- Direct log evidence: the multi-peer selectors classified B/C as `CPA_RISK` at approximately `0.340 m`; the preregistered adjacent parking-centre spacing was `0.335 m`, below the controller's frozen `0.340 m` proximity trigger.
- Consequence: both robots re-entered peer avoidance after arrival. Robot C later entered the unchanged local-sensor failsafe and stopped with `DURATION_CEILING`.
- Methodological disposition: excluded from successful formal statistics, retained as a failed formal attempt. It must never be overwritten or relabelled as a pass.

## Formal paired Trial 01, OFF attempt01 - valid but not paired after geometry revision

- Execution commit: `aa827c8b920b38a5a2ffa28e6d0eb31d053e83d5`.
- Outcome: `VALID` data, `SUCCESS` task outcome, with no observed anomaly.
- Raw evidence is preserved under `/home/eamon/epuck_comm_bags/shared_exit_n3_n3_exit_comm_off_trial01_attempt01` and its diagnostic-log sibling.
- Methodological disposition: valid evidence for the old parking geometry, but excluded from the corrected paired comparison because its ON counterpart failed and the parking geometry was subsequently revised.

## Corrective action and planned comparison

The N=3-only reception area is widened and adjacent parking-centre spacing is increased to `0.450 m`. Communication avoidance, local IR/ToF avoidance, `safety_radius_m=0.14`, and the controller's `trigger_distance_m=0.34` remain enabled and unchanged. The corrected configuration requires new OFF and ON pilots, followed by a matched OFF/ON formal attempt02.

The dissertation may use the excluded attempt as a failure-analysis case: an apparently successful parking event was invalidated by a geometry/controller interaction, diagnosed from human observation plus selector and controller logs, corrected without weakening safety, and then retested against the same completion criteria.

## Formal Trial 04, ON attempt01 - excluded startup failure

- Execution commit: `311f96e98385b19a1ed654d67970bcb41d25bd64`.
- Outcome: `VALID` recorded data, `FAIL` task outcome, watchdog termination.
- Raw evidence is preserved under `/home/eamon/epuck_comm_bags/shared_exit_n3_n3_exit_comm_on_trial04_attempt01` and its diagnostic-log sibling.
- Direct cause: the epuck2 `diffdrive_controller` load service timed out during Webots startup. The controller was present but not activated, so epuck2 published state with `validity_flags=0x06` instead of `0x07` and never supplied valid odometry.
- Safety response: every multi-peer selector withheld output under the unchanged fail-closed policy; all three controllers remained in `SAFE_STOP_STALE`. No task motion or goal announcement occurred.
- Methodological disposition: excluded startup failure, never counted as a successful formal trial and never overwritten.
- Corrective action: the orchestrator now requires all three state streams to produce `validity_flags=7` and all three selected-peer topics to produce real messages before rosbag recording and task timing begin. Topic-name existence alone is no longer treated as readiness.

## Formal Trial 04, OFF attempt02 - excluded pre-record orchestration failure

- Execution commit: `e74137f267d3b049362362d571e2b5ccd138b6a3`.
- Outcome: the new readiness function stopped with an unbound local shell variable before rosbag recording, controllers, navigators, or task timing began.
- No task-level result exists and this attempt is not counted in formal statistics.
- The diagnostic directory is preserved and the name is never reused.
- Corrective action: the timeout and deadline local variables are initialized in separate statements under `set -u`; the corrected matched Trial 04 pair uses attempt03.
