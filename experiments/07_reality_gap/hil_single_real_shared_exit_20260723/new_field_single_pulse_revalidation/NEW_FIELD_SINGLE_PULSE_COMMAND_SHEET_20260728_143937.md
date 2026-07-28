# NEW_FIELD_SINGLE_PULSE_COMMAND_SHEET_20260728_143937

**Status: offline documentation only. No physical process, no RUN_ID
regeneration, no Pi contact, and no evidence collection occurred as
part of preparing this document.** This is the single, dedicated,
exact executable command sheet for `NEW_FIELD_SINGLE_PULSE_REVALIDATION`
under the frozen session identifier `20260728_143937` -- see
`NEW_FIELD_SINGLE_PULSE_REVALIDATION_SPEC.md` for the full design
rationale and acceptance/exclusion rules, which this sheet does not
repeat.

## Frozen identifiers for this session (do not regenerate)

```
RUN_ID=20260728_143937
PI_JSONL=/home/pi/real_robot_avoidance_v1/command_audit_20260728_143937.jsonl
WSL_OUTPUT_ROOT=/home/eamon/epuck_comm_bags/new_field_single_pulse_revalidation_20260728_143937
WSL_CSV=/home/eamon/epuck_comm_bags/new_field_single_pulse_revalidation_20260728_143937/command_evidence.csv
WSL_MANIFEST=/home/eamon/epuck_comm_bags/new_field_single_pulse_revalidation_20260728_143937/manifest.json
```

## New-field parameter override (binding)

The live preflight must consume the new-field geometry file, never the
historical `ground_diagnostic_params.json`:

```
GROUND_DIAGNOSTIC_PARAMS=<repo>/experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/new_field_geometry_params.json
```

## Recorder command (exact, binding)

```bash
bash run_hil_command_evidence_recorder.sh start \
  --output-root /home/eamon/epuck_comm_bags/new_field_single_pulse_revalidation_20260728_143937 \
  --upstream-cmd-vel-topic cmd_vel_unguarded \
  --guarded-cmd-vel-topic cmd_vel \
  --arm-topic /hil_guard/arm \
  --state-topic /epuck1/state \
  --bridge-status-topic /epuck_bridge/status \
  --flush-interval-s 1 \
  --duration-s 3600
```

`--flush-interval-s 1` is the existing default value, used as-is. It
must not be changed to the shorter interval used for repeatability
trials elsewhere in this project unless a previously reviewed and
committed new-field requirement explicitly documents that this
specific field or session requires it. No such requirement has been
recorded for this revalidation, so the default is used unchanged, not
assumed or substituted.

## Pulse command (exact, binding)

```bash
python3 hil_ground_single_pulse_test.py \
  --upstream-cmd-vel-topic cmd_vel_unguarded \
  --pulse-linear-mps 0.015 \
  --zero-hold-s 1 \
  --pulse-s 6.67 \
  --post-hold-s 1
```

## Prohibitions (binding)

- Do not use `hil_wheel_suspension_test.py` for this run -- this run's
  wheels bear weight on the ground, they are not suspended.
- Do not use `--pulse-s 2`, the historical two-second suspended-wheel
  pulse duration, for this run.
- Do not use `--output-csv` with the recorder wrapper for this run --
  it rejects that bare flag outright with no process started;
  `--output-root` above is the only supported override.
- Do not publish directly to `/cmd_vel`.
- `cmd_vel_unguarded` is the only permitted pulse input topic.
- `hil_cmd_vel_guard` is the sole permitted `/cmd_vel` publisher.
- `angular.z` must remain exactly `0.0` throughout -- there is no CLI
  override for it in the pulse tool, by design.
- Exactly one pulse is permitted under this command sheet.
- No manual, automatic, or recovery repeat of the pulse is permitted.
- The recorder must remain running for the complete duration of the
  pulse.
- The recorder must be stopped last, through the frozen manifest above.
- Any early recorder exit makes the run `EXCLUDED`.

## Required gates before motion (binding)

- `LIVE_ZERO_STATE_CHECK_PASS` and all required pose/readiness checks
  must pass before ground placement.
- `APPROVED_FOR_SINGLE_PULSE=YES` must be given immediately before
  arming or any motion -- a separate, explicit confirmation, distinct
  from every gate above it.
- Any unexpected motion, sound, rotation, acceleration, connection
  loss, evidence-chain failure, or command-source uncertainty requires
  immediate physical power-off (per the confirmed emergency power-off
  arrangement in `NEW_FIELD_SINGLE_PULSE_REVALIDATION_SPEC.md`) and
  `EXCLUDED` classification.
