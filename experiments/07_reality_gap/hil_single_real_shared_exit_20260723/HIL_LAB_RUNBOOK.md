# HIL lab runbook (2026-07-23)

This is the step-by-step operator procedure. It does not authorize
skipping any step in `HIL_SAFETY_CHECKLIST.md` -- it only sequences
them. Stop immediately and report if any step's actual result differs
from its expected result.

## 1. Offline checks (no hardware, no network)

```bash
cd experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools
python3 -m unittest discover -s . -p "test_hil_*.py" -v
bash run_hil_shared_exit_trial.sh --check-only
bash run_hil_shared_exit_trial.sh --dry-run
```

Expected: all unit tests pass; `--check-only` prints
`hil_preflight.py`'s JSON report; while `hil_frozen_params.json` still
has any `UNCONFIRMED_PHYSICAL_MEASUREMENT` field, its `status` is
`BLOCKED_AWAITING_LAB_MEASUREMENT`, which is the correct/expected
result at this stage, not a failure.

## 2. Read-only physical connectivity check

Only after step 1. Verifies the robot is reachable, the driver/bridge
are running, and topics/validity flags look sane -- publishes nothing,
starts no navigation, causes no ground motion. See
`HIL_SAFETY_CHECKLIST.md` for the full precondition list before this
step, and report using the named statuses (`PHYSICAL_DEVICE_REACHABLE`,
`DRIVER_STATUS`, `BRIDGE_STATUS`, `SENSOR_TOPICS_READY`,
`VALIDITY_FLAGS`, `CMD_VEL_PUBLISHER_COUNT`, `RESIDUAL_PROCESS_CHECK`,
`CLOCK_DOMAIN_STATUS`, `STILL_UNCONFIRMED_PARAMETERS`).

## 3. Wheel-suspension confirmation and test (STOP POINT)

Do not proceed past step 2 without the user providing, verbatim, in
the same session:

```
ROBOT_ON_STAND=YES
WHEELS_CLEAR_OF_GROUND=YES
USER_AT_EMERGENCY_STOP=YES
TEST_AREA_CLEAR=YES
```

Then run the short suspended-wheel sequence described in
`HIL_SAFETY_CHECKLIST.md`. After it, stop all motion nodes, verify
zero velocity, verify exactly one guarded `cmd_vel` publisher, stop the
actual PIDs directly (never `pkill`), confirm `PROCESSES_CLEAN`, and
wait for the user's own report on wheel directions / stopping /
anomalies / guard effectiveness before continuing.

## 4. Field geometry measurement

Only after step 3 passes. Measure (do not estimate): arena dimensions,
start pose, exit location/size, parking zone(s), walls/boundary,
obstacles, emergency-stop position, WiFi coverage, minimum safety
clearance. Write these into `hil_frozen_params.json`'s
`field_geometry` block, replacing each `UNCONFIRMED_PHYSICAL_MEASUREMENT`
with the measured value, then commit as
"docs: freeze measured hardware-in-loop geometry". Never adjust
geometry after the fact to force a later step to pass.

## 5. First ground pilot

Only once every item in `HIL_SAFETY_CHECKLIST.md`'s
"Before the first ground `EXCLUSIONARY_HIL_PILOT`" section holds.
`HIL_COMM_OFF` first, one run, user watching with Ctrl+C ready. Then
stop, verify clean teardown, and wait for the user's own review of the
recorded data before `HIL_COMM_ON` is even handed to them as a command
to run themselves.
