# Pi verifier deployment checklist (prepared, not executed)

One command at a time, operator-executed. This checklist is only
prepared here -- it has not been run, and nothing on this list starts
a physical process, ROS node, or hardware interaction. It deploys
exactly the reviewed, two-file, hash-pinned unit documented in
`GROUND_DIAGNOSTIC_RUNBOOK.md` step 8:

| File | Reviewed SHA-256 |
|---|---|
| `pi_ground_diagnostic_audit_verifier.py` | `ff9390597cdf27a6e485bb0470bf9fa9a66d53ff8fb3da0bf79c4e88c0972565` |
| `analyze_ground_diagnostic.py` | `faa14871314ab8bcbfa401204679f45173e661573379893b53a086864eb4b73f` |

**Precondition, verbatim from the operator, before step 1:**
```
ROBOT_POWERED_OFF=YES
PI_POWERED_OFF=YES
```
This checklist copies files only -- there is no reason for either the
robot or the Pi to be powered on during file preparation, and doing so
is not authorized by this checklist.

## 1. Compute local (repository-side) hashes to confirm against the table above

`[PowerShell Window 1 -- operator transfer and host checks]`
```bash
sha256sum experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/pi_ground_diagnostic_audit_verifier.py experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/analyze_ground_diagnostic.py
```
Expected: both hashes match the table above exactly. Stop and report
if either does not match -- do not proceed with a mismatched file.

## 2. Check the intended Pi paths do not already exist

`[PowerShell Window 1 -- operator transfer and host checks]` -- opens
one SSH session, password entered manually by the operator:
```bash
ssh pi@<PI_IP> "test ! -e /home/pi/real_robot_avoidance_v1/pi_ground_diagnostic_audit_verifier.py && test ! -e /home/pi/real_robot_avoidance_v1/analyze_ground_diagnostic.py && echo BOTH_PATHS_AVAILABLE"
```
Expected: `BOTH_PATHS_AVAILABLE`. If either path already exists, stop
and report -- **never overwrite an existing file on the Pi.** Choose a
different destination name or resolve the conflict before continuing.

## 3. Copy both files together, in one command

`[PowerShell Window 1 -- operator transfer and host checks]` -- one
explicit, interactive SCP invocation, password entered manually:
```bash
scp experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/pi_ground_diagnostic_audit_verifier.py experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/analyze_ground_diagnostic.py pi@<PI_IP>:/home/pi/real_robot_avoidance_v1/
```
Both files are copied together in the same invocation so the
deployment unit can never be left partially copied.

## 4. Verify both deployed hashes on the Pi

`[PowerShell Window 1 -- operator transfer and host checks]`:
```bash
ssh pi@<PI_IP> "sha256sum /home/pi/real_robot_avoidance_v1/pi_ground_diagnostic_audit_verifier.py /home/pi/real_robot_avoidance_v1/analyze_ground_diagnostic.py"
```
Expected: both hashes match the table above exactly. Stop and report
if either does not match -- treat this as a failed, not a retried,
deployment; do not silently re-copy and re-check.

## 5. Confirm both files are syntactically valid Python -- no ROS process

`[PowerShell Window 1 -- operator transfer and host checks]`:
```bash
ssh pi@<PI_IP> "python3 -m py_compile /home/pi/real_robot_avoidance_v1/pi_ground_diagnostic_audit_verifier.py /home/pi/real_robot_avoidance_v1/analyze_ground_diagnostic.py && echo PY_COMPILE_OK"
```
Expected: `PY_COMPILE_OK`. This only byte-compiles the two files -- it
does not import ROS, does not start the driver, server, bridge, or
guard, and does not run the verifier itself.

## 6. Record the deployed paths and hashes

Fill this in (into the run's own evidence record, e.g. a future
`SUMMARY.md`) once steps 1-5 all pass:

| File | Deployed path | Verified SHA-256 |
|---|---|---|
| `pi_ground_diagnostic_audit_verifier.py` | `/home/pi/real_robot_avoidance_v1/pi_ground_diagnostic_audit_verifier.py` | |
| `analyze_ground_diagnostic.py` | `/home/pi/real_robot_avoidance_v1/analyze_ground_diagnostic.py` | |

## Stop here

Deployment ends at step 6. This checklist does not authorize starting
the driver, server, bridge, guard, or any physical attempt -- that
remains a separate, explicit decision, gated by its own runbook steps.
