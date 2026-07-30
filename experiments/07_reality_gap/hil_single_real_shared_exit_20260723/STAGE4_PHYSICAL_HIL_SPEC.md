# Stage 4 physical HIL specification -- one real robot + one virtual scout

Status as of this document (2026-07-30, revision 3): **design and
implementation completed offline only, including a live hardware-free
ROS-graph rehearsal with the real committed runtime nodes**. **No
physical run is authorised yet.** The Pi has not been contacted, no
formal physical RUN_ID has been generated, no Webots process was
started, and nothing in this directory has been staged or committed.
This document describes what the committed offline implementation
does; it is not itself a run report, and a rehearsal PASS recorded
anywhere is never a physical PASS (see Section 12).

## 1. Objective (exactly, no more)

Virtual scout enters its virtual exit region -> automatic
GoalAnnouncement -> real `cooperative_avoider` adopts the target ->
guarded bounded straight physical response -> automatic zero and
disarm.

This is explicitly **not**: a full physical exit-navigation trial; an
A-G impairment experiment; an n=5 repeatability batch; a physical
navigation-performance comparison; evidence of physical communication
impairment.

## 2. Frozen parameters (verbatim)

```
field: 1.00 m x 1.40 m
real-robot start: x=0.30 m, y=0.50 m, yaw=0
corridor: y in [0.30, 0.70] m
exit target: x=1.20 m, y=0.50 m
nominal physical response: 0.10 m forward
max linear speed: 0.015 m/s
max motion window (verifier hard bound): 6.67 s
internal supervisor cutoff: 6.50 s
angular.z: exactly 0.0 (tolerance 1e-6 rad/s)
hard max observed travel: 0.15 m
min manual forward displacement: 0.05 m
min boundary clearance: 0.10 m
guard: sole permitted /cmd_vel publisher
```

## 3. Coordinate frame (audited, see below)

Origin: rear-left physical corner of the 1.40 m x 1.00 m field. +x
forward along the 1.40 m length; +y across the 1.00 m width, toward the
operator's right when facing +x; yaw=0 points forward along +x
(`tools/new_field_geometry_params.json`,
`new_field_single_pulse_revalidation/NEW_FIELD_MEASUREMENT_FORM.md`).

Positive `linear.x` is confirmed, by a prior physical run using this
exact field and this exact start pose
(`new_field_single_pulse_revalidation`, RUN_ID `20260728_143937`), to
produce +x (field-forward) motion: start x=0.30 -> measured body-centre
x≈0.40 after a forward pulse. `(0.30,0.50) -> (0.40,0.50)` is that
previously-validated movement.

`angular.z` sign convention (CCW-positive) is confirmed only for
wheel-rotation direction under a suspended-wheel test
(`angular_suspension_diagnostic_20260723/SUMMARY.md`), not for
ground-yaw rotation. This does not block Stage 4: Stage 4 never
commands a nonzero angular.z on purpose, and rejects any nonzero
magnitude regardless of sign.

## 4. Command path

```
cooperative_avoider (-r cmd_vel:=cmd_vel_stage4_raw, -r state:=..., -r nav_intent:=...)
  -> cmd_vel_stage4_raw
  -> hil_stage4_motion_supervisor.py
       - rejects the entire raw Twist (whole-message rejection, never a
         clamp, never a partial forward) if any component is
         non-finite, if angular.x/y/z magnitude exceeds 1e-6, if
         linear.x is negative, or if linear.x exceeds 0.015 m/s -- any
         of these latches FAILED immediately (genuine safety violation)
       - a zero/below-0.001 m/s linear.x sample is NOT a safety
         violation and does NOT latch FAILED by itself -- it is simply
         not armed on, and the supervisor keeps waiting (bounded by
         RAW_COMMAND_TIMEOUT_S=5.0s). This was refined after the live
         rehearsal showed cooperative_avoider's own real control loop
         can legitimately still publish a pre-ramp/pre-intent-update
         zero for one or more ticks immediately after adoption.
       - only a fully-valid, non-zero command is forwarded, verbatim,
         to cmd_vel_unguarded, and only once the supervisor has already
         independently confirmed event -> announcement -> adoption via
         a schema-validated adoption-evidence message (Section 5)
  -> cmd_vel_unguarded
  -> hil_cmd_vel_guard.py (UNMODIFIED, --max-angular-speed-rps 0.0,
     independent second backstop, sole /cmd_vel publisher)
  -> /cmd_vel -> physical bridge -> Pi -> robot
```

`cooperative_avoider.py` subscribes to two more hardcoded relative topic
names beyond `cmd_vel` -- `state` and `nav_intent` (no parameters exist
for either). Both remaps were missing from the first orchestrator draft
and were caught only by the live ROS-graph rehearsal (Section 5a);
without them the node never sees real state or intent and stays at
zero forever, silently, with no error.

## 5. Online adoption gate (audit result)

`hil_goal_announcement_evidence.py`'s existing
`HIL_GOAL_ANNOUNCEMENT_EVIDENCE` line was a **console log line only** --
no machine-readable, subscribable signal existed for "this receiver
adopted this exact goal_id/coordinates" before this implementation.
Treating announcement reception as equivalent to adoption, or scraping
that log line, would have been unsafe.

**Smallest additive fix implemented**: `hil_goal_announcement_evidence.py`
now also publishes one JSON-encoded `std_msgs/String` per received
announcement on `/hil/adoption_evidence`
(`STAGE4_ADOPTION_EVIDENCE_TOPIC`), containing a frozen schema
(`schema_version="1.0.0"`), `goal_id`, `source_robot_id`,
`source_sequence`, `accepted`, `duplicate`, `target_x_m`, `target_y_m`,
`adapter_receive_time_s` (ROS/wall), and `adapter_receive_monotonic_s`
-- the same facts already computed for the log line, added after it,
never replacing it. `goal_navigator.py` is untouched. Proven live (real
rclpy, isolated test-only topics) in
`test_hil_goal_announcement_adoption.py::test_adoption_evidence_message_is_published_machine_readably_on_acceptance`
and `::test_adoption_evidence_message_marks_duplicate_as_not_accepted`.

The topic name itself (`/hil/adoption_evidence`) is a fixed module
constant, not CLI-configurable -- the same name is used in both
production (`run_hil_stage4_trial.sh`) and the rehearsal
(`test_hil_stage4_live_graph_rehearsal.py`), isolated in the latter
case by `ROS_DOMAIN_ID` rather than a private topic namespace.

### 5a. Adoption-evidence fail-closed validation

`hil_stage4_motion_supervisor.py`'s `parse_and_validate_adoption_evidence()`
independently validates every field before any of it is trusted --
malformed JSON, a non-object payload, any missing required field, any
wrong field type (including `accepted="true"` as a string, or a bool
where an int is required), a `schema_version` other than exactly
`"1.0.0"`, a NaN/Inf coordinate, and staleness in either direction
(`|now - adapter_receive_time_s| > 2.0s`) are all rejected with a
specific reason and never treated as adoption. `accepted=false` and
`duplicate=true` are rejected separately even after schema validation
passes. An adoption-evidence topic with a publisher count other than
exactly 1 is treated as a supervisor self-check failure (latches
`FAILED`), not a normal per-message rejection. The very first raw
Twist accepted for arming is additionally required to have arrived
strictly after the adoption-evidence record that unblocked it
(`RAW_COMMAND_NOT_STRICTLY_AFTER_ADOPTION` otherwise) -- enforced
structurally by not even creating the raw-command subscription until
adoption is confirmed, so no pre-adoption backlog can exist.

## 6. Supervisor state machine

`hil_stage4_motion_supervisor.py`, states:

```
PREPARED -> WAITING_FOR_EVENT -> VALIDATING_RAW_COMMAND -> ACTIVE
  -> ZERO_BURST -> DISARMED -> COMPLETE   (terminal, latched)
                            -> FAILED     (terminal, latched, via ABORT_ZERO path)
```

One-shot: `COMPLETE`/`FAILED` permanently refuse any further approval,
release, or raw-command input in the same process. Timeouts:
`EVENT_TIMEOUT_S=30.0`, `ADOPTION_TIMEOUT_S=5.0`,
`RAW_COMMAND_TIMEOUT_S=5.0` -- each latches `FAILED` directly (guard
stays DISARMED throughout, since arming never happened). Internal
cutoff `INTERNAL_ACTIVE_CUTOFF_S=6.50` (checked every 0.05 s, matching
the node's own timer period) always fires before the verifier's hard
`HARD_MAX_NONZERO_DURATION_S=6.67` bound.

## 7. Operator approval and arm authority

Exactly one human action: the operator types
`APPROVED_FOR_SINGLE_HIL_EVENT=YES` at the orchestrator's stop-point
prompt (`run_hil_stage4_trial.sh`, `--run` mode). The orchestrator then
launches the supervisor with a one-time `--operator-approval-token`.
The supervisor is the only process that ever publishes to
`/hil_guard/arm`; no `ros2 topic pub` to the arm topic is ever printed
or expected from the operator (`test_run_hil_stage4_trial_static.py`
asserts this by absence, not merely by convention).

`--run` mode additionally requires an externally-supplied
`EXPECTED_HEAD` and verifies, before creating any output directory,
that the repository's actual `HEAD` matches it and that every
Stage-4-critical file's working-tree content (via `git hash-object`)
matches its committed content at that exact commit (the same technique
already proven in `HIL_OFFLINE_STAGE3_RUNBOOK.md`). Manual physical
bring-up (Pi driver, audited Pi server, WSL bridge, **real
`state_publisher.py`**) must already be complete before this step --
the orchestrator never starts or touches the Pi, and owns everything
from the WSL command-evidence recorder onward (recorder, adapter,
`cooperative_avoider`, guard, supervisor, virtual peer, PID/status/
evidence manifests, exact cleanup).

## 8. Deterministic virtual-scout release

`hil_virtual_peer.py` has no internal pause/release flag -- it moves
toward its target as soon as the process starts. Release is therefore
enforced by **when the orchestrator spawns it**: only after readiness,
zero-state, and publisher-count gates all pass and the supervisor has
accepted the approval token (state `WAITING_FOR_EVENT`) does the
orchestrator publish `/hil_stage4/virtual_scout_released=true` and spawn
`hil_virtual_peer.py`, exactly once. No Stage 3 harness (synthetic
state injection, scripted duplicate announcement) is used anywhere in
Stage 4.

## 9. Topic ownership

| Topic | Expected publishers |
|---|---|
| `/epuck1/state` | 1 (real `state_publisher.py`) |
| `/epuck_virtual_peer/state` | 1 (`hil_virtual_peer.py`, post-release) |
| `/hil/goal_announcement` | 1 message total |
| `cmd_vel_stage4_raw` | 1 (`cooperative_avoider`) |
| `cmd_vel_unguarded` | 1 (the supervisor) |
| `/cmd_vel` | 1 (the guard) |
| `/hil_guard/arm` | 1 (the supervisor; no manual publisher) |

Pre- and post-start publisher-count checks are in
`run_hil_stage4_trial.sh` `--run` mode.

## 10. Crash semantics

If the supervisor process dies during `ACTIVE`, its `cmd_vel_unguarded`
publisher disappears. `hil_cmd_vel_guard.py`'s existing, unmodified
`decide_command()` already treats `upstream_cmd_vel_publisher_count !=
1` as an independently-sufficient block, forcing `linear=0.0,
angular=0.0, armed_effective=False` regardless of any other input --
proven in
`test_hil_stage4_motion_supervisor.py::GuardZeroesAfterSupervisorDeathTest`.
A supervisor crash mid-trial is classified `FAIL_VALID_EVIDENCE`; no
automatic restart occurs.

## 11. Evidence and binding verifier

Supervisor JSONL (`stage4_supervisor_evidence.jsonl`): one record per
raw Twist (pre-filter), one per state transition, one per
approval/release/adoption/arm/active/deadline/zero/disarm/timeout
event. Every record carries `monotonic_time_s`, `ros_time_s`, `state`,
`event`, `reason`, `raw`, `run_id`, `goal_id`. Combined with the
existing WSL command-evidence recorder and Pi command-audit JSONL
(unchanged, reused), the orchestrator's `pid_manifest.json`, and a
`residual_check.json` written by the orchestrator's own cleanup trap.

`hil_stage4_post_run_verifier.py` consumes only explicit file paths
(never scans a directory) and produces exactly one of `PASS`,
`FAIL_VALID_EVIDENCE`, `INVALID_EVIDENCE`, plus a JSON report. It
checks evidence schema/ordering, the full causal chain (exactly one
release/adoption/arm/window, adoption before any motion, no prohibited
raw component during `ACTIVE`), timing against both
`INTERNAL_ACTIVE_CUTOFF_S` and `HARD_MAX_NONZERO_DURATION_S`, the
terminal-state/disarm tail sequence, hash/PID-manifest/residual-process
evidence when paths are supplied, and (physical mode only) explicit
operator measurements. Missing evidence is always `INVALID_EVIDENCE`,
never defaulted to zero/false/success.

## 12. Physical vs. rehearsal outcome thresholds -- never conflated

`hil_stage4_post_run_verifier.py --mode rehearsal` PASS means only:
software-contract PASS on hardware-free evidence. It is never usable as
physical evidence.

`--mode physical` PASS additionally requires an explicit, operator-
supplied measurements file with manually measured body-centre forward
displacement in `[0.05, 0.15]` m (recorded separately from odometry,
never averaged or substituted), `corridor_crossed=false`,
`stop_line_crossed=false`, `min_boundary_clearance_m > 0.10`, and no
unexpected rotation/direction/sound/acceleration/interruption -- on top
of every rehearsal-mode check (exactly one announcement, no duplicate,
adoption before motion, all raw angular within tolerance, exactly one
motion window, guarded speed <=0.015 m/s, active duration <=6.67 s,
final zero, supervisor+guard disarmed, recorder stopped last, evidence
integrity PASS). A rehearsal PASS is structurally incapable of becoming
a physical PASS: physical mode with no measurements file returns
`INVALID_EVIDENCE`, never a silent pass-through
(`test_hil_stage4_post_run_verifier.py::PhysicalModePassTest::test_rehearsal_pass_is_not_physical_pass`).

## 13. Live hardware-free ROS-graph rehearsal result

`test_hil_stage4_live_graph_rehearsal.py` runs the real, unmodified
`hil_virtual_peer.py`, the real adoption-evidence path, the real
installed `cooperative_avoider` executable, the real
`hil_stage4_motion_supervisor.py`, and the real `hil_cmd_vel_guard.py`
as actual OS subprocesses on a fixed, reserved, isolated
`ROS_DOMAIN_ID=93` with every physical-output topic remapped into a
private `/pytest_stage4_live/...` namespace (except the fixed
`/hil/adoption_evidence` name, isolated by domain instead). Only the
real robot's physical state is synthetic
(`synthetic_stage4_physical_state_publisher.py`, reusing Stage 3's own
already-committed clear-sensor fixture). 5/5 scenarios pass: complete
successful sequence (COMPLETE, full ARM->ACTIVE->ZERO_BURST->DISARM
tail); supervisor killed mid-`ACTIVE` (the existing, unmodified guard's
own publisher-count check reaches zero independently); nonzero-angular
raw command end-to-end rejection; missing-adoption-evidence timeout;
duplicate/wrong-goal adoption-evidence rejection.

This rehearsal caught two real bugs later fixed in
`run_hil_stage4_trial.sh` itself (missing `-r state:=...` and
`-r nav_intent:=...` remaps for `cooperative_avoider.py`, both
hardcoded relative topic names with no parameter equivalent -- without
them the node would silently stay at zero forever on a real physical
run too) and led to the zero-command-tolerance refinement in Section 4.

## 14. Files (this implementation)

New: `tools/hil_stage4_motion_supervisor.py`,
`tools/test_hil_stage4_motion_supervisor.py`,
`tools/run_hil_stage4_trial.sh`,
`tools/test_run_hil_stage4_trial_static.py`,
`tools/synthetic_stage4_physical_state_publisher.py`,
`tools/test_hil_stage4_live_graph_rehearsal.py`,
`tools/hil_stage4_post_run_verifier.py`,
`tools/test_hil_stage4_post_run_verifier.py`, this spec, and
`STAGE4_COMMAND_SHEET_TEMPLATE.md`.

Modified (additive only, reasons stated inline and in each file's own
docstrings): `tools/hil_goal_announcement_evidence.py` (adoption
evidence publisher + schema version), `tools/test_hil_goal_announcement_adoption.py`
(tests registering that publisher), `tools/run_isolated_test_suite.sh`
(registers the five new safety-critical test modules above so a
deletion or failed collection is a hard runner failure).

Unmodified: `goal_navigator.py`, `hil_virtual_peer.py`,
`hil_topic_adapter.py`, `cooperative_avoider.py`,
`hil_cmd_vel_guard.py`.
