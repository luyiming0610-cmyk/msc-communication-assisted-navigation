# HIL Offline Stage 3 Runbook

**Classification: `OFFLINE_INTEGRATION_VALIDATION`.** This is a hardware-free,
mixed-node ROS 2 validation of the message chain virtual scout ->
GoalAnnouncement -> adapter-hosted GoalNavigator reception/adoption ->
NavigationIntent -> requested cmd_vel -> guard evaluation -> guarded
(test-only) command -> evidence capture. It is explicitly **not**: a
physical HIL trial, a Webots navigation experiment, a physical
navigation trial, angular calibration, a dual-physical-robot
experiment, formal A-G or N2/N3 data, or evidence that physical
steering is safe. A successful run proves the software chain is wired
and behaves as designed in software only.

## Authoritative Git HEAD requirement

Before starting any Stage 3 run, confirm the working tree's HEAD is the
commit this runbook and its supporting tools were reviewed against
(`10ad3a9768369bb5163d778d095416dd9a3b4362` at the time this runbook was
first written, or a later HEAD that has been independently re-verified
to contain the same, unmodified `hil_virtual_peer.py`,
`goal_navigator.py`, `cooperative_avoider.py`, `hil_cmd_vel_guard.py`,
and `hil_command_evidence_recorder.py`). Do not proceed if any of those
five files differ from what this runbook assumes.

## Isolated topic table

All Stage 3 topics live under `/hil_offline_stage3/...`. None reuse a
production topic name.

| Purpose | Isolated topic | Message type |
|---|---|---|
| Own-robot test state | `/hil_offline_stage3/epuck1/state` | `EpuckState` |
| Virtual peer source state (published by `hil_virtual_peer.py`, unmodified) | `/hil_offline_stage3/virtual_peer/source_state` | `EpuckState` |
| Virtual peer state after the test gate (consumed by the guard) | `/hil_offline_stage3/virtual_peer/guard_input_state` | `EpuckState` |
| GoalAnnouncement | `/hil_offline_stage3/goal_announcement` | `GoalAnnouncement` |
| NavigationIntent | `/hil_offline_stage3/epuck1/nav_intent` | `NavigationIntent` |
| Requested (pre-guard) cmd_vel | `/hil_offline_stage3/cmd_vel_unguarded_test_only` | `Twist` |
| Guarded (post-guard) cmd_vel | `/hil_offline_stage3/cmd_vel_guarded_test_only` | `Twist` |
| Guard arm state | `/hil_offline_stage3/guard_arm_test_only` | `Bool` |
| Bridge-status substitute | `/hil_offline_stage3/bridge_status_test_only` | `std_msgs/String` (JSON `{"connected": bool, "rx_count": int}`) |
| Phase/gate/adoption/duplicate evidence events | `/hil_offline_stage3/phase_event_test_only` | `std_msgs/String` (JSON; keys among `phase`, `gate_open`, `gate_epoch`, `adoption_confirmed`, `duplicate_sent`, `duplicate_rejected`, `guard_blocked_reasons`) -- recorded in the CSV under the fixed pseudo-topic name `PHASE_EVENT`, not the real topic string, so the verifier can find every such event unambiguously regardless of the actual topic name used for a given run |
| Gate-owned structured forward-decision evidence | `/hil_offline_stage3/gate_decision_test_only` | `std_msgs/String` (JSON; one event per source message processed by the gate, emitted synchronously at the gate's own decision point -- keys: `event_type` (always `"GATE_DECISION"`), `gate_epoch`, `gate_state`, `source_protocol_version`, `source_robot_id`, `source_sequence`, `source_production_stamp_s`, `decision` (`"FORWARDED"` or `"REJECTED_GATE_CLOSED"`), `decision_timestamp_s`, `first_source_after_reopen`, `forwarded_destination_topic`) -- recorded in the CSV under the fixed pseudo-topic name `GATE_DECISION_EVENT` |
| Guarded (post-guard) cmd_vel, as observed by the harness's automatic runner for its own zero/bounded phase checks | `/hil_offline_stage3/cmd_vel_guarded_test_only` (same topic as above; the harness subscribes to it read-only, in addition to the guard publishing it) | `Twist` |

## ROS_DOMAIN_ID

**`ROS_DOMAIN_ID=91`** for any real Stage 3 run. Confirmed distinct from:
- `0` (ROS default / any physical process);
- `77` (Stage 2 hardware-free verification domain);
- `89` (the project's standing pytest-isolation domain, `src/epuck2_comm/test/conftest.py`'s `TEST_ROS_DOMAIN_ID`).

During Stage 3 *preparation* (writing/testing the tooling below, not a
real Stage 3 run), focused harness/gate tests use **`ROS_DOMAIN_ID=92`**,
and the limited recorder-verifier integration test (see below) uses
**`ROS_DOMAIN_ID=93`** -- both distinct from 0/77/89/91 and from each
other, so preparation-time test runs can never collide with a real
Stage 3 run's domain, or with each other, even if several existed at
once. The pre-existing Stage 2 adapter/adoption live test
(`test_hil_goal_announcement_adoption.py`) uses its own dedicated
**`ROS_DOMAIN_ID=90`**.

## ROS_LOCALHOST_ONLY

**Every ROS-based test in this directory (new Stage 3 preparation tests
and pre-existing Stage 1/Stage 2 live tests alike) must set
`ROS_LOCALHOST_ONLY=1` alongside its assigned domain.** Each live test's
`setUp()` asserts this explicitly and fails closed if it is unset --
this is not merely a documentation convention, it is enforced in code.
Example invocation form:

```bash
ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_offline_stage3_harness_live.py -v
ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_offline_stage3_recorder_verifier_integration.py -v
ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_hil_goal_announcement_adoption.py -v
```

## Environment setup (per terminal session used for the real run)

```bash
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
for f in /opt/ros/*/setup.bash; do [ -f "$f" ] && source "$f" && break; done
source "$HOME/epuck_ws/install/setup.bash"
```

## Test-only angular bound disclaimer

Any angular-speed bound passed to `hil_cmd_vel_guard.py --max-angular-speed-rps`
for a Stage 3 run is a **`TEST_ONLY_SOFTWARE_BOUND_NOT_A_PHYSICAL_LIMIT`**
(see `hil_offline_stage3_harness.py`'s constant of that name). It must
never be copied into `hil_frozen_params.json` and must never be cited
as a ground-contact angular calibration result.

## Evidence directory convention

One directory per run, named `hil_offline_stage3_<RUN_ID>/`, containing
`evidence.csv` (written by `hil_offline_stage3_evidence_recorder.py`),
`summary.json` (same tool), and `post_run_verification.json` (written
by `hil_offline_stage3_post_run_verifier.py`). No raw evidence from this
directory is ever committed to the repository, matching the existing
project convention for native/raw evidence.

## Exact launch order

Every process is launched in its own terminal/window. Capture its PID
with `$!` (or the shell's direct child-PID mechanism) **immediately**
after launch -- never with `pgrep -f` (reserved only for the pre-run
forbidden-process check and the post-run residual-process check).

| Step | Process | Command (abbreviated; see tool `--help` for full flags) | Readiness condition |
|---|---|---|---|
| 0 | Forbidden-process check | `pgrep -af 'webots-bin\|cooperative_avoider\|state_publisher\|hil_\|ros2 bag record\|piserver\|pi_driver\|epuck_bridge'` | must report nothing |
| 1 | Evidence recorder | `python3 hil_offline_stage3_evidence_recorder.py --own-state-topic ... --output-csv ... --output-summary-json ...` | `HIL_OFFLINE_STAGE3_EVIDENCE_RECORDER_READY` logged |
| 2 | Guard | `python3 hil_cmd_vel_guard.py --upstream-cmd-vel-topic /hil_offline_stage3/cmd_vel_unguarded_test_only --guarded-cmd-vel-topic /hil_offline_stage3/cmd_vel_guarded_test_only --arm-topic /hil_offline_stage3/guard_arm_test_only --physical-state-topic /hil_offline_stage3/epuck1/state --virtual-peer-topic /hil_offline_stage3/virtual_peer/guard_input_state --require-virtual-peer --max-angular-speed-rps <TEST_ONLY_BOUND>` | `HIL_CMD_VEL_GUARD_READY armed=False` logged |
| 3 | Adapter-hosted navigator | `python3 hil_topic_adapter.py` with isolated `--state-topic`, `--nav-intent-topic`, `--goal-announcement-topic` | `goal_navigator READY` logged |
| 4 | `cooperative_avoider` | via its own existing launch mechanism, `state`/`nav_intent`/`cmd_vel` remapped to isolated topics, `-p armed:=true -p enable_dynamic_heading:=true -p enable_dynamic_speed:=true` | its own READY logged |
| 5 | Stage 3 harness | `python3 hil_offline_stage3_harness.py --own-state-topic ... --bridge-status-topic ... --arm-topic ... --goal-announcement-topic ... --virtual-peer-source-topic ... --virtual-peer-guard-input-topic ... --gate-decision-topic ... --nav-intent-topic ... --guarded-cmd-vel-topic ... [--auto-run]` | `HIL_OFFLINE_STAGE3_HARNESS_READY` logged |
| 6 | Virtual peer | `python3 hil_virtual_peer.py` with isolated `--state-topic`/`--announcement-topic`, target position == start position | `HIL_VIRTUAL_PEER_READY` logged |

## Recorder started first, stopped last

The evidence recorder (step 1) is launched before every other process
and shut down after every other process has already exited, so its CSV
captures each other process's own final (zero) output.

## Exact `kill -INT <PID>` shutdown order

Reverse of launch: virtual peer -> harness -> `cooperative_avoider` ->
adapter-hosted navigator -> guard -> recorder. Each `kill -INT` uses the
exact PID captured at that process's own launch in step order above.
No `pkill` at shutdown.

## Forbidden-process and forbidden-topic checks

- **Pre-run**: `pgrep -af` for `webots-bin|cooperative_avoider|state_publisher|hil_|ros2 bag record|piserver|pi_driver|epuck_bridge` must report nothing.
- **Forbidden-topic check**: before step 1, confirm (e.g. `ros2 topic list`) that none of `/cmd_vel`, `/cmd_vel_unguarded`, `/epuck1/state`, `/epuck_bridge/status`, `/hil_guard/arm` already exist on the bus under this domain.
- **Post-run**: `pgrep -af` (same pattern) must report nothing; confirm via `git status`/`git diff` that no repository file changed during the run.

## Abort conditions

Any physical/Pi/bridge/Webots process detected at any point; any
publisher/subscriber on a real production topic; any node reporting
`ROS_DOMAIN_ID` other than `91`; any topic/type mismatch against the
table above; guard output outside the declared test-only bound; more
than one adoption event; missing expected evidence row/log line;
residual process after shutdown; unexpected repository modification.

## Post-run verifier command

```bash
python3 hil_offline_stage3_post_run_verifier.py \
  --csv <evidence.csv> --summary-json <summary.json> \
  --test-only-angular-bound-rps <TEST_ONLY_BOUND> \
  --test-only-linear-bound-mps 0.02 \
  [--residual-process-detected]
```

## Automatic orchestration runner

`hil_offline_stage3_harness.py`'s `Stage3AutomaticRunner` drives the
harness through all 11 `Stage3Phase` values end-to-end automatically,
using only the harness's own observable evidence (adoption confirmation
from the real `NavigationIntent` stream, the guarded-cmd-vel topic's own
zero/bounded values, the gate's own forwarding evidence) -- it contains
no navigation, GoalAnnouncement-acceptance, guard-decision, or
virtual-peer motion logic of its own. It enforces a per-phase timeout
and an overall run timeout (`RunnerTimeoutError`, fail-closed), never
skips or repeats a phase (delegated entirely to the existing
`PhaseMachine`), and never permits any action after `COMPLETE`
(delegated to the existing `DuplicateAnnouncementController`). Invoke it
by passing `--auto-run` (plus `--runner-*` tuning flags) to
`hil_offline_stage3_harness.py`; without `--auto-run`, the harness
behaves exactly as before, waiting for an external caller to drive
`advance_phase()`/`close_gate()`/etc.

## Duplicate-announcement ordering (enforced internally, not left to the caller)

`hil_offline_stage3_harness.py`'s `DuplicateAnnouncementController` enforces,
inside the harness itself: the duplicate `GoalAnnouncement` may only be
published after exactly one adoption rising-edge has been observed on
the `NavigationIntent` stream; a second call after the first successful
publication always fails closed (`DuplicateOrderingError`); a call
before adoption or after the run reaches `COMPLETE` always fails closed;
and a second adoption rising-edge (which the frozen navigation logic's
own idempotent latch should never actually produce) aborts the run
(`AdoptionCountExceededError`). No external orchestration script is
relied upon to get this order right.

## DATA_VALIDITY / TASK_OUTCOME separation

`DATA_VALIDITY` (infrastructure/measurement question, computed first,
independently) covers: file existence/non-emptiness, topic/type
contract, sanctioned `ROS_DOMAIN_ID`, presence of every required
evidence stream (including the gate-decision topic), own-state
`validity_flags==7` throughout, monotonic timestamps, no production
topic in the contract, no residual process, the `[PEER_GATE_CLOSED,
PEER_GATE_REOPENED)` boundary-event proof (both events exist exactly
once, close strictly precedes reopen, the source topic continued
publishing during the interval, at least one gate-input row exists
before closure), and gate-decision-event **structural** well-formedness
(every event has a matched `event_type`/`decision`/`gate_epoch`/finite
`decision_timestamp_s`, every `FORWARDED` event's
`forwarded_destination_topic` matches the real gate-input topic, every
`REJECTED_GATE_CLOSED` event carries none).

`TASK_OUTCOME` (only meaningful when `DATA_VALIDITY=VALID`) is one of a
**non-physical** taxonomy -- `SUCCESS`, `GUARD_BOUND_VIOLATION`,
`STALE_ZERO_FAILURE`, `RECOVERY_FAILURE`, `ADOPTION_FAILURE`,
`DUPLICATE_HANDLING_FAILURE`, `GATE_FORWARDING_FAILURE`,
`BACKLOG_REPLAY_DETECTED`, `NOT_EVALUABLE` -- never a physical-safety
word like "UNSAFE_FAILURE". Every verifier result additionally carries
`result_type="OFFLINE_SOFTWARE_CONTRACT_RESULT"` and an explicit
`physical_claim` disclaimer stating this is not evidence of physical
collision or physical unsafe motion. Checks: exactly one announcement
accepted/adopted; exactly one duplicate successfully sent (not merely
attempted); all guarded commands within the declared test-only bound;
`STALE_ZERO_CONFIRMED` timed strictly after the peer timeout has
elapsed past closure and strictly before reopening; `RECOVERY_CONFIRMED`
timed strictly after the first fresh, gate-decision-proven post-reopen
forward; clean completion within the bounded runtime; and the **strict
first-post-reopen forwarding contract**, proven exclusively from the
gate's own `GATE_DECISION_EVENT` rows (`evaluate_gate_forwarding_outcome()`
in `hil_offline_stage3_post_run_verifier.py`) -- never by comparing this
recorder's rows for the gate-input topic against its rows for the
source topic, since those are two independently-scheduled subscriber
callbacks with no guaranteed relative ordering:
- the events span at least two gate epochs (a reopen actually occurred);
- every event recorded while the gate was `CLOSED` has
  `decision=REJECTED_GATE_CLOSED` (none `FORWARDED`), and at least one
  such event exists (the source kept being processed while closed);
- exactly one event in the final (post-reopen) epoch is marked
  `first_source_after_reopen=true`, and its decision is `FORWARDED`;
- no `FORWARDED` event in the post-reopen epoch carries a
  `source_sequence` less than or equal to the highest `source_sequence`
  already seen while the gate was `CLOSED` -- a violation of this rule
  is reported as its own, more specific `BACKLOG_REPLAY_DETECTED`
  outcome (using the message's own sequence number, never local receipt
  time, which can never distinguish a replayed old message from a fresh
  one); any other contract violation is `GATE_FORWARDING_FAILURE`.

A valid data chain with a failed task outcome (e.g. a guarded command
exceeding the test-only bound, reported as `GUARD_BOUND_VIOLATION`) is a
real, valid failure and must be reported as such -- never hidden,
retried automatically, or reclassified.

## Recorder-verifier integration test (preparation-time only, not the final graph)

`test_hil_offline_stage3_recorder_verifier_integration.py` starts only
the evidence recorder plus plain stimulus publishers (never
`cooperative_avoider`, never the real adapter/virtual-peer/guard graph
together, never a bridge) under topics namespaced
`/hil_offline_stage3_preparation_test/...` and
`ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1`, writes evidence to a
`tempfile.TemporaryDirectory` (deleted automatically), and confirms the
post-run verifier reads the produced files and reports
`DATA_VALIDITY=VALID`/`TASK_OUTCOME=SUCCESS` for a synthetic,
textbook-correct scenario that includes: one original `GoalAnnouncement`
and exactly one adoption event; one duplicate `GoalAnnouncement`
publication and one recorded duplicate-rejection event, correctly
ordered (duplicate-sent strictly after adoption, duplicate-rejected
strictly after duplicate-sent); gate-decision events proving the source
continued to be processed and rejected while closed and the first
post-reopen message was forwarded, with no backlog replay. This proves
the recorder and verifier work together end-to-end; it is not, and must
never be cited as, a real Stage 3 run.

## No automatic progression

A completed Stage 3 run does not authorize, and must never be cited
as, Stage 4, any physical work, angular calibration, or HIL field
geometry population. Each of those requires its own separate,
explicit operator authorization.
