# physical_single_device_zero_impairment_baseline_v1 — design only, NOT executed

Status: design proposal, submitted for confirmation. Nothing in this
document has been run. `Trial 01` has not started. Robot wheels remain
suspended; no `/cmd_vel` has been sent at any point.

## What this is / is not

Formal-intent (n≥5) zero-impairment physical baseline layered on top of
`physical_expanded_bridge_epuckstate_integration_pilot01_attempt01`
(`PASS_WITH_LIMITATION`, commit `f85df97`) and the recorder fix (commit
`8b9871e`). Uses the expanded bridge + expanded server + current
`epuck2_comm_interfaces/msg/EpuckState` protocol throughout. **Still
stationary, still no controller, still no ground motion.** This design
does not itself decide whether 5 trials of this shape are sufficient to
call it "the" formal physical Objective 5 baseline -- that is a
downstream decision after the data exists.

## Trial independence policy (the part explicitly required to not be faked)

Two process tiers, handled differently, both recorded explicitly per
trial in that trial's `runtime_manifest.json`:

- **Long-running infrastructure** (Pi driver, Pi expanded server
  `pi_epuck_tcp_server_sensors.py`, WSL expanded bridge
  `wsl_epuck_tcp_bridge_sensors.py`): proposed to stay running
  continuously across all 5 trials, rather than being restarted each
  time. Restarting these 5x each introduces real reconnection risk and
  manual overhead for marginal benefit, since their own health
  (`connected`, `crc_errors`) is independently re-verified at every
  trial's pre-flight check regardless of whether the process itself
  is fresh.
- **`state_publisher`**: proposed to be **restarted fresh for each
  trial**. This gives each trial its own `EpuckState.sequence` starting
  near 0 (clean tier-B boundaries, not a continuation of the previous
  trial's counter) and is cheap/low-risk to restart (pure software,
  no TCP reconnection involved).
- **Tier A implication**: because the bridge's own cumulative counters
  (`state_seq_first`/`state_seq_last`/`state_unique_received`/
  `state_missing`/`state_out_of_order`) are NOT reset by a
  `state_publisher` restart (they belong to the long-running bridge
  process), each trial's tier-A `APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO`
  must be computed as a **delta between the bridge's own counter values
  at trial-start vs. trial-end** (same technique already used for
  `crc_errors_delta` in both prior pilots), never from the bridge's
  all-time first/last values. This is a deliberate, explicit design
  choice -- not an oversight -- and must be stated in every trial's
  `summary.md` so a reader never mistakes a delta for an absolute count.
- Each trial's `runtime_manifest.json` explicitly states, per process,
  whether it was **REUSED** (already running, not touched) or **FRESH**
  (started for this trial) -- this is the mechanism that prevents "one
  continuous 1500s session disguised as 5 trials": a reviewer can check
  that `state_publisher`'s PID differs across all 5 trials and that its
  own `EpuckState.sequence` genuinely restarts near 0 each time, while
  the bridge's tier-A counters are visibly continuous (by design, not
  by accident) and analyzed via deltas accordingly.

## Fixed per-trial structure (same shape as attempt01)

- 300s total: 30s warmup / 240s main / 30s tail.
- Native WSL ext4 bag path
  (`/home/eamon/epuck_comm_bags/physical_single_device_zero_impairment_baseline_v1_trial0N`),
  copied to the Windows tree and SHA-256-verified only after the
  recorder/bag/sampler have cleanly stopped -- never written to
  `/mnt/c` directly.
- Unique directory, unique CSV filenames, unique `runtime_manifest.json`
  per trial (`trial01` .. `trial05`) -- no attempt directory or filename
  is ever reused or overwritten, matching the discipline already
  established this session.
- Wheels suspended throughout. No controller. WSL-side `/cmd_vel`
  publisher count checked at pre-flight (must be 0) and at
  start/mid/end checkpoints (must stay 0) exactly as in attempt01. Pi
  side: only the 20Hz zero-velocity watchdog is expected; any nonzero
  value observed anywhere (bag or live) is an immediate trial FAIL and
  a full stop, not a retry-in-place.
- Uses the **fixed** `wsl_expanded_pilot_recorder.py` (commit
  `8b9871e`) -- the double-shutdown bug from `attempt01` should not
  recur, and if it somehow does, the trial is flagged for investigation
  rather than the exception being pre-emptively dismissed as harmless.

## Metrics recorded per trial (all required items, mapped to how each is computed)

| metric | source | note |
|---|---|---|
| `APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO` | bridge status, trial-start vs. trial-end delta | tier A |
| Pi sequence `missing` / `out_of_order` | same delta | tier A |
| `duplicate_count` | **NOT_MEASURABLE** | stays NOT_MEASURABLE unless a separate, independently tested change to the bridge's own duplicate tracking is implemented first -- not attempted as part of this baseline |
| EpuckState bag `capture_ratio` | bag `/epuck1/state` sequence, main window | tier B, fresh per trial since `state_publisher` restarts |
| sensor topic rate / max gap / total stall | bag `/odom /scan /tof /ps0-7`, main window | tier C, no PDR claimed |
| RTT snapshot distribution + tail (`>50/100/200ms`, longest run) | bridge status main window | explicitly a 1Hz snapshot, not a transaction census, same as both prior pilots |
| `state_age_s` | same formula as documented in both prior pilots (single WSL clock domain) | VALID |
| `validity_flags` duration breakdown | bag `/epuck1/state`, main window | |
| NaN / protocol-allowed-Inf / unexpected-Inf accounting | bag `/epuck1/state`, main window | |
| CPU / RAM / Wi-Fi (Pi + WSL) | 1Hz local samplers, main window | |
| nonzero `/cmd_vel` | bag (0 messages expected) + 3 live checkpoints | any nonzero anywhere = trial FAIL |
| recorder exit integrity | subprocess exit code + stderr traceback check + file-completeness check | now backed by the fix and its own test suite (commit `8b9871e`) |
| one-way Pi→WSL latency | **NOT COMPUTED** | no clock-sync procedure exists; stays out of scope for this baseline entirely |

## RTT tail repeatability (explicitly not decided yet)

`attempt01`'s RTT snapshots showed a bimodal-looking pattern: median
8.4ms but 27.1% of samples > 100ms. **No cause is attributed here.**
The n=5 batch's analysis must report, across all 5 trials: whether the
`>50ms`/`>100ms`/`>200ms` percentages and the longest-high-RTT-run length
are similar trial-to-trial (suggesting a repeatable structural pattern)
or vary widely (suggesting a one-off or environment-dependent effect) --
and report that comparison as a finding, not as a diagnosis of why it
happens.

## Trial 01 — manual steps required

Given the "long-running infrastructure reused, `state_publisher` fresh"
policy above, Trial 01 needs:
1. Confirm infrastructure still healthy (pre-flight check, same as
   `attempt01` -- I can do this myself, no user action).
2. **User action**: start a fresh Pi-side system sampler with a
   trial-unique filename (same pattern as before: `nohup python3
   ~/pi_system_sampler.py ~/physical_pilot_pi_metrics_baseline_v1_trial01.csv
   1.0 ...`), report the PID.
3. I stop the existing `state_publisher` (PID 2813/2834, targeted,
   confirmed) and start a fresh one for this trial.
4. I run the 300s orchestration (adapted from
   `run_expanded_bridge_epuckstate_pilot.sh`, new bag/CSV names).
5. **User action**: stop the Pi sampler (targeted PID), `scp` the CSV
   into the trial's native WSL diagnostic directory (never `/mnt/c`,
   never via me directly -- I don't have and must never receive Pi/
   hotspot credentials).
6. I verify SHA-256 both sides, compute the real overlap window, run
   the analyzer, generate the four required artifacts, copy to Windows,
   verify SHA-256 again, commit.

## Trials 02-05 — can they run without the user, without reducing safety?

**Partially, with one deliberate manual checkpoint kept in each trial
specifically because it is the credential boundary, not a safety
shortcut:**

- Steps that **can** be fully automated by me alone, safely, because
  they are the same pre-verified, no-motion, no-controller envelope
  already proven in `attempt01` and (if it passes) Trial 01: restarting
  `state_publisher`, running the 300s orchestration script, all
  pre-flight/live `/cmd_vel` checks, bag closure/verification, SHA-256
  manifest, analysis, and Windows copy.
- Steps that **cannot** be automated by me, ever, in this project: the
  Pi-side system sampler start/stop and its `scp` retrieval, because
  both require the Pi's own credential, which I do not have and must
  never be given (matches the standing constraint at the top of this
  whole session). This is a hard boundary, not a convenience decision.
- **Proposed middle ground, for your decision**: instead of a fresh
  Pi-sampler start/stop per trial (5x manual round-trips), the user
  could start ONE long-running Pi-side sampler ONCE before Trial 01,
  writing a single continuous CSV across all 5 trials (with the trial
  boundaries recovered afterward from timestamps, same as the
  trial-delta technique already used for tier A) -- reducing the
  Pi-side manual touchpoints from 10 (5x start + 5x stop) to 2 (1x
  start + 1x stop), with one final `scp`+SHA-256 step instead of 5. If
  you prefer fully independent per-trial Pi CSVs instead (more
  isolation, more manual steps), say so and I will keep the per-trial
  design instead.
- Either way, **I will not start Trial 02 automatically after Trial 01
  finishes** -- each trial's result is reported before the next one
  begins, matching the standing "do not auto-run the next experiment"
  instruction, unless you explicitly authorize a run-all-5 sequence.

## Execution attempts (append-only, never overwritten/reused)

| attempt | trial slot | orchestrator | status | formal_eligible | reason |
|---|---|---|---|---|---|
| `trial01_attempt01_short_window` | trial01 | `run_baseline_v1_trial.sh` (v1, now DEPRECATED) | `EXCLUDED_SHORT_WINDOW` | `false` | true 3-source (bag/status_csv/system_csv) overlap 295.432s < required 300.000s (shortfall 4.568s). See `window_audit_status_final.json` in the attempt's diag directory. |
| `trial02_attempt01_short_window` | trial02 | `run_baseline_v1_trial.sh` (v1, now DEPRECATED) | `EXCLUDED_SHORT_WINDOW` | `false` | same systematic defect, confirmed independently: overlap 294.345s < required 300.000s (shortfall 5.655s). See `window_audit_status_final.json` in the attempt's diag directory. |

Both attempts' data (rosbag, CSVs, logs) is preserved in full under
`/home/eamon/epuck_comm_bags/`, never deleted or overwritten, and is
explicitly `formal_eligible: false` / not counted toward the formal n=5
batch. Directory names carry the `_short_window` marker so no later
summarization step can mistake them for eligible trials. Root cause and
fix: see commit `a7f2a7e` (`run_baseline_v1_trial_v2.sh` + `window_calc.py`
+ `wait_for_ready.py`). `run_baseline_v1_trial.sh` is hard-disabled (exits
immediately with `DEPRECATED_SHORT_WINDOW_ORCHESTRATOR`, code 3) and must
never be used again; only `run_baseline_v1_trial_v2.sh` is used for any
subsequent attempt, starting with `trial01_attempt02`.

## Safety teardown available on request

If you need to step away or pause for more than ~10 minutes at any
point, ask and I will give a single, targeted, non-`pkill` shutdown
sequence for whatever is running at that moment (currently: Pi
expanded server PID 1168, WSL expanded bridge PID 2535, `state_publisher`
PID 2813/2834) -- not executed unless you ask for it.
