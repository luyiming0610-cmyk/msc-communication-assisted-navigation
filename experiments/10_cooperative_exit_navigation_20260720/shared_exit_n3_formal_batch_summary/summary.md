# Three-robot shared-exit formal paired batch

## Verdict

`FINAL_BATCH_PASS` for data validity and task completion: all ten counted formal runs completed successfully, and all five OFF/ON pairs are available. The efficiency result is `MIXED_EFFICIENCY_SMALL_POSITIVE_MEAN`, not a uniform improvement claim.

## Main result

Across five paired Webots trials, mean makespan decreased from 111.384 s without communication to 109.700 s with communication. Mean paired saving was 1.684 s, equivalent to a mean paired improvement of 1.561%. Communication improved makespan in three trials and was slightly slower in two trials. The largest saving was 5.440 s and the largest slowdown was 0.780 s.

All five communication-enabled trials showed the required causal event chain: robot A physically entered the exit region, sent the first `GoalAnnouncement`, and robots B and C switched from their independent search routes to the discovered exit. No counted run contained a local-sensor failsafe, collision marker or watchdog termination.

## Interpretation

The result supports a modest average task-level benefit from communication in this scenario, but it does not show that communication improves every individual run. With n=5 paired simulation trials, the analysis is descriptive and should not be generalised broadly. Startup failures and the earlier parking-geometry interaction are retained separately in `N3_ATTEMPT_HISTORY.md` and excluded from the successful formal statistics.

## Evidence

- Raw WSL source: `/home/eamon/epuck_comm_bags`
- Windows immutable copy: `experiments/10_cooperative_exit_navigation_20260720/bags/shared_exit_n3_formal_20260722`
- SHA-256 verification: 175/175 files match; 0 mismatch
- Frozen world SHA-256: `b4721d04064c3546350f0727f7f4ba038fe7bb64dfed8eccb92adcd188e064d4`
