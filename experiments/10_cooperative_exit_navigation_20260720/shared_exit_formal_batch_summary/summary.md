# Shared-exit communication experiment  formal batch summary

## Verdict

**FINAL_BATCH_PASS  5/5 COMM_OFF and 5/5 COMM_ON trials completed successfully.**

All ten runs ended through the task-completion monitor, both robots entered their assigned parking regions and settled, no collision or visible instability was observed, and no run ended through the maximum-runtime limit.

## Main result

| Metric | COMM_OFF | COMM_ON |
|---|---:|---:|
| Robot B mean completion time | 94.184  3.029 s | 88.184  2.298 s |
| Success rate | 5/5 | 5/5 |
| Collision count | 0 | 0 |

The mean paired reduction in Robot B completion time and overall makespan was **6.000  1.709 s**, corresponding to a mean paired improvement of **6.345%**. Every paired trial showed improvement; individual reductions ranged from 3.440 s to 7.560 s.

## Mechanism verification

All five communication-enabled trials followed the required causal order:

1. Robot A physically entered and discovered the exit;
2. Robot A transmitted the first valid exit announcement;
3. Robot B changed from deterministic search to direct exit navigation.

No premature search-to-goal switch was observed.

## Interpretation

Communication benefited the uninformed searching robot rather than the already-informed robot. It reduced task completion time without reducing task success or observed safety.

## Evidence integrity

The ten native WSL bags and ten diagnostic-log directories were copied after recording had closed. SHA-256 comparison verified all **175/175 files**, with zero mismatches. Raw bags are retained locally and excluded from Git; summaries, manifests and checksums remain suitable for version control.

## Limitation

This is an n=5 paired simulation batch. The result is consistent across all five pairs, but it should be presented descriptively rather than as broad statistical generalisation.
