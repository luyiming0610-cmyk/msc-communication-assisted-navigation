# Objective 5 Condition F formal batch summary

## Verdict

`FINAL_BATCH_PASS`: 5/5 formal Webots trials produced valid data and successful two-robot task completion under periodic bidirectional outage.

## Frozen manipulation and scope

- Outage: 0.7 s every 15.0 s, phase 10.0 s; no fixed delay, jitter, or independent random loss.
- Execution commit: `4fc4c516ec4dedd52e62ee570b7660328aa6bf2e`.
- Intended claim: stale-state safe stopping and recovery during an already-active CPA avoidance manoeuvre.
- Not claimed: impairment of the initial CPA trigger.

## Results

- All stale-state safe stops recovered; total relay-authoritative outage drops: `260`.
- Mean consumer capture ratio: `0.957420` (epuck1 to epuck2), `0.957077` (epuck2 to epuck1).
- Duplicate/out-of-order messages: 0/0 in both directions for all trials.
- Minimum-separation mean: `0.148096 m`; minimum across the batch: `0.143598 m`.
- Safety-margin mean: `8.096 mm`; tightest: `3.598 mm`.
- Local safety layer engaged in Trials `[1, 2, 4, 5]` and did not engage in Trial `[3]`. This is reported descriptively; no causal attribution to outage is made.
- Native-to-Windows raw evidence: `70/70` files SHA-256 matched; all five bags were readable with `ros2 bag info`; post-batch process check was clean.

## Interpretation

The intended outage caused repeatable `SAFE_STOP_STALE` behavior. Each robot stopped safely when peer state became stale and resumed after communication returned. All five tasks completed above the frozen 0.14 m safety radius. The trial therefore demonstrates safe degradation and recovery under the registered burst-outage schedule, with the local sensing layer available as an additional fallback in four trials.

## Limitations

This is an n=5 simulation result under one fixed outage schedule. It should not be generalized to all network outages, and it does not replace dual-physical-robot testing.
