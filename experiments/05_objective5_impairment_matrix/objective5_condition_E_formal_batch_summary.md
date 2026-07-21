# Objective 5 Condition E formal batch

**Final verdict: FINAL_BATCH_PASS (5/5).** Condition E applied independent Bernoulli message loss with `drop_probability=0.15`, zero delay, zero jitter, and no outage. All five trials completed the two-robot task, retained positive separation above the frozen 0.14 m safety radius, and triggered pure communication-based CPA avoidance without local-layer engagement.

| Trial | Verdict | Drop 1→2 | Drop 2→1 | Min distance (m) | Margin (mm) | Observation |
|---:|---|---:|---:|---:|---:|---|
| 01 | PASS | 14.78% | 17.05% | 0.146627 | 6.627 | CONFIRMED |
| 02 | PASS | 14.70% | 16.19% | 0.147683 | 7.683 | NOT_INDIVIDUALLY_OBSERVED |
| 03 | PASS | 12.92% | 11.03% | 0.149539 | 9.539 | NOT_INDIVIDUALLY_OBSERVED |
| 04 | PASS | 15.78% | 13.67% | 0.147700 | 7.700 | NOT_INDIVIDUALLY_OBSERVED |
| 05 | PASS | 17.65% | 16.13% | 0.144734 | 4.734 | NOT_INDIVIDUALLY_OBSERVED |

## Aggregate result

- Direction 1→2 drop fraction: mean 0.151646, sample SD 0.017282; pooled 327/2156 = 0.151670.
- Direction 2→1 drop fraction: mean 0.148138, sample SD 0.024601; pooled 327/2205 = 0.148299.
- Minimum separation across the batch: 0.144734 m (Trial 05), leaving 4.734 mm above the frozen threshold.
- Raw evidence: 70/70 copied files matched their native WSL sources by SHA-256; 0 mismatch.
- Trial 01 was manually observed and confirmed; Trials 02–05 were authorized automated continuations.

## Measurement interpretation

Total loss is measured from the impairment relay's received, forwarded, and independent-drop counters. Sequence-counter gaps cover only the interior of the consumer-visible first-to-last sequence window; leading or trailing dropped messages are boundary-censored. Therefore sequence-gap counts are retained as diagnostic evidence but are not used as the authoritative total-loss metric.

This is an n=5 Webots batch with fixed random seeds. The result is descriptive, not a claim of broad statistical generalisation.
