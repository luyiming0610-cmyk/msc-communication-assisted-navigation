# Condition E Trial 05 formal result

- Classification: `FORMAL_SIM`
- Data validity: `VALID`
- Task outcome: `SUCCESS`
- Trial verdict: `PASS`
- Configured independent loss probability: `0.15`
- Observed drop fraction: `0.1765` (epuck1 to epuck2), `0.1613` (epuck2 to epuck1)
- Authoritative relay delivery ratio: `0.8235`, `0.8387`
- Interior sequence gaps: `75`, `70`; duplicates/out-of-order: `0/0` both directions
- Sequence-window boundary censoring: `False`, `False`. Total loss is taken from relay received/forwarded/drop counters, because leading or trailing dropped sequence numbers cannot be inferred from the consumer's first/last received sequence alone.
- Minimum separation: `0.144733788 m`
- Safety margin above 0.14 m: `4.734 mm`
- Trigger mechanism: `PURE_COMMUNICATION_CPA_AVOIDANCE`; all `LOCAL_*` counters zero
- Queue drained: `true`; bag readable: `true`; native-to-Windows SHA-256 match: `14/14`
- Manual observation: `NOT_INDIVIDUALLY_OBSERVED` — Not individually observed; automated continuation authorized after Trial 01 passed all formal gates.

This file is trial-level evidence only until all five Condition E trials are complete and a batch summary is generated.
