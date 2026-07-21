# Condition E Trial 01 formal result

- Classification: `FORMAL_SIM`
- Data validity: `VALID`
- Task outcome: `SUCCESS`
- Trial verdict: `PASS`
- Configured independent loss probability: `0.15`
- Observed drop fraction: `0.1478` (epuck1 to epuck2), `0.1705` (epuck2 to epuck1)
- Authoritative relay delivery ratio: `0.8522`, `0.8295`
- Interior sequence gaps: `64`, `75`; duplicates/out-of-order: `0/0` both directions
- Sequence-window boundary censoring: `False`, `False`. Total loss is taken from relay received/forwarded/drop counters, because leading or trailing dropped sequence numbers cannot be inferred from the consumer's first/last received sequence alone.
- Minimum separation: `0.146626503 m`
- Safety margin above 0.14 m: `6.627 mm`
- Trigger mechanism: `PURE_COMMUNICATION_CPA_AVOIDANCE`; all `LOCAL_*` counters zero
- Queue drained: `true`; bag readable: `true`; native-to-Windows SHA-256 match: `14/14`
- Manual observation: `CONFIRMED` — User observed successful avoidance with no collision, oscillation, stalling, or other anomaly; both robots completed and stopped. A sub-second startup offset was visually noted.

This file is trial-level evidence only until all five Condition E trials are complete and a batch summary is generated.
