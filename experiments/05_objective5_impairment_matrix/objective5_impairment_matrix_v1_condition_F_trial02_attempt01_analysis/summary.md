# Condition F Trial 02 formal result

- Classification: `FORMAL_SIM`
- Data validity: `VALID`
- Task outcome: `SUCCESS`
- Trial verdict: `PASS`
- Periodic outage: `0.7 s` every `15.0 s`, phase `10.0 s`; no independent random loss
- Relay outage drops: `24` (epuck1 to epuck2), `24` (epuck2 to epuck1)
- Consumer capture ratio: `0.962675`, `0.962963`; duplicates/out-of-order: `0/0` both directions
- Minimum separation: `0.151948579 m`
- Safety margin above 0.14 m: `11.949 mm`
- Stale-state safety behavior: every `SAFE_STOP_STALE` interval recovered; an interval during active communication avoidance was confirmed
- Mechanism classification: `EXPECTED_SAFE_DEGRADATION_WITH_RECOVERY_AND_LOCAL_LAYER_ENGAGEMENT`
- Local safety layer engaged: `true`. This is an observation, not a claim that outage causally triggered the local layer.
- Queue drained: `true`; ROS bag readable: `true`; native-to-Windows SHA-256 match: `14/14`
- Manual observation: `NOT_INDIVIDUALLY_OBSERVED` — Automated sequential continuation was authorized after Trial 01 passed; no separate human observation was supplied for this trial.

Condition F tests safe stopping and recovery during an already-active CPA avoidance manoeuvre. It does not test whether an outage prevents the initial CPA trigger.
