# Condition F Trial 01 formal result

- Classification: `FORMAL_SIM`
- Data validity: `VALID`
- Task outcome: `SUCCESS`
- Trial verdict: `PASS`
- Periodic outage: `0.7 s` every `15.0 s`, phase `10.0 s`; no independent random loss
- Relay outage drops: `24` (epuck1 to epuck2), `26` (epuck2 to epuck1)
- Consumer capture ratio: `0.962500`, `0.959815`; duplicates/out-of-order: `0/0` both directions
- Minimum separation: `0.143597951 m`
- Safety margin above 0.14 m: `3.598 mm`
- Stale-state safety behavior: every `SAFE_STOP_STALE` interval recovered; an interval during active communication avoidance was confirmed
- Mechanism classification: `EXPECTED_SAFE_DEGRADATION_WITH_RECOVERY_AND_LOCAL_LAYER_ENGAGEMENT`
- Local safety layer engaged: `true`. This is an observation, not a claim that outage causally triggered the local layer.
- Queue drained: `true`; ROS bag readable: `true`; native-to-Windows SHA-256 match: `14/14`
- Manual observation: `CONFIRMED` — User observed successful avoidance with no visible anomaly. A sub-second pause during avoidance was noticed; offline timestamps identify it as synchronized SAFE_STOP_STALE behavior caused by the configured outage, not evidence of an unexplained PC freeze.

Condition F tests safe stopping and recovery during an already-active CPA avoidance manoeuvre. It does not test whether an outage prevents the initial CPA trigger.
