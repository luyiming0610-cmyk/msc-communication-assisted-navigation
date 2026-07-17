# Local static obstacle trial 01 — diagnostic summary

Date: 2026-07-16

## Configuration

- Webots two-robot namespaced world with a 0.06 m wooden obstacle approximately 0.125 m in front of epuck1.
- epuck1 armed at 0.010 m/s maximum nominal speed; epuck2 remained stationary while publishing its state.
- Local IR/ToF layer enabled and peer-state CPA layer enabled.
- Eight-second controller runtime with immediate zero command at completion.

## Safety result

- epuck1 turned to its own right and did not contact the wooden obstacle.
- epuck2 remained stationary.
- epuck1 stopped completely at the runtime limit.
- The trial therefore passed collision avoidance and terminal-stop checks.

## Observed control defect

Visible left-right oscillation occurred during avoidance. The log shows the front
range alternating as the narrow ToF beam moved across the box:

- `0.127 m -> 1.088 m -> 0.113 m -> 1.063 m -> 0.099 m -> 1.043 m`.

The controller consequently alternated between:

- `LOCAL_FRONT_WARN` or `LOCAL_FRONT_DANGER`, which commanded a right turn; and
- `CRUISE`, whose heading correction immediately commanded a left turn.

This is a perception/control boundary effect rather than packet loss or CPA
instability: peer distance remained approximately 0.67–0.70 m and the CPA risk
condition was not active.

## Corrective action

Trial 01 is retained as diagnostic evidence and excluded from final performance
statistics. The controller was revised to use:

1. a one-second local direction latch through brief range dropouts;
2. a `LOCAL_CLEARANCE` phase that preserves the avoidance direction;
3. a distance-based `LOCAL_BYPASS` phase before heading recovery;
4. a rate-limited `LOCAL_RECOVER` phase.

The next trial must demonstrate no repeated opposite-sign turn commands while
maintaining no collision, a stationary epuck2, and a complete terminal stop.
