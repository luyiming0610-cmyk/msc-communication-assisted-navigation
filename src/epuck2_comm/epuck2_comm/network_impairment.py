"""Pure, deterministic delay/jitter/loss decision logic for the network
impairment relay.

Kept separate from the ROS Node wrapper (network_impairment_relay.py) so
the actual random-number-driven decision can be unit tested without
rclpy. Given a fixed seed, ImpairmentDecider is fully reproducible: the
same seed always produces the same sequence of drop/forward and
delay/jitter decisions, which is required so a trial's impairment
parameters and seed together fully describe what happened -- see the
communication-impairment-matrix instructions requiring the seed to be
written into trial metadata.

Jitter formula (exact, for anyone reading a config and predicting the
release-delay distribution without running the code): given
`jitter_s` (the FULL peak-to-peak spread, not a half-amplitude), each
decision draws `jitter ~ Uniform(-jitter_s/2, +jitter_s/2)` and computes
`release_delay = max(0.0, delay_s + jitter)`. The floor to 0.0 is
applied AFTER adding jitter to delay, on the summed value, never to
delay_s or jitter_s individually. If `delay_s >= jitter_s / 2.0`, the
floor never actually clips anything (the summed range's own minimum is
already >= 0), so the realized distribution is exactly
`Uniform(delay_s - jitter_s/2, delay_s + jitter_s/2)` with no
probability-mass spike at 0.0. If `delay_s < jitter_s / 2.0`, the floor
does clip the lower tail into an atom at exactly 0.0, and the realized
distribution is no longer uniform (see test_network_impairment.py's
`test_large_jitter_is_clamped_to_never_go_negative` for the case this
matters).

Burst/outage extension (v1.1, additive -- see
`experiments/05_objective5_impairment_matrix/objective5_impairment_matrix_design_v1.md`
section 3's "Condition F" for the scientific rationale): `decide()` now
takes an `elapsed_s` argument (seconds since the caller's own reference
start time, e.g. the relay node's construction time, under whatever
clock the caller uses -- this module has no clock of its own). If
`outage_period_s > 0` and `outage_duration_s > 0`, every message whose
`elapsed_s` falls inside a scheduled outage window is dropped
deterministically (no RNG draw for this decision), independent of and
prior to the existing Bernoulli drop check. Outage windows recur every
`outage_period_s` seconds, each `outage_duration_s` seconds long,
starting at `outage_phase_s` (so window k spans
`[outage_phase_s + k*outage_period_s, outage_phase_s + k*outage_period_s + outage_duration_s)`
for k=0,1,2,...). The window test uses
`(elapsed_s - outage_phase_s) % outage_period_s < outage_duration_s`,
which is a PURE function of `elapsed_s` with no internal state -- it is
therefore automatically correct even if `elapsed_s` goes backward
between calls (e.g. a Webots simulation-time reset), unlike a
stateful/accumulating scheme would be; Python's `%` on a positive
divisor always returns a non-negative result, so this holds for
`elapsed_s` values before `outage_phase_s` too (no outage is ever
reported before the first window starts). With `outage_period_s=0.0`
(the default), the outage check is skipped entirely and behavior is
byte-for-byte identical to the pre-extension relay, per message --
confirmed by `test_default_outage_params_are_message_equivalent_to_pre_extension_relay`.
"""

import random
from dataclasses import dataclass


@dataclass(frozen=True)
class ImpairmentConfig:
    delay_s: float = 0.0
    jitter_s: float = 0.0
    drop_probability: float = 0.0
    seed: int = 0
    outage_period_s: float = 0.0
    outage_duration_s: float = 0.0
    outage_phase_s: float = 0.0


@dataclass(frozen=True)
class RelayDecision:
    forward: bool
    release_delay_s: float  # extra delay applied on top of "now", 0.0 if forwarding immediately or dropped
    drop_reason: str = ""  # "" if forwarded, "outage" or "independent" if dropped


class ImpairmentDecider:
    """Not thread-safe; intended for single-threaded use from one ROS
    subscription callback, matching how the relay Node uses it."""

    def __init__(self, config: ImpairmentConfig):
        self.config = config
        self._rng = random.Random(config.seed)

    def _in_outage(self, elapsed_s: float) -> bool:
        return self.outage_status(elapsed_s)["active"]

    def outage_status(self, elapsed_s: float) -> dict:
        """Read-only outage classification at a given elapsed_s -- used
        both by decide() (to decide the current message) and by the
        relay's relay_status topic (to report outage_active/
        current_outage_index independent of any specific message,
        e.g. once per second even when no message arrives that tick).
        `index` is the 0-based ordinal of the outage window elapsed_s
        falls in or has most recently passed (None if elapsed_s is
        before the first window or outage is disabled)."""
        if self.config.outage_period_s <= 0.0 or self.config.outage_duration_s <= 0.0:
            return {"active": False, "index": None}
        if elapsed_s < self.config.outage_phase_s:
            return {"active": False, "index": None}
        since_phase = elapsed_s - self.config.outage_phase_s
        index = int(since_phase // self.config.outage_period_s)
        phase = since_phase % self.config.outage_period_s
        return {"active": phase < self.config.outage_duration_s, "index": index}

    def decide(self, elapsed_s: float = 0.0) -> RelayDecision:
        if self._in_outage(elapsed_s):
            return RelayDecision(forward=False, release_delay_s=0.0, drop_reason="outage")
        if self.config.drop_probability > 0.0 and self._rng.random() < self.config.drop_probability:
            return RelayDecision(forward=False, release_delay_s=0.0, drop_reason="independent")
        jitter = 0.0
        if self.config.jitter_s > 0.0:
            jitter = self._rng.uniform(-self.config.jitter_s / 2.0, self.config.jitter_s / 2.0)
        release_delay = max(0.0, self.config.delay_s + jitter)
        return RelayDecision(forward=True, release_delay_s=release_delay)

    def is_zero_impairment(self) -> bool:
        return (
            self.config.delay_s <= 0.0
            and self.config.jitter_s <= 0.0
            and self.config.drop_probability <= 0.0
            and (self.config.outage_period_s <= 0.0 or self.config.outage_duration_s <= 0.0)
        )

    def max_release_delay_s(self) -> float:
        """The largest possible release_delay_s decide() can ever return
        for this config -- used by the orchestrator's queue-drain rule
        (max_configured_delivery_delay, see the impairment-matrix design
        doc section 5) to know how long to wait after task completion
        before it is safe to assume the relay's pending queue is empty."""
        return self.config.delay_s + max(0.0, self.config.jitter_s / 2.0)
