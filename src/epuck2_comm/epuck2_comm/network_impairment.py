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
"""

import random
from dataclasses import dataclass


@dataclass(frozen=True)
class ImpairmentConfig:
    delay_s: float = 0.0
    jitter_s: float = 0.0
    drop_probability: float = 0.0
    seed: int = 0


@dataclass(frozen=True)
class RelayDecision:
    forward: bool
    release_delay_s: float  # extra delay applied on top of "now", 0.0 if forwarding immediately or dropped


class ImpairmentDecider:
    """Not thread-safe; intended for single-threaded use from one ROS
    subscription callback, matching how the relay Node uses it."""

    def __init__(self, config: ImpairmentConfig):
        self.config = config
        self._rng = random.Random(config.seed)

    def decide(self) -> RelayDecision:
        if self.config.drop_probability > 0.0 and self._rng.random() < self.config.drop_probability:
            return RelayDecision(forward=False, release_delay_s=0.0)
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
        )
