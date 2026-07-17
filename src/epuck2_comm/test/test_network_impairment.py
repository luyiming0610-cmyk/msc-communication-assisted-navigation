from epuck2_comm.network_impairment import ImpairmentConfig, ImpairmentDecider


def test_zero_impairment_always_forwards_with_no_delay():
    decider = ImpairmentDecider(ImpairmentConfig())
    assert decider.is_zero_impairment()
    for _ in range(100):
        decision = decider.decide()
        assert decision.forward is True
        assert decision.release_delay_s == 0.0


def test_drop_probability_one_always_drops():
    decider = ImpairmentDecider(ImpairmentConfig(drop_probability=1.0, seed=1))
    for _ in range(50):
        decision = decider.decide()
        assert decision.forward is False


def test_drop_probability_zero_never_drops_regardless_of_seed():
    for seed in range(5):
        decider = ImpairmentDecider(ImpairmentConfig(drop_probability=0.0, seed=seed))
        for _ in range(50):
            assert decider.decide().forward is True


def test_fixed_delay_with_no_jitter_is_exact():
    decider = ImpairmentDecider(ImpairmentConfig(delay_s=0.25, seed=3))
    for _ in range(20):
        decision = decider.decide()
        assert decision.forward is True
        assert decision.release_delay_s == 0.25


def test_jitter_stays_within_symmetric_band_and_never_negative():
    decider = ImpairmentDecider(ImpairmentConfig(delay_s=0.1, jitter_s=0.05, seed=7))
    for _ in range(500):
        decision = decider.decide()
        assert decision.forward is True
        assert 0.0 <= decision.release_delay_s <= 0.1 + 0.025 + 1e-9
        assert decision.release_delay_s >= 0.1 - 0.025 - 1e-9


def test_large_jitter_is_clamped_to_never_go_negative():
    # delay_s smaller than jitter_s/2 could mathematically request a
    # negative release delay -- must clamp to 0.0, never go back in time.
    decider = ImpairmentDecider(ImpairmentConfig(delay_s=0.01, jitter_s=0.5, seed=11))
    for _ in range(500):
        decision = decider.decide()
        if decision.forward:
            assert decision.release_delay_s >= 0.0


def test_same_seed_reproduces_the_identical_decision_sequence():
    config = ImpairmentConfig(delay_s=0.1, jitter_s=0.05, drop_probability=0.3, seed=42)
    decider_a = ImpairmentDecider(config)
    decider_b = ImpairmentDecider(config)
    sequence_a = [decider_a.decide() for _ in range(200)]
    sequence_b = [decider_b.decide() for _ in range(200)]
    assert sequence_a == sequence_b


def test_different_seeds_produce_different_sequences():
    base = ImpairmentConfig(delay_s=0.1, jitter_s=0.05, drop_probability=0.3, seed=1)
    other = ImpairmentConfig(delay_s=0.1, jitter_s=0.05, drop_probability=0.3, seed=2)
    sequence_a = [ImpairmentDecider(base).decide() for _ in range(50)]
    sequence_b = [ImpairmentDecider(other).decide() for _ in range(50)]
    assert sequence_a != sequence_b


def test_drop_probability_roughly_matches_observed_rate_over_many_samples():
    decider = ImpairmentDecider(ImpairmentConfig(drop_probability=0.5, seed=99))
    outcomes = [decider.decide().forward for _ in range(20000)]
    drop_rate = 1.0 - (sum(outcomes) / len(outcomes))
    assert abs(drop_rate - 0.5) < 0.02
