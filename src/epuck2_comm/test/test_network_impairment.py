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


# --- v1.1 burst/outage extension ---


def test_default_outage_params_are_message_equivalent_to_pre_extension_relay():
    """outage_period_s/outage_duration_s default to 0.0 -- decide() must
    behave identically to the pre-extension decider for every elapsed_s,
    message for message, at a fixed seed."""
    config = ImpairmentConfig(delay_s=0.1, jitter_s=0.05, drop_probability=0.3, seed=42)
    decider = ImpairmentDecider(config)
    for elapsed in (0.0, 1.0, 100.0, 12345.6):
        decision = decider.decide(elapsed)
        assert decision.drop_reason in ("", "bernoulli")  # never "outage" with outage disabled


def test_outage_window_boundaries_are_closed_open():
    # outage window [5.0, 5.7) given phase=5.0, duration=0.7
    config = ImpairmentConfig(outage_period_s=15.0, outage_duration_s=0.7, outage_phase_s=5.0, seed=1)
    decider = ImpairmentDecider(config)
    assert decider.decide(4.999).forward is True  # just before window
    assert decider.decide(5.0).forward is False  # exact start, inclusive
    assert decider.decide(5.0).drop_reason == "outage"
    assert decider.decide(5.699).forward is False  # just inside end
    assert decider.decide(5.7).forward is True  # exact end, exclusive
    assert decider.decide(5.701).forward is True  # just after window


def test_outage_recurs_every_period():
    config = ImpairmentConfig(outage_period_s=15.0, outage_duration_s=0.7, outage_phase_s=5.0, seed=1)
    decider = ImpairmentDecider(config)
    # second window: [20.0, 20.7); third: [35.0, 35.7)
    assert decider.decide(20.3).forward is False
    assert decider.decide(20.3).drop_reason == "outage"
    assert decider.decide(35.3).forward is False
    assert decider.decide(19.9).forward is True
    assert decider.decide(34.9).forward is True


def test_outage_before_first_phase_is_not_a_false_positive():
    config = ImpairmentConfig(outage_period_s=15.0, outage_duration_s=0.7, outage_phase_s=5.0, seed=1)
    decider = ImpairmentDecider(config)
    for elapsed in (0.0, 1.0, 2.0, 4.99):
        assert decider.decide(elapsed).forward is True


def test_outage_is_a_pure_function_correct_under_backward_time_jump():
    """The outage check has no accumulated state -- calling decide() with
    a smaller elapsed_s than a previous call (e.g. a sim-time reset) must
    still classify correctly, not raise, and not "remember" the earlier,
    larger elapsed_s."""
    config = ImpairmentConfig(outage_period_s=15.0, outage_duration_s=0.7, outage_phase_s=5.0, seed=1)
    decider = ImpairmentDecider(config)
    assert decider.decide(20.3).forward is False  # inside 2nd window
    assert decider.decide(1.0).forward is True  # backward jump to before phase, must not raise/misclassify
    assert decider.decide(5.3).forward is False  # backward jump but still lands inside 1st window


def test_outage_combined_with_bernoulli_drop_never_double_counts():
    """When both outage and independent Bernoulli drop are configured,
    a message inside an outage window is dropped for the OUTAGE reason
    even if drop_probability=1.0 would also have dropped it -- outage is
    checked first and short-circuits the Bernoulli draw entirely (no RNG
    call consumed for a message already dropped by the outage)."""
    config = ImpairmentConfig(
        drop_probability=1.0, outage_period_s=15.0, outage_duration_s=0.7,
        outage_phase_s=5.0, seed=1,
    )
    decider = ImpairmentDecider(config)
    decision = decider.decide(5.3)
    assert decision.forward is False
    assert decision.drop_reason == "outage"
    # outside the outage window, drop_probability=1.0 alone still drops everything
    decision2 = decider.decide(10.0)
    assert decision2.forward is False
    assert decision2.drop_reason == "bernoulli"


def test_outage_zero_duration_or_zero_period_disables_outage():
    config_a = ImpairmentConfig(outage_period_s=15.0, outage_duration_s=0.0, outage_phase_s=5.0)
    config_b = ImpairmentConfig(outage_period_s=0.0, outage_duration_s=0.7, outage_phase_s=5.0)
    for config in (config_a, config_b):
        decider = ImpairmentDecider(config)
        assert decider.decide(5.3).forward is True
        assert decider.is_zero_impairment() is True


def test_is_zero_impairment_false_when_outage_configured():
    config = ImpairmentConfig(outage_period_s=15.0, outage_duration_s=0.7)
    assert ImpairmentDecider(config).is_zero_impairment() is False


# --- jitter formula: exact range, no clamping bias, matched-seed direction independence ---


def test_jitter_formula_matches_documented_uniform_half_amplitude():
    """jitter_s is a FULL peak-to-peak spread; the actual draw is
    Uniform(-jitter_s/2, +jitter_s/2). delay_s=jitter_s/2 exactly means
    the summed range's minimum touches 0.0 without being clamped (no
    atom at 0) -- this is Condition D's exact configuration
    (delay_s=0.15, jitter_s=0.30) from the impairment matrix design."""
    config = ImpairmentConfig(delay_s=0.15, jitter_s=0.30, seed=7)
    decider = ImpairmentDecider(config)
    samples = [decider.decide().release_delay_s for _ in range(20000)]
    assert min(samples) >= 0.0
    # with 20000 samples the true minimum should land very close to 0.0,
    # not be bounded away from it the way a clamped distribution would be
    assert min(samples) < 0.002
    assert max(samples) <= 0.30 + 1e-9
    assert max(samples) > 0.298
    mean = sum(samples) / len(samples)
    assert abs(mean - 0.15) < 0.01  # theoretical mean of Uniform(0, 0.30)


def test_jitter_formula_condition_g_range_has_no_floor_clamping():
    """Condition G (delay_s=0.20, jitter_s=0.20): draw range
    (-0.10,+0.10), summed range [0.10, 0.30] -- delay_s > jitter_s/2, so
    the floor never engages at all; realized distribution is exactly
    Uniform(0.10, 0.30)."""
    config = ImpairmentConfig(delay_s=0.20, jitter_s=0.20, seed=8)
    decider = ImpairmentDecider(config)
    samples = [decider.decide().release_delay_s for _ in range(20000)]
    assert min(samples) >= 0.10 - 1e-9
    assert min(samples) < 0.105  # should get close to the true 0.10 floor, not be clamped-away from it
    assert max(samples) <= 0.30 + 1e-9
    mean = sum(samples) / len(samples)
    assert abs(mean - 0.20) < 0.01  # theoretical mean of Uniform(0.10, 0.30)
    variance = sum((s - mean) ** 2 for s in samples) / len(samples)
    theoretical_variance = (0.30 - 0.10) ** 2 / 12.0
    assert abs(variance - theoretical_variance) < 0.001


def test_reordering_is_possible_when_jitter_spread_exceeds_publish_period():
    """Two consecutively-generated decisions, at a jitter spread larger
    than the source publish period, must be able to produce a release
    order different from receive order (over enough trials) -- this is
    what makes Condition D's reordering claim checkable in code, not just
    asserted in prose. Simulates two messages arriving 0.1151s apart
    (the measured publish period) and checks whether their absolute
    release times ever cross over across many repeated seed draws."""
    publish_period_s = 0.1151
    crossovers = 0
    trials = 2000
    for seed in range(trials):
        decider = ImpairmentDecider(ImpairmentConfig(delay_s=0.15, jitter_s=0.30, seed=seed))
        release_a = 0.0 + decider.decide().release_delay_s
        release_b = publish_period_s + decider.decide().release_delay_s
        if release_b < release_a:
            crossovers += 1
    assert crossovers > 0, "jitter spread of 0.30s at a 0.1151s publish period should produce reordering"


def test_max_release_delay_s_matches_the_actual_observed_maximum():
    config = ImpairmentConfig(delay_s=0.20, jitter_s=0.20, seed=9)
    decider = ImpairmentDecider(config)
    reported_max = decider.max_release_delay_s()
    observed_max = max(decider.decide().release_delay_s for _ in range(20000))
    assert observed_max <= reported_max + 1e-9
    assert abs(observed_max - reported_max) < 0.01  # should be a tight, not loose, bound


def test_two_direction_seeds_base_and_base_plus_one_produce_different_sequences():
    """Matches the matrix design's epuck1/epuck2 seed convention
    (base, base+1, per run_relay_counter_configurable.py's existing
    precedent) -- confirms the two directions do not receive identical
    drop/jitter sequences merely because they share a base seed value."""
    base_seed = 4001
    config_a = ImpairmentConfig(delay_s=0.2, jitter_s=0.2, drop_probability=0.1, seed=base_seed)
    config_b = ImpairmentConfig(delay_s=0.2, jitter_s=0.2, drop_probability=0.1, seed=base_seed + 1)
    sequence_a = [ImpairmentDecider(config_a).decide() for _ in range(100)]
    sequence_b = [ImpairmentDecider(config_b).decide() for _ in range(100)]
    assert sequence_a != sequence_b
