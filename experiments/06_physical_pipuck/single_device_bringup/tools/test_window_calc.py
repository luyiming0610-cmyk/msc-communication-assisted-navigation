"""Tests for window_calc.py's overlap/centered-window computation.

Covers the boundary cases the user explicitly required: an overlap of
exactly 300.000s must pass with exactly 30.000s buffers on both sides, and
an overlap of 299.999s must fail -- the threshold is never fuzzy. Also
covers the two real failure modes observed in trial01/trial02 (bag starts
late, status CSV ends early) as synthetic reproductions, and a normal
315s-total run with asymmetric offsets to check the centered-window math
is genuinely computed, not assumed symmetric.
"""
from window_calc import evaluate_window, compute_overlap, MAIN_SPAN_S, MIN_BUFFER_S, REQUIRED_TOTAL_S


def test_exactly_300s_overlap_passes_with_exact_30s_buffers():
    sources = {
        "bag": (1000.0, 1300.0),
        "status_csv": (1000.0, 1300.0),
        "system_csv": (1000.0, 1300.0),
    }
    result = evaluate_window(sources)
    assert result["verdict"] == "OK"
    assert result["common_overlap_span_s"] == 300.0
    assert abs(result["left_buffer_s"] - 30.0) < 1e-9
    assert abs(result["right_buffer_s"] - 30.0) < 1e-9
    assert abs((result["main_window_end_unix_s"] - result["main_window_start_unix_s"]) - 240.0) < 1e-9


def test_299_999s_overlap_fails():
    sources = {
        "bag": (1000.0, 1299.999),
        "status_csv": (1000.0, 1299.999),
        "system_csv": (1000.0, 1299.999),
    }
    result = evaluate_window(sources)
    assert result["verdict"] == "SHORT_WINDOW"
    assert result["shortfall_s"] > 0
    assert abs(result["shortfall_s"] - 0.001) < 1e-6


def test_bag_starts_late_reproduces_trial01_style_shortfall():
    """Reproduction of the actual trial01 failure mode: bag's first message
    arrives after status/system recorders are already flowing, and status
    CSV stops before the bag does -- exactly the asymmetric pattern
    observed twice in real runs."""
    sources = {
        "bag": (1000.6, 1299.2),       # starts 0.6s late
        "status_csv": (1000.0, 1295.9),  # ends ~3.9s early
        "system_csv": (999.8, 1299.6),
    }
    result = evaluate_window(sources)
    assert result["verdict"] == "SHORT_WINDOW"
    assert result["limiting_start_source"] == "bag"
    assert result["limiting_end_source"] == "status_csv"


def test_status_recorder_stopped_early_alone_causes_short_window():
    sources = {
        "bag": (1000.0, 1320.0),
        "status_csv": (1000.0, 1280.0),  # stopped 40s before everything else
        "system_csv": (1000.0, 1320.0),
    }
    result = evaluate_window(sources)
    assert result["verdict"] == "SHORT_WINDOW"
    assert result["limiting_end_source"] == "status_csv"


def test_bag_stopped_early_alone_causes_short_window():
    sources = {
        "bag": (1000.0, 1280.0),  # bag itself stopped early this time
        "status_csv": (1000.0, 1320.0),
        "system_csv": (1000.0, 1320.0),
    }
    result = evaluate_window(sources)
    assert result["verdict"] == "SHORT_WINDOW"
    assert result["limiting_end_source"] == "bag"


def test_normal_315s_run_with_asymmetric_offsets_computes_correct_centered_window():
    """Simulates the intended fixed-orchestrator shape: T0-based 315s run
    where each source's actual start/end differs slightly (sub-second
    subscription lag etc.), but overlap is comfortably >= 300s so a
    correctly centered 240s window should be produced. Centering around
    the overlap's own midpoint means the two buffers are mathematically
    always equal to each other by construction (that's what "centered"
    means) -- what this test actually checks is that they are correctly
    DERIVED from the real, asymmetric overlap bounds (limited by "bag" on
    the left and "status_csv" on the right, not by whichever source
    happens to be listed first), not from any assumed/hardcoded split."""
    t0 = 2_000_000_000.0
    sources = {
        "bag": (t0 + 0.42, t0 + 315.05),
        "status_csv": (t0 + 0.10, t0 + 309.50),
        "system_csv": (t0 + 0.05, t0 + 315.20),
    }
    result = evaluate_window(sources)
    assert result["verdict"] == "OK"
    common_start, common_end, limiting_start, limiting_end = compute_overlap(sources)
    assert limiting_start == "bag"
    assert limiting_end == "status_csv"
    expected_center = (common_start + common_end) / 2.0
    expected_main_start = expected_center - MAIN_SPAN_S / 2.0
    expected_main_end = expected_center + MAIN_SPAN_S / 2.0
    assert abs(result["main_window_start_unix_s"] - expected_main_start) < 1e-9
    assert abs(result["main_window_end_unix_s"] - expected_main_end) < 1e-9
    assert result["left_buffer_s"] >= MIN_BUFFER_S
    assert result["right_buffer_s"] >= MIN_BUFFER_S
    # Centered => buffers equal each other, but only because they were both
    # correctly computed from common_start/common_end (asserted above), not
    # because the test assumes symmetry independently of those bounds.
    assert abs(result["left_buffer_s"] - result["right_buffer_s"]) < 1e-9


def test_four_source_overlap_pi_metrics_added_post_batch():
    """Post-batch: Pi metrics CSV is folded in as a 4th source. If the Pi
    CSV's own coverage is narrower than the other three, it becomes the
    new limiting factor -- verifies the function generalizes to N sources,
    not hardcoded to exactly three."""
    sources = {
        "bag": (1000.0, 1320.0),
        "status_csv": (1000.0, 1320.0),
        "system_csv": (1000.0, 1320.0),
        "pi_metrics_csv": (1005.0, 1310.0),
    }
    result = evaluate_window(sources)
    assert result["verdict"] == "OK"
    assert result["limiting_start_source"] == "pi_metrics_csv"
    assert result["limiting_end_source"] == "pi_metrics_csv"


def test_required_total_matches_240_plus_2x30():
    assert REQUIRED_TOTAL_S == MAIN_SPAN_S + 2 * MIN_BUFFER_S == 300.0
