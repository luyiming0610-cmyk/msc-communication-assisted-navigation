from min_interrobot_distance import min_distance_from_paired_positions


def test_min_distance_simple_two_point_case():
    positions_a = [(0, 0.0, 0.0)]
    positions_b = [(0, 3.0, 4.0)]
    assert min_distance_from_paired_positions(positions_a, positions_b) == 5.0


def test_min_distance_picks_the_closest_approach_over_time():
    positions_a = [(0, 0.0, 0.0), (1_000_000_000, 1.0, 0.0), (2_000_000_000, 2.0, 0.0)]
    positions_b = [(0, 5.0, 0.0), (1_000_000_000, 1.05, 0.0), (2_000_000_000, 5.0, 0.0)]
    result = min_distance_from_paired_positions(positions_a, positions_b)
    assert abs(result - 0.05) < 1e-9


def test_min_distance_returns_none_when_either_side_empty():
    assert min_distance_from_paired_positions([], [(0, 0.0, 0.0)]) is None
    assert min_distance_from_paired_positions([(0, 0.0, 0.0)], []) is None
    assert min_distance_from_paired_positions([], []) is None


def test_min_distance_nearest_neighbor_pairing_not_index_pairing():
    """positions_a and positions_b are NOT the same length -- pairing
    must be by nearest timestamp, not by list index (which would crash
    or silently misalign on unequal-length real bag data)."""
    positions_a = [(0, 0.0, 0.0), (5_000_000_000, 10.0, 10.0)]
    positions_b = [(0, 0.1, 0.0), (1_000_000_000, 0.2, 0.0), (5_000_000_000, 10.05, 10.0)]
    result = min_distance_from_paired_positions(positions_a, positions_b)
    assert result < 0.1  # the t=5s pair (10,10)-(10.05,10) is the closest
