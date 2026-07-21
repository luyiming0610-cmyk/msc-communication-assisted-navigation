from dataclasses import replace

from multi_peer_selector import RankedPeer, choose_peer


def item(robot_id, distance, tcpa, dcpa, risk):
    return RankedPeer(robot_id, None, distance, tcpa, dcpa, risk)


def test_risk_wins_over_nearer_nonrisk():
    selected = choose_peer([item(2, 0.1, 1.0, 0.3, False), item(3, 0.4, 0.5, 0.1, True)])
    assert selected.robot_id == 3


def test_soonest_cpa_wins():
    selected = choose_peer([item(2, 0.2, 0.7, 0.1, True), item(3, 0.3, 0.4, 0.12, True)])
    assert selected.robot_id == 3


def test_distance_at_cpa_breaks_risk_tie():
    selected = choose_peer([item(2, 0.2, 0.5, 0.11, True), item(3, 0.3, 0.5, 0.08, True)])
    assert selected.robot_id == 3


def test_nearest_fresh_fallback():
    selected = choose_peer([item(2, 0.4, 0.0, 0.4, False), item(3, 0.3, 0.0, 0.3, False)])
    assert selected.robot_id == 3


def test_robot_id_is_deterministic_final_tiebreaker():
    selected = choose_peer([item(3, 0.3, 0.5, 0.1, True), item(2, 0.3, 0.5, 0.1, True)])
    assert selected.robot_id == 2


def test_empty_returns_none():
    assert choose_peer([]) is None
