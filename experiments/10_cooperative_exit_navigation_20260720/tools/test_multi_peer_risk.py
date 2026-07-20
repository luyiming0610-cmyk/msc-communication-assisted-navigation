import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from multi_peer_risk import PeerCandidate, any_peer_stale, select_highest_priority_conflict


def _c(peer_id, tcpa, dcpa, dist=1.0, risk=True):
    return PeerCandidate(peer_id, tcpa, dcpa, dist, risk)


def test_no_candidates_no_conflict():
    assert select_highest_priority_conflict([]) is None


def test_single_peer_at_risk_selected():
    c = _c("p2", 1.5, 0.05)
    assert select_highest_priority_conflict([c]) is c


def test_no_peer_at_risk_returns_none():
    candidates = [_c("p2", 1.0, 0.05, risk=False), _c("p3", 0.5, 0.02, risk=False)]
    assert select_highest_priority_conflict(candidates) is None


def test_soonest_tcpa_wins_among_multiple_conflicts():
    # 3-robot case (N=3, 2 peers): p2 conflicts sooner than p3.
    p2 = _c("p2", 1.0, 0.05)
    p3 = _c("p3", 2.5, 0.01)
    assert select_highest_priority_conflict([p2, p3]) is p2


def test_distance_at_cpa_is_tiebreaker_on_equal_tcpa():
    # 4-robot case (N=4, 3 peers): p2/p3 tie on tcpa, p4 is not at risk.
    p2 = _c("p2", 1.0, 0.08)
    p3 = _c("p3", 1.0, 0.03)
    p4 = _c("p4", 0.9, 0.20, risk=False)
    winner = select_highest_priority_conflict([p2, p3, p4])
    assert winner is p3  # smaller distance_at_cpa_m wins the tie


def test_non_risk_candidate_never_beats_a_risk_candidate_even_with_better_tcpa():
    # A peer with objectively "better" (smaller) tcpa but is_risk=False
    # (e.g. closing_speed too low to be a real conflict per collision_risk())
    # must never be selected over a genuine, slower-approaching conflict.
    fast_but_not_risky = _c("p2", 0.1, 0.01, risk=False)
    slower_but_risky = _c("p3", 3.0, 0.05, risk=True)
    winner = select_highest_priority_conflict([fast_but_not_risky, slower_but_risky])
    assert winner is slower_but_risky


def test_any_peer_stale_empty_list_is_not_stale():
    assert any_peer_stale([]) is False


def test_any_peer_stale_all_fresh():
    assert any_peer_stale([True, True, True]) is False


def test_any_peer_stale_one_stale_among_many_fails_closed():
    # N=4 (3 peers): 2 fresh, 1 stale -> overall stale (fail-closed).
    assert any_peer_stale([True, False, True]) is True


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([os.path.abspath(__file__), "-v"]))
