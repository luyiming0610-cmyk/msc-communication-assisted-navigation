"""Pure-Python multi-peer CPA risk ranking (design-only preparatory module).

Implements exactly the ranking rule described in
multi_peer_extension_design_20260720.md section 3: given zero or more
FRESH, USABLE peer CpaResult-like records, select the single
highest-priority conflict (soonest predicted CPA first, nearest predicted
approach distance as tiebreaker), or report no conflict.

NOT wired into cooperative_avoider.py. This module exists to let the
ranking rule be reviewed and unit-tested BEFORE any N3/N4 controller
change, per instruction ("如需改动，先提交最小设计，不得直接拼接临时代码").
Only src/epuck2_comm/epuck2_comm/collision_math.py's existing
CpaResult/collision_risk are reused (imported, not reimplemented).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PeerCandidate:
    """One fresh, usable peer's identity plus its CPA metrics and whether
    collision_risk() flagged it. peer_id is opaque (topic name, robot_id,
    whatever the caller uses) -- this module does not interpret it."""
    peer_id: str
    time_to_cpa_s: float
    distance_at_cpa_m: float
    current_distance_m: float
    is_risk: bool


def select_highest_priority_conflict(
    candidates: list[PeerCandidate],
) -> Optional[PeerCandidate]:
    """Returns the single highest-priority peer in genuine conflict
    (is_risk=True), or None if no candidate is in conflict.

    Ranking: smallest time_to_cpa_s wins (soonest predicted conflict takes
    priority); distance_at_cpa_m is the tiebreaker (closer predicted
    approach wins). Candidates with is_risk=False are never selected, even
    if their CPA numbers would otherwise rank first -- risk is a hard
    precondition, not part of the sort key.
    """
    at_risk = [c for c in candidates if c.is_risk]
    if not at_risk:
        return None
    at_risk.sort(key=lambda c: (c.time_to_cpa_s, c.distance_at_cpa_m))
    return at_risk[0]


def any_peer_stale(peer_fresh_flags: list[bool]) -> bool:
    """Fail-closed rule from the design doc section 2: ANY single stale
    peer (out of a possibly-empty list) means the robot must treat the
    overall peer-link state as unsafe. An empty list (no peers configured,
    e.g. COMM_OFF or N=1) is never stale by vacuous truth."""
    return any(not fresh for fresh in peer_fresh_flags)
