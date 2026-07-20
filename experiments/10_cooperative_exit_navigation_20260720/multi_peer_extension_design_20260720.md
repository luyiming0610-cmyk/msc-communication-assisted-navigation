# Multi-peer extension — minimal-change design (NOT IMPLEMENTED this round)

Required by instruction before any N3/N4 controller change is made. This
is a design document only. `cooperative_avoider.py` is NOT modified by
this document; N3/N4 pilots are not authorized this round and this design
is not wired into the running controller.

## Scope

For N=3 (2 peers) or N=4 (3 peers), a robot must simultaneously track
freshness and CPA risk against multiple peers, and resolve conflicting
avoidance guidance into one command.

## 1. Multi-peer subscription table

Replace the scalar `peer_topic` / `peer_state` / `peer_received` with a
dict keyed by peer robot_id, populated from a list parameter:

```python
self.declare_parameter("peer_state_topics", ["/epuck2/state"])  # was a single string
...
self.peer_topics: list[str] = list(self.get_parameter("peer_state_topics").value)
self.peers: dict[str, PeerRecord] = {}  # keyed by topic (or resolved robot_id)

@dataclass
class PeerRecord:
    state: EpuckState | None = None
    received_at: float | None = None

for topic in self.peer_topics:
    self.peers[topic] = PeerRecord()
    self.create_subscription(
        EpuckState, topic,
        functools.partial(self._peer_callback, topic=topic), 20,
    )

def _peer_callback(self, message: EpuckState, *, topic: str) -> None:
    record = self.peers[topic]
    record.state = message
    record.received_at = self._now_s()
```

No change to `EpuckState.msg`, `state_publisher.py`, the relay, or
`sequence_counter.py` — each already supports exactly this (one relay/
counter instance per robot, as documented in the architecture audit).

## 2. Per-peer freshness

```python
def _peer_fresh(self, record: PeerRecord, now: float) -> bool:
    return record.received_at is not None and now - record.received_at <= self.peer_timeout

any_peer_stale = any(
    not self._peer_fresh(r, now) for r in self.peers.values()
) if self.enable_peer_avoidance else False
```

Design decision (stated explicitly, not assumed): **any single stale peer
triggers the same `SAFE_STOP_STALE` as today's binary check** — a
degraded/lost link to ANY teammate is treated as unsafe to continue,
consistent with the existing fail-closed philosophy (`cooperative_avoider.py`'s
existing `SAFE_STOP_STALE` already fails closed on a single peer). This
must be revisited if partial-connectivity scenarios become a research
question in their own right.

## 3. Multi-peer CPA risk ranking

Compute `CpaResult` per fresh peer (reusing `collision_math.closest_point_of_approach`
unmodified — already pairwise/N-agnostic, see architecture audit section 3),
then select the SINGLE highest-priority conflict to act on:

```python
def _multi_peer_risk(self, now: float):
    """Returns (highest_risk_peer_topic, CpaResult, is_risk) or (None, None, False)."""
    candidates = []
    for topic, record in self.peers.items():
        if not self._peer_fresh(record, now) or not self._state_usable(record.state):
            continue
        metrics = self._metrics_for(record.state)  # existing closest_point_of_approach call, parameterized by peer
        if self._risk(metrics):
            candidates.append((topic, metrics))
    if not candidates:
        return None, None, False
    # Ranking rule: smallest time_to_cpa_s wins (soonest conflict takes
    # priority) with distance_at_cpa_m as tiebreaker (closer predicted
    # approach wins). Both fields already exist on CpaResult -- no new
    # math, only a sort key.
    candidates.sort(key=lambda item: (item[1].time_to_cpa_s, item[1].distance_at_cpa_m))
    topic, metrics = candidates[0]
    return topic, metrics, True
```

This ranking rule (soonest-CPA-first) is the proposed default — a
documented design CHOICE requiring the same review as any other frozen
safety threshold, not something to silently pick after seeing results.
Alternative rules (closest-current-distance-first, or a full potential-
field sum across all conflicting peers) are NOT chosen here; soonest-CPA-
first is simplest to reason about and test, and degrades gracefully to
the existing single-peer behavior when there is only 1 peer (N=2).

## 4. Decision-conflict handling

Once the highest-priority peer conflict is selected, the AVOID_TURN /
AVOID_PASS / RECOVER state machine proceeds EXACTLY as today, just
parameterized by the selected peer's metrics instead of the sole
`self.peer_state`. Concretely: `self.mode`, `self.previous_pass_error`,
`self.encounter_complete` remain per-robot scalars (not per-peer) — a
robot handles ONE active avoidance encounter at a time, against whichever
peer currently has the highest-priority conflict. If a second peer becomes
higher-priority mid-encounter (e.g., its own CPA becomes more urgent),
this design does NOT interrupt an in-progress AVOID_TURN/AVOID_PASS to
switch targets — it finishes the current encounter, then re-evaluates.
This is a documented safety-conservatism choice (never abandon a
turn-in-progress) that must be validated by dedicated N3/N4 pilots before
being trusted, exactly as `enable_local_avoidance`'s priority-over-CPA
ordering was validated by the original A-D pilots.

## 5. What this design does NOT change

- `collision_math.py` — reused unmodified.
- `local_obstacle_logic.py` — reused unmodified (still robot-count-agnostic).
- `state_publisher.py`, `network_impairment_relay.py`,
  `sequence_counter.py` — reused unmodified, launched once per additional
  robot.
- `EpuckState.msg` — unchanged, `PROTOCOL_VERSION=1` unaffected.
- Frozen safety thresholds (`safety_radius_m=0.14`, `trigger_distance_m`,
  `cpa_horizon_s`, etc.) — unchanged.

## 6. Preconditions before this design may be implemented

1. Explicit user authorization for N3/N4 pilots (not given this round).
2. The `10_cooperative_exit_navigation_20260720` N2 pilots (this round)
   must PASS first, confirming the goal/exit-region task-completion
   criterion and analyzer are correct on the simplest case before adding
   multi-peer complexity on top.
3. A dedicated design-review pass on the ranking rule (section 3) and the
   conflict-handling policy (section 4), analogous to the review this
   document itself represents — not silently implemented inline with
   N3/N4's first pilot run.
