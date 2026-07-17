from dataclasses import dataclass, field
from typing import Any, Dict, Optional


UINT32_MODULUS = 2**32
UINT32_HALF_RANGE = 2**31


@dataclass
class NeighborStats:
    received: int = 0
    inferred_lost: int = 0
    duplicates: int = 0
    out_of_order: int = 0
    latency_sum_ms: float = 0.0
    latency_max_ms: float = 0.0
    serialized_bytes: int = 0

    @property
    def average_latency_ms(self) -> float:
        return self.latency_sum_ms / self.received if self.received else 0.0

    @property
    def delivery_ratio(self) -> float:
        expected = self.received + self.inferred_lost
        return self.received / expected if expected else 1.0


@dataclass
class NeighborEntry:
    robot_id: int
    state: Any
    sequence: int
    source_stamp_ns: int
    receive_stamp_ns: int
    receive_monotonic: float
    first_receive_monotonic: float
    stats: NeighborStats = field(default_factory=NeighborStats)


@dataclass(frozen=True)
class NeighborUpdate:
    robot_id: int
    sequence: int
    inferred_gap: int
    duplicate: bool
    out_of_order: bool
    latency_ms: float


class NeighborCache:
    """Latest-state cache with wrap-safe sequence accounting and freshness checks."""

    def __init__(self):
        self._entries: Dict[int, NeighborEntry] = {}

    def update(
        self,
        *,
        robot_id: int,
        sequence: int,
        source_stamp_ns: int,
        receive_stamp_ns: int,
        receive_monotonic: float,
        serialized_bytes: int,
        state: Any,
    ) -> NeighborUpdate:
        sequence = int(sequence) % UINT32_MODULUS
        latency_ms = max(0.0, (receive_stamp_ns - source_stamp_ns) / 1_000_000.0)
        entry = self._entries.get(robot_id)

        inferred_gap = 0
        duplicate = False
        out_of_order = False

        if entry is None:
            stats = NeighborStats(
                received=1,
                latency_sum_ms=latency_ms,
                latency_max_ms=latency_ms,
                serialized_bytes=max(0, int(serialized_bytes)),
            )
            self._entries[robot_id] = NeighborEntry(
                robot_id=robot_id,
                state=state,
                sequence=sequence,
                source_stamp_ns=source_stamp_ns,
                receive_stamp_ns=receive_stamp_ns,
                receive_monotonic=receive_monotonic,
                first_receive_monotonic=receive_monotonic,
                stats=stats,
            )
        else:
            delta = (sequence - entry.sequence) % UINT32_MODULUS
            if delta == 0:
                duplicate = True
                entry.stats.duplicates += 1
            elif delta < UINT32_HALF_RANGE:
                inferred_gap = max(0, delta - 1)
                entry.stats.inferred_lost += inferred_gap
            else:
                out_of_order = True
                entry.stats.out_of_order += 1

            entry.stats.received += 1
            entry.stats.latency_sum_ms += latency_ms
            entry.stats.latency_max_ms = max(entry.stats.latency_max_ms, latency_ms)
            entry.stats.serialized_bytes += max(0, int(serialized_bytes))

            # Duplicates and old out-of-order samples contribute to transport
            # statistics but must not replace the newest usable state.
            if not duplicate and not out_of_order:
                entry.state = state
                entry.sequence = sequence
                entry.source_stamp_ns = source_stamp_ns
                entry.receive_stamp_ns = receive_stamp_ns
                entry.receive_monotonic = receive_monotonic

        return NeighborUpdate(
            robot_id=robot_id,
            sequence=sequence,
            inferred_gap=inferred_gap,
            duplicate=duplicate,
            out_of_order=out_of_order,
            latency_ms=latency_ms,
        )

    def get(self, robot_id: int) -> Optional[NeighborEntry]:
        return self._entries.get(robot_id)

    def get_fresh(self, robot_id: int, *, now_monotonic: float, max_age_s: float):
        entry = self._entries.get(robot_id)
        if entry is None:
            return None
        if now_monotonic - entry.receive_monotonic > max_age_s:
            return None
        return entry.state

    def entries(self):
        return tuple(self._entries.values())
