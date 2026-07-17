from epuck2_comm.neighbor_cache import NeighborCache, UINT32_MODULUS


def update(cache, sequence, monotonic=1.0, robot_id=1):
    return cache.update(
        robot_id=robot_id,
        sequence=sequence,
        source_stamp_ns=1_000_000_000,
        receive_stamp_ns=1_002_000_000,
        receive_monotonic=monotonic,
        serialized_bytes=80,
        state={"sequence": sequence},
    )


def test_sequence_gap_is_counted():
    cache = NeighborCache()
    update(cache, 10)
    result = update(cache, 13, monotonic=1.1)
    assert result.inferred_gap == 2
    assert cache.get(1).stats.inferred_lost == 2


def test_duplicate_does_not_replace_latest_state():
    cache = NeighborCache()
    update(cache, 5)
    result = update(cache, 5, monotonic=1.1)
    assert result.duplicate
    assert cache.get(1).sequence == 5
    assert cache.get(1).stats.duplicates == 1


def test_out_of_order_does_not_replace_latest_state():
    cache = NeighborCache()
    update(cache, 20)
    result = update(cache, 19, monotonic=1.1)
    assert result.out_of_order
    assert cache.get(1).sequence == 20


def test_uint32_wrap_is_forward_progress():
    cache = NeighborCache()
    update(cache, UINT32_MODULUS - 1)
    result = update(cache, 0, monotonic=1.1)
    assert not result.out_of_order
    assert result.inferred_gap == 0
    assert cache.get(1).sequence == 0


def test_get_fresh_uses_local_receive_age():
    cache = NeighborCache()
    update(cache, 1, monotonic=10.0)
    assert cache.get_fresh(1, now_monotonic=10.4, max_age_s=0.5) is not None
    assert cache.get_fresh(1, now_monotonic=10.6, max_age_s=0.5) is None


def test_latency_and_delivery_ratio():
    cache = NeighborCache()
    update(cache, 1)
    update(cache, 3, monotonic=1.1)
    stats = cache.get(1).stats
    assert stats.average_latency_ms == 2.0
    assert stats.delivery_ratio == 2.0 / 3.0
