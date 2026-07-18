#!/usr/bin/env python3
"""Queue-drain rule for the impairment-matrix orchestrator (design doc
section 5): after task completion, the relay/clock/counter/bag must keep
running, with both robots' commanded velocity held at zero, for at least
`max_configured_delivery_delay + 2 publish periods` before the relay's
`relay_status` topic is polled to confirm `pending_queue_depth == 0` --
only then may the relay and bag recorder be stopped. destroy_node() does
NOT flush the pending queue (network_impairment_relay.py, unchanged in
v1.1), so skipping this wait risks miscounting still-pending delayed
messages as "dropped" by the shutdown itself, contaminating the
condition's own delay/jitter results with an artifact of stopping too
early -- exactly the failure mode this module exists to prevent.

Pure functions only; no ROS, no subprocess -- the orchestrator wraps
these with the actual `ros2 topic echo relay_status --once` polling.
"""
from __future__ import annotations

import json


def max_configured_delivery_delay_s(delay_s: float, jitter_s: float) -> float:
    """Matches ImpairmentDecider.max_release_delay_s() exactly (kept as a
    separate, independently-testable copy here so the orchestrator-side
    drain-duration computation can be verified without importing the ROS
    package) -- delay_s + half the jitter spread, the largest
    release_delay_s decide() can ever produce for this config. Outage
    windows are NOT included: a message inside an outage window is
    dropped immediately (release_delay_s=0.0), it never sits in the
    queue, so outage cannot extend the drain wait."""
    return delay_s + max(0.0, jitter_s / 2.0)


def compute_drain_duration_s(
    delay_s: float, jitter_s: float, publish_period_s: float, periods_margin: int = 2,
) -> float:
    """max_configured_delivery_delay + periods_margin full publish
    periods of margin (per instruction: "至少为max_configured_delivery_delay
    + 2个发布周期"). publish_period_s should be the trial's own MEASURED
    period where available (design doc section 2.1's ~0.1151s), not an
    assumed nominal value."""
    if periods_margin < 0:
        raise ValueError("periods_margin must be >= 0")
    return max_configured_delivery_delay_s(delay_s, jitter_s) + periods_margin * publish_period_s


def parse_relay_status(raw_json_text: str) -> dict:
    """Parses one relay_status String message payload (see
    NetworkImpairmentRelay._publish_status's exact field set). Raises
    ValueError (not a silent default) if pending_queue_depth is missing
    -- a status message this module can't verify is drained must never
    be treated as if it reported an empty queue."""
    payload = json.loads(raw_json_text)
    if "pending_queue_depth" not in payload:
        raise ValueError("relay_status payload missing pending_queue_depth")
    return payload


def is_drained(status_payload: dict) -> bool:
    return int(status_payload["pending_queue_depth"]) == 0


class DrainTimeoutError(RuntimeError):
    """Raised when the queue did not reach pending_queue_depth==0 within
    the allotted drain window -- the caller must treat this trial as
    DATA_VALIDITY=INVALID (design doc section 5 item 5: 'shutdown未排空
    不得计为网络损伤结果'), never as a TASK_OUTCOME, and never silently
    proceed to stop the relay/bag anyway."""


def poll_until_drained(read_status_fn, timeout_s: float, poll_interval_s: float = 0.5,
                        time_fn=None, sleep_fn=None):
    """read_status_fn() -> dict (already parsed, e.g. via parse_relay_status)
    called repeatedly until is_drained() is true or timeout_s elapses.
    time_fn/sleep_fn default to time.monotonic/time.sleep; both are
    injectable so tests never need a real sleep. Returns the final
    (drained) status dict on success; raises DrainTimeoutError on
    timeout, including the last observed status for diagnosis."""
    import time as _time_module

    time_fn = time_fn or _time_module.monotonic
    sleep_fn = sleep_fn or _time_module.sleep
    deadline = time_fn() + timeout_s
    last_status = None
    while time_fn() <= deadline:
        last_status = read_status_fn()
        if is_drained(last_status):
            return last_status
        sleep_fn(poll_interval_s)
    raise DrainTimeoutError(
        f"pending_queue_depth did not reach 0 within {timeout_s:.3f}s; "
        f"last observed status: {last_status}"
    )
