"""Pure-Python GoalAnnouncement sequence/timing analysis, no ROS
dependency so it can be unit tested without sourcing ROS. Mirrors
sequence_counter.py's missing/duplicate/out-of-order accounting
approach, applied to the recorded GoalAnnouncement message stream
instead of EpuckState.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class AnnouncementRecord:
    sequence: int
    production_stamp_s: float
    recv_stamp_s: float
    valid: bool


@dataclass
class AnnouncementSequenceStats:
    message_count: int
    missing_count: int
    duplicate_count: int
    out_of_order_count: int
    mean_age_s: float | None
    max_age_s: float | None


def analyze_announcement_sequence(records: list[AnnouncementRecord]) -> AnnouncementSequenceStats:
    """records must be in RECEIPT order (as they arrived / were recorded),
    not necessarily sequence order -- that is exactly what lets this
    detect out-of-order delivery."""
    if not records:
        return AnnouncementSequenceStats(0, 0, 0, 0, None, None)

    seen_sequences = set()
    duplicate_count = 0
    out_of_order_count = 0
    highest_seen = None
    ages = []

    for rec in records:
        if rec.sequence in seen_sequences:
            duplicate_count += 1
        else:
            seen_sequences.add(rec.sequence)
        if highest_seen is not None and rec.sequence < highest_seen:
            out_of_order_count += 1
        else:
            highest_seen = rec.sequence if highest_seen is None else max(highest_seen, rec.sequence)
        age = rec.recv_stamp_s - rec.production_stamp_s
        ages.append(age)

    unique_sequences = seen_sequences
    if unique_sequences:
        expected_span = max(unique_sequences) - min(unique_sequences) + 1
        missing_count = expected_span - len(unique_sequences)
    else:
        missing_count = 0

    mean_age = sum(ages) / len(ages) if ages else None
    max_age = max(ages) if ages else None

    return AnnouncementSequenceStats(
        message_count=len(records),
        missing_count=max(0, missing_count),
        duplicate_count=duplicate_count,
        out_of_order_count=out_of_order_count,
        mean_age_s=mean_age,
        max_age_s=max_age,
    )


def normalize_trial_relative(absolute_time_s: float, trial_epoch_s: float) -> float:
    """Converts an absolute sim-clock timestamp to trial-relative time
    (seconds since the trial's own first sample) -- never report a raw
    absolute sim-clock value as if it were a duration."""
    return absolute_time_s - trial_epoch_s


NOT_APPLICABLE = "NOT_APPLICABLE"


def build_off_communication_summary(off_leak_message_count: int) -> dict:
    """COMM_OFF must report every communication-contribution field as the
    literal string NOT_APPLICABLE, never a fake 0 -- by construction,
    always, regardless of what off_leak_message_count turns out to be.
    off_leak_message_count is reported separately, as itself, precisely
    so a nonzero value is visible rather than being silently coerced
    into NOT_APPLICABLE too."""
    return {
        "exit_announcement_tx_time_s": NOT_APPLICABLE,
        "exit_announcement_rx_time_s": NOT_APPLICABLE,
        "robot_b_search_to_goal_switch_time_s": NOT_APPLICABLE,
        "message_count": NOT_APPLICABLE,
        "missing_count": NOT_APPLICABLE,
        "duplicate_count": NOT_APPLICABLE,
        "out_of_order_count": NOT_APPLICABLE,
        "mean_age_s": NOT_APPLICABLE,
        "max_age_s": NOT_APPLICABLE,
        "off_leak_check_message_count": off_leak_message_count,
        "off_leak_detected": off_leak_message_count > 0,
    }


def build_on_communication_summary(
    tx_time_s, rx_time_s, switch_time_s, seq_stats: AnnouncementSequenceStats
) -> dict:
    return {
        "exit_announcement_tx_time_s": tx_time_s,
        "exit_announcement_rx_time_s": rx_time_s,
        "robot_b_search_to_goal_switch_time_s": switch_time_s,
        "message_count": seq_stats.message_count,
        "missing_count": seq_stats.missing_count,
        "duplicate_count": seq_stats.duplicate_count,
        "out_of_order_count": seq_stats.out_of_order_count,
        "mean_age_s": seq_stats.mean_age_s,
        "max_age_s": seq_stats.max_age_s,
    }
