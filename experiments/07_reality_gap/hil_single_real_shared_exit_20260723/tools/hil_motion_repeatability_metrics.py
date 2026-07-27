#!/usr/bin/env python3
"""Pure offline motion-repeatability metrics for
SINGLE_ROBOT_GROUND_REPEATABILITY_BASELINE, added 2026-07-27.

Computes longitudinal/lateral displacement, final yaw error, and
centre-to-stop-line clearance from state_x_m/state_y_m/state_yaw_rad
-- the three additive command-evidence-recorder columns captured from
the SAME /epuck1/state subscription the recorder already held (real
dead-reckoning odometry, published by state_publisher.py from its own
/odom subscription; not a new capability, only a newly-recorded one).

Deliberately separate from analyze_ground_diagnostic.py:
compute_odometry_displacement()/compute_average_ground_speed() in that
file are asserted by an existing test to always return NotAvailable,
and that assertion must stay true for evidence that genuinely lacks
pose data (including the already-accepted RUN_ID 20260727_102033).
This module is additive, reused (not duplicating) analyze_ground_
diagnostic.load_wsl_csv_rows and analyze_ground_diagnostic's
count_nonzero_pulses (itself reused, not duplicated, by
ground_diagnostic_post_run_verifier.py).

Never touches evaluate_verdict() or any acceptance threshold -- these
metrics are reported as measured facts only, per
SINGLE_ROBOT_GROUND_REPEATABILITY_BASELINE_SPEC.md's explicit
prohibition on inventing a displacement PASS tolerance.

Timestamp selection and staleness rules (binding for this module):
  - The START sample is the LAST valid pose sample with
    local_time_ns <= the single guarded pulse's first-nonzero time
    (the most recent known pose immediately before commanded motion
    began). If that sample is more than max_sample_staleness_s before
    the pulse start, it is rejected as stale, not silently used.
  - The END sample is the LAST valid pose sample recorded for the
    state topic at all (the final observed resting pose). If that
    sample's time is earlier than the pulse's last-nonzero time, the
    evidence does not actually cover the full motion and the result is
    NOT_AVAILABLE, never a fabricated "final" position.
  - A pose sample is "valid" only if state_x_m/state_y_m/state_yaw_rad
    are all present and finite (math.isfinite) -- a NaN/Inf/missing
    value silently excludes that one sample, never the whole run.
  - Exactly one pulse is required. Zero or more than one pulse on the
    guarded topic makes displacement/drift/yaw NOT_AVAILABLE with a
    specific reason -- this module never guesses which of several
    pulses is "the" trial pulse.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from analyze_ground_diagnostic import count_nonzero_pulses

DEFAULT_MAX_SAMPLE_STALENESS_S = 1.0


@dataclass(frozen=True)
class StateSample:
    local_time_ns: int
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class MotionMetricsResult:
    available: bool
    reason: Optional[str]
    start_sample_time_ns: Optional[int] = None
    end_sample_time_ns: Optional[int] = None
    longitudinal_displacement_m: Optional[float] = None
    lateral_displacement_m: Optional[float] = None
    final_yaw_error_rad: Optional[float] = None
    stop_line_clearance_m: Optional[float] = None

    def to_dict(self) -> dict:
        return {
            "available": self.available,
            "reason": self.reason,
            "start_sample_time_ns": self.start_sample_time_ns,
            "end_sample_time_ns": self.end_sample_time_ns,
            "longitudinal_displacement_m": self.longitudinal_displacement_m,
            "lateral_displacement_m": self.lateral_displacement_m,
            "final_yaw_error_rad": self.final_yaw_error_rad,
            "stop_line_clearance_m": self.stop_line_clearance_m,
        }


def _not_available(reason: str) -> MotionMetricsResult:
    return MotionMetricsResult(available=False, reason=reason)


def _wrap_to_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def extract_valid_state_samples(wsl_rows: list, state_topic: str) -> list:
    """Rows on `state_topic` with all three pose fields present as
    finite floats, in original (chronological) list order. A row
    missing any field (old-format CSV, or a row from before this
    instrumentation existed) or carrying a non-finite value is
    excluded silently -- never fabricated, never raised."""
    samples = []
    for r in wsl_rows:
        if r.get("topic") != state_topic:
            continue
        if not isinstance(r.get("local_time_ns"), int):
            continue
        x, y, yaw = r.get("state_x_m"), r.get("state_y_m"), r.get("state_yaw_rad")
        if not (isinstance(x, float) and isinstance(y, float) and isinstance(yaw, float)):
            continue
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            continue
        samples.append(StateSample(r["local_time_ns"], x, y, yaw))
    return samples


def compute_motion_metrics(
    wsl_rows: list,
    state_topic: str,
    guarded_topic: str,
    frozen_start_yaw_rad: float = 0.0,
    stop_line_distance_m: float = 0.10,
    max_sample_staleness_s: float = DEFAULT_MAX_SAMPLE_STALENESS_S,
) -> MotionMetricsResult:
    samples = extract_valid_state_samples(wsl_rows, state_topic)
    if not samples:
        return _not_available("NO_POSE_SAMPLES_AVAILABLE")

    pulses = count_nonzero_pulses(wsl_rows, guarded_topic)
    if len(pulses) == 0:
        return _not_available("NO_PULSE_DETECTED")
    if len(pulses) > 1:
        return _not_available("MULTIPLE_PULSES_DETECTED")
    pulse = pulses[0]

    candidates_before = [s for s in samples if s.local_time_ns <= pulse.start_time_ns]
    if not candidates_before:
        return _not_available("NO_STATE_SAMPLE_BEFORE_PULSE_START")
    start_sample = candidates_before[-1]
    if (pulse.start_time_ns - start_sample.local_time_ns) / 1e9 > max_sample_staleness_s:
        return _not_available("START_SAMPLE_TOO_STALE")

    end_sample = samples[-1]
    if end_sample.local_time_ns < pulse.end_time_ns:
        return _not_available("FINAL_STATE_SAMPLE_BEFORE_PULSE_END")

    dx = end_sample.x_m - start_sample.x_m
    dy = end_sample.y_m - start_sample.y_m
    c, s = math.cos(frozen_start_yaw_rad), math.sin(frozen_start_yaw_rad)
    longitudinal = dx * c + dy * s
    lateral = -dx * s + dy * c
    yaw_error = _wrap_to_pi(end_sample.yaw_rad - start_sample.yaw_rad)
    clearance = stop_line_distance_m - longitudinal

    return MotionMetricsResult(
        available=True,
        reason=None,
        start_sample_time_ns=start_sample.local_time_ns,
        end_sample_time_ns=end_sample.local_time_ns,
        longitudinal_displacement_m=longitudinal,
        lateral_displacement_m=lateral,
        final_yaw_error_rad=yaw_error,
        stop_line_clearance_m=clearance,
    )
