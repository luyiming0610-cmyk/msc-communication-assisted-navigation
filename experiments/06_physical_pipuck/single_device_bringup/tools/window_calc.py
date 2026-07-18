#!/usr/bin/env python3
"""Pure window-overlap / centered-main-window computation.

No file I/O, no ROS, no subprocess -- takes already-extracted
(start_unix_s, end_unix_s) pairs per data source and computes:

  1. the common overlap interval across all supplied sources,
  2. whether that overlap is long enough to hold a centered 240.000s main
     window with >=30.000s buffer on both sides (default REQUIRED_TOTAL_S
     = 300.000s = 240 + 30 + 30),
  3. if so, the exact main-window bounds and the two buffer lengths.

If the required span is not met, the verdict is SHORT_WINDOW and no main
window is computed -- callers must not shrink MAIN_SPAN_S or MIN_BUFFER_S
after the fact to force a pass. Both boundary cases matter and are tested
explicitly: an overlap of exactly REQUIRED_TOTAL_S must pass (buffers of
exactly MIN_BUFFER_S are valid, ">=" not ">"), and an overlap even 0.001s
short of that must fail.
"""
from __future__ import annotations

MAIN_SPAN_S = 240.0
MIN_BUFFER_S = 30.0
REQUIRED_TOTAL_S = MAIN_SPAN_S + 2 * MIN_BUFFER_S  # 300.0


def compute_overlap(sources: dict) -> tuple:
    """sources: {label: (start_unix_s, end_unix_s)}. Returns
    (common_start, common_end, limiting_start_label, limiting_end_label)."""
    if not sources:
        raise ValueError("no sources supplied")
    starts = {label: bounds[0] for label, bounds in sources.items()}
    ends = {label: bounds[1] for label, bounds in sources.items()}
    limiting_start_label = max(starts, key=starts.get)
    limiting_end_label = min(ends, key=ends.get)
    common_start = starts[limiting_start_label]
    common_end = ends[limiting_end_label]
    return common_start, common_end, limiting_start_label, limiting_end_label


def evaluate_window(sources: dict, main_span_s: float = MAIN_SPAN_S,
                     min_buffer_s: float = MIN_BUFFER_S,
                     required_total_s: float = REQUIRED_TOTAL_S) -> dict:
    """Full pipeline: overlap -> pass/fail -> centered window. Never shrinks
    main_span_s/min_buffer_s to force a pass; a short overlap is always
    reported as SHORT_WINDOW, never silently downgraded to a smaller
    'main window'."""
    common_start, common_end, limiting_start_label, limiting_end_label = compute_overlap(sources)
    span = common_end - common_start

    result = {
        "sources": {k: {"start_unix_s": v[0], "end_unix_s": v[1], "span_s": v[1] - v[0]} for k, v in sources.items()},
        "common_overlap_start_unix_s": common_start,
        "common_overlap_end_unix_s": common_end,
        "common_overlap_span_s": span,
        "limiting_start_source": limiting_start_label,
        "limiting_end_source": limiting_end_label,
        "required_total_s": required_total_s,
        "main_span_s": main_span_s,
        "min_buffer_s": min_buffer_s,
    }

    if span < required_total_s:
        result["verdict"] = "SHORT_WINDOW"
        result["shortfall_s"] = required_total_s - span
        return result

    center = (common_start + common_end) / 2.0
    main_start = center - main_span_s / 2.0
    main_end = center + main_span_s / 2.0
    left_buffer = main_start - common_start
    right_buffer = common_end - main_end

    if left_buffer < min_buffer_s or right_buffer < min_buffer_s:
        # Should not happen if span >= required_total_s and the window is
        # centered, but checked explicitly rather than assumed.
        result["verdict"] = "SHORT_WINDOW"
        result["shortfall_s"] = max(min_buffer_s - left_buffer, min_buffer_s - right_buffer, 0.0)
        return result

    result["verdict"] = "OK"
    result["main_window_start_unix_s"] = main_start
    result["main_window_end_unix_s"] = main_end
    result["left_buffer_s"] = left_buffer
    result["right_buffer_s"] = right_buffer
    return result


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Evaluate a centered main window from source start/end pairs.")
    parser.add_argument("--source", action="append", required=True,
                         metavar="LABEL:START:END",
                         help="Repeatable. e.g. --source bag:1700000000.1:1700000300.4")
    args = parser.parse_args()
    sources = {}
    for spec in args.source:
        label, start_s, end_s = spec.split(":", 2)
        sources[label] = (float(start_s), float(end_s))
    print(json.dumps(evaluate_window(sources), indent=2))
