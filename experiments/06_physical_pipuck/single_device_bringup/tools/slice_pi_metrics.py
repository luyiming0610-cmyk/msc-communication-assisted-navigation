#!/usr/bin/env python3
"""Slices the batch-level Pi raw metrics CSV to one trial's real main
window, using real Unix timestamps (never row-number alignment, never
the full 9631-row file). Writes a derived pi_system_metrics_window.csv
plus returns the provenance fields required in that trial's
runtime_manifest.json: source path, source SHA-256, slice start/end
unix times, the ORIGINAL row-number range sliced (1-indexed, header is
row 0), the derived file's own SHA-256, and summary Pi resource stats
computed only from the sliced rows (never from rows outside the trial's
main window).
"""
from __future__ import annotations

import csv
import hashlib
import math
import statistics
from pathlib import Path


def _percentile(sorted_values, fraction):
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    index = fraction * (len(sorted_values) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return sorted_values[int(index)]
    weight = index - lower
    return sorted_values[lower] * (1 - weight) + sorted_values[upper] * weight


def _dist(values):
    values = [v for v in values if v is not None]
    if not values:
        return {"sample_count": 0, "mean": None, "median": None, "p95": None, "p99": None, "max": None, "min": None}
    s = sorted(values)
    return {
        "sample_count": len(values),
        "mean": statistics.fmean(values),
        "median": _percentile(s, 0.5),
        "p95": _percentile(s, 0.95),
        "p99": _percentile(s, 0.99),
        "max": s[-1],
        "min": s[0],
    }


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def slice_pi_metrics(batch_csv_path: Path, window_start: float, window_end: float, output_path: Path) -> dict:
    """Reads the batch raw CSV once, selects rows whose unix_time_s falls
    within [window_start, window_end] (inclusive), writes those rows
    (with header) to output_path, and returns provenance + summary stats.
    Row numbers are 1-indexed counting the header as row 1 (so the first
    data row is row 2), matching a human opening the file in a text
    editor."""
    with batch_csv_path.open(encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        fieldnames = reader.fieldnames
        selected_rows = []
        first_selected_row_num = None
        last_selected_row_num = None
        for row_num, row in enumerate(reader, start=2):  # row 1 is the header
            t = float(row["unix_time_s"])
            if window_start <= t <= window_end:
                selected_rows.append(row)
                if first_selected_row_num is None:
                    first_selected_row_num = row_num
                last_selected_row_num = row_num

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(selected_rows)

    cpu = _dist([float(r["cpu_percent"]) for r in selected_rows if r.get("cpu_percent") not in (None, "")])
    mem_used = _dist([float(r["mem_used_mb"]) for r in selected_rows if r.get("mem_used_mb") not in (None, "")])
    mem_avail = _dist([float(r["mem_available_mb"]) for r in selected_rows if r.get("mem_available_mb") not in (None, "")])
    wifi_quality = _dist([float(r["wifi_link_quality"]) for r in selected_rows if r.get("wifi_link_quality") not in (None, "")])
    wifi_dbm = _dist([float(r["wifi_signal_dbm"]) for r in selected_rows if r.get("wifi_signal_dbm") not in (None, "")])
    net_rx = _dist([float(r["net_rx_bytes_delta"]) for r in selected_rows if r.get("net_rx_bytes_delta") not in (None, "")])
    net_tx = _dist([float(r["net_tx_bytes_delta"]) for r in selected_rows if r.get("net_tx_bytes_delta") not in (None, "")])

    return {
        "source_batch_csv_path": str(batch_csv_path),
        "source_batch_csv_sha256": sha256_of(batch_csv_path),
        "slice_window_start_unix_s": window_start,
        "slice_window_end_unix_s": window_end,
        "original_row_number_range": [first_selected_row_num, last_selected_row_num],
        "sliced_sample_count": len(selected_rows),
        "derived_file_path": str(output_path),
        "derived_file_sha256": sha256_of(output_path),
        "slicing_method": "real Unix-timestamp inclusion in [window_start, window_end], never row-number alignment, never the full batch file",
        "tool_version": "slice_pi_metrics.py",
        "pi_cpu_percent": cpu,
        "pi_mem_used_mb": mem_used,
        "pi_mem_available_mb": mem_avail,
        "pi_wifi_link_quality": wifi_quality,
        "pi_wifi_signal_dbm": wifi_dbm,
        "pi_net_rx_bytes_delta": net_rx,
        "pi_net_tx_bytes_delta": net_tx,
    }
