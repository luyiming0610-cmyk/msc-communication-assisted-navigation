"""Tests for slice_pi_metrics.py: confirms slicing is by real Unix
timestamp (not row position), the row-number range reported is accurate,
and stats are computed only from the sliced rows."""
from pathlib import Path

from slice_pi_metrics import slice_pi_metrics


def _write_batch_csv(path: Path):
    header = "unix_time_s,cpu_percent,mem_used_mb,mem_available_mb,net_rx_bytes_delta,net_tx_bytes_delta,wifi_link_quality,wifi_signal_dbm\n"
    rows = []
    for i in range(20):
        t = 1000.0 + i  # 1000..1019
        rows.append(f"{t},{10 + i},{200 + i},{100 - i},{100},{50},70.0,-30.0\n")
    path.write_text(header + "".join(rows), encoding="utf-8")


def test_slice_selects_only_rows_in_real_timestamp_window(tmp_path):
    batch_csv = tmp_path / "batch.csv"
    _write_batch_csv(batch_csv)
    out = tmp_path / "window.csv"

    result = slice_pi_metrics(batch_csv, window_start=1005.0, window_end=1009.0, output_path=out)

    assert result["sliced_sample_count"] == 5  # t=1005..1009
    # row 2 is the first data row (t=1000); t=1005 is the 6th data row -> row 7
    assert result["original_row_number_range"] == [7, 11]
    assert result["pi_cpu_percent"]["sample_count"] == 5
    assert result["pi_cpu_percent"]["mean"] == (15 + 16 + 17 + 18 + 19) / 5

    out_lines = out.read_text(encoding="utf-8").strip().splitlines()
    assert len(out_lines) == 1 + 5  # header + 5 rows


def test_slice_excludes_rows_outside_window_even_if_adjacent(tmp_path):
    batch_csv = tmp_path / "batch.csv"
    _write_batch_csv(batch_csv)
    out = tmp_path / "window.csv"
    result = slice_pi_metrics(batch_csv, window_start=1010.0, window_end=1010.0, output_path=out)
    assert result["sliced_sample_count"] == 1
    assert result["original_row_number_range"] == [12, 12]


def test_slice_reports_correct_source_and_derived_sha256(tmp_path):
    batch_csv = tmp_path / "batch.csv"
    _write_batch_csv(batch_csv)
    out = tmp_path / "window.csv"
    result = slice_pi_metrics(batch_csv, window_start=1000.0, window_end=1019.0, output_path=out)
    assert len(result["source_batch_csv_sha256"]) == 64
    assert len(result["derived_file_sha256"]) == 64
    assert result["source_batch_csv_sha256"] != result["derived_file_sha256"]


def test_slice_empty_window_produces_zero_samples(tmp_path):
    batch_csv = tmp_path / "batch.csv"
    _write_batch_csv(batch_csv)
    out = tmp_path / "window.csv"
    result = slice_pi_metrics(batch_csv, window_start=5000.0, window_end=5001.0, output_path=out)
    assert result["sliced_sample_count"] == 0
    assert result["original_row_number_range"] == [None, None]
    assert result["pi_cpu_percent"]["sample_count"] == 0
