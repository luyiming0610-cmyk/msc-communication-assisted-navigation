import csv
import json
import shutil
from pathlib import Path

import pytest

from aggregate_objective5_matrix_a_to_g import (
    AggregationError,
    NOT_AVAILABLE,
    aggregate,
    classify_drop_mechanism,
    run,
    trial_dir_name,
    write_plot_data_csvs,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
MATRIX_ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------
# Real-evidence tests (read-only against the actual preserved A-G evidence)
# ---------------------------------------------------------------------------

def test_exact_35_row_formal_dataset():
    rows, excluded, ctx = aggregate(MATRIX_ROOT, ["A", "B", "C", "D", "E", "F", "G"])
    assert len(rows) == 35


def test_c05_retained_as_valid_unsafe_failure():
    rows, _, _ = aggregate(MATRIX_ROOT, ["C"])
    c05 = next(r for r in rows if r["trial_id"].endswith("trial05_attempt01"))
    assert c05["data_validity"] == "VALID"
    assert c05["task_outcome"] == "UNSAFE_FAILURE"
    assert c05["minimum_interrobot_distance_m"] == pytest.approx(0.13890857361707953)
    assert c05["safety_margin_m"] < 0


def test_g02_retained_as_valid_unsafe_failure():
    rows, _, _ = aggregate(MATRIX_ROOT, ["G"])
    g02 = next(r for r in rows if r["trial_id"].endswith("trial02_attempt01"))
    assert g02["data_validity"] == "VALID"
    assert g02["task_outcome"] == "UNSAFE_FAILURE"
    assert g02["minimum_interrobot_distance_m"] == pytest.approx(0.1396690407705024)
    assert g02["safety_margin_m"] < 0


def test_d04_excluded_and_d06_counted():
    rows, excluded, _ = aggregate(MATRIX_ROOT, ["D"])
    trial_ids = {r["trial_id"] for r in rows}
    assert "objective5_impairment_matrix_v1_condition_D_trial04_attempt01" not in trial_ids
    assert "objective5_impairment_matrix_v1_condition_D_trial06_attempt01" in trial_ids
    assert len(rows) == 5
    assert len(excluded) == 1
    assert excluded[0]["trial_index"] == 4
    assert excluded[0]["classification"] == "EXCLUDED_MEASUREMENT_CHAIN_ATTEMPT"


def test_missing_metric_remains_not_available_task_completion_time():
    rows, _, _ = aggregate(MATRIX_ROOT, ["A"])
    for row in rows:
        assert row["task_completion_time_s"] == NOT_AVAILABLE
        assert row["recovery_time_s"] == NOT_AVAILABLE


def test_f_stale_stop_duration_is_available_others_are_not():
    rows, _, _ = aggregate(MATRIX_ROOT, ["F"])
    for row in rows:
        assert row["stale_stop_duration_s"] != NOT_AVAILABLE
        assert isinstance(row["stale_stop_duration_s"], float)
    rows_a, _, _ = aggregate(MATRIX_ROOT, ["A"])
    for row in rows_a:
        assert row["stale_stop_duration_s"] == NOT_AVAILABLE


def test_p95_message_age_is_never_converted_to_zero_when_genuinely_missing(tmp_path, monkeypatch):
    """This is a synthetic-fixture regression test: a matrix_analysis.json
    whose latency block genuinely omits p95_message_age_s must surface as
    NOT_AVAILABLE in the canonical row, never as 0.0. (Note: the real
    Condition E per-trial matrix_analysis.json DOES carry p95 -- an
    earlier prose summary in this project incorrectly implied E's p95 was
    unavailable; that was a batch-summary-document omission, not a raw-
    evidence gap. This test targets the general robustness rule with a
    synthetic fixture rather than mislabeling E.)"""
    import aggregate_objective5_matrix_a_to_g as mod
    root = _make_single_trial_fixture(
        tmp_path, condition_id="Z", trial_index=1, attempt=1,
        latency_overrides={"p95_message_age_s": None},
        strip_latency_keys=["p95_message_age_s"],
    )
    # Exercise load_trial_row directly rather than aggregate(), since this
    # probe deliberately supplies only one synthetic trial and the "exactly
    # 5 counted trials" rule is out of scope for this specific check.
    ctx = mod.AggregationContext()
    row = mod.load_trial_row(root, "Z", 1, 1, ctx)
    assert row["measured_message_age_p95_epuck1_to_epuck2"] == NOT_AVAILABLE


def test_e_evidence_status_differs_from_f_and_g():
    rows_e, _, _ = aggregate(MATRIX_ROOT, ["E"])
    rows_f, _, _ = aggregate(MATRIX_ROOT, ["F"])
    rows_g, _, _ = aggregate(MATRIX_ROOT, ["G"])
    assert rows_e[0]["evidence_hash_status"] == "REPORTED_VERIFIED_NO_STANDALONE_MANIFEST"
    assert rows_f[0]["evidence_hash_status"] == "VERIFIED_WITH_STANDALONE_MANIFEST"
    assert rows_g[0]["evidence_hash_status"] == "VERIFIED_WITH_STANDALONE_MANIFEST"
    assert rows_e[0]["evidence_hash_status"] != rows_f[0]["evidence_hash_status"]


def test_every_input_read_appears_in_manifest(tmp_path):
    out_dir = tmp_path / "out"
    result = run(MATRIX_ROOT, out_dir, ["G"], dry_run=False)
    manifest_path = out_dir / "input_manifest_sha256.csv"
    with open(manifest_path, encoding="utf-8") as f:
        manifest_paths = {row["path"] for row in csv.DictReader(f)}
    # every G frozen_params/trial_verdict/matrix_analysis path must appear
    for trial_index in (1, 2, 3, 4, 5):
        d = MATRIX_ROOT / trial_dir_name("G", trial_index, 1)
        for fname in ("frozen_params.json", "trial_verdict.json", "matrix_analysis.json"):
            assert str(d / fname) in manifest_paths


def test_existing_output_directory_blocks_all_writes(tmp_path):
    out_dir = tmp_path / "out"
    out_dir.mkdir()
    (out_dir / "pre_existing_file.txt").write_text("do not touch")
    with pytest.raises(AggregationError):
        run(MATRIX_ROOT, out_dir, ["G"], dry_run=False)
    # the pre-existing file must remain untouched and no new files added
    assert list(out_dir.iterdir()) == [out_dir / "pre_existing_file.txt"]


def test_plot_data_row_counts_agree_with_per_trial_canonical(tmp_path):
    out_dir = tmp_path / "out"
    run(MATRIX_ROOT, out_dir, ["A", "B"], dry_run=False)
    with open(out_dir / "per_trial_canonical.csv", encoding="utf-8") as f:
        canonical_rows = list(csv.DictReader(f))
    with open(out_dir / "plot_data" / "min_interrobot_distance.csv", encoding="utf-8") as f:
        plot_rows = list(csv.DictReader(f))
    assert len(canonical_rows) == len(plot_rows) == 10


def test_no_existing_evidence_file_is_modified(tmp_path):
    watched = [
        MATRIX_ROOT / trial_dir_name("G", 2, 1) / "trial_verdict.json",
        MATRIX_ROOT / "objective5_condition_G_formal_batch_summary.json",
    ]
    before = {p: p.read_bytes() for p in watched}
    out_dir = tmp_path / "out"
    run(MATRIX_ROOT, out_dir, ["A", "B", "C", "D", "E", "F", "G"], dry_run=False)
    after = {p: p.read_bytes() for p in watched}
    assert before == after


def test_plot_data_numeric_columns_parse_as_float_for_real_output(tmp_path):
    """Every numeric plot-data column in the real (non-synthetic) A-G
    output must parse as float for every row -- confirming Tableau will
    read the column as a continuous numeric field, not text."""
    out_dir = tmp_path / "out"
    run(MATRIX_ROOT, out_dir, ["A", "B", "C", "D", "E", "F", "G"], dry_run=False, skip_plots=True)
    checks = [
        ("min_interrobot_distance.csv", ["minimum_interrobot_distance_m"]),
        ("safety_margin.csv", ["safety_margin_m"]),
        ("realised_loss.csv", ["authoritative_drop_fraction_epuck1_to_epuck2",
                                "authoritative_drop_fraction_epuck2_to_epuck1"]),
        ("message_age.csv", ["measured_message_age_mean_epuck1_to_epuck2",
                              "measured_message_age_mean_epuck2_to_epuck1"]),
        ("reordered_count.csv", ["reordered_count_epuck1_to_epuck2",
                                  "reordered_count_epuck2_to_epuck1"]),
    ]
    for fname, fields in checks:
        with open(out_dir / "plot_data" / fname, encoding="utf-8") as f:
            for row in csv.DictReader(f):
                for field_name in fields:
                    float(row[field_name])  # raises ValueError if not numeric-parseable
                    assert row[f"{field_name}_value_status"] == "AVAILABLE"


def test_plot_data_unavailable_value_exports_as_null_with_status_never_zero(tmp_path):
    """Synthetic-fixture regression test for the Tableau numeric-compatibility
    rule: a row whose numeric field is NOT_AVAILABLE must export as an EMPTY
    string (Tableau null) with value_status=NOT_AVAILABLE, never as 0 or as
    the literal text NOT_AVAILABLE inside the numeric column itself."""
    rows = [
        {"condition_id": "Z", "trial_id": "z_trial01", "minimum_interrobot_distance_m": 0.20,
         "safety_margin_m": NOT_AVAILABLE, "task_outcome": "SUCCESS", "drop_mechanism": "NONE",
         "authoritative_drop_fraction_epuck1_to_epuck2": NOT_AVAILABLE,
         "authoritative_drop_fraction_epuck2_to_epuck1": 0.1,
         "measured_message_age_mean_epuck1_to_epuck2": 0.01,
         "measured_message_age_mean_epuck2_to_epuck1": 0.01,
         "reordered_count_epuck1_to_epuck2": 0, "reordered_count_epuck2_to_epuck1": 0},
    ]
    out_dir = tmp_path / "out"
    write_plot_data_csvs(out_dir, rows)
    with open(out_dir / "plot_data" / "safety_margin.csv", encoding="utf-8") as f:
        row = next(csv.DictReader(f))
    assert row["safety_margin_m"] == ""  # empty/null, not "0" and not the literal NOT_AVAILABLE
    assert row["safety_margin_m_value_status"] == NOT_AVAILABLE
    with open(out_dir / "plot_data" / "realised_loss.csv", encoding="utf-8") as f:
        row = next(csv.DictReader(f))
    assert row["authoritative_drop_fraction_epuck1_to_epuck2"] == ""
    assert row["authoritative_drop_fraction_epuck1_to_epuck2_value_status"] == NOT_AVAILABLE
    assert row["authoritative_drop_fraction_epuck2_to_epuck1"] == "0.1"
    assert row["authoritative_drop_fraction_epuck2_to_epuck1_value_status"] == "AVAILABLE"
    # a genuine 0 (reordered_count) must remain 0, not be confused with NOT_AVAILABLE
    with open(out_dir / "plot_data" / "reordered_count.csv", encoding="utf-8") as f:
        row = next(csv.DictReader(f))
    assert row["reordered_count_epuck1_to_epuck2"] == "0"
    assert row["reordered_count_epuck1_to_epuck2_value_status"] == "AVAILABLE"


def test_skip_plots_produces_only_verified_data_outputs_no_svg_png(tmp_path):
    out_dir = tmp_path / "out"
    result = run(MATRIX_ROOT, out_dir, ["A", "B"], dry_run=False, skip_plots=True)
    assert result["skip_plots"] is True
    assert result["plot_files"] == []
    assert not (out_dir / "plots").exists()
    for f in out_dir.rglob("*"):
        assert f.suffix not in (".svg", ".png")
    expected = {
        "per_trial_canonical.csv", "per_condition_summary.csv", "excluded_trials.csv",
        "configured_vs_realised_impairment.csv", "evidence_status.csv",
        "input_manifest_sha256.csv", "A_to_G_aggregation_report.md",
    }
    top_level_names = {p.name for p in out_dir.iterdir() if p.is_file()}
    assert expected.issubset(top_level_names)
    assert (out_dir / "plot_data").is_dir()


def test_dry_run_writes_nothing(tmp_path):
    out_dir = tmp_path / "should_not_exist"
    result = run(MATRIX_ROOT, out_dir, ["A"], dry_run=True)
    assert result["dry_run"] is True
    assert not out_dir.exists()


# ---------------------------------------------------------------------------
# Fail-closed tests (synthetic fixtures only -- never touch real evidence)
# ---------------------------------------------------------------------------

def _write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data), encoding="utf-8")


def _make_single_trial_fixture(tmp_path, condition_id, trial_index, attempt,
                                data_validity="VALID", task_outcome="SUCCESS",
                                latency_overrides=None, strip_latency_keys=None,
                                min_dist=0.20):
    root = tmp_path / "root"
    d = root / trial_dir_name(condition_id, trial_index, attempt)
    d.mkdir(parents=True)
    _write_json(d / "frozen_params.json", {
        "condition_id": condition_id, "trial_index": trial_index, "attempt": attempt,
        "delay_s": 0.0, "jitter_s": 0.0, "drop_probability": 0.0,
        "outage_period_s": 0.0, "outage_duration_s": 0.0, "outage_phase_s": 0.0,
        "seed_epuck1_to_epuck2": 1, "seed_epuck2_to_epuck1": 2,
    })
    _write_json(d / "trial_verdict.json", {
        "condition_id": condition_id, "trial_index": trial_index, "attempt": attempt,
        "controller_complete_count": 2, "min_interrobot_distance_m": min_dist,
        "data_validity": data_validity, "task_outcome": task_outcome,
    })
    latency = {
        "mean_message_age_s": 0.01, "p95_message_age_s": 0.02,
    }
    if latency_overrides:
        latency.update(latency_overrides)
    if strip_latency_keys:
        for k in strip_latency_keys:
            latency.pop(k, None)
    direction = lambda name: {
        "direction": name,
        "relay": {"received_count": 100, "forwarded_count": 100, "dropped_count": 0},
        "sequence": {"out_of_order_count": 0, "duplicate_count": 0},
        "latency": latency,
    }
    _write_json(d / "matrix_analysis.json", {
        "condition_id": condition_id, "trial_index": trial_index,
        "directions": [direction("epuck1_to_epuck2"), direction("epuck2_to_epuck1")],
        "queue_drain": {"queue_drained": True},
        "task_outcome_inputs": {"controller_crashed": False},
        "measurement_validity": "VALID",
        "errors": [],
    })
    return root


def test_malformed_enum_fails_closed(tmp_path, monkeypatch):
    root = _make_single_trial_fixture(tmp_path, "Z", 1, 1, data_validity="MAYBE")
    monkeypatch.setitem(__import__("aggregate_objective5_matrix_a_to_g").CONDITION_TRIALS, "Z", [(1, 1)])
    with pytest.raises(AggregationError, match="unrecognised data_validity"):
        aggregate(root, ["Z"])


def test_duplicate_trial_id_fails_closed(tmp_path, monkeypatch):
    import aggregate_objective5_matrix_a_to_g as mod
    root = _make_single_trial_fixture(tmp_path, "Z", 1, 1)
    monkeypatch.setitem(mod.CONDITION_TRIALS, "Z", [(1, 1), (1, 1)])
    with pytest.raises(AggregationError, match="duplicate trial_id"):
        aggregate(root, ["Z"])


def test_missing_trial_fails_closed(tmp_path, monkeypatch):
    import aggregate_objective5_matrix_a_to_g as mod
    root = _make_single_trial_fixture(tmp_path, "Z", 1, 1)
    monkeypatch.setitem(mod.CONDITION_TRIALS, "Z", [(1, 1), (2, 1)])
    with pytest.raises(AggregationError, match="missing required source file"):
        aggregate(root, ["Z"])


def test_batch_summary_mismatch_fails_closed(tmp_path, monkeypatch):
    import aggregate_objective5_matrix_a_to_g as mod
    root = tmp_path / "root"
    for trial_index in range(1, 6):
        d = root / mod.trial_dir_name("Z", trial_index, 1)
        d.mkdir(parents=True)
        single = _make_single_trial_fixture(tmp_path / f"src{trial_index}", "Z", trial_index, 1, min_dist=0.20)
        src_dir = single / mod.trial_dir_name("Z", trial_index, 1)
        for fname in ("frozen_params.json", "trial_verdict.json", "matrix_analysis.json"):
            (d / fname).write_bytes((src_dir / fname).read_bytes())
    monkeypatch.setitem(mod.CONDITION_TRIALS, "Z", [(i, 1) for i in range(1, 6)])
    monkeypatch.setitem(mod._BATCH_SUMMARY_CANDIDATES, "Z", "z_summary.json")
    _write_json(root / "z_summary.json", {
        "individual_trial_verdicts": [
            {"trial_id": "objective5_impairment_matrix_v1_condition_Z_trial01_attempt01",
             "min_interrobot_distance_m": 0.99, "task_outcome": "SUCCESS", "data_validity": "VALID"}
        ]
    })
    with pytest.raises(AggregationError, match="disagrees with recomputed"):
        aggregate(root, ["Z"])


def test_output_directory_existing_and_nonempty_fails_closed(tmp_path):
    out_dir = tmp_path / "out"
    out_dir.mkdir()
    (out_dir / "x.txt").write_text("existing")
    with pytest.raises(AggregationError, match="already exists and is non-empty"):
        run(MATRIX_ROOT, out_dir, ["A"], dry_run=False)


def test_classify_drop_mechanism():
    assert classify_drop_mechanism(0.0, 0.0) == "NONE"
    assert classify_drop_mechanism(0.15, 0.0) == "INDEPENDENT_BERNOULLI"
    assert classify_drop_mechanism(0.0, 0.7) == "SCHEDULED_OUTAGE"
    assert classify_drop_mechanism(0.10, 0.7) == "COMBINED"
