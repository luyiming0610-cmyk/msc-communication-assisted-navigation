import importlib.util
import json
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("audit_condition_f_outage_timing.py")
SPEC = importlib.util.spec_from_file_location("condition_f_audit", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_outage_windows_are_absolute_and_frozen():
    assert MODULE.outage_windows(60.0) == [
        (10.0, 10.7), (25.0, 25.7), (40.0, 40.7), (55.0, 55.7)
    ]


def test_classifies_active_avoidance_without_claiming_initial_entry():
    trial = {
        "trial_id": "example",
        "source": "example.json",
        "timebase_init": {"epuck1": 23.0, "epuck2": 25.0},
        "cruise": {"epuck1": 28.0, "epuck2": 30.0},
        "avoid_turn": {"epuck1": 36.5, "epuck2": 36.5},
        "recover": {"epuck1": 58.0, "epuck2": 58.0},
    }
    result = MODULE.classify_trial(trial)
    assert result["window_40_fully_inside_active_avoidance"] is True
    assert result["window_55_fully_inside_active_avoidance"] is True
    assert result["windows_covering_initial_avoid_entry"] == []


def test_reads_compact_e_schema(tmp_path):
    path = tmp_path / "startup_sync_audit.json"
    path.write_text(json.dumps({
        "trial_id": "e01",
        "cruise_start_ros_time_s": {"epuck1": 29.0, "epuck2": 30.0},
        "avoid_turn_ros_time_s": {"epuck1": 37.0, "epuck2": 37.0},
        "recover_ros_time_s": {"epuck1": 58.0, "epuck2": 58.0},
    }))
    result = MODULE.read_trial_times(path)
    assert result["timebase_init"] is None
    assert result["avoid_turn"]["epuck1"] == 37.0


def test_fails_when_40_second_window_precedes_avoidance():
    trial = {
        "trial_id": "late",
        "source": "late.json",
        "timebase_init": None,
        "cruise": {"epuck1": 39.0, "epuck2": 39.0},
        "avoid_turn": {"epuck1": 41.0, "epuck2": 41.0},
        "recover": {"epuck1": 58.0, "epuck2": 58.0},
    }
    result = MODULE.classify_trial(trial)
    assert result["window_40_fully_inside_active_avoidance"] is False
