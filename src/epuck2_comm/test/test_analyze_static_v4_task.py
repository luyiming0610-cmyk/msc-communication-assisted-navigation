"""controller_v4_full_sensor_bypass_20260717: analyzer reset-artifact test.

pilot_a3's own "epuck1_passed_box=true" verdict was corrupted by a single
post-shutdown /epuck1/state sample at x=0,y=0,yaw=0 -- a state_publisher/
Webots teardown artifact recorded after the controller had already reached
max_runtime_s. This test imports the standalone analyzer script (outside the
ROS package, run directly by the v4 pilot shell scripts) via its file path
and exercises its two filtering helpers directly, without needing a real
rosbag.
"""

import importlib.util
import math
from pathlib import Path


_RELATIVE_SUBPATH = (
    "experiments/controller_v4_full_sensor_bypass_20260717/"
    "config/static_box_v4/analyze_static_v4_task.py"
)
# WSL only mirrors src/ into the colcon workspace (see the repo's own
# "WSL mirror paths" convention); experiments/ lives solely in the
# Windows-repo git checkout, reachable from WSL via /mnt/c/. Try the normal
# relative-to-this-file path first (works if the test is ever run directly
# from the Windows-repo checkout), then fall back to the known /mnt/c/ path.
_CANDIDATES = [
    Path(__file__).resolve().parents[3] / _RELATIVE_SUBPATH,
    Path(
        "/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/" + _RELATIVE_SUBPATH
    ),
]
_ANALYZER_PATH = next((p for p in _CANDIDATES if p.exists()), _CANDIDATES[0])


def _load_analyzer():
    spec = importlib.util.spec_from_file_location("analyze_static_v4_task", _ANALYZER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_analyzer_module_is_present_and_loadable():
    assert _ANALYZER_PATH.exists(), f"expected analyzer at {_ANALYZER_PATH}"
    module = _load_analyzer()
    assert hasattr(module, "_is_reset_artifact")
    assert hasattr(module, "_controller_window")


def test_reset_artifact_triple_is_detected():
    module = _load_analyzer()
    assert module._is_reset_artifact(0.0, 0.0, 0.0) is True
    assert module._is_reset_artifact(-0.001, 0.0, 0.0) is False
    assert module._is_reset_artifact(0.0, 0.0, 0.05) is False


def test_controller_window_reads_first_and_last_timestamp(tmp_path):
    module = _load_analyzer()
    log_path = tmp_path / "controller.log"
    log_path.write_text(
        "[100.500] [INFO] first line mode=STARTUP_HOLD\n"
        "no timestamp bracket here, must be skipped\n"
        "[142.250] [INFO] last line mode=CRUISE\n",
        encoding="utf-8",
    )
    first_t, last_t = module._controller_window(log_path)
    assert first_t == 100.500
    assert last_t == 142.250


def test_controller_window_handles_empty_log(tmp_path):
    module = _load_analyzer()
    log_path = tmp_path / "empty.log"
    log_path.write_text("no timestamps at all\n", encoding="utf-8")
    first_t, last_t = module._controller_window(log_path)
    assert first_t is None
    assert last_t is None
