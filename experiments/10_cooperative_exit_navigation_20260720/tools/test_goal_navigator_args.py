"""Tests for goal_navigator.py's argparse contract: the completion-region
and parking-zone geometry, hold time, and nominal speed have NO default
value and NO hardcoded fallback anywhere in this module -- they must be
supplied by the caller (the orchestrator, sourced from
shared_exit_frozen_params.json) on every invocation, or the process
refuses to start."""
import pytest

from goal_navigator import parse_args

REQUIRED_GEOMETRY_FLAGS = [
    "--nominal-speed-mps=0.04",
    "--exit-center-x=0.50",
    "--exit-center-y=0.50",
    "--exit-radius=0.10",
    "--parking-x=0.64",
    "--parking-y=0.50",
    "--parking-radius=0.04",
    "--goal-hold-time-s=2.0",
]

BASE_ARGS = [
    "--robot-id=1",
    "--state-topic=/epuck1/state",
    "--nav-intent-topic=/epuck1/nav_intent",
    "--mode=informed",
]


def test_full_args_parse_succeeds():
    args = parse_args(BASE_ARGS + REQUIRED_GEOMETRY_FLAGS)
    assert args.nominal_speed_mps == 0.04
    assert args.exit_center_x == 0.50
    assert args.parking_x == 0.64
    assert args.goal_hold_time_s == 2.0


@pytest.mark.parametrize("missing_flag", REQUIRED_GEOMETRY_FLAGS)
def test_missing_any_geometry_flag_refuses_to_parse(missing_flag):
    incomplete = [f for f in REQUIRED_GEOMETRY_FLAGS if f != missing_flag]
    with pytest.raises(SystemExit):
        parse_args(BASE_ARGS + incomplete)
