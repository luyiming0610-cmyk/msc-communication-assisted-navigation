import math

from epuck2_comm.analyze_static_bag import (
    box_surface_clearance,
    trajectory_metrics,
)


def test_box_clearance_matches_long_course_initial_geometry():
    clearance = box_surface_clearance(-0.55, 0.0, -0.25, 0.0, 0.06, 0.06, 0.035)
    assert math.isclose(clearance, 0.235, abs_tol=1.0e-9)


def test_box_clearance_is_negative_for_contact():
    clearance = box_surface_clearance(-0.31, 0.0, -0.25, 0.0, 0.06, 0.06, 0.035)
    assert clearance < 0.0


def test_trajectory_metrics_report_progress_lateral_motion_and_pass():
    records = [
        (0.0, -0.55, 0.00, 0.0, 0.235, math.inf, math.inf),
        (1.0, -0.35, -0.08, -0.4, 0.10, math.inf, math.inf),
        (2.0, -0.10, -0.08, 0.0, 0.80, math.inf, math.inf),
    ]
    result = trajectory_metrics(
        records,
        course_heading_rad=0.0,
        box_x_m=-0.25,
        box_y_m=0.0,
        box_size_x_m=0.06,
        box_size_y_m=0.06,
        robot_radius_m=0.035,
    )
    assert result["obstacle_passed"]
    assert math.isclose(result["forward_progress_m"], 0.45, abs_tol=1.0e-9)
    assert math.isclose(
        result["maximum_abs_lateral_deviation_m"], 0.08, abs_tol=1.0e-9
    )
    assert not result["box_collision_detected"]
