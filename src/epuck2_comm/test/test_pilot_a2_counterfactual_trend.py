"""controller_v3_unified_encounter_20260717: pilot_a2 counterfactual
short-term trend check.

*** This is NOT a safety-acceptance test. ***  Per
controller_v3_unified_encounter_design_20260717.md section 9: pilot_a2's
recorded trajectory reflects controller_v2's actual commands; once
controller_v3 issues different commands, every subsequent position and
sensor reading would genuinely differ, and no replay of old data can know
what the real closed-loop sensor stream would have been under a new
policy. This test only checks a low-fidelity, short-horizon, comparative
trend against pilot_a2's own real recorded outcome. It never asserts an
absolute clearance bound. Only a real, Webots-Supervisor-instrumented
pilot_a3 run may be cited as evidence of the >=0.005m clearance gate
(design doc sections 6.2, 6.6, 13).

Ground-truth values below are taken directly from the forensic timeline
in controller_v3_unified_encounter_design_20260717.md section 1, itself
rebuilt from raw /epuck1/state and /epuck1/cmd_vel in bag
controller_v2_local_latch_20260717_static_box_pilot_a2 -- not re-derived
or approximated here.
"""

import math


BOX_X_M = -0.25
BOX_Y_M = 0.0
BOX_HALF_M = 0.03
ROBOT_RADIUS_M = 0.035


def _clearance(x, y):
    dx = max(abs(x - BOX_X_M) - BOX_HALF_M, 0.0)
    dy = max(abs(y - BOX_Y_M) - BOX_HALF_M, 0.0)
    return math.hypot(dx, dy) - ROBOT_RADIUS_M


# Real pilot_a2 (controller_v2) recorded state at the instant CAPPED_BYPASS
# was entered (t=24.673s bag-relative): x, y, yaw, and the raw left_distance_m
# reading at that exact tick (still inside the side_release_m=0.058 band,
# i.e. raw side is still "active" by decide_local_obstacle()'s own
# hysteresis rule).
CONSTRAINED_ENTRY_X = -0.309
CONSTRAINED_ENTRY_Y = -0.039
CONSTRAINED_ENTRY_YAW = -0.834
CONSTRAINED_ENTRY_LEFT_M = 0.051
SIDE_RELEASE_M = 0.058

# Real pilot_a2 recorded outcome ~1.25s later (t=25.921s, the recorded
# minimum-clearance instant) under controller_v2's actual CAPPED_BYPASS
# commands (linear_mps=0.012, angular_rps=0.0, constant, confirmed by the
# raw /epuck1/cmd_vel trace in the same window).
BASELINE_LATER_X = -0.301
BASELINE_LATER_Y = -0.049
BASELINE_WINDOW_S = 25.921 - 24.673


def test_baseline_matches_recorded_pilot_a2_clearance_regression():
    """Sanity check on the hardcoded ground truth itself: confirms the
    real pilot_a2 baseline really did get worse over this window, which is
    the whole reason this design exists. If this assertion ever fails, the
    hardcoded constants above have drifted from the source bag and must be
    re-extracted, not the test loosened."""
    entry_clearance = _clearance(CONSTRAINED_ENTRY_X, CONSTRAINED_ENTRY_Y)
    later_clearance = _clearance(BASELINE_LATER_X, BASELINE_LATER_Y)
    assert entry_clearance < 0.0  # already penetrating on entry
    assert later_clearance < entry_clearance  # got worse under v2's blind creep


def test_v3_hold_candidate_does_not_worsen_pilot_a2s_recorded_trend():
    """Counterfactual trend check, not a safety proof (see module
    docstring). Models the v3 CONSTRAINED policy applied from the exact
    recorded entry state: raw left_distance_m=0.051m is inside the
    side_release_m=0.058m hysteresis band, so decide_local_obstacle() would
    still report LOCAL_LEFT_SIDE active at this instant -> v3's CONSTRAINED
    policy commands HOLD (zero linear, zero angular), not the CREEP a
    from-scratch model of pilot_a2's own sensor trace would need to justify.
    Position is therefore frozen for this short window under the
    candidate policy."""
    assert CONSTRAINED_ENTRY_LEFT_M <= SIDE_RELEASE_M  # raw side still active -> HOLD

    entry_clearance = _clearance(CONSTRAINED_ENTRY_X, CONSTRAINED_ENTRY_Y)
    # HOLD: candidate position does not move for BASELINE_WINDOW_S.
    candidate_clearance = _clearance(CONSTRAINED_ENTRY_X, CONSTRAINED_ENTRY_Y)
    baseline_clearance = _clearance(BASELINE_LATER_X, BASELINE_LATER_Y)

    assert candidate_clearance == entry_clearance  # HOLD: unchanged, not worsened
    assert candidate_clearance >= baseline_clearance, (
        "counterfactual trend: HOLD candidate must not be worse than "
        "controller_v2's actual recorded CREEP-the-whole-time outcome "
        "over the same window"
    )
    # Explicitly NOT asserted here, and must never be cited as such:
    # candidate_clearance >= 0.005 (the formal safety gate) -- HOLD only
    # prevents further worsening, it does not recover positive clearance.
    # See design doc section 6.5/13: only pilot_a3 may certify that gate.
