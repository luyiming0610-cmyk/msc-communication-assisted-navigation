#!/usr/bin/env bash
# HIL shared-exit trial launcher.
#
# Modes (mutually exclusive):
#   --check-only   (DEFAULT if no mode flag is given) Runs hil_preflight.py's
#                   offline checks only. Never starts Webots, the driver,
#                   the bridge, SSH, any controller, rosbag, and never
#                   publishes cmd_vel or writes raw trial data.
#   --dry-run       Prints the sequence of steps a real run WOULD take
#                    (including the exact commands), without running any
#                    of them.
#
# There is no flag that starts ground motion. Until the lab
# wheel-suspension validation is complete (recorded by the user
# explicitly updating hil_frozen_params.json's field_geometry and
# hil_guard_limits.max_angular_speed_rps away from
# UNCONFIRMED_PHYSICAL_MEASUREMENT), this script always returns
# PHYSICAL_MOTION_LOCKED_UNTIL_LAB_VALIDATION for any mode that isn't
# --check-only/--dry-run. There is deliberately no bypass flag, hidden
# env var, or "force" option -- the only way past this gate is for the
# referenced JSON fields to actually contain real measurements.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
FROZEN_PARAMS="${HIL_ROOT}/hil_frozen_params.json"

MODE="--check-only"
if [[ $# -gt 0 ]]; then
    MODE="$1"
fi

case "${MODE}" in
    --check-only)
        echo "[run_hil_shared_exit_trial] MODE=--check-only (offline checks only, no processes started)"
        python3 "${SCRIPT_DIR}/hil_preflight.py" --frozen-params "${FROZEN_PARAMS}"
        exit $?
        ;;
    --dry-run)
        echo "[run_hil_shared_exit_trial] MODE=--dry-run (printing planned steps only, nothing executed)"
        cat <<'EOF'
Planned steps for a real HIL trial run (NONE of these are executed in --dry-run):
  1. Run hil_preflight.py offline checks (must return OFFLINE_CHECKS_PASS).
  2. Run a read-only physical connectivity check (network/driver/bridge/topic
     reachability only -- no nonzero cmd_vel, no navigation start).
  3. Require four explicit user confirmations (ROBOT_ON_STAND, WHEELS_CLEAR_OF_GROUND,
     USER_AT_EMERGENCY_STOP, TEST_AREA_CLEAR) before any nonzero /cmd_vel.
  4. Run the short suspended-wheel test sequence at capped low speed.
  5. Guide field-geometry measurement and freeze it into hil_frozen_params.json.
  6. Start hil_cmd_vel_guard.py (DISARMED), hil_topic_adapter.py, hil_virtual_peer.py.
  7. Start the recorder per hil_recorder_plan.py's topic list.
  8. Arm the guard only after every precondition above is satisfied, run
     exactly one EXCLUSIONARY_HIL_PILOT in HIL_COMM_OFF.
  9. Stop all motion nodes, verify zero velocity and clean process teardown.
 10. Only after a user-reviewed data audit, hand back (not run) the HIL_COMM_ON command.
EOF
        exit 0
        ;;
    *)
        echo "[run_hil_shared_exit_trial] PHYSICAL_MOTION_LOCKED_UNTIL_LAB_VALIDATION"
        echo "[run_hil_shared_exit_trial] Reason: mode '${MODE}' would require ground motion or a live physical session, and hil_frozen_params.json still has one or more required fields set to UNCONFIRMED_PHYSICAL_MEASUREMENT (see required_before_ground_motion). This launcher has no bypass for that condition."
        exit 2
        ;;
esac
