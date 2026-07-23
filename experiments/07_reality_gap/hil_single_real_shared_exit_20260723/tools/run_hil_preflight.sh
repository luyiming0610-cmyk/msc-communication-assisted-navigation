#!/usr/bin/env bash
# Single, permanent preflight command a user runs before every HIL
# session. Combines the two existing preflight layers in sequence:
#   1. hil_preflight.py --frozen-params (offline: is every required
#      geometry/angular-cap field confirmed, not UNCONFIRMED_PHYSICAL_
#      MEASUREMENT?).
#   2. run_hil_physical_preflight.sh (read-only physical: is the
#      device reachable, driver/bridge/state healthy, validity_flags=7,
#      /cmd_vel currently at 0 publishers?).
#
# Read-only throughout. Never starts the driver, bridge, a controller,
# the guard, Webots, or rosbag, and never publishes /cmd_vel.
#
# Usage: run_hil_preflight.sh
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
FROZEN_PARAMS="${HIL_ROOT}/hil_frozen_params.json"

echo "=== [1/2] Offline check: hil_preflight.py ==="
OFFLINE_JSON="$(python3 "${SCRIPT_DIR}/hil_preflight.py" --frozen-params "${FROZEN_PARAMS}")"
echo "${OFFLINE_JSON}"
OFFLINE_STATUS="$(python3 -c "import json,sys; print(json.loads(sys.argv[1])['status'])" "${OFFLINE_JSON}" 2>/dev/null || echo "UNKNOWN")"
echo "offline_status=${OFFLINE_STATUS}"

echo ""
echo "=== [2/2] Physical read-only check: run_hil_physical_preflight.sh ==="
PHYSICAL_OUTPUT="$(bash "${SCRIPT_DIR}/run_hil_physical_preflight.sh" 2>&1)"
PHYSICAL_EXIT=$?
echo "${PHYSICAL_OUTPUT}"

echo ""
if [[ "${OFFLINE_STATUS}" == "OFFLINE_CHECKS_PASS" && "${PHYSICAL_EXIT}" -eq 0 ]]; then
    echo "HIL_PREFLIGHT_PASS"
    exit 0
else
    echo "HIL_PREFLIGHT_BLOCKED offline_status=${OFFLINE_STATUS} physical_exit=${PHYSICAL_EXIT}"
    exit 1
fi
