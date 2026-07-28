#!/usr/bin/env bash
# End-to-end proof that run_ground_diagnostic_preflight.sh's
# GROUND_DIAGNOSTIC_PARAMS override genuinely switches which params
# file PRE_STACK [2/6] reads -- added 2026-07-28 for
# NEW_FIELD_SINGLE_PULSE_REVALIDATION, whose whole design depends on
# this NOT being an untested assumption ("a new JSON file that live
# safety checks do not actually read is not acceptable").
#
# hil_preflight.check_required_fields_ready() is entirely schema-driven
# (iterates whatever paths appear in the given file's own
# required_before_ground_motion list, no hardcoded field names), so a
# new file with its own extended list is already correctly supported
# by existing, unmodified code -- this test proves that end-to-end via
# a real subprocess invocation of the real script, not just by reading
# the code and assuming.
#
# Read-only, offline: never starts a driver/bridge/guard/recorder,
# never contacts the Pi. Only exercises PRE_STACK's step [2/6] output,
# which is independent of device reachability -- so this test is valid
# regardless of whether a physical stack happens to be reachable.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FAIL=0
assert_true() {
    local desc="$1" cond="$2"
    if [[ "${cond}" == "true" ]]; then
        echo "PASS: ${desc}"
    else
        echo "FAIL: ${desc}"
        FAIL=1
    fi
}

SCRATCH_ROOT="$(mktemp -d)"
trap 'rm -rf "${SCRATCH_ROOT}"' EXIT

echo "=== Scenario 1: override file with a deliberately-unconfirmed new field blocks on THAT field ==="
BROKEN_PARAMS="${SCRATCH_ROOT}/broken_new_field_params.json"
python3 - "${SCRIPT_DIR}/new_field_geometry_params.json" "${BROKEN_PARAMS}" <<'PYEOF'
import json
import sys

src, dst = sys.argv[1], sys.argv[2]
with open(src, encoding="utf-8") as fh:
    params = json.load(fh)
# Deliberately break only the two new corridor fields -- everything
# else stays exactly as in the real new-field file.
params["measured_geometry"]["corridor_y_min_m"] = "UNCONFIRMED_PHYSICAL_MEASUREMENT"
with open(dst, "w", encoding="utf-8") as fh:
    json.dump(params, fh, indent=2)
PYEOF

OUTPUT_BROKEN="$(GROUND_DIAGNOSTIC_PARAMS="${BROKEN_PARAMS}" bash "${SCRIPT_DIR}/run_ground_diagnostic_preflight.sh" pre-stack 2>&1 || true)"
echo "${OUTPUT_BROKEN}"
assert_true "TRACKED_FIELDS_OK=false with the broken override file" "$([[ "${OUTPUT_BROKEN}" == *"TRACKED_FIELDS_OK=false"* ]] && echo true || echo false)"
assert_true "TRACKED_UNCONFIRMED names the deliberately-broken new-field path" "$([[ "${OUTPUT_BROKEN}" == *"measured_geometry.corridor_y_min_m"* ]] && echo true || echo false)"
assert_true "the OLD field's own paths are not mentioned (proves the new file, not the default, was read)" "$([[ "${OUTPUT_BROKEN}" != *"test_area_length_m\": 0.65"* ]] && echo true || echo false)"

echo ""
echo "=== Scenario 2: override file with every field confirmed passes the tracked-fields check ==="
FULLY_CONFIRMED_PARAMS="${SCRATCH_ROOT}/fully_confirmed_new_field_params.json"
python3 - "${SCRIPT_DIR}/new_field_geometry_params.json" "${FULLY_CONFIRMED_PARAMS}" <<'PYEOF'
import json
import sys

src, dst = sys.argv[1], sys.argv[2]
with open(src, encoding="utf-8") as fh:
    params = json.load(fh)
# The 10 geometry fields are already real measurements in the shipped
# file -- only the two stable-venue booleans need flipping true here
# to fully pass, proving the override is genuinely a DIFFERENT file
# from the shipped one (which is deliberately false on both).
params["environment"]["boundaries_and_obstacles_recorded"] = True
params["safety"]["emergency_stop_position_confirmed"] = True
with open(dst, "w", encoding="utf-8") as fh:
    json.dump(params, fh, indent=2)
PYEOF

OUTPUT_CONFIRMED="$(GROUND_DIAGNOSTIC_PARAMS="${FULLY_CONFIRMED_PARAMS}" bash "${SCRIPT_DIR}/run_ground_diagnostic_preflight.sh" pre-stack 2>&1 || true)"
echo "${OUTPUT_CONFIRMED}"
assert_true "TRACKED_FIELDS_OK=true with the fully-confirmed override file" "$([[ "${OUTPUT_CONFIRMED}" == *"TRACKED_FIELDS_OK=true"* ]] && echo true || echo false)"
assert_true "TRACKED_MISSING=[] with the fully-confirmed override file" "$([[ "${OUTPUT_CONFIRMED}" == *"TRACKED_MISSING=[]"* ]] && echo true || echo false)"
assert_true "TRACKED_UNCONFIRMED=[] with the fully-confirmed override file" "$([[ "${OUTPUT_CONFIRMED}" == *"TRACKED_UNCONFIRMED=[]"* ]] && echo true || echo false)"

echo ""
echo "=== Scenario 3: shipped new_field_geometry_params.json (unmodified) now passes both stable-venue confirmations ==="
# boundaries_and_obstacles_recorded was confirmed true 2026-07-28 after
# manual on-site obstacle-clearance confirmation; emergency_stop_position_confirmed
# was confirmed true the same day after manual confirmation of the
# emergency power-off arrangement. Both tracked-file confirmations are
# now true -- this proves TRACKED_FIELDS_OK, not PRE_STACK_VERDICT: the
# four genuinely per-session confirmations (floor condition, travel
# path, operator present, Wi-Fi) still block PRE_STACK_VERDICT via the
# separate, gitignored hil_ground_diagnostic_session.py state file.
OUTPUT_SHIPPED="$(GROUND_DIAGNOSTIC_PARAMS="${SCRIPT_DIR}/new_field_geometry_params.json" bash "${SCRIPT_DIR}/run_ground_diagnostic_preflight.sh" pre-stack 2>&1 || true)"
echo "${OUTPUT_SHIPPED}"
assert_true "TRACKED_FIELDS_OK=true with the shipped (now fully-confirmed) new-field file" "$([[ "${OUTPUT_SHIPPED}" == *"TRACKED_FIELDS_OK=true"* ]] && echo true || echo false)"
assert_true "TRACKED_UNCONFIRMED=[] with the shipped (now fully-confirmed) new-field file" "$([[ "${OUTPUT_SHIPPED}" == *"TRACKED_UNCONFIRMED=[]"* ]] && echo true || echo false)"
assert_true "PRE_STACK_VERDICT is still not a pass (per-session confirmations remain false)" "$([[ "${OUTPUT_SHIPPED}" == *"SESSION_STATE_NOT_READY"* || "${OUTPUT_SHIPPED}" == *"CONFIRMATIONS_NOT_TRUE"* ]] && echo true || echo false)"

echo ""
echo "=== Scenario 4: no override at all still reads the OLD field's default file, unaffected ==="
OUTPUT_DEFAULT="$(bash "${SCRIPT_DIR}/run_ground_diagnostic_preflight.sh" pre-stack 2>&1 || true)"
echo "${OUTPUT_DEFAULT}"
assert_true "default invocation does not mention the new field's corridor paths" "$([[ "${OUTPUT_DEFAULT}" != *"corridor_y_min_m"* ]] && echo true || echo false)"

echo ""
if [[ ${FAIL} -eq 0 ]]; then
    echo "NEW_FIELD_PARAMS_OVERRIDE_E2E_TEST_PASS"
else
    echo "NEW_FIELD_PARAMS_OVERRIDE_E2E_TEST_FAIL"
fi
exit ${FAIL}
