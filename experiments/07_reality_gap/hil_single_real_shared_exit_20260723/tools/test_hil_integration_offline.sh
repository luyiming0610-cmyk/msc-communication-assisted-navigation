#!/usr/bin/env bash
# Offline end-to-end HIL integration test. Uses ONLY test-namespaced
# topics (/hil_offline_test/...) for everything -- never /epuck1/state,
# never /cmd_vel_unguarded, never /cmd_vel, and never the guard's
# GLOBAL default arm topic (/hil_guard/arm) either: a real or another
# test's guard instance could be subscribed to that same global topic,
# so this script arms ONLY its own test-namespaced arm topic
# (/hil_offline_test/hil_guard/arm), passed explicitly via --arm-topic.
# The guard's final output in this test goes to a non-hardware sink
# topic (/hil_offline_test/cmd_vel_sink), never the real /cmd_vel. The
# real /cmd_vel's Publisher count is checked before and after and must
# stay 0 throughout.
#
# Never starts Webots, the real driver, the real bridge, or the real
# guard/controller against real topics. Never publishes to the real
# /cmd_vel. Safe to run at any time, on any machine with this ROS
# workspace sourced, whether or not the physical robot is connected.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash
set -u

FAIL=0
check() {
    local desc="$1" actual="$2" expected="$3"
    if [[ "$actual" == "$expected" ]]; then
        echo "PASS: $desc (got '$actual')"
    else
        echo "FAIL: $desc (expected '$expected', got '$actual')"
        FAIL=1
    fi
}

REAL_CMD_VEL_BEFORE="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo 0)"
echo "real_cmd_vel_publisher_count_before=${REAL_CMD_VEL_BEFORE}"

PIDS=()
cleanup() {
    for pid in "${PIDS[@]:-}"; do
        [[ -n "${pid}" ]] || continue
        kill -INT "${pid}" 2>/dev/null || true
    done
    for pid in "${PIDS[@]:-}"; do
        [[ -n "${pid}" ]] || continue
        wait "${pid}" 2>/dev/null || true
    done
    REAL_CMD_VEL_AFTER="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo 0)"
    echo "real_cmd_vel_publisher_count_after=${REAL_CMD_VEL_AFTER}"
    check "real /cmd_vel untouched throughout (after)" "${REAL_CMD_VEL_AFTER}" "0"
}
trap cleanup EXIT

start() {
    local name="$1"; shift
    "$@" > "/tmp/hil_offline_test_${name}.log" 2>&1 &
    PIDS+=("$!")
    echo "started ${name} pid=$!"
}

echo "=== Test 1: baseline armed pulse reaches the sink, never /cmd_vel ==="
start fake_state python3 "${SCRIPT_DIR}/hil_integration_test_fake_state_publisher.py" \
    --state-topic /hil_offline_test/epuck1/state --validity-flags 7
sleep 1

start guard python3 "${SCRIPT_DIR}/hil_cmd_vel_guard.py" \
    --physical-state-topic /hil_offline_test/epuck1/state \
    --upstream-cmd-vel-topic /hil_offline_test/cmd_vel_unguarded \
    --guarded-cmd-vel-topic /hil_offline_test/cmd_vel_sink \
    --arm-topic /hil_offline_test/hil_guard/arm \
    --max-linear-speed-mps 0.02 --max-angular-speed-rps 0.1 \
    --required-validity-flags 7
sleep 1

ros2 topic pub --once /hil_offline_test/hil_guard/arm std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
sleep 0.5

# Capture the WHOLE sink stream for the pulse's duration (not --once,
# which raced the disarm->armed/heartbeat-establishment window and
# could capture an earlier zero-hold sample instead of the pulse) --
# this proves the guard actually passes a legitimate command through
# when armed and healthy, not merely that it blocks bad ones.
timeout 6 ros2 topic echo /hil_offline_test/cmd_vel_sink > /tmp/hil_offline_sink_echo.log 2>&1 &
SINK_ECHO_PID=$!

python3 "${SCRIPT_DIR}/hil_wheel_suspension_test.py" \
    --upstream-cmd-vel-topic /hil_offline_test/cmd_vel_unguarded \
    --pulse-linear-mps 0.015 --zero-hold-s 1 --pulse-s 2 --post-hold-s 1 \
    > /tmp/hil_offline_test_pulse.log 2>&1
wait "${SINK_ECHO_PID}" 2>/dev/null || true

if grep -qE "x: 0\.01[0-9]" /tmp/hil_offline_sink_echo.log; then
    echo "PASS: guarded sink received a nonzero (clamped) command while armed and healthy"
else
    echo "FAIL: guarded sink never showed a nonzero command during the armed, healthy pulse window"
    FAIL=1
fi

REAL_CMD_VEL_MID="$(ros2 topic info /cmd_vel 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo 0)"
check "real /cmd_vel untouched during test 1" "${REAL_CMD_VEL_MID}" "0"

echo ""
echo "=== Test 2: validity_flags=0 fails closed ==="
kill -INT "${PIDS[0]}" 2>/dev/null || true  # stop fake_state (flags=7)
wait "${PIDS[0]}" 2>/dev/null || true
start fake_state_zero python3 "${SCRIPT_DIR}/hil_integration_test_fake_state_publisher.py" \
    --state-topic /hil_offline_test/epuck1/state --validity-flags 0
sleep 1
timeout 3 ros2 topic echo /hil_offline_test/cmd_vel_sink --once --field linear.x > /tmp/hil_offline_flags0.log 2>&1 || true
FLAGS0_VALUE="$(grep -m1 -E '^[0-9.-]+$' /tmp/hil_offline_flags0.log | tr -d '[:space:]' || echo "")"
check "sink linear.x is 0.0 with validity_flags=0" "${FLAGS0_VALUE}" "0.0"

echo ""
echo "=== Test 3: multiple upstream publishers fails closed ==="
start extra_upstream ros2 topic pub -r 10 /hil_offline_test/cmd_vel_unguarded geometry_msgs/msg/Twist "{linear: {x: 0.015}}"
sleep 1
timeout 3 ros2 topic echo /hil_offline_test/cmd_vel_sink --once --field linear.x > /tmp/hil_offline_multipub.log 2>&1 || true
MULTIPUB_VALUE="$(grep -m1 -E '^[0-9.-]+$' /tmp/hil_offline_multipub.log | tr -d '[:space:]' || echo "")"
check "sink linear.x is 0.0 with 2 upstream publishers" "${MULTIPUB_VALUE}" "0.0"

echo ""
echo "=== HIL_INTEGRATION_OFFLINE_TEST_$( [[ $FAIL -eq 0 ]] && echo PASS || echo FAIL ) ==="
exit "${FAIL}"
