#!/usr/bin/env bash
# HIL physical preflight -- STRICTLY READ-ONLY.
#
# Checks network reachability, driver/server/bridge process health, ROS
# topic presence/type, publisher/subscriber counts (including the
# final, driver-facing /cmd_vel topic), rates, validity_flags, bridge
# status, and residual processes. Reuses the exact check patterns
# already proven in this repository's physical scripts (confirmed by
# reading them directly): the /cmd_vel "Publisher count" grep from
# run_baseline_v1_trial_v2.sh and run_baseline_v1_batch_trials.sh, and
# the /epuck_bridge/status "connected" JSON check from
# run_expanded_bridge_epuckstate_pilot.sh and
# run_transport_diagnostic_pilot.sh.
#
# This script MUST NOT and DOES NOT: start the driver, start the
# bridge, SSH to the Pi, publish /cmd_vel, start a controller, start
# Webots, or start rosbag. It only reads already-running ROS graph
# state and network reachability. If the device is unreachable, it
# reports PHYSICAL_READ_ONLY_PREFLIGHT_BLOCKED_DEVICE_UNREACHABLE and
# exits nonzero -- this is a safety block, not treated as a script bug.
set -uo pipefail

PI_IP="${HIL_PI_IP:-192.168.137.71}"
STATE_TOPIC="${HIL_STATE_TOPIC:-/epuck1/state}"
CMD_VEL_TOPIC="${HIL_CMD_VEL_TOPIC:-/cmd_vel}"
BRIDGE_STATUS_TOPIC="${HIL_BRIDGE_STATUS_TOPIC:-/epuck_bridge/status}"

echo "=== HIL physical preflight (READ-ONLY: no driver/bridge/SSH/cmd_vel/controller/Webots/rosbag start) ==="

PHYSICAL_DEVICE_REACHABLE="NO"
if ping -c 2 -W 2 "$PI_IP" >/dev/null 2>&1; then
  PHYSICAL_DEVICE_REACHABLE="YES"
fi
echo "PHYSICAL_DEVICE_REACHABLE=$PHYSICAL_DEVICE_REACHABLE (pi_ip=$PI_IP)"

if [[ "$PHYSICAL_DEVICE_REACHABLE" != "YES" ]]; then
  echo "DRIVER_STATUS=NOT_CHECKED"
  echo "BRIDGE_STATUS=NOT_CHECKED"
  echo "SENSOR_TOPICS_READY=NOT_CHECKED"
  echo "VALIDITY_FLAGS=NOT_CHECKED"
  echo "CMD_VEL_PUBLISHER_COUNT=NOT_CHECKED"
  echo "CMD_VEL_PUBLISHERS=NOT_CHECKED"
  RESIDUAL="$(pgrep -af 'state_publisher|wsl_epuck_tcp_bridge|cooperative_avoider|goal_navigator|hil_cmd_vel_guard|hil_virtual_peer|hil_topic_adapter' 2>/dev/null | grep -v 'bash -lc' || true)"
  if [[ -n "$RESIDUAL" ]]; then
    echo "RESIDUAL_PROCESS_CHECK=FOUND"
    echo "$RESIDUAL"
  else
    echo "RESIDUAL_PROCESS_CHECK=CLEAN"
  fi
  echo "CLOCK_DOMAIN_STATUS=NOT_CHECKED"
  echo "PHYSICAL_READ_ONLY_PREFLIGHT_BLOCKED_DEVICE_UNREACHABLE"
  exit 3
fi

DRIVER_STATUS="UNKNOWN"
if command -v ros2 >/dev/null 2>&1; then
  SCAN_PUB_COUNT="$(ros2 topic info /scan 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo "")"
  if [[ "$SCAN_PUB_COUNT" == "1" ]]; then
    DRIVER_STATUS="RUNNING (/scan publisher count = 1)"
  elif [[ -z "$SCAN_PUB_COUNT" ]]; then
    DRIVER_STATUS="NOT_RUNNING (/scan not present)"
  else
    DRIVER_STATUS="UNEXPECTED (/scan publisher count = $SCAN_PUB_COUNT, expected 1)"
  fi
else
  DRIVER_STATUS="NOT_CHECKED (ros2 CLI not on PATH in this shell -- source /opt/ros/humble/setup.bash first)"
fi
echo "DRIVER_STATUS=$DRIVER_STATUS"

BRIDGE_STATUS="UNKNOWN"
if command -v ros2 >/dev/null 2>&1; then
  STATUS_JSON="$(timeout 3 ros2 topic echo "$BRIDGE_STATUS_TOPIC" --once --field data 2>/dev/null || true)"
  if [[ -z "$STATUS_JSON" ]]; then
    BRIDGE_STATUS="NOT_RUNNING ($BRIDGE_STATUS_TOPIC not publishing)"
  elif [[ "$STATUS_JSON" == *'"connected": true'* || "$STATUS_JSON" == *'"connected":true'* ]]; then
    BRIDGE_STATUS="CONNECTED"
  else
    BRIDGE_STATUS="RUNNING_NOT_CONNECTED"
  fi
  echo "EXPANDED_SERVER_STATUS=$BRIDGE_STATUS (inferred from $BRIDGE_STATUS_TOPIC connected field -- same signal used for both the Pi expanded TCP server and the WSL bridge in existing scripts, which cannot be independently distinguished from this topic alone)"
else
  echo "EXPANDED_SERVER_STATUS=NOT_CHECKED (ros2 CLI not on PATH)"
fi
echo "BRIDGE_STATUS=$BRIDGE_STATUS"

SENSOR_TOPICS_READY="NOT_CHECKED"
VALIDITY_FLAGS="NOT_CHECKED"
if command -v ros2 >/dev/null 2>&1; then
  MISSING=""
  for t in "$STATE_TOPIC" /odom /scan /tof /ps0 /ps1 /ps2 /ps3 /ps4 /ps5 /ps6 /ps7; do
    ros2 topic list 2>/dev/null | grep -qx "$t" || MISSING="$MISSING $t"
  done
  if [[ -z "$MISSING" ]]; then
    SENSOR_TOPICS_READY="YES"
  else
    SENSOR_TOPICS_READY="NO (missing:$MISSING)"
  fi
  echo "SENSOR_TOPICS_READY=$SENSOR_TOPICS_READY"

  FLAGS_RAW="$(timeout 4 ros2 topic echo "$STATE_TOPIC" --field validity_flags --once 2>/dev/null || true)"
  FLAGS_VALUE="$(grep -m1 -E '^[[:space:]]*[0-9]+[[:space:]]*$' <<<"$FLAGS_RAW" | tr -d '[:space:]' || true)"
  if [[ -n "$FLAGS_VALUE" ]]; then
    VALIDITY_FLAGS="$FLAGS_VALUE (7 = ODOM|IR|TOF all valid; anything else means the HIL guard's default required_validity_flags_for_motion=7 will not be satisfied)"
  else
    VALIDITY_FLAGS="NOT_AVAILABLE ($STATE_TOPIC not publishing or field unreadable)"
  fi
  echo "VALIDITY_FLAGS=$VALIDITY_FLAGS"

  for t in /odom /scan /tof; do
    HZ="$(timeout 4 ros2 topic hz "$t" --window 20 2>/dev/null | grep -m1 'average rate' || echo "NOT_AVAILABLE")"
    echo "TOPIC_RATE[$t]=$HZ"
  done
else
  echo "SENSOR_TOPICS_READY=NOT_CHECKED (ros2 CLI not on PATH)"
  echo "VALIDITY_FLAGS=NOT_CHECKED (ros2 CLI not on PATH)"
fi

CMD_VEL_PUBLISHER_COUNT="NOT_CHECKED"
if command -v ros2 >/dev/null 2>&1; then
  CMD_VEL_PUBLISHER_COUNT="$(ros2 topic info "$CMD_VEL_TOPIC" 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*' || echo "unknown")"
  echo "CMD_VEL_PUBLISHER_COUNT=$CMD_VEL_PUBLISHER_COUNT (topic=$CMD_VEL_TOPIC)"
  if [[ "$CMD_VEL_PUBLISHER_COUNT" == "0" || "$CMD_VEL_PUBLISHER_COUNT" == "unknown" ]]; then
    echo "CMD_VEL_PUBLISHERS=NONE"
  else
    echo "CMD_VEL_PUBLISHERS (node name / namespace, read-only graph query):"
    ros2 topic info -v "$CMD_VEL_TOPIC" 2>/dev/null | grep -A4 'Publisher count' || true
  fi
  if [[ "$CMD_VEL_PUBLISHER_COUNT" != "0" ]]; then
    echo "WARNING: $CMD_VEL_TOPIC already has $CMD_VEL_PUBLISHER_COUNT publisher(s) before any HIL node has started -- 0 is expected at this stage."
  fi
else
  echo "CMD_VEL_PUBLISHER_COUNT=NOT_CHECKED (ros2 CLI not on PATH)"
fi

RESIDUAL="$(pgrep -af 'state_publisher|wsl_epuck_tcp_bridge|cooperative_avoider|goal_navigator|hil_cmd_vel_guard|hil_virtual_peer|hil_topic_adapter' 2>/dev/null | grep -v 'bash -lc' || true)"
if [[ -n "$RESIDUAL" ]]; then
  echo "RESIDUAL_PROCESS_CHECK=FOUND"
  echo "$RESIDUAL"
else
  echo "RESIDUAL_PROCESS_CHECK=CLEAN"
fi

echo "CLOCK_DOMAIN_STATUS=SEE_HIL_DESIGN.md (real robot=wall_clock_use_sim_time_false, virtual peer=independent clock; freshness checks use receive-side timestamps only, no sync required)"

echo "STILL_UNCONFIRMED_PARAMETERS: see hil_frozen_params.json's required_before_ground_motion list (this script does not read or judge that file -- run_hil_shared_exit_trial.sh --check-only does)"

echo "PHYSICAL_READ_ONLY_PREFLIGHT_DONE"
exit 0
