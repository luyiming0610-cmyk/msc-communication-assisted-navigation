#!/usr/bin/env bash
set -eo pipefail
#
# READ-ONLY diagnostic, NOT a pilot: launches the single-purpose
# sensor_ray_height_audit_world.wbt (epuck1 stationary, a single test
# marker at a configurable height directly 0.15m ahead) and prints
# epuck1's fused front_distance_m once state_publisher has produced a
# stable reading. No controller motion, no scoring, no bag/evidence
# directory -- a single empirical measurement to settle whether
# Webots' e-puck DistanceSensor ray-casting is height-selective, per
# the shared-exit-navigation exit-geometry redesign.
#
# Usage: sensor_ray_height_audit.sh

REPO="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm"
WORK_DIR="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/2-1.仿真通信实验/working"

source /opt/ros/humble/setup.bash
source "$HOME/epuck_ws/install/setup.bash"
export WEBOTS_HOME="/mnt/c/Program Files/Webots"
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib/controller:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$WEBOTS_HOME/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
set -u

RESIDUAL="$(pgrep -af 'webots-bin|state_publisher' 2>/dev/null | grep -v 'bash -lc' || true)"
if [[ -n "$RESIDUAL" ]]; then
  echo "RESIDUAL_PROCESSES_FOUND:" >&2
  echo "$RESIDUAL" >&2
  exit 1
fi

SIM_PID=""; STATE_PID=""
cleanup() {
  [[ -n "$STATE_PID" ]] && kill -INT "-$STATE_PID" 2>/dev/null || true
  sleep 0.5
  [[ -n "$SIM_PID" ]] && kill -INT "-$SIM_PID" 2>/dev/null || true
  sleep 1
  [[ -n "$STATE_PID" ]] && kill -KILL "-$STATE_PID" 2>/dev/null || true
  [[ -n "$SIM_PID" ]] && kill -KILL "-$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT

export EPUCK_WORLD_FILE="sensor_ray_height_audit_world.wbt"
setsid bash -c "cd '$WORK_DIR' && exec python3 run_shared_exit_n2.py" >/tmp/sensor_audit_sim.log 2>&1 &
SIM_PID=$!

deadline=$((SECONDS + 90))
while (( SECONDS < deadline )); do
  if ros2 topic list 2>/dev/null | grep -Fxq /epuck1/tof; then break; fi
  sleep 1
done
echo "[sensor topics ready]"

setsid ros2 run epuck2_comm state_publisher --ros-args \
  -r __ns:=/epuck1 -p robot_id:=1 -p use_sim_time:=true -p mode:=periodic \
  >/tmp/sensor_audit_state.log 2>&1 &
STATE_PID=$!

sleep 8
echo "=== epuck1/state (single sample) ==="
timeout 8 ros2 topic echo /epuck1/state --once || echo "NO SAMPLE RECEIVED"
