#!/usr/bin/env python3
"""WSL-side recorder for
physical_expanded_bridge_epuckstate_integration_pilot01_attempt01.

Read-only. Never publishes anything (including /cmd_vel). Subscribes to
/epuck_bridge/status (1Hz, carries the expanded bridge's own Pi-to-WSL
application-level sequence stats -- tier A) and records one CSV row per
status tick. Separately samples /cmd_vel publisher count (via
get_publishers_info_by_topic, a read-only graph query, not a subscription)
at caller-specified checkpoints (start/mid/end), writing them to a small
JSON file as they happen.

Tier B (state_publisher output -> bag, via EpuckState.sequence) and tier C
(raw sensor topic rates/gaps) are computed by the offline analyzer directly
from the rosbag, not by this live recorder -- avoids two different tools
disagreeing about the same bag-derived numbers.

Shutdown design (fixed after a real run hit a trailing "rcl_shutdown
already called" exception -- root-caused, not guessed):
rclpy.init()'s DEFAULT signal handling installs its own SIGINT handler
that can invalidate the rclpy context from a different execution path
while this script's main loop is still inside rclpy.spin_once() --
confirmed directly by a traceback raised FROM WITHIN spin_once() itself,
not from a plain Python KeyboardInterrupt at a loop boundary. The fix is
to become the single lifecycle owner: disable rclpy's automatic signal
handling (SignalHandlerOptions.NO) and install one plain `signal.signal`
handler that only ever sets a flag; the main loop polls that flag between
spin_once() calls, so a SIGINT can never land inside an in-flight rclpy
call. Shutdown itself is centralized in _shutdown_once(), guarded by a
local flag AND an rclpy.ok() check, called from exactly one place
(the `finally` block) on all three exit paths: SIGINT (flag set),
external context invalidation (rclpy.ok() goes false, loop exits on its
own), or a genuine unhandled exception (still re-raised after cleanup --
never swallowed).
"""
import argparse
import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


class ExpandedPilotRecorder(Node):
    def __init__(self, status_csv_path: str, cmd_vel_checkpoints_path: str):
        super().__init__("wsl_expanded_pilot_recorder")
        self._csv = open(status_csv_path, "w", encoding="utf-8")
        self._csv.write(
            "wsl_unix_time_s,connected,rx_count,crc_errors,last_rtt_ms,"
            "estimated_one_way_ms,wall_clock_delta_ms,last_state_age_s,"
            "state_seq_first,state_seq_last,state_unique_received,"
            "state_missing,state_out_of_order,state_delivery_ratio\n"
        )
        self._csv.flush()
        self._cmd_vel_checkpoints_path = cmd_vel_checkpoints_path
        self._checkpoints = []
        self.create_subscription(String, "/epuck_bridge/status", self._on_status, 10)
        self.get_logger().info(f"wsl_expanded_pilot_recorder writing to {status_csv_path}")

    def _on_status(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        self._csv.write(
            f"{now:.3f},{status.get('connected')},{status.get('rx_count')},"
            f"{status.get('crc_errors')},{status.get('last_rtt_ms')},"
            f"{status.get('estimated_one_way_ms')},{status.get('wall_clock_delta_ms')},"
            f"{status.get('last_state_age_s')},"
            f"{status.get('state_seq_first')},{status.get('state_seq_last')},"
            f"{status.get('state_unique_received')},{status.get('state_missing')},"
            f"{status.get('state_out_of_order')},{status.get('state_delivery_ratio')}\n"
        )
        self._csv.flush()

    def record_cmd_vel_checkpoint(self, label: str):
        infos = self.get_publishers_info_by_topic("/cmd_vel")
        entry = {
            "label": label,
            "unix_time_s": time.time(),
            "publisher_count": len(infos),
            "publisher_node_names": [f"{i.node_namespace}/{i.node_name}" for i in infos],
        }
        self._checkpoints.append(entry)
        with open(self._cmd_vel_checkpoints_path, "w", encoding="utf-8") as fh:
            json.dump(self._checkpoints, fh, indent=2)
        self.get_logger().info(f"cmd_vel checkpoint [{label}]: publisher_count={len(infos)}")

    def close(self):
        self._csv.close()


def run(node: ExpandedPilotRecorder, checkpoint_schedule_s, stop_requested, spin_once_fn=None):
    """The testable core loop, independent of argparse/signal setup. Returns
    the list of checkpoint labels actually recorded (for tests to assert
    against). `stop_requested` is a callable returning True once a stop has
    been requested (real SIGINT flag in production; a fake counter/flag in
    tests). `spin_once_fn` defaults to a real rclpy.spin_once call but can
    be swapped in tests to avoid needing a live ROS graph."""
    if spin_once_fn is None:
        def spin_once_fn():
            rclpy.spin_once(node, timeout_sec=0.2)

    labels = ["start", "mid", "end"]
    start = time.monotonic()
    next_idx = 0
    recorded = []
    while rclpy.ok() and not stop_requested():
        spin_once_fn()
        elapsed = time.monotonic() - start
        if next_idx < len(checkpoint_schedule_s) and elapsed >= checkpoint_schedule_s[next_idx]:
            label = labels[next_idx] if next_idx < len(labels) else f"checkpoint{next_idx}"
            node.record_cmd_vel_checkpoint(label)
            recorded.append(label)
            next_idx += 1
    if next_idx < len(checkpoint_schedule_s):
        try:
            node.record_cmd_vel_checkpoint("end_on_interrupt")
            recorded.append("end_on_interrupt")
        except Exception:
            pass
    return recorded


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--status-csv", required=True)
    parser.add_argument("--cmd-vel-checkpoints-json", required=True)
    parser.add_argument("--checkpoint-schedule-s", type=float, nargs="+", default=[0.0, 150.0, 299.0],
                         help="Seconds after start at which to sample /cmd_vel publisher count (start/mid/end).")
    args = parser.parse_args()

    # Single lifecycle owner: disable rclpy's own automatic SIGINT handling
    # (it was racing this script's manual spin_once loop -- see module
    # docstring) and install a plain flag-setting handler instead.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_flag = {"set": False}

    def _on_sigint(signum, frame):
        stop_flag["set"] = True

    signal.signal(signal.SIGINT, _on_sigint)

    node = ExpandedPilotRecorder(args.status_csv, args.cmd_vel_checkpoints_json)
    shutdown_done = {"done": False}

    def _shutdown_once():
        if shutdown_done["done"]:
            return
        shutdown_done["done"] = True
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    try:
        run(node, args.checkpoint_schedule_s, lambda: stop_flag["set"])
    finally:
        _shutdown_once()

    return 0


if __name__ == "__main__":
    sys.exit(main())
