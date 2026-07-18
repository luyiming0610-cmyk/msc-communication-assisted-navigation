#!/usr/bin/env python3
"""WSL-side recorder for physical_single_device_transport_diagnostic_pilot01.

Subscribes read-only to /epuck_bridge/status, /scan, /odom, /cmd_vel
(never publishes anything, never sends a command). Writes one CSV row per
/epuck_bridge/status message (it is published at 1Hz by the bridge) with
the bridge's own reported counters/RTT, plus running counts of /scan,
/odom, /cmd_vel messages observed in the same 1-second window.

IMPORTANT, matches a finding from reading wsl_epuck_tcp_bridge.py directly:
the currently-running BASE bridge does not expose a paired source sequence
number on any WSL-side topic (it uses the Pi's internal "seq" purely for
de-duplication before discarding it) -- so this recorder does NOT attempt
to compute sequence-gap/duplicate/out-of-order/PDR here. That must be
computed from the rosbag (recorded separately by the orchestration script)
against /epuck_bridge/status's rx_count growth, and even then is only a
coverage-style count, not a true paired-sequence PDR -- the analyzer marks
true PDR NOT_MEASURABLE for this reason, per instruction.
"""
import argparse
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class TransportRecorder(Node):
    def __init__(self, output_csv_path: str):
        super().__init__("wsl_transport_recorder")
        self._csv = open(output_csv_path, "w", encoding="utf-8")
        self._csv.write(
            "wsl_unix_time_s,connected,rx_count,crc_errors,last_rtt_ms,"
            "estimated_one_way_ms,wall_clock_delta_ms,last_state_age_s,"
            "scan_count_this_window,odom_count_this_window,"
            "nonzero_cmd_vel_count_this_window\n"
        )
        self._scan_count = 0
        self._odom_count = 0
        self._nonzero_cmd_vel_count = 0
        self._total_scan_count = 0
        self._total_odom_count = 0
        self._total_nonzero_cmd_vel_count = 0

        self.create_subscription(String, "/epuck_bridge/status", self._on_status, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.get_logger().info(f"wsl_transport_recorder writing to {output_csv_path}")

    def _on_scan(self, msg):
        self._scan_count += 1
        self._total_scan_count += 1

    def _on_odom(self, msg):
        self._odom_count += 1
        self._total_odom_count += 1

    def _on_cmd_vel(self, msg):
        # Read-only observation. This recorder never publishes /cmd_vel
        # itself; it only counts whether anything else on the ROS graph did.
        if abs(msg.linear.x) > 1e-9 or abs(msg.angular.z) > 1e-9:
            self._nonzero_cmd_vel_count += 1
            self._total_nonzero_cmd_vel_count += 1
            self.get_logger().warning(
                f"NONZERO /cmd_vel OBSERVED: linear.x={msg.linear.x} angular.z={msg.angular.z}"
            )

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
            f"{self._scan_count},{self._odom_count},{self._nonzero_cmd_vel_count}\n"
        )
        self._csv.flush()
        self._scan_count = 0
        self._odom_count = 0
        self._nonzero_cmd_vel_count = 0

    def totals(self):
        return {
            "total_scan_count": self._total_scan_count,
            "total_odom_count": self._total_odom_count,
            "total_nonzero_cmd_vel_count": self._total_nonzero_cmd_vel_count,
        }

    def close(self):
        self._csv.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--totals-json", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = TransportRecorder(args.output_csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        totals = node.totals()
        with open(args.totals_json, "w", encoding="utf-8") as fh:
            json.dump(totals, fh, indent=2)
        node.close()
        node.destroy_node()
        # rclpy's own SIGINT handler may already have called shutdown()
        # before this finally block runs (observed directly: "rcl_shutdown
        # already called on the given context" after a clean SIGINT stop,
        # with all data above already written correctly) -- same class of
        # bug already fixed in sequence_counter.py earlier this session.
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
