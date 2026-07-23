#!/usr/bin/env python3
"""WSL ROS 2 Humble side of the e-puck TCP bridge -- INSTRUMENTED variant.

This file is a separate copy of wsl_epuck_tcp_bridge_sensors.py
(unmodified original preserved at its own path, SHA-256
09852be639f6d51a44e240134b8e1a2c7825639315ff5d07e5823c948b876bc0,
confirmed matching the currently-running PID 5477 on 2026-07-23,
before this file was written). It exists to investigate a repeatable
~33s, ~0.9-1.0s common-input-topic pause found by
TARGETED_STATIONARY_DIAGNOSTIC (see
experiments/07_reality_gap/hil_single_real_shared_exit_20260723/
targeted_stationary_diagnostic_20260723/SUMMARY.md in the main repo).

Every transport/publishing/watchdog/timing behavior is byte-for-byte
identical to the original -- the diagnostic additions below are pure
observation, gated behind a single `diagnostic_mode` parameter that
defaults to False (or the EPUCK_BRIDGE_DIAGNOSTIC=1 environment
variable). With diagnostic_mode off, this file behaves exactly like
the original (proven by test_wsl_epuck_tcp_bridge_sensors_instrumented.py's
default-off and unchanged-message-content tests).

Diagnostic additions, ALL gated behind diagnostic_mode, ALL read-only
observation (no gc.disable/gc.set_threshold/gc.collect call, no timer
period change, no transport/watchdog/threshold change):
  - monotonic timestamp of every successful TCP "state" payload decode
  - entry/exit timestamps and expected-vs-actual interval of the
    _publish_latest_state() 0.02s timer
  - entry timestamp and expected-vs-actual interval of the
    _publish_status() 1.0s timer
  - a gc.callbacks hook recording generation, start/end timestamp, and
    duration of every real Python garbage-collection cycle
  - process CPU time (time.process_time()) recorded alongside every
    wall/monotonic timestamp above, so a process-wide scheduling pause
    (wall time elapses, CPU time barely advances) can be distinguished
    from genuine CPU-bound work (both advance together)

Nothing is logged to the console per-event; all diagnostic rows are
buffered in memory and written to CSV exactly once, at shutdown
(mirrors the project's established hil_targeted_validity_diagnostic_
recorder.py pattern, for the same reason: the recorder's own I/O must
not itself introduce a periodic disturbance).
"""

import gc
import json
import math
import os
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String

from bridge_protocol import LineBuffer, ProtocolError, decode_message_line, encode_message


def _number(value, default=0.0):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def _restore_range(value):
    return float("inf") if value is None else float(value)


def _set_stamp(target, source):
    source = source or {}
    target.sec = int(source.get("sec", 0))
    target.nanosec = int(source.get("nanosec", 0))


def update_sequence_stats(last_seq, unique_received, missing, out_of_order, seq):
    """Update monotonic state-sequence delivery counters. Unchanged
    from the original file -- copied verbatim, not touched."""
    seq = int(seq)
    if seq < 0:
        return last_seq, unique_received, missing, out_of_order
    if last_seq is None:
        return seq, unique_received + 1, missing, out_of_order
    if seq > last_seq:
        missing += max(0, seq - last_seq - 1)
        return seq, unique_received + 1, missing, out_of_order
    return last_seq, unique_received, missing, out_of_order + 1


RANGE_SENSOR_NAMES = tuple(["ps{}".format(i) for i in range(8)] + ["tof"])


# --------------------------------------------------------------------
# Message-construction pure functions -- extracted VERBATIM from the
# original _publish_latest_state() body so their output is provably
# identical regardless of diagnostic_mode (test_*: unchanged-content).
# No ROS node/context is required to call these -- they only build
# message objects, so they are directly unit-testable offline.
# --------------------------------------------------------------------
def build_scan_msg(scan_data):
    msg = LaserScan()
    _set_stamp(msg.header.stamp, scan_data.get("stamp"))
    msg.header.frame_id = str(scan_data.get("frame_id", "base_scan"))
    msg.angle_min = _number(scan_data.get("angle_min"))
    msg.angle_max = _number(scan_data.get("angle_max"))
    msg.angle_increment = _number(scan_data.get("angle_increment"))
    msg.time_increment = _number(scan_data.get("time_increment"))
    msg.scan_time = _number(scan_data.get("scan_time"))
    msg.range_min = _number(scan_data.get("range_min"))
    msg.range_max = _number(scan_data.get("range_max"))
    msg.ranges = [_restore_range(v) for v in scan_data.get("ranges", [])]
    msg.intensities = [_number(v) for v in scan_data.get("intensities", [])]
    return msg


def build_odom_msg(odom_data):
    msg = Odometry()
    _set_stamp(msg.header.stamp, odom_data.get("stamp"))
    msg.header.frame_id = str(odom_data.get("frame_id", "odom"))
    msg.child_frame_id = str(odom_data.get("child_frame_id", "base_link"))

    position = odom_data.get("position") or {}
    orientation = odom_data.get("orientation") or {}
    linear = odom_data.get("linear") or {}
    angular = odom_data.get("angular") or {}
    msg.pose.pose.position.x = _number(position.get("x"))
    msg.pose.pose.position.y = _number(position.get("y"))
    msg.pose.pose.position.z = _number(position.get("z"))
    msg.pose.pose.orientation.x = _number(orientation.get("x"))
    msg.pose.pose.orientation.y = _number(orientation.get("y"))
    msg.pose.pose.orientation.z = _number(orientation.get("z"))
    msg.pose.pose.orientation.w = _number(orientation.get("w"), 1.0)
    msg.twist.twist.linear.x = _number(linear.get("x"))
    msg.twist.twist.linear.y = _number(linear.get("y"))
    msg.twist.twist.linear.z = _number(linear.get("z"))
    msg.twist.twist.angular.x = _number(angular.get("x"))
    msg.twist.twist.angular.y = _number(angular.get("y"))
    msg.twist.twist.angular.z = _number(angular.get("z"))

    pose_covariance = odom_data.get("pose_covariance") or []
    twist_covariance = odom_data.get("twist_covariance") or []
    if len(pose_covariance) == 36:
        msg.pose.covariance = [float(v) for v in pose_covariance]
    if len(twist_covariance) == 36:
        msg.twist.covariance = [float(v) for v in twist_covariance]
    return msg


def build_range_msg(sensor_name, sensor_data):
    msg = Range()
    _set_stamp(msg.header.stamp, sensor_data.get("stamp"))
    msg.header.frame_id = str(sensor_data.get("frame_id", sensor_name))
    msg.radiation_type = int(sensor_data.get("radiation_type", Range.INFRARED))
    msg.field_of_view = _number(sensor_data.get("field_of_view"))
    msg.min_range = _number(sensor_data.get("min_range"))
    msg.max_range = _number(sensor_data.get("max_range"))
    msg.range = _number(sensor_data.get("range"))
    return msg


# --------------------------------------------------------------------
# Pure diagnostic-recording logic -- no ROS dependency, independently
# unit-tested. Only ever appends to an in-memory list; the caller
# decides when (once, at shutdown) to flush it to disk.
# --------------------------------------------------------------------
DIAG_CSV_FIELDS = [
    "event",
    "monotonic_ns",
    "process_cpu_time_s",
    "generation",
    "expected_interval_s",
    "actual_interval_s",
    "duration_s",
]


class DiagnosticRecorder:
    def __init__(self):
        self.rows = []

    def record_tcp_received(self, monotonic_ns, process_cpu_time_s):
        self.rows.append({
            "event": "tcp_state_received",
            "monotonic_ns": monotonic_ns,
            "process_cpu_time_s": process_cpu_time_s,
            "generation": None,
            "expected_interval_s": None,
            "actual_interval_s": None,
            "duration_s": None,
        })

    def record_timer_span(self, event_name, entry_ns, exit_ns, process_cpu_time_s, expected_interval_s, actual_interval_s):
        self.rows.append({
            "event": event_name,
            "monotonic_ns": entry_ns,
            "process_cpu_time_s": process_cpu_time_s,
            "generation": None,
            "expected_interval_s": expected_interval_s,
            "actual_interval_s": actual_interval_s,
            "duration_s": None if exit_ns is None else (exit_ns - entry_ns) / 1e9,
        })

    def record_gc_event(self, generation, start_ns, end_ns, process_cpu_time_s):
        self.rows.append({
            "event": "gc_cycle",
            "monotonic_ns": start_ns,
            "process_cpu_time_s": process_cpu_time_s,
            "generation": generation,
            "expected_interval_s": None,
            "actual_interval_s": None,
            "duration_s": None if end_ns is None else (end_ns - start_ns) / 1e9,
        })

    def flush(self, path):
        import csv
        with open(path, "w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=DIAG_CSV_FIELDS)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(row)
        return len(self.rows)


def diagnostic_mode_enabled(param_value):
    """Single source of truth for whether diagnostic mode is on:
    the ROS parameter wins if explicitly set true; otherwise the
    EPUCK_BRIDGE_DIAGNOSTIC=1 environment variable can also enable it.
    Both default to OFF."""
    if bool(param_value):
        return True
    return os.environ.get("EPUCK_BRIDGE_DIAGNOSTIC", "0") == "1"


class WslEpuckTcpBridge(Node):
    def __init__(self):
        super().__init__("wsl_epuck_tcp_bridge_sensors_instrumented")

        self.declare_parameter("server_host", "192.168.137.71")
        self.declare_parameter("port", 5809)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("diagnostic_mode", False)
        self.declare_parameter("diagnostic_output_csv", "")

        self.server_host = str(self.get_parameter("server_host").value)
        self.port = int(self.get_parameter("port").value)
        self.command_timeout_s = max(0.1, float(self.get_parameter("command_timeout_s").value))
        self.reconnect_delay_s = max(0.2, float(self.get_parameter("reconnect_delay_s").value))

        self._diagnostic_mode = diagnostic_mode_enabled(self.get_parameter("diagnostic_mode").value)
        self._diagnostic_output_csv = str(self.get_parameter("diagnostic_output_csv").value)
        self._diag = DiagnosticRecorder() if self._diagnostic_mode else None
        self._last_publish_latest_state_ns = None
        self._last_publish_status_ns = None
        self._gc_start_ns = {}
        if self._diagnostic_mode:
            if not self._diagnostic_output_csv:
                raise ValueError("diagnostic_mode=true requires --diagnostic-output-csv / diagnostic_output_csv to be set")
            gc.callbacks.append(self._on_gc_event)
            self.get_logger().warn(
                f"DIAGNOSTIC_MODE_ENABLED output_csv={self._diagnostic_output_csv} "
                "-- transport/publishing/watchdog/timing behavior is unchanged; "
                "this only adds read-only observation."
            )

        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._range_pubs = {
            name: self.create_publisher(Range, "/" + name, 10)
            for name in RANGE_SENSOR_NAMES
        }
        self._status_pub = self.create_publisher(String, "/epuck_bridge/status", 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_callback, 10)

        self._command_lock = threading.Lock()
        self._command_linear = 0.0
        self._command_angular = 0.0
        self._last_command_monotonic = 0.0
        self._command_seq = 0

        self._pending_lock = threading.Lock()
        self._pending_commands = {}

        self._state_lock = threading.Lock()
        self._latest_state = None
        self._last_published_state_seq = -1

        self._stats_lock = threading.Lock()
        self._connected = False
        self._rx_count = 0
        self._crc_errors = 0
        self._state_seq_first = None
        self._state_seq_last = None
        self._state_unique_received = 0
        self._state_missing = 0
        self._state_out_of_order = 0
        self._last_rtt_ms = None
        self._last_clock_delta_ms = None
        self._last_state_time = 0.0

        self._stop_event = threading.Event()
        self._socket = None
        self._network_thread = threading.Thread(target=self._network_main)
        self._network_thread.daemon = True
        self._network_thread.start()

        self.create_timer(0.02, self._publish_latest_state)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            "WSL TCP bridge connecting to {}:{}; local command watchdog {:.2f}s".format(
                self.server_host, self.port, self.command_timeout_s
            )
        )

    def _cmd_callback(self, msg):
        linear = _number(msg.linear.x)
        angular = _number(msg.angular.z)
        with self._command_lock:
            self._command_linear = linear
            self._command_angular = angular
            self._last_command_monotonic = time.monotonic()

    def _command_payload(self, force_zero=False):
        with self._command_lock:
            fresh = time.monotonic() - self._last_command_monotonic <= self.command_timeout_s
            linear = self._command_linear if fresh and not force_zero else 0.0
            angular = self._command_angular if fresh and not force_zero else 0.0
            self._command_seq += 1
            seq = self._command_seq
        return {
            "type": "cmd_vel",
            "seq": seq,
            "sent_time": time.time(),
            "linear_x": linear,
            "angular_z": angular,
        }

    def _remember_command_sent(self, seq):
        with self._pending_lock:
            self._pending_commands[int(seq)] = time.monotonic()
            cutoff = int(seq) - 100
            for old_seq in list(self._pending_commands):
                if old_seq < cutoff:
                    del self._pending_commands[old_seq]

    def _record_state(self, payload):
        if payload.get("type") != "state":
            return
        try:
            state_seq = int(payload.get("seq", -1))
        except (TypeError, ValueError):
            state_seq = -1
        sent_time = _number(payload.get("sent_time"), time.time())
        clock_delta_ms = (time.time() - sent_time) * 1000.0
        try:
            ack_seq = int(payload.get("command_ack_seq", -1))
        except (TypeError, ValueError):
            ack_seq = -1
        rtt_ms = None
        with self._pending_lock:
            command_sent = self._pending_commands.pop(ack_seq, None)
            for old_seq in list(self._pending_commands):
                if old_seq < ack_seq:
                    del self._pending_commands[old_seq]
        if command_sent is not None:
            rtt_ms = (time.monotonic() - command_sent) * 1000.0
        with self._state_lock:
            self._latest_state = payload
        with self._stats_lock:
            self._rx_count += 1
            if state_seq >= 0 and self._state_seq_first is None:
                self._state_seq_first = state_seq
            (
                self._state_seq_last,
                self._state_unique_received,
                self._state_missing,
                self._state_out_of_order,
            ) = update_sequence_stats(
                self._state_seq_last,
                self._state_unique_received,
                self._state_missing,
                self._state_out_of_order,
                state_seq,
            )
            if rtt_ms is not None:
                self._last_rtt_ms = rtt_ms
            self._last_clock_delta_ms = clock_delta_ms
            self._last_state_time = time.time()

        if self._diag is not None:
            self._diag.record_tcp_received(time.monotonic_ns(), time.process_time())

    def _network_main(self):
        while not self._stop_event.is_set():
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            try:
                sock.connect((self.server_host, self.port))
                sock.settimeout(0.05)
                self._socket = sock
                with self._stats_lock:
                    self._connected = True
                self.get_logger().info("TCP bridge connected")

                line_buffer = LineBuffer()
                next_send = 0.0
                while not self._stop_event.is_set():
                    try:
                        data = sock.recv(65536)
                        if not data:
                            break
                        for line in line_buffer.feed(data):
                            try:
                                self._record_state(decode_message_line(line))
                            except ProtocolError as exc:
                                with self._stats_lock:
                                    self._crc_errors += 1
                                self.get_logger().warning("Rejected state message: {}".format(exc))
                    except socket.timeout:
                        pass

                    now = time.monotonic()
                    if now >= next_send:
                        command = self._command_payload()
                        sock.sendall(encode_message(command))
                        self._remember_command_sent(command["seq"])
                        next_send = now + 0.1
            except (OSError, ProtocolError) as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warning("TCP bridge unavailable: {}".format(exc))
            finally:
                if self._socket is sock:
                    try:
                        sock.sendall(encode_message(self._command_payload(force_zero=True)))
                    except (OSError, ProtocolError):
                        pass
                try:
                    sock.close()
                except OSError:
                    pass
                self._socket = None
                with self._stats_lock:
                    self._connected = False

            if not self._stop_event.wait(self.reconnect_delay_s):
                continue

    def _publish_latest_state(self):
        entry_ns = time.monotonic_ns() if self._diag is not None else None

        with self._state_lock:
            payload = self._latest_state
        if not payload:
            if self._diag is not None:
                self._record_publish_timer_span(entry_ns, 0.02)
            return
        seq = int(payload.get("seq", -1))
        if seq == self._last_published_state_seq:
            if self._diag is not None:
                self._record_publish_timer_span(entry_ns, 0.02)
            return
        self._last_published_state_seq = seq

        scan_data = payload.get("scan")
        if isinstance(scan_data, dict):
            self._scan_pub.publish(build_scan_msg(scan_data))

        odom_data = payload.get("odom")
        if isinstance(odom_data, dict):
            self._odom_pub.publish(build_odom_msg(odom_data))

        range_sensors = payload.get("range_sensors")
        if isinstance(range_sensors, dict):
            for sensor_name in RANGE_SENSOR_NAMES:
                sensor_data = range_sensors.get(sensor_name)
                if not isinstance(sensor_data, dict):
                    continue
                self._range_pubs[sensor_name].publish(build_range_msg(sensor_name, sensor_data))

        if self._diag is not None:
            self._record_publish_timer_span(entry_ns, 0.02)

    def _record_publish_timer_span(self, entry_ns, expected_interval_s):
        actual_interval_s = None
        if self._last_publish_latest_state_ns is not None:
            actual_interval_s = (entry_ns - self._last_publish_latest_state_ns) / 1e9
        self._last_publish_latest_state_ns = entry_ns
        self._diag.record_timer_span(
            "publish_latest_state", entry_ns, time.monotonic_ns(), time.process_time(),
            expected_interval_s, actual_interval_s,
        )

    def _publish_status(self):
        entry_ns = time.monotonic_ns() if self._diag is not None else None

        with self._stats_lock:
            state_expected = self._state_unique_received + self._state_missing
            status = {
                "connected": self._connected,
                "rx_count": self._rx_count,
                "crc_errors": self._crc_errors,
                "state_seq_first": self._state_seq_first,
                "state_seq_last": self._state_seq_last,
                "state_unique_received": self._state_unique_received,
                "state_missing": self._state_missing,
                "state_out_of_order": self._state_out_of_order,
                "state_delivery_ratio": None
                if state_expected == 0
                else self._state_unique_received / state_expected,
                "last_rtt_ms": self._last_rtt_ms,
                "estimated_one_way_ms": None if self._last_rtt_ms is None else self._last_rtt_ms / 2.0,
                "wall_clock_delta_ms": self._last_clock_delta_ms,
                "last_state_age_s": None if self._last_state_time == 0.0 else time.time() - self._last_state_time,
                "server": "{}:{}".format(self.server_host, self.port),
            }
        msg = String()
        msg.data = json.dumps(status, sort_keys=True)
        self._status_pub.publish(msg)

        if self._diag is not None:
            actual_interval_s = None
            if self._last_publish_status_ns is not None:
                actual_interval_s = (entry_ns - self._last_publish_status_ns) / 1e9
            self._last_publish_status_ns = entry_ns
            self._diag.record_timer_span(
                "publish_status", entry_ns, time.monotonic_ns(), time.process_time(),
                1.0, actual_interval_s,
            )

    def _on_gc_event(self, phase, info):
        """gc.callbacks hook -- read-only observation ONLY. Never
        calls gc.disable()/gc.set_threshold()/gc.collect() itself, and
        never suppresses or alters a real collection; it only records
        that one happened and how long it took."""
        generation = info.get("generation") if isinstance(info, dict) else None
        now_ns = time.monotonic_ns()
        if phase == "start":
            self._gc_start_ns[generation] = now_ns
        elif phase == "stop":
            start_ns = self._gc_start_ns.pop(generation, None)
            if self._diag is not None:
                self._diag.record_gc_event(generation, start_ns if start_ns is not None else now_ns, now_ns, time.process_time())

    def flush_diagnostics(self):
        if self._diag is None:
            return 0
        return self._diag.flush(self._diagnostic_output_csv)

    def stop(self):
        self._stop_event.set()
        sock = self._socket
        if sock is not None:
            try:
                sock.sendall(encode_message(self._command_payload(force_zero=True)))
            except (OSError, ProtocolError):
                pass
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._network_thread.is_alive():
            self._network_thread.join(timeout=2.0)
        if self._diag is not None:
            gc.callbacks.remove(self._on_gc_event)
            count = self.flush_diagnostics()
            self.get_logger().warn(f"DIAGNOSTIC_MODE_FLUSHED rows={count} output={self._diagnostic_output_csv}")


def main(args=None):
    rclpy.init(args=args)
    node = WslEpuckTcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
