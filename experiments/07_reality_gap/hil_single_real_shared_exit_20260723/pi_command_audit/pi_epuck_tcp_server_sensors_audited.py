#!/usr/bin/env python3
"""Pi-puck ROS 2 Foxy side of the e-puck TCP bridge -- AUDITED VARIANT.

Proposed change only -- NOT deployed to the Pi. See PROVENANCE.md in
this directory for the original file's SHA-256 and an exact diff
summary against pi_epuck_tcp_server_sensors_original_mirror.py (a
byte-exact copy of the file currently running on the Pi).

Adds OPTIONAL, explicitly-enabled, timestamped command auditing,
requested after the two 2026-07-23 UNEXPECTED_PHYSICAL_MOTION safety
incidents to close the "no positive record of what was actually
forwarded to the Pi motor controller" evidence gap identified in both
incident records
(experiments/07_reality_gap/hil_single_real_shared_exit_20260723/
safety_incident_unexpected_motion_20260723/SUMMARY.md and
safety_incident_unexpected_motion_2_20260723/SUMMARY.md).

Design constraints (binding):
  - Disabled by default (`command_audit_enabled` ROS parameter,
    default False). When disabled, every code path is byte-identical
    to the original in ordering and effect -- the only difference is
    an early-return no-op check before each audit call site.
  - Transport, watchdog timeout logic, and clamping/motor behaviour are
    UNCHANGED. The audit layer only OBSERVES values already computed
    by the original logic; it never feeds back into
    `_command_linear`/`_command_angular`/`_cmd_pub`, and never
    publishes, retransmits, or replays anything.
  - All audit-record construction is done by pure functions
    (`parse_and_clamp_command`, `compute_zero_reason`, the `build_*`
    functions below) with no I/O and no rclpy dependency, so the
    entire audit-record contract can be unit tested without a ROS
    graph or a real socket -- mirroring this project's established
    pattern (hil_cmd_vel_guard.py's pure decide_command() +thin Node
    wrapper).
"""
from __future__ import annotations

import json
import math
import socket
import threading
import time
from dataclasses import dataclass, field
from functools import partial
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range

from bridge_protocol import LineBuffer, ProtocolError, decode_message_line, encode_message


def _finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def _stamp_dict(stamp):
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


RANGE_SENSOR_NAMES = tuple(["ps{}".format(i) for i in range(8)] + ["tof"])


# --------------------------------------------------------------------
# Pure, rclpy-free command-audit logic. Every function here is a plain
# transformation of plain data -- no sockets, no ROS, no file I/O, no
# mutation of anything outside its own return value.
# --------------------------------------------------------------------


@dataclass(frozen=True)
class ParsedCommand:
    linear_raw: float
    angular_raw: float
    linear_applied: float
    angular_applied: float
    seq: int
    clamped: bool


def parse_and_clamp_command(
    payload, max_linear_mps: float, max_angular_rps: float
) -> Optional[ParsedCommand]:
    """Identical parsing/clamping logic to the original
    _handle_command's inline code, extracted as a pure function so it
    can be unit tested directly. Returns None for anything the
    original would have silently rejected (wrong type, non-numeric,
    non-finite) -- never raises."""
    if not isinstance(payload, dict) or payload.get("type") != "cmd_vel":
        return None
    try:
        linear_raw = float(payload.get("linear_x", 0.0))
        angular_raw = float(payload.get("angular_z", 0.0))
        seq = int(payload.get("seq", -1))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(linear_raw) or not math.isfinite(angular_raw):
        return None

    linear_applied = max(-max_linear_mps, min(max_linear_mps, linear_raw))
    angular_applied = max(-max_angular_rps, min(max_angular_rps, angular_raw))
    clamped = (linear_applied != linear_raw) or (angular_applied != angular_raw)
    return ParsedCommand(
        linear_raw=linear_raw,
        angular_raw=angular_raw,
        linear_applied=linear_applied,
        angular_applied=angular_applied,
        seq=seq,
        clamped=clamped,
    )


def compute_zero_reason(
    *,
    client_connected: bool,
    last_command_monotonic: float,
    now_monotonic: float,
    command_timeout_s: float,
    linear: float,
    angular: float,
) -> Optional[str]:
    """Reason a tick's applied command is zero, or None if it is a
    genuine nonzero command. Mirrors the original _command_timer's
    `fresh` computation exactly -- this is an audit ANNOTATION of that
    same logic, not a new or different gating rule."""
    if not client_connected:
        return "DISCONNECTED"
    if last_command_monotonic <= 0.0:
        return "NEVER_RECEIVED"
    if now_monotonic - last_command_monotonic > command_timeout_s:
        return "WATCHDOG_STALE_TIMEOUT"
    if linear == 0.0 and angular == 0.0:
        return "COMMANDED_ZERO"
    return None


def build_command_received_record(
    *, wall_time: float, monotonic_time: float, connection_id: int, parsed: ParsedCommand
) -> dict:
    return {
        "event": "command_received",
        "wall_time": wall_time,
        "monotonic_time": monotonic_time,
        "connection_id": connection_id,
        "seq": parsed.seq,
        "linear_raw": parsed.linear_raw,
        "angular_raw": parsed.angular_raw,
        "linear_applied_clamped": parsed.linear_applied,
        "angular_applied_clamped": parsed.angular_applied,
        "clamped": parsed.clamped,
    }


def build_command_rejected_record(
    *, wall_time: float, monotonic_time: float, connection_id: int, reason: str
) -> dict:
    return {
        "event": "command_rejected_malformed",
        "wall_time": wall_time,
        "monotonic_time": monotonic_time,
        "connection_id": connection_id,
        "reason": reason,
    }


def build_tick_applied_record(
    *,
    wall_time: float,
    monotonic_time: float,
    connection_id: int,
    linear: float,
    angular: float,
    zero_reason: Optional[str],
) -> dict:
    return {
        "event": "tick_applied",
        "wall_time": wall_time,
        "monotonic_time": monotonic_time,
        "connection_id": connection_id,
        "linear": linear,
        "angular": angular,
        "zero_reason": zero_reason,
    }


def build_socket_connected_record(
    *, wall_time: float, monotonic_time: float, connection_id: int, peer: str
) -> dict:
    return {
        "event": "socket_connected",
        "wall_time": wall_time,
        "monotonic_time": monotonic_time,
        "connection_id": connection_id,
        "peer": peer,
    }


def build_socket_disconnected_record(
    *, wall_time: float, monotonic_time: float, connection_id: int, reason: str
) -> dict:
    return {
        "event": "socket_disconnected",
        "wall_time": wall_time,
        "monotonic_time": monotonic_time,
        "connection_id": connection_id,
        "reason": reason,
    }


class CommandAuditSink:
    """Thin, injectable append-only sink for audit records. Writes one
    JSON line per record and flushes immediately (append-only, safe
    for a crash/kill -INT mid-session to still leave every prior
    record intact). Never reads, never seeks, never truncates -- pure
    write-only observation. Passing `enabled=False` makes every method
    a zero-cost no-op, matching the original's behaviour exactly."""

    def __init__(self, path: Optional[str], enabled: bool):
        self._enabled = bool(enabled and path)
        self._path = path
        self._lock = threading.Lock()
        self._fh = None
        if self._enabled:
            self._fh = open(self._path, "a", encoding="utf-8")

    @property
    def enabled(self) -> bool:
        return self._enabled

    def write(self, record: dict) -> None:
        if not self._enabled:
            return
        line = json.dumps(record, sort_keys=True) + "\n"
        with self._lock:
            self._fh.write(line)
            self._fh.flush()

    def close(self) -> None:
        if self._fh is not None:
            with self._lock:
                self._fh.close()
            self._fh = None


class PiEpuckTcpServer(Node):
    def __init__(self):
        super().__init__("pi_epuck_tcp_server_sensors")

        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("port", 5809)
        self.declare_parameter("state_rate_hz", 10.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("max_linear_mps", 0.04)
        self.declare_parameter("max_angular_rps", 2.0)
        # New, optional, disabled-by-default audit parameters.
        self.declare_parameter("command_audit_enabled", False)
        self.declare_parameter("command_audit_path", "")

        self.listen_host = str(self.get_parameter("listen_host").value)
        self.port = int(self.get_parameter("port").value)
        self.state_rate_hz = max(1.0, float(self.get_parameter("state_rate_hz").value))
        self.command_timeout_s = max(0.1, float(self.get_parameter("command_timeout_s").value))
        self.max_linear_mps = abs(float(self.get_parameter("max_linear_mps").value))
        self.max_angular_rps = abs(float(self.get_parameter("max_angular_rps").value))

        audit_enabled = bool(self.get_parameter("command_audit_enabled").value)
        audit_path = str(self.get_parameter("command_audit_path").value) or None
        self._audit = CommandAuditSink(audit_path, audit_enabled)

        self._state_lock = threading.Lock()
        self._latest_scan = None
        self._latest_odom = None
        self._latest_range_sensors = {}
        self._state_seq = 0

        self._command_lock = threading.Lock()
        self._command_linear = 0.0
        self._command_angular = 0.0
        self._command_seq = -1
        self._last_command_monotonic = 0.0
        self._client_connected = False
        self._last_published_zero = True
        self._connection_id = 0

        self._stop_event = threading.Event()
        self._server_socket = None
        self._client_socket = None

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom_callback, qos_profile_sensor_data)
        for sensor_name in RANGE_SENSOR_NAMES:
            self.create_subscription(
                Range,
                "/" + sensor_name,
                partial(self._range_callback, sensor_name),
                qos_profile_sensor_data,
            )
        self.create_timer(0.05, self._command_timer)

        self._network_thread = threading.Thread(target=self._network_main)
        self._network_thread.daemon = True
        self._network_thread.start()

        self.get_logger().info(
            "Pi TCP bridge listening on {}:{}; watchdog {:.2f}s; limits {:.3f}m/s {:.3f}rad/s; "
            "command_audit_enabled={}".format(
                self.listen_host,
                self.port,
                self.command_timeout_s,
                self.max_linear_mps,
                self.max_angular_rps,
                self._audit.enabled,
            )
        )

    def _scan_callback(self, msg):
        data = {
            "stamp": _stamp_dict(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "angle_min": float(msg.angle_min),
            "angle_max": float(msg.angle_max),
            "angle_increment": float(msg.angle_increment),
            "time_increment": float(msg.time_increment),
            "scan_time": float(msg.scan_time),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "ranges": [_finite_or_none(v) for v in msg.ranges],
            "intensities": [_finite_or_none(v) for v in msg.intensities],
        }
        with self._state_lock:
            self._latest_scan = data

    def _odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        data = {
            "stamp": _stamp_dict(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "position": {"x": float(p.x), "y": float(p.y), "z": float(p.z)},
            "orientation": {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)},
            "pose_covariance": [float(v) for v in msg.pose.covariance],
            "linear": {"x": float(linear.x), "y": float(linear.y), "z": float(linear.z)},
            "angular": {"x": float(angular.x), "y": float(angular.y), "z": float(angular.z)},
            "twist_covariance": [float(v) for v in msg.twist.covariance],
        }
        with self._state_lock:
            self._latest_odom = data

    def _range_callback(self, sensor_name, msg):
        data = {
            "stamp": _stamp_dict(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "radiation_type": int(msg.radiation_type),
            "field_of_view": _finite_or_none(msg.field_of_view),
            "min_range": _finite_or_none(msg.min_range),
            "max_range": _finite_or_none(msg.max_range),
            "range": _finite_or_none(msg.range),
        }
        with self._state_lock:
            self._latest_range_sensors[sensor_name] = data

    def _state_payload(self):
        with self._state_lock:
            scan = self._latest_scan
            odom = self._latest_odom
            range_sensors = dict(self._latest_range_sensors)
            seq = self._state_seq
            self._state_seq += 1
        with self._command_lock:
            command_seq = self._command_seq

        return {
            "type": "state",
            "robot_id": "epuck1",
            "source": "pi_foxy",
            "seq": seq,
            "sent_time": time.time(),
            "command_ack_seq": command_seq,
            "scan": scan,
            "odom": odom,
            "range_sensors": range_sensors,
        }

    def _handle_command(self, payload):
        if payload.get("type") != "cmd_vel":
            return
        connection_id = self._connection_id
        parsed = parse_and_clamp_command(payload, self.max_linear_mps, self.max_angular_rps)
        if parsed is None:
            if self._audit.enabled:
                self._audit.write(
                    build_command_rejected_record(
                        wall_time=time.time(),
                        monotonic_time=time.monotonic(),
                        connection_id=connection_id,
                        reason="unparseable_or_non_finite",
                    )
                )
            return

        with self._command_lock:
            self._command_linear = parsed.linear_applied
            self._command_angular = parsed.angular_applied
            self._command_seq = parsed.seq
            self._last_command_monotonic = time.monotonic()

        if self._audit.enabled:
            self._audit.write(
                build_command_received_record(
                    wall_time=time.time(),
                    monotonic_time=time.monotonic(),
                    connection_id=connection_id,
                    parsed=parsed,
                )
            )

    def _set_client_connected(self, connected):
        with self._command_lock:
            self._client_connected = bool(connected)
            if not connected:
                self._command_linear = 0.0
                self._command_angular = 0.0
                self._last_command_monotonic = 0.0

    def _command_timer(self):
        now = time.monotonic()
        connection_id = self._connection_id
        with self._command_lock:
            client_connected = self._client_connected
            last_command_monotonic = self._last_command_monotonic
            fresh = (
                client_connected
                and last_command_monotonic > 0.0
                and now - last_command_monotonic <= self.command_timeout_s
            )
            linear = self._command_linear if fresh else 0.0
            angular = self._command_angular if fresh else 0.0

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)
        self._last_published_zero = linear == 0.0 and angular == 0.0

        if self._audit.enabled:
            zero_reason = compute_zero_reason(
                client_connected=client_connected,
                last_command_monotonic=last_command_monotonic,
                now_monotonic=now,
                command_timeout_s=self.command_timeout_s,
                linear=linear,
                angular=angular,
            )
            self._audit.write(
                build_tick_applied_record(
                    wall_time=time.time(),
                    monotonic_time=now,
                    connection_id=connection_id,
                    linear=linear,
                    angular=angular,
                    zero_reason=zero_reason,
                )
            )

    def _network_main(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket = server
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((self.listen_host, self.port))
            server.listen(1)
            server.settimeout(1.0)
        except OSError as exc:
            self.get_logger().error("TCP bridge could not listen: {}".format(exc))
            return

        while not self._stop_event.is_set():
            try:
                client, address = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self._connection_id += 1
            connection_id = self._connection_id
            self._client_socket = client
            client.settimeout(0.05)
            line_buffer = LineBuffer()
            self._set_client_connected(True)
            self.get_logger().info("TCP client connected from {}:{}".format(address[0], address[1]))
            if self._audit.enabled:
                self._audit.write(
                    build_socket_connected_record(
                        wall_time=time.time(),
                        monotonic_time=time.monotonic(),
                        connection_id=connection_id,
                        peer="{}:{}".format(address[0], address[1]),
                    )
                )
            next_send = 0.0

            disconnect_reason = "closed_cleanly"
            try:
                while not self._stop_event.is_set():
                    try:
                        data = client.recv(65536)
                        if not data:
                            disconnect_reason = "peer_closed"
                            break
                        for line in line_buffer.feed(data):
                            try:
                                self._handle_command(decode_message_line(line))
                            except ProtocolError as exc:
                                self.get_logger().warning("Rejected TCP command: {}".format(exc))
                                if self._audit.enabled:
                                    self._audit.write(
                                        build_command_rejected_record(
                                            wall_time=time.time(),
                                            monotonic_time=time.monotonic(),
                                            connection_id=connection_id,
                                            reason="protocol_error:{}".format(exc),
                                        )
                                    )
                    except socket.timeout:
                        pass

                    now = time.monotonic()
                    if now >= next_send:
                        client.sendall(encode_message(self._state_payload()))
                        next_send = now + (1.0 / self.state_rate_hz)
            except (OSError, ProtocolError) as exc:
                disconnect_reason = "error:{}".format(exc)
                if not self._stop_event.is_set():
                    self.get_logger().warning("TCP client disconnected: {}".format(exc))
            finally:
                self._set_client_connected(False)
                try:
                    client.close()
                except OSError:
                    pass
                self._client_socket = None
                self.get_logger().info("TCP client closed; command watchdog forced zero")
                if self._audit.enabled:
                    self._audit.write(
                        build_socket_disconnected_record(
                            wall_time=time.time(),
                            monotonic_time=time.monotonic(),
                            connection_id=connection_id,
                            reason=disconnect_reason,
                        )
                    )

        try:
            server.close()
        except OSError:
            pass

    def stop(self):
        self._set_client_connected(False)
        zero = Twist()
        for _ in range(3):
            self._cmd_pub.publish(zero)
            time.sleep(0.05)
        self._stop_event.set()
        for sock in (self._client_socket, self._server_socket):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass
        if self._network_thread.is_alive():
            self._network_thread.join(timeout=2.0)
        self._audit.close()


def main(args=None):
    rclpy.init(args=args)
    node = PiEpuckTcpServer()
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
