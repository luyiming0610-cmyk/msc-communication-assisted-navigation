#!/usr/bin/env python3
"""Pi-puck ROS 2 Foxy side of the e-puck TCP bridge."""

import json
import math
import socket
import threading
import time
from functools import partial

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


class PiEpuckTcpServer(Node):
    def __init__(self):
        super().__init__("pi_epuck_tcp_server_sensors")

        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("port", 5809)
        self.declare_parameter("state_rate_hz", 10.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("max_linear_mps", 0.04)
        self.declare_parameter("max_angular_rps", 2.0)

        self.listen_host = str(self.get_parameter("listen_host").value)
        self.port = int(self.get_parameter("port").value)
        self.state_rate_hz = max(1.0, float(self.get_parameter("state_rate_hz").value))
        self.command_timeout_s = max(0.1, float(self.get_parameter("command_timeout_s").value))
        self.max_linear_mps = abs(float(self.get_parameter("max_linear_mps").value))
        self.max_angular_rps = abs(float(self.get_parameter("max_angular_rps").value))

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
            "Pi TCP bridge listening on {}:{}; watchdog {:.2f}s; limits {:.3f}m/s {:.3f}rad/s".format(
                self.listen_host,
                self.port,
                self.command_timeout_s,
                self.max_linear_mps,
                self.max_angular_rps,
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
        try:
            linear = float(payload.get("linear_x", 0.0))
            angular = float(payload.get("angular_z", 0.0))
            seq = int(payload.get("seq", -1))
        except (TypeError, ValueError):
            return
        if not math.isfinite(linear) or not math.isfinite(angular):
            return

        linear = max(-self.max_linear_mps, min(self.max_linear_mps, linear))
        angular = max(-self.max_angular_rps, min(self.max_angular_rps, angular))
        with self._command_lock:
            self._command_linear = linear
            self._command_angular = angular
            self._command_seq = seq
            self._last_command_monotonic = time.monotonic()

    def _set_client_connected(self, connected):
        with self._command_lock:
            self._client_connected = bool(connected)
            if not connected:
                self._command_linear = 0.0
                self._command_angular = 0.0
                self._last_command_monotonic = 0.0

    def _command_timer(self):
        now = time.monotonic()
        with self._command_lock:
            fresh = (
                self._client_connected
                and self._last_command_monotonic > 0.0
                and now - self._last_command_monotonic <= self.command_timeout_s
            )
            linear = self._command_linear if fresh else 0.0
            angular = self._command_angular if fresh else 0.0

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)
        self._last_published_zero = linear == 0.0 and angular == 0.0

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

            self._client_socket = client
            client.settimeout(0.05)
            line_buffer = LineBuffer()
            self._set_client_connected(True)
            self.get_logger().info("TCP client connected from {}:{}".format(address[0], address[1]))
            next_send = 0.0

            try:
                while not self._stop_event.is_set():
                    try:
                        data = client.recv(65536)
                        if not data:
                            break
                        for line in line_buffer.feed(data):
                            try:
                                self._handle_command(decode_message_line(line))
                            except ProtocolError as exc:
                                self.get_logger().warning("Rejected TCP command: {}".format(exc))
                    except socket.timeout:
                        pass

                    now = time.monotonic()
                    if now >= next_send:
                        client.sendall(encode_message(self._state_payload()))
                        next_send = now + (1.0 / self.state_rate_hz)
            except (OSError, ProtocolError) as exc:
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
