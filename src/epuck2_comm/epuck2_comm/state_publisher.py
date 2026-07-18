import math
import time
from functools import partial
from typing import Dict, Optional

import rclpy
from epuck2_comm_interfaces.msg import EpuckState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from .collision_math import local_to_global
from .models import RobotStateSnapshot
from .transmission_policy import EventTriggeredPolicy, PeriodicPolicy


IR_NAMES = tuple(f"ps{i}" for i in range(8))


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _finite_min(values) -> float:
    finite_values = [value for value in values if value is not None and math.isfinite(value) and value >= 0.0]
    return min(finite_values) if finite_values else math.inf


class StatePublisher(Node):
    def _now_s(self) -> float:
        """controller_v4_ros_time_consistency: ROS node clock (follows
        Webots sim time under use_sim_time=true) for sensor/odom freshness
        gating -- these flags (FLAG_ODOM_VALID/FLAG_IR_VALID/FLAG_TOF_VALID)
        feed cooperative_avoider.py's own safety checks, so both nodes must
        agree on which clock domain "fresh" is measured in."""
        return self.get_clock().now().nanoseconds / 1.0e9

    def __init__(self):
        super().__init__("state_publisher")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("source", "webots")
        self.declare_parameter("mode", "periodic")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("heartbeat_hz", 2.0)
        self.declare_parameter("odom_timeout_s", 0.5)
        self.declare_parameter("sensor_timeout_s", 0.5)
        self.declare_parameter("obstacle_distance_m", 0.055)
        self.declare_parameter("ir_no_detection_m", 0.060)
        self.declare_parameter("position_threshold_m", 0.01)
        self.declare_parameter("yaw_threshold_rad", 0.05)
        self.declare_parameter("linear_velocity_threshold_mps", 0.005)
        self.declare_parameter("angular_velocity_threshold_rps", 0.05)
        self.declare_parameter("distance_threshold_m", 0.01)
        self.declare_parameter("origin_x_m", 0.0)
        self.declare_parameter("origin_y_m", 0.0)
        self.declare_parameter("origin_yaw_rad", 0.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.source = self._source_code(str(self.get_parameter("source").value))
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        heartbeat_hz = max(0.1, float(self.get_parameter("heartbeat_hz").value))
        self.odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)
        self.sensor_timeout_s = float(self.get_parameter("sensor_timeout_s").value)
        self.obstacle_distance_m = float(self.get_parameter("obstacle_distance_m").value)
        self.ir_no_detection_m = float(self.get_parameter("ir_no_detection_m").value)
        self.origin_x_m = float(self.get_parameter("origin_x_m").value)
        self.origin_y_m = float(self.get_parameter("origin_y_m").value)
        self.origin_yaw_rad = float(self.get_parameter("origin_yaw_rad").value)

        minimum_interval_s = 1.0 / publish_rate_hz
        if self.mode == "periodic":
            self.policy = PeriodicPolicy(min_interval_s=minimum_interval_s)
        elif self.mode == "event":
            self.policy = EventTriggeredPolicy(
                min_interval_s=minimum_interval_s,
                heartbeat_interval_s=1.0 / heartbeat_hz,
                position_threshold_m=float(self.get_parameter("position_threshold_m").value),
                yaw_threshold_rad=float(self.get_parameter("yaw_threshold_rad").value),
                linear_velocity_threshold_mps=float(self.get_parameter("linear_velocity_threshold_mps").value),
                angular_velocity_threshold_rps=float(self.get_parameter("angular_velocity_threshold_rps").value),
                distance_threshold_m=float(self.get_parameter("distance_threshold_m").value),
            )
        else:
            raise ValueError("mode must be 'periodic' or 'event'")

        self.publisher = self.create_publisher(EpuckState, "state", 10)
        self.create_subscription(Odometry, "odom", self._odom_callback, qos_profile_sensor_data)
        for sensor_name in IR_NAMES:
            self.create_subscription(
                Range,
                sensor_name,
                partial(self._range_callback, sensor_name),
                qos_profile_sensor_data,
            )
        self.create_subscription(Range, "tof", partial(self._range_callback, "tof"), qos_profile_sensor_data)

        self.odom: Optional[Odometry] = None
        self.odom_received_at: Optional[float] = None
        self.ranges: Dict[str, float] = {}
        self.range_received_at: Dict[str, float] = {}
        self.sequence = 0
        self.published_count = 0
        self.start_time = time.monotonic()
        # protocol_v1.1_stamp_semantics: msg.stamp is only trustworthy once
        # the ROS clock has a valid, nonzero sample (under use_sim_time=true,
        # before the first /clock message this reads back as 0.0). Publishing
        # a formal EpuckState with a fake zero stamp would corrupt any
        # downstream latency calculation, so hold here instead -- same
        # principle as cooperative_avoider's WAITING_FOR_CLOCK gate, applied
        # to this node's own publish path (this node is not the frozen
        # controller, so this addition is in scope).
        self._clock_wait_logged = False

        self.timer = self.create_timer(min(0.02, minimum_interval_s), self._timer_callback)
        self.get_logger().info(
            f"state publisher robot_id={self.robot_id} mode={self.mode} "
            f"max_rate={publish_rate_hz:.1f}Hz heartbeat={heartbeat_hz:.1f}Hz"
        )
        self.get_logger().info(
            f"shared-frame origin=({self.origin_x_m:.3f}, {self.origin_y_m:.3f}, "
            f"{self.origin_yaw_rad:.3f}rad)"
        )

    @staticmethod
    def _source_code(value: str) -> int:
        mapping = {
            "unknown": EpuckState.SOURCE_UNKNOWN,
            "webots": EpuckState.SOURCE_WEBOTS,
            "hardware": EpuckState.SOURCE_HARDWARE,
            "virtual": EpuckState.SOURCE_VIRTUAL,
        }
        if value not in mapping:
            raise ValueError(f"unsupported source: {value}")
        return mapping[value]

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom = msg
        self.odom_received_at = self._now_s()

    def _range_callback(self, sensor_name: str, msg: Range) -> None:
        self.ranges[sensor_name] = float(msg.range)
        self.range_received_at[sensor_name] = self._now_s()

    def _is_fresh(self, received_at: Optional[float], timeout_s: float, now: float) -> bool:
        return received_at is not None and now - received_at <= timeout_s

    def _fresh_range(self, sensor_name: str, now: float) -> Optional[float]:
        if not self._is_fresh(self.range_received_at.get(sensor_name), self.sensor_timeout_s, now):
            return None
        value = self.ranges.get(sensor_name)
        if value is None or not math.isfinite(value) or value < 0.0:
            return None
        return value

    def _snapshot(self, now: float) -> RobotStateSnapshot:
        validity_flags = 0
        x_m = y_m = yaw_rad = 0.0
        linear_velocity_mps = angular_velocity_rps = 0.0

        if self.odom is not None and self._is_fresh(self.odom_received_at, self.odom_timeout_s, now):
            validity_flags |= EpuckState.FLAG_ODOM_VALID
            pose = self.odom.pose.pose
            twist = self.odom.twist.twist
            x_m, y_m, yaw_rad = local_to_global(
                float(pose.position.x),
                float(pose.position.y),
                _yaw_from_quaternion(pose.orientation),
                self.origin_x_m,
                self.origin_y_m,
                self.origin_yaw_rad,
            )
            linear_velocity_mps = float(twist.linear.x)
            angular_velocity_rps = float(twist.angular.z)

        raw_ir = {name: self._fresh_range(name, now) for name in IR_NAMES}
        if all(raw_ir[name] is not None for name in ("ps0", "ps1", "ps2", "ps5", "ps6", "ps7")):
            validity_flags |= EpuckState.FLAG_IR_VALID

        # The e-puck IR bridge reports a finite clear-space baseline (roughly
        # 0.07 m in the validated Webots and hardware data) rather than +Inf.
        # Convert values outside the useful near-obstacle region to +Inf so the
        # state message does not report a false obstacle in open space.
        filtered_ir = {
            name: (
                value
                if value is not None and value < self.ir_no_detection_m
                else math.inf
            )
            for name, value in raw_ir.items()
        }

        tof = self._fresh_range("tof", now)
        if tof is not None:
            validity_flags |= EpuckState.FLAG_TOF_VALID

        front_distance_m = _finite_min((filtered_ir["ps0"], filtered_ir["ps7"], tof))
        left_distance_m = _finite_min((filtered_ir["ps5"], filtered_ir["ps6"], filtered_ir["ps7"]))
        right_distance_m = _finite_min((filtered_ir["ps0"], filtered_ir["ps1"], filtered_ir["ps2"]))
        obstacle_status = self._classify_obstacle(front_distance_m, left_distance_m, right_distance_m)

        # controller_v4_full_sensor_bypass_20260717: zone split of the same
        # 8 raw ps readings, adding the two sensors (ps3, ps4 -- the
        # right-rear/left-rear pair) that front/left/right never read at
        # all. Zone assignment follows the standard e-puck ring (ps0/ps7
        # near front, ps1/ps6 front-diagonal, ps2/ps5 abeam, ps3/ps4 rear
        # diagonal): left_front=ps7, left_mid=min(ps5,ps6), left_rear=ps4,
        # right_front=ps0, right_mid=min(ps1,ps2), right_rear=ps3. This is
        # forensic ps0-ps7 mapping, not a new calibration -- same
        # ir_no_detection_m clear-space filtering as front/left/right.
        left_front_m = filtered_ir["ps7"]
        left_mid_m = _finite_min((filtered_ir["ps5"], filtered_ir["ps6"]))
        left_rear_m = filtered_ir["ps4"]
        right_front_m = filtered_ir["ps0"]
        right_mid_m = _finite_min((filtered_ir["ps1"], filtered_ir["ps2"]))
        right_rear_m = filtered_ir["ps3"]

        return RobotStateSnapshot(
            x_m=x_m,
            y_m=y_m,
            yaw_rad=yaw_rad,
            linear_velocity_mps=linear_velocity_mps,
            angular_velocity_rps=angular_velocity_rps,
            front_distance_m=front_distance_m,
            left_distance_m=left_distance_m,
            right_distance_m=right_distance_m,
            obstacle_status=obstacle_status,
            validity_flags=validity_flags,
            left_front_m=left_front_m,
            left_mid_m=left_mid_m,
            left_rear_m=left_rear_m,
            right_front_m=right_front_m,
            right_mid_m=right_mid_m,
            right_rear_m=right_rear_m,
        )

    def _classify_obstacle(self, front: float, left: float, right: float) -> int:
        close = [
            front < self.obstacle_distance_m,
            left < self.obstacle_distance_m,
            right < self.obstacle_distance_m,
        ]
        if sum(close) > 1:
            return EpuckState.OBSTACLE_MULTIPLE
        if close[0]:
            return EpuckState.OBSTACLE_FRONT
        if close[1]:
            return EpuckState.OBSTACLE_LEFT
        if close[2]:
            return EpuckState.OBSTACLE_RIGHT
        if all(math.isinf(value) for value in (front, left, right)):
            return EpuckState.OBSTACLE_UNKNOWN
        return EpuckState.OBSTACLE_CLEAR

    def _timer_callback(self) -> None:
        now = self._now_s()
        if now <= 0.0:
            if not self._clock_wait_logged:
                self.get_logger().info("WAITING_FOR_CLOCK: ROS clock not yet valid, holding state publication")
                self._clock_wait_logged = True
            return
        snapshot = self._snapshot(now)
        if not self.policy.should_publish(snapshot, now):
            return

        msg = EpuckState()
        msg.version = EpuckState.PROTOCOL_VERSION
        msg.source = self.source
        msg.robot_id = self.robot_id
        msg.sequence = self.sequence
        msg.stamp = self.get_clock().now().to_msg()
        msg.x_m = snapshot.x_m
        msg.y_m = snapshot.y_m
        msg.yaw_rad = snapshot.yaw_rad
        msg.linear_velocity_mps = snapshot.linear_velocity_mps
        msg.angular_velocity_rps = snapshot.angular_velocity_rps
        msg.front_distance_m = snapshot.front_distance_m
        msg.left_distance_m = snapshot.left_distance_m
        msg.right_distance_m = snapshot.right_distance_m
        msg.obstacle_status = snapshot.obstacle_status
        msg.validity_flags = snapshot.validity_flags
        msg.left_front_m = snapshot.left_front_m
        msg.left_mid_m = snapshot.left_mid_m
        msg.left_rear_m = snapshot.left_rear_m
        msg.right_front_m = snapshot.right_front_m
        msg.right_mid_m = snapshot.right_mid_m
        msg.right_rear_m = snapshot.right_rear_m
        self.publisher.publish(msg)

        self.policy.mark_published(snapshot, now)
        self.sequence = (self.sequence + 1) % (2**32)
        self.published_count += 1
        if self.published_count % 100 == 0:
            elapsed = max(0.001, now - self.start_time)
            self.get_logger().info(
                f"published={self.published_count} average_rate={self.published_count / elapsed:.2f}Hz "
                f"validity=0x{snapshot.validity_flags:02x}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
