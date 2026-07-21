#!/usr/bin/env python3
"""Select one conservative peer-state stream for the frozen pairwise controller.

The existing controller remains unchanged.  For each robot this node consumes
its own state and all teammate states, ranks genuine CPA conflicts, and
republishes exactly one teammate state.  If any configured teammate becomes
stale, output is withheld so the controller's existing stale-data stop engages.
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from epuck2_comm.collision_math import closest_point_of_approach, collision_risk, velocity_vector
from epuck2_comm_interfaces.msg import EpuckState


@dataclass(frozen=True)
class RankedPeer:
    robot_id: int
    message: EpuckState
    current_distance_m: float
    time_to_cpa_s: float
    distance_at_cpa_m: float
    is_risk: bool


def choose_peer(candidates):
    risky = [item for item in candidates if item.is_risk]
    if risky:
        return min(risky, key=lambda item: (item.time_to_cpa_s, item.distance_at_cpa_m, item.robot_id))
    if candidates:
        return min(candidates, key=lambda item: (item.current_distance_m, item.robot_id))
    return None


class MultiPeerSelector(Node):
    def __init__(self, args):
        super().__init__(f"multi_peer_selector_{args.robot_id}")
        self.args = args
        self.own = None
        self.own_at = None
        self.peers = {topic: [None, None] for topic in args.peer_topics}
        self.publisher = self.create_publisher(EpuckState, args.output_topic, 20)
        self.create_subscription(EpuckState, args.own_topic, self._own_callback, 20)
        for topic in args.peer_topics:
            self.create_subscription(EpuckState, topic, lambda msg, t=topic: self._peer_callback(t, msg), 20)
        self.last_selected = None
        self.last_stale_log = -1.0
        self.timer = self.create_timer(1.0 / args.rate_hz, self._tick)
        self.get_logger().info(
            f"MULTI_PEER_SELECTOR_READY robot_id={args.robot_id} own={args.own_topic} "
            f"peers={','.join(args.peer_topics)} output={args.output_topic}"
        )

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _own_callback(self, msg):
        self.own, self.own_at = msg, self._now()

    def _peer_callback(self, topic, msg):
        self.peers[topic] = [msg, self._now()]

    @staticmethod
    def _usable(msg):
        return (
            msg is not None
            and int(msg.version) == int(EpuckState.PROTOCOL_VERSION)
            and (int(msg.validity_flags) & int(EpuckState.FLAG_ODOM_VALID)) != 0
        )

    def _tick(self):
        now = self._now()
        if not self._usable(self.own) or self.own_at is None or now - self.own_at > self.args.timeout_s:
            return
        stale = [topic for topic, (_, at) in self.peers.items() if at is None or now - at > self.args.timeout_s]
        if stale:
            if now - self.last_stale_log >= 1.0:
                self.get_logger().warning(f"MULTI_PEER_STALE withholding_output topics={','.join(stale)}")
                self.last_stale_log = now
            return
        own_vx, own_vy = velocity_vector(self.own.linear_velocity_mps, self.own.yaw_rad)
        candidates = []
        for message, _ in self.peers.values():
            if not self._usable(message):
                return
            peer_vx, peer_vy = velocity_vector(message.linear_velocity_mps, message.yaw_rad)
            metrics = closest_point_of_approach(
                self.own.x_m, self.own.y_m, own_vx, own_vy,
                message.x_m, message.y_m, peer_vx, peer_vy,
                self.args.cpa_horizon_s,
            )
            candidates.append(RankedPeer(
                int(message.robot_id), message, metrics.current_distance_m,
                metrics.time_to_cpa_s, metrics.distance_at_cpa_m,
                collision_risk(
                    metrics,
                    horizon_s=self.args.cpa_horizon_s,
                    safety_radius_m=self.args.safety_radius_m,
                    trigger_distance_m=self.args.trigger_distance_m,
                ),
            ))
        selected = choose_peer(candidates)
        if selected is None:
            return
        self.publisher.publish(selected.message)
        reason = "CPA_RISK" if selected.is_risk else "NEAREST_FRESH"
        identity = (selected.robot_id, reason)
        if identity != self.last_selected:
            self.get_logger().info(
                f"MULTI_PEER_SELECTED peer_robot_id={selected.robot_id} reason={reason} "
                f"distance={selected.current_distance_m:.3f} tcpa={selected.time_to_cpa_s:.3f} "
                f"dcpa={selected.distance_at_cpa_m:.3f}"
            )
            self.last_selected = identity


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-id", type=int, required=True)
    parser.add_argument("--own-topic", required=True)
    parser.add_argument("--peer-topics", required=True)
    parser.add_argument("--output-topic", required=True)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--cpa-horizon-s", type=float, default=4.0)
    parser.add_argument("--safety-radius-m", type=float, default=0.14)
    parser.add_argument("--trigger-distance-m", type=float, default=0.34)
    args = parser.parse_args()
    args.peer_topics = [item for item in args.peer_topics.split(",") if item]
    if len(args.peer_topics) != 2:
        parser.error("N=3 requires exactly two peer topics")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = MultiPeerSelector(args)
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
