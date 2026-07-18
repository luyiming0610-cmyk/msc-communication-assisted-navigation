"""ROS2 relay node for controlled communication impairment.

    /epuckN/state_raw  ->  network_impairment_relay  ->  /epuckN/state

Sits between the real state_publisher and the controller's peer-state
subscription. Applies fixed delay, optional symmetric jitter, and an
independent per-message drop probability -- all deterministic given a
fixed seed (see network_impairment.py for the pure decision logic this
wraps). NEVER modifies the frozen PROTOCOL_VERSION=1 EpuckState message
content; it only decides whether and when to forward the exact message it
received.

With delay_s=0, jitter_s=0, drop_probability=0 (the default), every
message is republished synchronously inside the subscription callback,
with no queueing and no added latency -- this default configuration must
be equivalent to a direct connection (no relay in the loop at all), which
the baseline pilot explicitly verifies.

Every subscription-namespace deployment uses one relay instance per robot
(remap `state_raw`/`state` via `-r __ns:=/epuckN`), and each instance logs
every message's outcome (received/forwarded/dropped, scheduled delay,
receive and release time) to a CSV via the `log_path` parameter -- the
impairment matrix's analysis compares this relay-side ground truth against
the consuming controller's own observed behaviour.
"""

import heapq

import rclpy
from rclpy.node import Node

from epuck2_comm_interfaces.msg import EpuckState

from .network_impairment import ImpairmentConfig, ImpairmentDecider


class NetworkImpairmentRelay(Node):
    def __init__(self):
        super().__init__("network_impairment_relay")
        self.declare_parameter("delay_s", 0.0)
        self.declare_parameter("jitter_s", 0.0)
        self.declare_parameter("drop_probability", 0.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("log_path", "")

        config = ImpairmentConfig(
            delay_s=float(self.get_parameter("delay_s").value),
            jitter_s=float(self.get_parameter("jitter_s").value),
            drop_probability=float(self.get_parameter("drop_probability").value),
            seed=int(self.get_parameter("seed").value),
        )
        self.decider = ImpairmentDecider(config)
        self._immediate_passthrough = self.decider.is_zero_impairment()

        self.publisher = self.create_publisher(EpuckState, "state", 20)
        self.create_subscription(EpuckState, "state_raw", self._on_message, 20)

        self._queue = []  # heap of (release_time_s, counter, msg, scheduled_delay_s, receive_time_s)
        self._counter = 0
        self.received_count = 0
        self.forwarded_count = 0
        self.dropped_count = 0

        log_path = str(self.get_parameter("log_path").value)
        self._log_file = open(log_path, "w", encoding="utf-8") if log_path else None
        if self._log_file:
            # source_stamp_s/actual_release_time_s appended (not inserted)
            # so existing readers keyed by column name (received_seq,
            # action, receive_time_s, release_time_s) are unaffected.
            # release_time_s remains the value SCHEDULED at receive time;
            # actual_release_time_s is when publish() actually ran (may
            # differ by up to one flush-timer period, 0.01s, for delayed
            # messages -- identical to release_time_s for immediate
            # passthrough, since that path is synchronous).
            self._log_file.write(
                "received_seq,action,scheduled_delay_s,receive_time_s,release_time_s,"
                "source_stamp_s,actual_release_time_s\n"
            )

        self.get_logger().info(
            f"network_impairment_relay: delay_s={config.delay_s:.4f} "
            f"jitter_s={config.jitter_s:.4f} drop_probability={config.drop_probability:.4f} "
            f"seed={config.seed} immediate_passthrough={self._immediate_passthrough}"
        )

        if not self._immediate_passthrough:
            self.timer = self.create_timer(0.01, self._flush_queue)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def _stamp_s(msg: EpuckState) -> float:
        return float(msg.stamp.sec) + float(msg.stamp.nanosec) / 1.0e9

    def _on_message(self, msg: EpuckState) -> None:
        self.received_count += 1
        decision = self.decider.decide()
        now = self._now_s()
        source_stamp = self._stamp_s(msg)
        if not decision.forward:
            self.dropped_count += 1
            self._log(msg.sequence, "dropped", 0.0, now, None, source_stamp, None)
            return
        if self._immediate_passthrough:
            self.publisher.publish(msg)
            self.forwarded_count += 1
            self._log(msg.sequence, "forwarded", 0.0, now, now, source_stamp, now)
            return
        release_time = now + decision.release_delay_s
        self._counter += 1
        heapq.heappush(
            self._queue,
            (release_time, self._counter, msg, decision.release_delay_s, now),
        )

    def _flush_queue(self) -> None:
        now = self._now_s()
        while self._queue and self._queue[0][0] <= now:
            release_time, _, msg, scheduled_delay, receive_time = heapq.heappop(self._queue)
            self.publisher.publish(msg)
            self.forwarded_count += 1
            actual_release_time = self._now_s()
            self._log(
                msg.sequence, "forwarded", scheduled_delay, receive_time, release_time,
                self._stamp_s(msg), actual_release_time,
            )

    def _log(
        self, seq, action, scheduled_delay_s, receive_time_s, release_time_s,
        source_stamp_s, actual_release_time_s,
    ) -> None:
        if self._log_file is None:
            return
        release_field = f"{release_time_s:.6f}" if release_time_s is not None else ""
        actual_release_field = f"{actual_release_time_s:.6f}" if actual_release_time_s is not None else ""
        self._log_file.write(
            f"{seq},{action},{scheduled_delay_s:.6f},{receive_time_s:.6f},{release_field},"
            f"{source_stamp_s:.6f},{actual_release_field}\n"
        )
        self._log_file.flush()

    def destroy_node(self) -> bool:
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = NetworkImpairmentRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
