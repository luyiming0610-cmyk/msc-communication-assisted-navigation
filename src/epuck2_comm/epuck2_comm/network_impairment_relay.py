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

v1.1: also supports a deterministic burst/outage window (see
`network_impairment.ImpairmentConfig`'s `outage_period_s`/
`outage_duration_s`/`outage_phase_s`) on top of the original fixed
delay + symmetric jitter + independent-Bernoulli drop, and publishes a
1Hz `relay_status` topic (received/forwarded/dropped_bernoulli/
dropped_outage counts + current pending queue depth) so an external
orchestrator can poll for "queue fully drained" before stopping this
node -- `destroy_node()` still does NOT flush the pending delayed-message
queue (unchanged from v1.0; this is why the drain-before-stop poll
exists as an orchestrator-level responsibility, not something this node
does for itself on shutdown).
"""

import heapq
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
        self.declare_parameter("outage_period_s", 0.0)
        self.declare_parameter("outage_duration_s", 0.0)
        self.declare_parameter("outage_phase_s", 0.0)

        config = ImpairmentConfig(
            delay_s=float(self.get_parameter("delay_s").value),
            jitter_s=float(self.get_parameter("jitter_s").value),
            drop_probability=float(self.get_parameter("drop_probability").value),
            seed=int(self.get_parameter("seed").value),
            outage_period_s=float(self.get_parameter("outage_period_s").value),
            outage_duration_s=float(self.get_parameter("outage_duration_s").value),
            outage_phase_s=float(self.get_parameter("outage_phase_s").value),
        )
        self.decider = ImpairmentDecider(config)
        self._immediate_passthrough = self.decider.is_zero_impairment()

        self.publisher = self.create_publisher(EpuckState, "state", 20)
        self.create_subscription(EpuckState, "state_raw", self._on_message, 20)

        self._queue = []  # heap of (release_time_s, counter, msg, scheduled_delay_s, receive_time_s)
        self._counter = 0
        self.received_count = 0
        self.forwarded_count = 0
        self.dropped_bernoulli_count = 0
        self.dropped_outage_count = 0

        # v1.1: elapsed_s fed to the (pure, stateless) outage-window check
        # is measured from this node's own construction instant, under
        # its own clock (sim time when use_sim_time=true, matching every
        # launch site -- see the module docstring). Because _in_outage()
        # is a pure function of elapsed_s with no accumulated state, a
        # backward jump in self._now_s() (e.g. a Webots sim-time reset)
        # is handled correctly automatically -- it just re-evaluates the
        # same periodic schedule at whatever elapsed_s results, never
        # raises, never needs the reset-detection machinery
        # cooperative_avoider.py's _ensure_timebase() needs for its own
        # ACCUMULATING elapsed-time state.
        self._start_s = self._now_s()

        log_path = str(self.get_parameter("log_path").value)
        self._log_file = open(log_path, "w", encoding="utf-8") if log_path else None
        if self._log_file:
            # source_stamp_s/actual_release_time_s/drop_reason appended
            # (not inserted) so existing readers keyed by column name
            # (received_seq, action, receive_time_s, release_time_s) are
            # unaffected. release_time_s remains the value SCHEDULED at
            # receive time; actual_release_time_s is when publish()
            # actually ran (may differ by up to one flush-timer period,
            # 0.01s, for delayed messages -- identical to release_time_s
            # for immediate passthrough, since that path is synchronous).
            # drop_reason is "" for forwarded rows, "bernoulli" or
            # "outage" for dropped rows -- the two loss mechanisms stay
            # distinguishable in the log even if both are configured at
            # once (per the impairment-matrix analysis plan's requirement
            # that "configured drop" and "burst outage drop" never be
            # conflated).
            self._log_file.write(
                "received_seq,action,scheduled_delay_s,receive_time_s,release_time_s,"
                "source_stamp_s,actual_release_time_s,drop_reason\n"
            )

        self.get_logger().info(
            f"network_impairment_relay: delay_s={config.delay_s:.4f} "
            f"jitter_s={config.jitter_s:.4f} drop_probability={config.drop_probability:.4f} "
            f"seed={config.seed} outage_period_s={config.outage_period_s:.4f} "
            f"outage_duration_s={config.outage_duration_s:.4f} outage_phase_s={config.outage_phase_s:.4f} "
            f"immediate_passthrough={self._immediate_passthrough}"
        )

        if not self._immediate_passthrough:
            self.timer = self.create_timer(0.01, self._flush_queue)

        self.status_publisher = self.create_publisher(String, "relay_status", 5)
        self.status_timer = self.create_timer(1.0, self._publish_status)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def _stamp_s(msg: EpuckState) -> float:
        return float(msg.stamp.sec) + float(msg.stamp.nanosec) / 1.0e9

    @property
    def dropped_count(self) -> int:
        """Backward-compatible total (v1.0 had a single dropped_count
        counter, before drop_reason existed to distinguish mechanisms)."""
        return self.dropped_bernoulli_count + self.dropped_outage_count

    def pending_queue_depth(self) -> int:
        """Read-only -- never used to alter scheduling, only exposed so an
        orchestrator's drain step (and this class's own status topic) can
        observe it without touching the private queue directly."""
        return len(self._queue)

    def _on_message(self, msg: EpuckState) -> None:
        self.received_count += 1
        elapsed = self._now_s() - self._start_s
        decision = self.decider.decide(elapsed)
        now = self._now_s()
        source_stamp = self._stamp_s(msg)
        if not decision.forward:
            if decision.drop_reason == "outage":
                self.dropped_outage_count += 1
            else:
                self.dropped_bernoulli_count += 1
            self._log(msg.sequence, "dropped", 0.0, now, None, source_stamp, None, decision.drop_reason)
            return
        if self._immediate_passthrough:
            self.publisher.publish(msg)
            self.forwarded_count += 1
            self._log(msg.sequence, "forwarded", 0.0, now, now, source_stamp, now, "")
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
                self._stamp_s(msg), actual_release_time, "",
            )

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps({
            "received_count": self.received_count,
            "forwarded_count": self.forwarded_count,
            "dropped_bernoulli_count": self.dropped_bernoulli_count,
            "dropped_outage_count": self.dropped_outage_count,
            "pending_queue_depth": self.pending_queue_depth(),
        })
        self.status_publisher.publish(msg)

    def _log(
        self, seq, action, scheduled_delay_s, receive_time_s, release_time_s,
        source_stamp_s, actual_release_time_s, drop_reason,
    ) -> None:
        if self._log_file is None:
            return
        release_field = f"{release_time_s:.6f}" if release_time_s is not None else ""
        actual_release_field = f"{actual_release_time_s:.6f}" if actual_release_time_s is not None else ""
        self._log_file.write(
            f"{seq},{action},{scheduled_delay_s:.6f},{receive_time_s:.6f},{release_field},"
            f"{source_stamp_s:.6f},{actual_release_field},{drop_reason}\n"
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
