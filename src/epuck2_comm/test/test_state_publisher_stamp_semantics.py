"""protocol_v1.1_stamp_semantics tests for state_publisher.py.

Covers: stamp is nonzero and monotonically non-decreasing with ROS time;
WAITING_FOR_CLOCK holds publication (no fake zero-stamped formal message)
while the clock is not yet valid; a real (hardware-clock-shaped) nonzero
clock never degrades into WAITING_FOR_CLOCK.
"""

import rclpy

from epuck2_comm.state_publisher import StatePublisher


def _stamp_to_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1.0e9


def test_no_publish_while_ros_clock_is_not_yet_valid(monkeypatch):
    rclpy.init(args=["--ros-args", "-r", "__ns:=/pytest_isolated"])
    try:
        node = StatePublisher()
        published = []
        node.publisher.publish = lambda msg: published.append(msg)
        monkeypatch.setattr(node, "_now_s", lambda: 0.0)

        node._timer_callback()

        assert published == [], "must not publish a formal state message before the clock is valid"
        assert node._clock_wait_logged is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stamp_is_nonzero_once_clock_is_valid(monkeypatch):
    rclpy.init(args=["--ros-args", "-r", "__ns:=/pytest_isolated"])
    try:
        node = StatePublisher()
        published = []
        node.publisher.publish = lambda msg: published.append(msg)
        monkeypatch.setattr(node, "_now_s", lambda: 12.5)

        node._timer_callback()

        assert len(published) == 1
        assert _stamp_to_s(published[0].stamp) > 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stamp_advances_monotonically_with_ros_time(monkeypatch):
    rclpy.init(args=["--ros-args", "-r", "__ns:=/pytest_isolated"])
    try:
        node = StatePublisher()
        published = []
        node.publisher.publish = lambda msg: published.append(msg)

        clock = {"t": 10.0}
        monkeypatch.setattr(node, "_now_s", lambda: clock["t"])
        node._timer_callback()

        clock["t"] = 10.5
        node._timer_callback()

        assert len(published) == 2
        assert _stamp_to_s(published[1].stamp) > _stamp_to_s(published[0].stamp)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_real_nonzero_clock_never_triggers_waiting_for_clock(monkeypatch):
    """Hardware mode (use_sim_time=false): the system clock is always a
    large positive Unix-epoch value from the first tick, so the `now <= 0.0`
    guard must never degrade a real hardware clock into a permanent hold."""
    rclpy.init(args=["--ros-args", "-r", "__ns:=/pytest_isolated"])
    try:
        node = StatePublisher()
        published = []
        node.publisher.publish = lambda msg: published.append(msg)
        monkeypatch.setattr(node, "_now_s", lambda: 1_784_360_000.123)

        node._timer_callback()

        assert len(published) == 1
        assert node._clock_wait_logged is False
    finally:
        node.destroy_node()
        rclpy.shutdown()
