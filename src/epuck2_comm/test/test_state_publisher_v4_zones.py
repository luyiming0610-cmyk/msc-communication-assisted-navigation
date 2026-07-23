"""controller_v4_full_sensor_bypass_20260717 ps0-ps7 zone-mapping tests.

Verifies state_publisher.py's zone split (left_front=ps7, left_mid=min(ps5,
ps6), left_rear=ps4, right_front=ps0, right_mid=min(ps1,ps2), right_rear=ps3)
-- the exact forensic mapping recovered from pilot_a3 -- including left/right
mirror symmetry and inf/no-detection handling. ps3/ps4 (the rear pair) were
never read by any prior version; this is the coverage gap v4 closes.
"""

import math

import rclpy
from sensor_msgs.msg import Range

from epuck2_comm.state_publisher import StatePublisher


CLEAR_BASELINE = 0.070  # matches the real Webots/hardware clear-space reading
IR_NO_DETECTION_M = 0.060


def _range_msg(value):
    msg = Range()
    msg.range = float(value)
    return msg


def _feed_all_clear(node, now):
    for name in (f"ps{i}" for i in range(8)):
        node._range_callback(name, _range_msg(CLEAR_BASELINE))
    node.range_received_at = {k: now for k in node.range_received_at}


def test_zone_mapping_and_mirror_symmetry():
    rclpy.init(args=["--ros-args", "-r", "__ns:=/pytest_isolated"])
    try:
        node = StatePublisher()
        import time

        now = time.monotonic()
        _feed_all_clear(node, now)
        # ps7 (left_front), ps4 (left_rear) close; mirror: ps0 (right_front),
        # ps3 (right_rear) close on the other side.
        node._range_callback("ps7", _range_msg(0.045))
        node._range_callback("ps4", _range_msg(0.040))
        node.range_received_at["ps7"] = now
        node.range_received_at["ps4"] = now

        snapshot = node._snapshot(now)
        assert snapshot.left_front_m == 0.045
        assert snapshot.left_rear_m == 0.040
        assert math.isinf(snapshot.left_mid_m)
        assert math.isinf(snapshot.right_front_m)
        assert math.isinf(snapshot.right_rear_m)

        node.destroy_node()

        # Mirror: right side close instead.
        node2 = StatePublisher()
        _feed_all_clear(node2, now)
        node2._range_callback("ps0", _range_msg(0.045))
        node2._range_callback("ps3", _range_msg(0.040))
        node2.range_received_at["ps0"] = now
        node2.range_received_at["ps3"] = now
        snapshot2 = node2._snapshot(now)
        assert snapshot2.right_front_m == 0.045
        assert snapshot2.right_rear_m == 0.040
        assert math.isinf(snapshot2.right_mid_m)
        assert math.isinf(snapshot2.left_front_m)
        assert math.isinf(snapshot2.left_rear_m)
        node2.destroy_node()
    finally:
        rclpy.shutdown()


def test_ps3_ps4_rear_pair_now_reaches_the_state_message():
    """Regression guard for the forensic finding that ps3/ps4 were never
    read by v1/v2/v3 -- feeding ONLY ps3/ps4 close must now show up as
    right_rear_m/left_rear_m, something no prior version's front/left/right
    triple could ever represent."""
    rclpy.init(args=["--ros-args", "-r", "__ns:=/pytest_isolated"])
    try:
        import time

        node = StatePublisher()
        now = time.monotonic()
        _feed_all_clear(node, now)
        node._range_callback("ps3", _range_msg(0.035))
        node._range_callback("ps4", _range_msg(0.038))
        node.range_received_at["ps3"] = now
        node.range_received_at["ps4"] = now
        snapshot = node._snapshot(now)
        assert snapshot.right_rear_m == 0.035
        assert snapshot.left_rear_m == 0.038
        # And the legacy front/left/right triple is untouched by ps3/ps4 --
        # confirms the historical blind spot this closes.
        assert math.isinf(snapshot.left_distance_m)
        assert math.isinf(snapshot.right_distance_m)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_clear_space_baseline_reported_as_inf_not_raw_baseline():
    rclpy.init(args=["--ros-args", "-r", "__ns:=/pytest_isolated"])
    try:
        import time

        node = StatePublisher()
        now = time.monotonic()
        _feed_all_clear(node, now)
        snapshot = node._snapshot(now)
        assert math.isinf(snapshot.left_front_m)
        assert math.isinf(snapshot.left_mid_m)
        assert math.isinf(snapshot.left_rear_m)
        assert math.isinf(snapshot.right_front_m)
        assert math.isinf(snapshot.right_mid_m)
        assert math.isinf(snapshot.right_rear_m)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_stale_ps_reading_treated_as_no_detection():
    rclpy.init(args=["--ros-args", "-r", "__ns:=/pytest_isolated"])
    try:
        import time

        node = StatePublisher()
        now = time.monotonic()
        _feed_all_clear(node, now)
        node._range_callback("ps4", _range_msg(0.020))
        node.range_received_at["ps4"] = now - 10.0  # far beyond sensor_timeout_s
        snapshot = node._snapshot(now)
        assert math.isinf(snapshot.left_rear_m)
        node.destroy_node()
    finally:
        rclpy.shutdown()
