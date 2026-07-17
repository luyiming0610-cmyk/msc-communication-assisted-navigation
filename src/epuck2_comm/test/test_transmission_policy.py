import math

from epuck2_comm.models import RobotStateSnapshot
from epuck2_comm.transmission_policy import EventTriggeredPolicy, PeriodicPolicy


def snapshot(**changes):
    values = dict(
        x_m=0.0,
        y_m=0.0,
        yaw_rad=0.0,
        linear_velocity_mps=0.0,
        angular_velocity_rps=0.0,
        front_distance_m=math.inf,
        left_distance_m=math.inf,
        right_distance_m=math.inf,
        obstacle_status=1,
        validity_flags=7,
    )
    values.update(changes)
    return RobotStateSnapshot(**values)


def event_policy():
    return EventTriggeredPolicy(
        min_interval_s=0.1,
        heartbeat_interval_s=0.5,
        position_threshold_m=0.01,
        yaw_threshold_rad=0.05,
        linear_velocity_threshold_mps=0.005,
        angular_velocity_threshold_rps=0.05,
        distance_threshold_m=0.01,
    )


def test_periodic_policy_respects_period():
    policy = PeriodicPolicy(min_interval_s=0.1)
    state = snapshot()
    assert policy.should_publish(state, 0.0)
    policy.mark_published(state, 0.0)
    assert not policy.should_publish(state, 0.05)
    assert policy.should_publish(state, 0.1)


def test_event_policy_publishes_on_position_change():
    policy = event_policy()
    initial = snapshot()
    policy.mark_published(initial, 0.0)
    assert not policy.should_publish(snapshot(x_m=0.005), 0.2)
    assert policy.should_publish(snapshot(x_m=0.011), 0.2)


def test_event_policy_publishes_on_status_or_validity_change():
    policy = event_policy()
    initial = snapshot()
    policy.mark_published(initial, 0.0)
    assert policy.should_publish(snapshot(obstacle_status=2), 0.2)
    assert policy.should_publish(snapshot(validity_flags=1), 0.2)


def test_event_policy_heartbeat_and_minimum_interval():
    policy = event_policy()
    initial = snapshot()
    policy.mark_published(initial, 0.0)
    assert not policy.should_publish(snapshot(x_m=1.0), 0.05)
    assert policy.should_publish(initial, 0.5)


def test_event_policy_detects_finite_distance_transition():
    policy = event_policy()
    initial = snapshot(front_distance_m=math.inf)
    policy.mark_published(initial, 0.0)
    assert policy.should_publish(snapshot(front_distance_m=0.2), 0.2)
