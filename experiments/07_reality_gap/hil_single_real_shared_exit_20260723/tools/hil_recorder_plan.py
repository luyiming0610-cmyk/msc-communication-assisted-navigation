#!/usr/bin/env python3
"""Computes the rosbag recording plan for a HIL trial -- topic list and
the `ros2 bag record` command -- without ever starting a recording
itself. This is deliberately read-only/print-only: the launcher script
is the only place a recording process is actually started, and only
past the wheel-suspension gate.

Recorded topics span: real robot state, virtual peer state, the
GoalAnnouncement/NavigationIntent channels, the GUARDED cmd_vel (never
the unguarded pre-guard stream, which is not the safety-relevant
signal), the guard's own arm/block-reason topic, and bridge/driver
status if present -- matching the auto-recording requirement (real +
virtual state, GoalAnnouncement, NavigationIntent, guarded cmd_vel,
bridge status, task completion, safety events).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RecorderPlan:
    topics: tuple[str, ...]
    output_dir: str

    def to_ros2_bag_record_command(self) -> list[str]:
        return ["ros2", "bag", "record", "-o", self.output_dir, *self.topics]


def build_recorder_plan(
    *,
    physical_state_topic: str,
    virtual_state_topic: str,
    goal_announcement_topic: str,
    nav_intent_topic: str,
    guarded_cmd_vel_topic: str,
    guard_arm_topic: str,
    bridge_status_topic: str,
    task_completion_topic: str,
    safety_event_topic: str,
    output_dir: str,
) -> RecorderPlan:
    topics = (
        physical_state_topic,
        virtual_state_topic,
        goal_announcement_topic,
        nav_intent_topic,
        guarded_cmd_vel_topic,
        guard_arm_topic,
        bridge_status_topic,
        task_completion_topic,
        safety_event_topic,
    )
    if any(not topic for topic in topics):
        raise ValueError("All recorder-plan topics must be non-empty -- refusing to build a plan with a silently omitted channel.")
    return RecorderPlan(topics=topics, output_dir=output_dir)


def main(argv=None) -> int:
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="Print (never run) the HIL rosbag recording plan.")
    parser.add_argument("--physical-state-topic", required=True)
    parser.add_argument("--virtual-state-topic", required=True)
    parser.add_argument("--goal-announcement-topic", required=True)
    parser.add_argument("--nav-intent-topic", required=True)
    parser.add_argument("--guarded-cmd-vel-topic", required=True)
    parser.add_argument("--guard-arm-topic", required=True)
    parser.add_argument("--bridge-status-topic", required=True)
    parser.add_argument("--task-completion-topic", required=True)
    parser.add_argument("--safety-event-topic", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    plan = build_recorder_plan(
        physical_state_topic=args.physical_state_topic,
        virtual_state_topic=args.virtual_state_topic,
        goal_announcement_topic=args.goal_announcement_topic,
        nav_intent_topic=args.nav_intent_topic,
        guarded_cmd_vel_topic=args.guarded_cmd_vel_topic,
        guard_arm_topic=args.guard_arm_topic,
        bridge_status_topic=args.bridge_status_topic,
        task_completion_topic=args.task_completion_topic,
        safety_event_topic=args.safety_event_topic,
        output_dir=args.output_dir,
    )
    print(" ".join(plan.to_ros2_bag_record_command()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
