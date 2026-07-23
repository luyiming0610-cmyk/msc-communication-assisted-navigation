#!/usr/bin/env python3
import unittest

from hil_recorder_plan import build_recorder_plan


def _kwargs(**overrides):
    kwargs = dict(
        physical_state_topic="/epuck5809/state",
        virtual_state_topic="/epuck_virtual_peer/state",
        goal_announcement_topic="/hil/goal_announcement",
        nav_intent_topic="/epuck5809/nav_intent",
        guarded_cmd_vel_topic="/epuck5809/cmd_vel",
        guard_arm_topic="/hil_guard/arm",
        bridge_status_topic="/hil/bridge_status",
        task_completion_topic="/hil/task_completion",
        safety_event_topic="/hil/safety_events",
        output_dir="/home/eamon/epuck_comm_bags/hil_test",
    )
    kwargs.update(overrides)
    return kwargs


class BuildRecorderPlanTest(unittest.TestCase):
    def test_all_topics_included(self):
        plan = build_recorder_plan(**_kwargs())
        self.assertEqual(len(plan.topics), 9)
        self.assertIn("/epuck5809/cmd_vel", plan.topics)

    def test_empty_topic_rejected(self):
        with self.assertRaises(ValueError):
            build_recorder_plan(**_kwargs(safety_event_topic=""))

    def test_command_includes_output_dir_and_topics(self):
        plan = build_recorder_plan(**_kwargs())
        command = plan.to_ros2_bag_record_command()
        self.assertEqual(command[0:3], ["ros2", "bag", "record"])
        self.assertIn(plan.output_dir, command)
        for topic in plan.topics:
            self.assertIn(topic, command)


if __name__ == "__main__":
    unittest.main()
