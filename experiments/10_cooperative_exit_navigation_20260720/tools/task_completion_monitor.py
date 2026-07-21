#!/usr/bin/env python3
"""Independent, read-only task-completion monitor for the cooperative
exit-navigation N-robot study.

This is a NEW component, not a modification of any frozen controller,
CPA, local IR/ToF avoidance, or EpuckState-protocol code. It exists
because the controller's own internal completion signal
(stop_after_recovery / _complete() in cooperative_avoider.py) marks
"a recovery maneuver finished", which is NOT the same event as "the
robot is inside the pre-registered goal/exit region" -- conflating the
two let earlier pilots run to max_runtime_s without the trial ever
actually stopping on genuine goal arrival. Per the pre-registered rule
in task_completion_analyzer.build_task_verdict(), a trial that ends via
max_runtime_s can never be scored SUCCESS regardless of where the robot
was -- so a live, independent goal-judgment mechanism is required for
TASK_OUTCOME=SUCCESS to ever be reachable at all.

Design constraints (binding):
  - Subscribes ONLY to each robot's own already-published EpuckState
    topic (the SAME topic that robot's own controller reads its own
    state from -- /epuckN/state in both COMM_OFF and COMM_ON). No new
    topics are added on the robot side.
  - NEVER publishes cmd_vel, a navigation target, or any command to any
    robot. It cannot steer or otherwise influence navigation -- it is a
    pure observer.
  - NEVER subscribes to Supervisor ground truth. Supervisor stays
    reserved for offline, post-hoc measurement only, exactly as before.
  - Does NOT forward one robot's state to the other's controller --
    under COMM_OFF this process still reads both robots' /epuckN/state
    topics (to judge whether BOTH have reached the goal, which is the
    task-level, not per-robot, success criterion), but that data goes
    only into this monitor's own log/verdict file, an external
    measurement artifact analogous to Supervisor-based post-hoc
    analysis running live instead of after the fact. It is never
    re-published to, or read by, either robot's own control loop, so
    the COMM_OFF "no access to any other robot's EpuckState" guarantee
    for the CONTROLLER remains intact.
  - Reuses the existing, frozen safe-stop mechanism rather than
    inventing a new one: on detecting all robots have satisfied the
    goal-hold requirement, this process only WRITES a single grep-able
    TASK_COMPLETE_GOAL log line and a monitor_verdict.json file, then
    exits. It does not send any signal to the controller processes
    itself. The orchestrator shell script watches for the log line and
    requests controller shutdown via the SAME stop_pid_group() SIGINT
    path it already uses at end-of-trial, which is what triggers
    cooperative_avoider.py's own unmodified stop() method (publishes a
    zero Twist three times from its SIGINT/KeyboardInterrupt handler in
    main()'s finally block) -- reused, not duplicated.

Per-robot goal judgment uses the same anti-single-frame continuous hold
state machine as the post-hoc analyzer, strengthened for formal task
completion by requiring both measured linear and angular velocity to
remain below frozen thresholds throughout the hold. Leaving the region
or exceeding either motion threshold resets the hold clock.
"""
from __future__ import annotations

import argparse
import json
import sys

import rclpy
from rclpy.node import Node

from epuck2_comm_interfaces.msg import EpuckState

from goal_hold_tracker import GoalHoldTracker


class TaskCompletionMonitor(Node):
    def __init__(self, robot_ids, state_topics, goal_centers_x_m, goal_centers_y_m,
                 goal_radii_m, goal_hold_time_s, max_linear_speed_mps,
                 max_angular_speed_rps, verdict_path):
        super().__init__("task_completion_monitor")
        self.robot_ids = robot_ids
        self.verdict_path = verdict_path
        self.done = False
        self.max_linear_speed_mps = max_linear_speed_mps
        self.max_angular_speed_rps = max_angular_speed_rps
        # Per-robot completion region (Part V: each robot's own parking
        # zone, distinct and non-colliding) -- NOT a single shared point,
        # so this monitor's completion criterion matches exactly what
        # each robot's own goal_navigator uses for its ARRIVED_HOLD latch.
        self.trackers = {
            rid: GoalHoldTracker(cx, cy, r, goal_hold_time_s)
            for rid, cx, cy, r in zip(robot_ids, goal_centers_x_m, goal_centers_y_m, goal_radii_m)
        }
        self._reported_complete = {rid: False for rid in robot_ids}
        self._subs = []
        for rid, topic in zip(robot_ids, state_topics):
            self._subs.append(
                self.create_subscription(EpuckState, topic, self._make_cb(rid), 20)
            )
        self.get_logger().info(
            "task_completion_monitor READY "
            f"watching={dict(zip(robot_ids, state_topics))} "
            f"goals={ {rid: (t.center_x_m, t.center_y_m, t.radius_m) for rid, t in self.trackers.items()} } "
            f"hold_time_s={goal_hold_time_s} "
            f"max_linear_speed_mps={max_linear_speed_mps} "
            f"max_angular_speed_rps={max_angular_speed_rps}"
        )

    def _make_cb(self, robot_id):
        def _cb(msg):
            if self.done:
                return
            t_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) / 1e9
            motion_settled = (
                abs(float(msg.linear_velocity_mps)) <= self.max_linear_speed_mps
                and abs(float(msg.angular_velocity_rps)) <= self.max_angular_speed_rps
            )
            self.trackers[robot_id].update(
                t_s, float(msg.x_m), float(msg.y_m), eligible=motion_settled
            )
            if self.trackers[robot_id].reached and not self._reported_complete[robot_id]:
                self._reported_complete[robot_id] = True
                self.get_logger().info(
                    f"ROBOT_ARRIVED robot_id={robot_id} t={t_s:.3f} "
                    f"completion_time_s={self.trackers[robot_id].completion_time_s:.3f} "
                    f"linear_velocity_mps={float(msg.linear_velocity_mps):.6f} "
                    f"angular_velocity_rps={float(msg.angular_velocity_rps):.6f}"
                )
            if all(tracker.reached for tracker in self.trackers.values()):
                self._on_all_complete()
        return _cb

    @property
    def per_robot_completed(self) -> dict:
        return dict(self._reported_complete)

    def _on_all_complete(self):
        self.done = True
        completion_times = {rid: tracker.completion_time_s for rid, tracker in self.trackers.items()}
        self.get_logger().info(
            "TASK_COMPLETE_GOAL all_robots_reached_goal=true "
            f"completion_times_s={completion_times}"
        )
        if self.verdict_path:
            with open(self.verdict_path, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "stop_reason": "TASK_COMPLETE_GOAL",
                        "all_robots_reached_goal": True,
                        "per_robot_completed": self.per_robot_completed,
                        "completion_times_s": completion_times,
                        "makespan_s": max(completion_times.values()),
                        "completion_max_linear_speed_mps": self.max_linear_speed_mps,
                        "completion_max_angular_speed_rps": self.max_angular_speed_rps,
                    },
                    f,
                    indent=2,
                )


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ids", required=True, help="comma-separated, e.g. epuck1,epuck2")
    parser.add_argument("--state-topics", required=True, help="comma-separated, same order as --robot-ids")
    parser.add_argument(
        "--goal-centers-x-m", required=True,
        help="comma-separated, same order as --robot-ids -- each robot's own completion-region center x"
    )
    parser.add_argument(
        "--goal-centers-y-m", required=True,
        help="comma-separated, same order as --robot-ids"
    )
    parser.add_argument(
        "--goal-radii-m", required=True,
        help="comma-separated, same order as --robot-ids"
    )
    parser.add_argument("--goal-hold-time-s", type=float, required=True)
    parser.add_argument("--max-linear-speed-mps", type=float, required=True)
    parser.add_argument("--max-angular-speed-rps", type=float, required=True)
    parser.add_argument("--verdict-path", default="")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    robot_ids = args.robot_ids.split(",")
    state_topics = args.state_topics.split(",")
    goal_centers_x_m = [float(v) for v in args.goal_centers_x_m.split(",")]
    goal_centers_y_m = [float(v) for v in args.goal_centers_y_m.split(",")]
    goal_radii_m = [float(v) for v in args.goal_radii_m.split(",")]
    if not (len(robot_ids) == len(state_topics) == len(goal_centers_x_m)
            == len(goal_centers_y_m) == len(goal_radii_m)):
        print(
            "robot-ids, state-topics, goal-centers-x-m, goal-centers-y-m, "
            "goal-radii-m must all have equal length",
            file=sys.stderr,
        )
        return 2

    rclpy.init(args=[])
    node = TaskCompletionMonitor(
        robot_ids, state_topics,
        goal_centers_x_m, goal_centers_y_m, goal_radii_m, args.goal_hold_time_s,
        args.max_linear_speed_mps, args.max_angular_speed_rps,
        args.verdict_path,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
