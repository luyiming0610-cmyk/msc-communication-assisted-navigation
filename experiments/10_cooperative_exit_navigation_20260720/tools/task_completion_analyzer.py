"""Pure-Python N-robot goal/exit-region task-completion analyzer.

Genuinely new logic for the 10_cooperative_exit_navigation experiment
category -- Conditions A-D never measured "did the robots reach a common
destination," only pairwise CPA-avoidance behavior. This module is
independent of ROS/rosbag (operates on plain per-robot time-series of
(t_s, x_m, y_m, yaw_rad, linear_velocity_mps) samples, e.g. already
extracted from a bag by a thin wrapper) so its correctness can be unit
tested without Webots.

DATA_VALIDITY (was the measurement chain itself sound) and TASK_OUTCOME
(did the task succeed) are computed and reported as SEPARATE fields,
never merged into one flag, per instruction.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import combinations


Sample = tuple[float, float, float, float, float]  # t_s, x_m, y_m, yaw_rad, linear_velocity_mps


@dataclass(frozen=True)
class GoalRegion:
    """A circular goal/exit region in the shared experiment frame."""
    center_x_m: float
    center_y_m: float
    radius_m: float

    def contains(self, x_m: float, y_m: float) -> bool:
        return math.hypot(x_m - self.center_x_m, y_m - self.center_y_m) <= self.radius_m


def robot_goal_completion_time(
    samples: list[Sample], goal: GoalRegion, hold_time_s: float
) -> tuple[float | None, str | None]:
    """First timestamp at which the robot has been CONTINUOUSLY inside
    `goal` for at least `hold_time_s`. Returns (completion_time_s, None)
    on success, or (None, reason) if the robot never satisfies the hold
    requirement.

    A single-frame dip into the goal region does NOT count -- this is the
    explicit anti-false-trigger requirement. Any sample outside the region
    resets the continuous-entry clock; the hold timer restarts from that
    later re-entry, never accumulates across a gap.
    """
    if not samples:
        return None, "NOT_MEASURABLE (no samples)"
    entry_time: float | None = None
    for t_s, x_m, y_m, _yaw, _v in samples:
        inside = goal.contains(x_m, y_m)
        if inside:
            if entry_time is None:
                entry_time = t_s
            elif t_s - entry_time >= hold_time_s:
                return entry_time + hold_time_s, None
        else:
            entry_time = None
    return None, "NEVER_HELD_GOAL_FOR_REQUIRED_DURATION"


@dataclass
class RobotGoalResult:
    robot_id: str
    reached: bool
    completion_time_s: float | None
    reason: str | None


def all_robots_goal_results(
    per_robot_samples: dict[str, list[Sample]],
    goal: GoalRegion,
    hold_time_s: float,
    per_robot_goals: dict[str, GoalRegion] | None = None,
) -> list[RobotGoalResult]:
    """`goal` is the shared/default region used for every robot unless
    `per_robot_goals` supplies a robot-specific override (Part V: each
    robot's own, distinct, non-colliding post-exit parking zone) --
    existing callers that never pass `per_robot_goals` are unaffected,
    byte-for-byte, by this addition."""
    results = []
    for robot_id, samples in per_robot_samples.items():
        robot_goal = (per_robot_goals or {}).get(robot_id, goal)
        completion_time, reason = robot_goal_completion_time(samples, robot_goal, hold_time_s)
        results.append(
            RobotGoalResult(robot_id, completion_time is not None, completion_time, reason)
        )
    return results


def all_robots_reached_goal(results: list[RobotGoalResult]) -> bool:
    """Success requires EVERY robot to individually satisfy the hold
    requirement -- explicitly not 'any one robot reached the goal'."""
    return len(results) > 0 and all(r.reached for r in results)


def makespan_s(results: list[RobotGoalResult]) -> float | None:
    """Last robot's completion time -- the primary efficiency metric.
    None if not every robot reached the goal (makespan is undefined for a
    task that did not fully succeed)."""
    if not all_robots_reached_goal(results):
        return None
    return max(r.completion_time_s for r in results)


def _interpolated_position_at(samples: list[Sample], t_s: float) -> tuple[float, float] | None:
    """Linear interpolation of (x_m, y_m) at time t_s from a sorted sample
    list. Returns None if t_s is outside the sample time range."""
    if not samples:
        return None
    if t_s < samples[0][0] or t_s > samples[-1][0]:
        return None
    for (t0, x0, y0, _, _), (t1, x1, y1, _, _) in zip(samples, samples[1:]):
        if t0 <= t_s <= t1:
            if t1 == t0:
                return x0, y0
            frac = (t_s - t0) / (t1 - t0)
            return x0 + frac * (x1 - x0), y0 + frac * (y1 - y0)
    return samples[-1][1], samples[-1][2]


def pairwise_min_distance(
    per_robot_samples: dict[str, list[Sample]],
) -> dict[tuple[str, str], float]:
    """Minimum inter-robot distance over the trial, for EVERY pairwise
    combination of robots (covers all C(N,2) pairs for any N, not just a
    single hardcoded pair) -- computed on the union of both robots'
    sample timestamps, interpolating the other robot's position at each
    timestamp so unevenly-sampled/unsynchronized streams are still
    compared fairly."""
    result: dict[tuple[str, str], float] = {}
    robot_ids = sorted(per_robot_samples)
    for a, b in combinations(robot_ids, 2):
        samples_a = per_robot_samples[a]
        samples_b = per_robot_samples[b]
        timestamps = sorted({t for t, *_ in samples_a} | {t for t, *_ in samples_b})
        min_dist = None
        for t_s in timestamps:
            pos_a = _interpolated_position_at(samples_a, t_s)
            pos_b = _interpolated_position_at(samples_b, t_s)
            if pos_a is None or pos_b is None:
                continue
            dist = math.hypot(pos_a[0] - pos_b[0], pos_a[1] - pos_b[1])
            if min_dist is None or dist < min_dist:
                min_dist = dist
        if min_dist is not None:
            result[(a, b)] = min_dist
    return result


def overall_minimum_pairwise_distance_m(pairwise: dict[tuple[str, str], float]) -> float | None:
    if not pairwise:
        return None
    return min(pairwise.values())


def path_length_m(samples: list[Sample]) -> float:
    total = 0.0
    for (_, x0, y0, _, _), (_, x1, y1, _, _) in zip(samples, samples[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def _unwrap(angles_rad: list[float]) -> list[float]:
    if not angles_rad:
        return []
    out = [angles_rad[0]]
    for angle in angles_rad[1:]:
        delta = (angle - out[-1] + math.pi) % (2.0 * math.pi) - math.pi
        out.append(out[-1] + delta)
    return out


def cumulative_absolute_heading_change_rad(samples: list[Sample]) -> float:
    yaws = _unwrap([s[3] for s in samples])
    return sum(abs(b - a) for a, b in zip(yaws, yaws[1:]))


def stop_duration_s(samples: list[Sample], stop_threshold_mps: float = 0.002) -> float:
    """Total wall-clock (sim-clock) time the robot's linear_velocity_mps
    stayed at or below stop_threshold_mps, trapezoid-integrated over the
    sample series."""
    total = 0.0
    for (t0, _, _, _, v0), (t1, _, _, _, v1) in zip(samples, samples[1:]):
        dt = max(0.0, t1 - t0)
        if v0 <= stop_threshold_mps and v1 <= stop_threshold_mps:
            total += dt
    return total


@dataclass
class TaskVerdict:
    """DATA_VALIDITY and TASK_OUTCOME are separate fields by design --
    neither is ever inferred from the other."""
    data_validity: str  # "VALID" | "INVALID"
    data_validity_reasons: list[str]
    task_outcome: str  # "SUCCESS" | "TASK_FAILURE" | "UNSAFE_FAILURE"
    task_outcome_reason: str
    all_robots_reached_goal: bool
    completed_robot_count: int
    total_robot_count: int
    makespan_s: float | None
    minimum_pairwise_distance_m: float | None
    safety_margin_m: float | None
    collision_count: int


def build_task_verdict(
    *,
    per_robot_samples: dict[str, list[Sample]],
    goal: GoalRegion,
    hold_time_s: float,
    safety_radius_m: float,
    collision_contact_distance_m: float,
    data_validity_reasons: list[str],
    latched_failsafe: bool,
    ended_by_max_runtime: bool,
    per_robot_goals: dict[str, GoalRegion] | None = None,
) -> TaskVerdict:
    """Assembles the final verdict. `latched_failsafe` and
    `ended_by_max_runtime` are supplied by the caller from the controller
    logs / bag (this module does not parse controller.log itself) --
    per instruction, max-runtime or process-exit must NEVER be read as
    success by this function: if ended_by_max_runtime is True, task
    outcome is forced to TASK_FAILURE regardless of goal-region results.
    """
    data_validity = "VALID" if not data_validity_reasons else "INVALID"

    results = all_robots_goal_results(per_robot_samples, goal, hold_time_s, per_robot_goals)
    reached = all_robots_reached_goal(results)
    completed_count = sum(1 for r in results if r.reached)

    pairwise = pairwise_min_distance(per_robot_samples)
    min_dist = overall_minimum_pairwise_distance_m(pairwise)
    safety_margin = (min_dist - safety_radius_m) if min_dist is not None else None
    collision_count = sum(1 for d in pairwise.values() if d < collision_contact_distance_m)

    if data_validity == "INVALID":
        task_outcome = "TASK_FAILURE"
        reason = "DATA_VALIDITY=INVALID -- task outcome not evaluable: " + "; ".join(data_validity_reasons)
    elif collision_count > 0:
        task_outcome = "UNSAFE_FAILURE"
        reason = f"collision_count={collision_count} (pairwise distance below contact threshold {collision_contact_distance_m}m)"
    elif min_dist is not None and min_dist < safety_radius_m:
        task_outcome = "UNSAFE_FAILURE"
        reason = f"minimum_pairwise_distance_m={min_dist:.6f} < safety_radius_m={safety_radius_m} (margin {safety_margin:.6f}m)"
    elif latched_failsafe:
        task_outcome = "TASK_FAILURE"
        reason = "a latched FAILSAFE occurred during the trial"
    elif ended_by_max_runtime:
        task_outcome = "TASK_FAILURE"
        reason = "trial ended via max_runtime_s, not genuine task completion -- never read as success"
    elif not reached:
        task_outcome = "TASK_FAILURE"
        missing = [r.robot_id for r in results if not r.reached]
        reason = f"not all robots reached the goal (missing: {missing})"
    else:
        task_outcome = "SUCCESS"
        reason = f"all {len(results)} robots held the goal region for >= {hold_time_s}s; no collision; min pairwise distance {min_dist:.6f}m >= safety radius"

    return TaskVerdict(
        data_validity=data_validity,
        data_validity_reasons=data_validity_reasons,
        task_outcome=task_outcome,
        task_outcome_reason=reason,
        all_robots_reached_goal=reached,
        completed_robot_count=completed_count,
        total_robot_count=len(results),
        makespan_s=makespan_s(results),
        minimum_pairwise_distance_m=min_dist,
        safety_margin_m=safety_margin,
        collision_count=collision_count,
    )
