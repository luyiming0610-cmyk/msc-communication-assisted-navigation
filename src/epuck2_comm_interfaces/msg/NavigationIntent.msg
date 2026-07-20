# A robot's own desired-heading advisory, published by a per-robot
# goal_navigator node and consumed only by that SAME robot's own
# cooperative_avoider (never cross-robot -- GoalAnnouncement is the
# cross-robot channel). Advisory only: CPA avoidance, local IR/ToF
# safety-stop, and stale-state handling in cooperative_avoider always
# take priority over this input.

uint32 PROTOCOL_VERSION=1

uint32 protocol_version
uint32 source_robot_id
uint32 sequence
builtin_interfaces/Time production_stamp
float64 desired_heading_rad
bool valid
