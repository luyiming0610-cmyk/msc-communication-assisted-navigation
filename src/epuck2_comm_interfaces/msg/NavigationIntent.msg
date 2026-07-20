# A robot's own desired-heading advisory, published by a per-robot
# goal_navigator node and consumed only by that SAME robot's own
# cooperative_avoider (never cross-robot -- GoalAnnouncement is the
# cross-robot channel). Advisory only: CPA avoidance, local IR/ToF
# safety-stop, and stale-state handling in cooperative_avoider always
# take priority over this input.

# navigation_phase values (string enum, not a Webots/ROS enum type --
# kept as a plain string so downstream log/CSV tooling needs no extra
# message-definition lookup):
#   SEARCH       -- robot has not yet located/switched to the exit
#   GO_TO_EXIT   -- robot is actively heading for the known exit target
#   ARRIVED_HOLD -- robot has latched arrival; irreversible for the
#                   rest of the trial. desired_linear_speed_mps is 0
#                   and desired_heading_rad is frozen at the last
#                   stable heading while in this phase.

uint32 PROTOCOL_VERSION=1

uint32 protocol_version
uint32 source_robot_id
uint32 sequence
builtin_interfaces/Time production_stamp
float64 desired_heading_rad
float64 desired_linear_speed_mps
string navigation_phase
bool valid
