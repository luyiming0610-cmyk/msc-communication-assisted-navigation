# One-shot task-level exit/goal announcement, published by a robot that
# has discovered or already knows a shared goal location, so a peer that
# does not can navigate to it. Independent of EpuckState (PROTOCOL_
# VERSION=1, unmodified) -- this is a task announcement, not a robot's
# own kinematic state.

uint32 PROTOCOL_VERSION=1

uint32 protocol_version
uint32 source_robot_id
uint32 sequence
builtin_interfaces/Time production_stamp
string goal_id
float64 goal_x_m
float64 goal_y_m
bool valid
