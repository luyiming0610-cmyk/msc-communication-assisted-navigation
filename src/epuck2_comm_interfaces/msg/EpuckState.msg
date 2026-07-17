# Compact robot state exchanged between e-puck2-Comm peers.
# SI units are used throughout. Sequence numbers are per publisher.

uint8 PROTOCOL_VERSION=1

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_WEBOTS=1
uint8 SOURCE_HARDWARE=2
uint8 SOURCE_VIRTUAL=3

uint8 OBSTACLE_UNKNOWN=0
uint8 OBSTACLE_CLEAR=1
uint8 OBSTACLE_FRONT=2
uint8 OBSTACLE_LEFT=3
uint8 OBSTACLE_RIGHT=4
uint8 OBSTACLE_MULTIPLE=5

uint8 FLAG_ODOM_VALID=1
uint8 FLAG_IR_VALID=2
uint8 FLAG_TOF_VALID=4

# Protocol and identity
uint8 version
uint8 source
uint16 robot_id
uint32 sequence
builtin_interfaces/Time stamp

# Planar pose and velocity
float32 x_m
float32 y_m
float32 yaw_rad
float32 linear_velocity_mps
float32 angular_velocity_rps

# Local obstacle summary. +Inf means no valid return within range.
float32 front_distance_m
float32 left_distance_m
float32 right_distance_m
uint8 obstacle_status
uint8 validity_flags

# controller_v4_full_sensor_bypass_20260717: additive fields, appended so
# v1/v2/v3 code (which only ever reads front/left/right_distance_m,
# obstacle_status, validity_flags) is unaffected. Zone-aggregated from all
# 8 raw ps0-ps7 IR sensors (state_publisher.py), filling the rear coverage
# gap that front/left/right (min(ps0,ps7,tof)/min(ps5,ps6,ps7)/
# min(ps0,ps1,ps2)) never had: ps3 (right-rear) and ps4 (left-rear) were
# never read by any prior version. +Inf means no valid return within range,
# matching front/left/right's existing convention.
float32 left_front_m
float32 left_mid_m
float32 left_rear_m
float32 right_front_m
float32 right_mid_m
float32 right_rear_m
