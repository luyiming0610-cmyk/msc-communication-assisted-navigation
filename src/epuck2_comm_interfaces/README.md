# epuck2_comm_interfaces

`EpuckState.msg` is the compact, strongly typed wire-level state for the
e-puck2-Comm project. It replaces the earlier `std_msgs/String` JSON prototype.

## Design decisions

- Numeric `robot_id` avoids repeatedly transmitting names such as `epuck1`.
- `sequence` supports gap-based loss estimation and duplicate detection.
- The publisher timestamp supports message-age and latency measurement.
- Pose and velocity enable closest-point-of-approach collision prediction.
- Three obstacle distances provide a controller-independent local summary.
- A validity bit mask avoids three separate validity fields.
- `float32` is sufficient for the e-puck arena scale and halves numeric payload
  size compared with `float64`.

## Planned evaluation

The serialized CDR size and serialization/deserialization time of this message
will be compared with the existing JSON prototype. Periodic and event-triggered
publication policies will then be compared using message rate, bandwidth,
latency, stale-state rate and collision-avoidance performance.
