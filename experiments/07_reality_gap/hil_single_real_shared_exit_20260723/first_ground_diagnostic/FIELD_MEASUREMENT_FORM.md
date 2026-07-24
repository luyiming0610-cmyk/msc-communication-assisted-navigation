# First ground diagnostic -- field measurement form

Fill in every field with a real, measured value before this diagnostic
may run. Use the literal string `UNCONFIRMED_PHYSICAL_MEASUREMENT` for
any numeric field not yet measured, and `false` for any not-yet-checked
confirmation field -- **never write an estimate, a guess, or a value
copied from a simulation world file in place of a real measurement, and
never mark a confirmation field true before it has actually been
checked.** The "Parameter path" column is the exact dotted path in
`../tools/ground_diagnostic_params.json` that this row must be
transferred into once measured/checked (never into
`hil_frozen_params.json` -- that file is the separate, formal
shared-exit geometry record). Rows with no parameter path are
documentation only and are never read by the preflight gate.

| Field | Value | Parameter path |
|---|---|---|
| Room | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Date | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Usable test-area length (m) | 0.65 | `measured_geometry.test_area_length_m` |
| Usable test-area width (m) | 0.25 | `measured_geometry.test_area_width_m` |
| Floor material and condition | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Floor condition checked and acceptable for the diagnostic | false | `environment.floor_condition_confirmed` |
| Coordinate origin marker (description/location) | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Start x (m) | 0.25 | `measured_geometry.start_x_m` |
| Start y (m) | 0.125 | `measured_geometry.start_y_m` |
| Start yaw (rad) | 0.0 | `measured_geometry.start_yaw_rad` |
| Intended travel direction | forward_along_length | `measured_geometry.travel_direction` |
| Stop-line distance from start (m) | 0.10 | `measured_geometry.stop_line_distance_m` |
| Minimum boundary clearance (m) | 0.10 | `measured_geometry.min_boundary_clearance_m` |
| Intended travel path checked clear of any obstruction | false | `environment.travel_path_clear_confirmed` |
| Wall and obstacle locations | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Wall and obstacle locations recorded | false | `environment.boundaries_and_obstacles_recorded` |
| Emergency-stop operator position | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Emergency-stop operator position checked | false | `safety.emergency_stop_position_confirmed` |
| Operator present at the emergency stop, confirmed | false | `safety.operator_present_confirmed` |
| Wi-Fi observation | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Wi-Fi checked in the test area | false | `network.wifi_checked_in_test_area` |
| Measured stopping clearance (m, post-run) | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only -- post-run, never a preflight gate) |
| Observer | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |

## Notes

- Recorded 2026-07-24: coordinate origin is the rear-left corner of the
  test area; +x points forward along the 0.65 m length, +y points from
  the left boundary toward the right/table-edge boundary, yaw=0 points
  forward along +x. Start x/y refer to the marked robot centre point.
  Stop-line distance from start (0.10 m) was measured from the robot
  centre, not from the front of the robot. All figures above are
  approximate tape measurements -- do not read them as millimetre-precise.
  Recorded (not a parameter-path field, not re-derived into the centre
  coordinates above) approximate body-edge clearances at the start
  pose: rear body edge to rear boundary ~0.205 m, front body edge to
  forward boundary ~0.370 m, left body edge to left boundary ~0.100 m,
  right body edge to right/table-edge boundary ~0.105 m.
- "Measured stopping clearance" is filled in **after** the run (the
  remaining distance between the robot's final position and the
  stop-line/boundary), not before -- it is a post-run observation, not
  a precondition, and has no parameter path in
  `ground_diagnostic_params.json`: it must never be added to
  `required_before_ground_motion` or otherwise gate execution. It is
  still recorded here so both plan and outcome live in one document.
- Room, Date, Coordinate origin marker, and Observer are documentation
  fields only, by design -- they are never transferred into
  `required_before_ground_motion` and never gate execution.
- The confirmation rows (floor condition, travel path, obstacle
  recording, emergency-stop position, operator presence, Wi-Fi) are
  separate from, and in addition to, the descriptive rows next to them:
  the descriptive row records what was observed; the confirmation row
  records that the check was actually done. Both must be filled in --
  the confirmation row must never be set `true` just because the
  descriptive row has text in it.
- This form intentionally does not include exit location, parking
  zone, or search-waypoint fields -- this diagnostic never uses them
  (see `FIRST_GROUND_DIAGNOSTIC_SPEC.md`'s exclusion list). Those
  remain the formal shared-exit experiment's own, separate,
  still-outstanding measurements in `hil_frozen_params.json`.
- Once every field with a parameter path above is a real, confirmed
  value (no numeric field still `UNCONFIRMED_PHYSICAL_MEASUREMENT`, no
  confirmation field still `false`), transfer them into
  `../tools/ground_diagnostic_params.json` at the exact path shown and
  re-run `run_ground_diagnostic_preflight.sh` -- it will report
  `GROUND_DIAGNOSTIC_PREFLIGHT_PASS` only once every
  `required_before_ground_motion` path in that file is confirmed (every
  numeric field measured, every confirmation field `true`).
