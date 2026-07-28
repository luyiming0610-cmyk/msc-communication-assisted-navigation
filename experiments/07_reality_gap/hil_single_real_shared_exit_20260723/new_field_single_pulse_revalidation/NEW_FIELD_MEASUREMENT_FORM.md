# New-field single-pulse revalidation -- field measurement form

Fill in every field with a real, measured value before this
revalidation may run. Use the literal string
`UNCONFIRMED_PHYSICAL_MEASUREMENT` for any numeric field not yet
measured, and `false` for any not-yet-checked confirmation field --
**never write an estimate, a guess, or a value copied from the old
0.65m x 0.25m field's own form in place of a real measurement, and
never mark a confirmation field true before it has actually been
checked.** The "Parameter path" column is the exact dotted path in
`../tools/new_field_geometry_params.json` that this row must be
transferred into once measured/checked (never into
`ground_diagnostic_params.json`, the old field's own record, and never
into `hil_frozen_params.json`, the separate formal shared-exit
geometry record). Rows with no parameter path are documentation only
and are never read by the preflight gate.

This form mirrors `../first_ground_diagnostic/FIELD_MEASUREMENT_FORM.md`'s
exact structure for a genuinely new, separate physical field -- it does
not modify, extend, or overwrite that form or the old field's own
record in any way.

| Field | Value | Parameter path |
|---|---|---|
| Room | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Date | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Usable test-area length (m) | 1.40 | `measured_geometry.test_area_length_m` |
| Usable test-area width (m) | 1.00 | `measured_geometry.test_area_width_m` |
| Floor material and condition | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Floor condition checked and acceptable for the diagnostic | (per-session, see note) | session file: `floor_condition_confirmed` (`hil_ground_diagnostic_session.py`, not this JSON file) |
| Coordinate origin marker (description/location) | Rear-left physical corner, standing behind the robot and looking along its intended forward travel direction | (documentation only) |
| Start x (m, robot body centre) | 0.30 | `measured_geometry.start_x_m` |
| Start y (m, robot body centre) | 0.50 | `measured_geometry.start_y_m` |
| Start yaw (rad) | 0.0 | `measured_geometry.start_yaw_rad` |
| Intended travel direction | forward_along_length | `measured_geometry.travel_direction` |
| Stop line, absolute x (m) | 1.20 | `documentation_only_reference.absolute_stop_line_x_m` (documentation/traceability only -- the gated field is the derived distance; a consistency test enforces `absolute_stop_line_x_m - start_x_m == stop_line_distance_m`) |
| Stop-line distance from start (m) | 0.90 | `measured_geometry.stop_line_distance_m` |
| Central corridor, y minimum (m) | 0.30 | `measured_geometry.corridor_y_min_m` |
| Central corridor, y maximum (m) | 0.70 | `measured_geometry.corridor_y_max_m` |
| Manual reference line, absolute x (m) | 0.40 | (documentation only -- never a gate, see note) |
| Manual reference line measured from robot body centre (confirmed) | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Minimum real-boundary body-edge clearance (m) | 0.10 | `measured_geometry.min_boundary_clearance_m` |
| Intended travel path checked clear of any obstruction | (per-session, see note) | session file: `travel_path_clear_confirmed` (`hil_ground_diagnostic_session.py`, not this JSON file) |
| Wall and obstacle locations | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Wall and obstacle locations recorded | false | `environment.boundaries_and_obstacles_recorded` |
| Emergency-stop operator position | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Emergency-stop operator position checked | false | `safety.emergency_stop_position_confirmed` |
| Operator present at the emergency stop, confirmed | (per-session, see note) | session file: `operator_present_confirmed` (`hil_ground_diagnostic_session.py`, not this JSON file) |
| Wi-Fi observation | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Wi-Fi checked in the test area | (per-session, see note) | session file: `wifi_checked_in_test_area` (`hil_ground_diagnostic_session.py`, not this JSON file) |
| Rear body-edge clearance at start pose (m) | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Front body-edge clearance at start pose (m) | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Left body-edge clearance at start pose (m) | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Right body-edge clearance at start pose (m) | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |
| Measured stopping clearance (m, post-run) | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only -- post-run, never a preflight gate) |
| Observer | UNCONFIRMED_PHYSICAL_MEASUREMENT | (documentation only) |

## Notes

- Coordinate frame: origin is the rear-left physical corner; +x points
  forward along the 1.40 m length; +y points across the 1.00 m width,
  toward the operator's right when facing +x; yaw=0 points forward
  along +x. Start x/y refer to the **robot body centre**, never the
  front edge, rear edge, or wheels.
- **`stop_line_distance_m` is a distance from start, not an absolute
  coordinate.** The marked stop line is at absolute x=1.20 m; start
  x=0.30 m; so `stop_line_distance_m = 1.20 - 0.30 = 0.90 m`. Both the
  absolute line position and the derived distance are recorded above
  so this conversion is never silently redone incorrectly later.
- **The central corridor tape (y=0.30 to y=0.70) and the manual
  reference line (x=0.40) are human-visible references only.** The
  robot has no means of sensing either (no vision, no line-following,
  no IR tape detection, no automatic tape-recognition claim of any
  kind). Neither is an automated runtime gate. Corridor compliance is
  decided **after** the pulse, by human observation of whether any
  part of the robot body crossed either tape edge -- if it did, the
  attempt is `EXCLUDED`. The reference line is a manual
  distance-measurement aid only, never a stop condition.
- **`min_boundary_clearance_m` applies to the nearest REAL PHYSICAL
  field boundary in any direction** (the true 1.40 m / 1.00 m edges),
  measured from the nearest robot body edge -- never the corridor tape,
  never the stop line. This is the actual safety-relevant clearance
  fed into `evaluate_verdict()` via `--robot-stayed-within-measured-area`,
  exactly as used for `SRGRB_20260727_02`'s own exclusion.
- The four body-edge clearance rows (rear/front/left/right at the
  start pose) are documentation only, mirroring the old field's own
  form -- recorded as free-text notes once measured, never compared
  against an invented number, never a parameter-path field.
- "Measured stopping clearance" is filled in **after** the run, exactly
  as in the old field's form -- a post-run observation, not a
  precondition, never added to `required_before_ground_motion`.
- Room, Date, Coordinate origin marker, Observer, and the reference-line
  measurement-basis confirmation are documentation fields only -- never
  transferred into `required_before_ground_motion`, never gate execution.
- The confirmation rows (floor condition, travel path, obstacle
  recording, emergency-stop position, operator presence, Wi-Fi) follow
  the exact same discipline as the old field's form: the descriptive
  row records what was observed; the confirmation row records that the
  check was actually done; floor condition, travel path, operator
  presence, and Wi-Fi are genuinely per-session and live only in the
  separate, gitignored session-state file (`hil_ground_diagnostic_session.py`),
  never committed here as a permanent fact.
- **`boundaries_and_obstacles_recorded` and
  `emergency_stop_position_confirmed` are deliberately `false` in the
  shipped `new_field_geometry_params.json`** -- this is a genuinely new
  physical venue, and the old field's own confirmed values must never
  be assumed to carry over. `run_ground_diagnostic_preflight.sh
  pre-stack` will correctly report these two as unconfirmed (and
  nothing else) until they are explicitly checked and transferred here.
- Once every tracked field above is a real, confirmed value, transfer
  it into `../tools/new_field_geometry_params.json` at the exact path
  shown, confirm all four per-session fields via
  `hil_ground_diagnostic_session.py`, then run
  `GROUND_DIAGNOSTIC_PARAMS=../tools/new_field_geometry_params.json
  bash run_ground_diagnostic_preflight.sh pre-stack` -- never the
  default invocation, which would silently check the OLD field's file
  instead.
