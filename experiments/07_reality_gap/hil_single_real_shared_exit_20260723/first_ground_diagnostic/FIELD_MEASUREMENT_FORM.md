# First ground diagnostic -- field measurement form

Fill in every field with a real, measured value before this diagnostic
may run. Use the literal string `UNCONFIRMED_PHYSICAL_MEASUREMENT` for
anything not yet measured -- **never write an estimate, a guess, or a
value copied from a simulation world file in place of a real
measurement.** Once completed, transfer the confirmed values into
`../tools/ground_diagnostic_params.json`'s `measured_geometry` block
(never into `hil_frozen_params.json` -- that file is the separate,
formal shared-exit geometry record).

| Field | Value |
|---|---|
| Room | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Date | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Usable test-area length (m) | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Usable test-area width (m) | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Floor material and condition | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Coordinate origin marker (description/location) | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Start x (m) | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Start y (m) | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Start yaw (rad) | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Intended travel direction | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Stop-line distance from start (m) | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Minimum boundary clearance (m) | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Wall and obstacle locations | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Emergency-stop operator position | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Wi-Fi observation | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Measured stopping clearance (m, post-run) | UNCONFIRMED_PHYSICAL_MEASUREMENT |
| Observer | UNCONFIRMED_PHYSICAL_MEASUREMENT |

## Notes

- "Measured stopping clearance" is filled in **after** the run (the
  remaining distance between the robot's final position and the
  stop-line/boundary), not before -- it is a post-run observation, not
  a precondition, but is still recorded here so both plan and outcome
  live in one document.
- This form intentionally does not include exit location, parking
  zone, or search-waypoint fields -- this diagnostic never uses them
  (see `FIRST_GROUND_DIAGNOSTIC_SPEC.md`'s exclusion list). Those
  remain the formal shared-exit experiment's own, separate,
  still-outstanding measurements in `hil_frozen_params.json`.
- Once every field above is a real value (not
  `UNCONFIRMED_PHYSICAL_MEASUREMENT`), transfer the pre-run fields into
  `../tools/ground_diagnostic_params.json`'s `measured_geometry` block
  and re-run `run_ground_diagnostic_preflight.sh` -- it will report
  `GROUND_DIAGNOSTIC_PREFLIGHT_PASS` only once every
  `required_before_ground_motion` path in that file is confirmed.
