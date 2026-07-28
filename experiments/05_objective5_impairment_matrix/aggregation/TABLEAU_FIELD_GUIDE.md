# Tableau field guide -- Objective 5 A-G aggregation

All source CSVs referenced below are produced by
`aggregate_objective5_matrix_a_to_g.py --skip-plots` and live in this
directory (`plot_data/` subfolder for the per-figure extracts,
`per_trial_canonical.csv` for the full 35-row dataset -- note the
canonical file is an **audit** artifact and still uses the literal text
`NOT_AVAILABLE` inside its columns; it is not the recommended Tableau
source. The `plot_data/` CSVs described below are the Tableau-safe
versions, built specifically so every numeric column stays a true
number.).

## Numeric-column policy (read this first)

Every `plot_data/*.csv` field that is plotted as a **continuous numeric
axis** is exported as a plain, parseable number (or an **empty** cell --
never the text `NOT_AVAILABLE`, and never `0`) together with a sibling
status column named `<field>_value_status`, whose value is either
`AVAILABLE` or `NOT_AVAILABLE`. Concretely, per file:

| File | Numeric column(s) | Sibling status column(s) |
|---|---|---|
| `min_interrobot_distance.csv` | `minimum_interrobot_distance_m` | `minimum_interrobot_distance_m_value_status` |
| `safety_margin.csv` | `safety_margin_m` | `safety_margin_m_value_status` |
| `realised_loss.csv` | `authoritative_drop_fraction_epuck1_to_epuck2`, `authoritative_drop_fraction_epuck2_to_epuck1` | one status column per numeric column, same naming pattern |
| `message_age.csv` | `measured_message_age_mean_epuck1_to_epuck2`, `measured_message_age_mean_epuck2_to_epuck1` | one status column per numeric column |
| `reordered_count.csv` | `reordered_count_epuck1_to_epuck2`, `reordered_count_epuck2_to_epuck1` | one status column per numeric column |
| `f_stale_stop_duration.csv` | `stale_stop_duration_s` | `stale_stop_duration_s_value_status` |
| `task_outcome.csv` | *(none -- categorical only, see below)* | *(none)* |

`condition_id`, `trial_id`, `drop_mechanism`, and `task_outcome` are
**categorical/text columns**, never plotted on a numeric axis, and never
carry a status column -- `task_outcome` in particular is always one of
the four defined enum values for a counted trial and is never
`NOT_AVAILABLE`.

**How to build every figure correctly in Tableau:**
1. Connect the CSV; confirm Tableau auto-detects each `_m`/`_s`/`_count`/`_fraction` column as **Number (decimal)** or **Number (whole)**, not String. If it detects String, the file has an unexpected non-numeric value in that column -- stop and re-check the CSV rather than forcing the type.
2. Put the numeric field on Rows/Columns as normal. Empty cells become Tableau **Null** automatically -- this is correct and requires no extra step.
3. **Never enable "Show missing values"** (right-click the header axis) on any sheet built from these files -- that option interpolates/zero-fills gaps in a discrete dimension's domain, which would fabricate a data point where `value_status=NOT_AVAILABLE` says none exists.
4. To make an unavailable point visible instead of silently absent, drag the matching `<field>_value_status` field onto Filter or onto Color/Shape as a secondary encoding, so a reviewer can see "this condition/trial had no measurement" explicitly rather than a blank gap that looks like an oversight.
5. **Null does not mean zero.** A `0` in `reordered_count_epuck1_to_epuck2` (with `value_status=AVAILABLE`) is a real, measured zero-reordering result. An empty cell (with `value_status=NOT_AVAILABLE`) means the metric could not be computed for that trial. Never treat these as interchangeable, and never let a Tableau default (e.g. a SUM() that silently treats Null as 0 in an aggregation) blur the distinction -- keep every mark at the individual-trial level (see below), where this cannot happen.

**Current-data note**: in the present 35-row A-G dataset, every numeric
`plot_data` column is `AVAILABLE` for all rows that appear in that file (0
occurrences of `NOT_AVAILABLE` were found in any `plot_data/*.csv` at the
time this guide was last verified). No plot-data column required a
correction for this run. The empty/null + status-column mechanism above
is a standing structural guarantee for any future condition (H, I, ...)
added to this aggregation, not a fix applied to existing values.

For every figure: build the mark as **individual data points** (Tableau
"Circle" or "Square" mark type with `trial_id` on the Detail shelf so all
5 trials per condition render as 5 separate marks), never as a single
aggregated bar per condition.

---

## 1. Minimum inter-robot distance by condition

- **Source CSV**: `plot_data/min_interrobot_distance.csv`
- **Columns**: `condition_id`
- **Rows**: `minimum_interrobot_distance_m` (disable default aggregation -- plot as individual marks, not `AVG`)
- **Detail**: `trial_id`
- **Outcome/category field**: none required from this CSV alone; optionally blend on `trial_id` with `per_trial_canonical.csv` to color marks by `task_outcome` (recommended, so C05/G02's `UNSAFE_FAILURE` points are visually distinct)
- **Required reference line**: constant reference line at **y = 0.14** (the frozen `safety_radius_m`), labeled "safety radius (0.14 m)"
- **Missing-value treatment**: `minimum_interrobot_distance_m_value_status` is `AVAILABLE` for all 35 current rows. If a future row is `NOT_AVAILABLE`, the numeric cell is empty (Tableau Null) -- filter it via the status column or display it as an explicit "not measured" annotation; never let it render as 0 m.
- **Required caption**: "n=5 formal valid trials per condition; descriptive results only; no statistical-significance or broad-generalisation claim."

## 2. Safety margin by condition

- **Source CSV**: `plot_data/safety_margin.csv`
- **Columns**: `condition_id`
- **Rows**: `safety_margin_m` (individual marks, not aggregated)
- **Detail**: `trial_id`
- **Outcome/category field**: optional blend with `task_outcome` from `per_trial_canonical.csv`, as above -- recommended to highlight C05 and G02's negative-margin points
- **Required reference line**: constant reference line at **y = 0.0**, labeled "zero safety margin"
- **Missing-value treatment**: `safety_margin_m_value_status` is `AVAILABLE` for all 35 current rows; same null-handling rule as figure 1 for any future gap.
- **Required caption**: "n=5 formal valid trials per condition; descriptive results only; no statistical-significance or broad-generalisation claim." Additionally state: "Trial C05 and Trial G02 are genuine, valid, unretried UNSAFE_FAILURE results (negative margin), not outliers removed from this figure."

## 3. Task outcome by condition

- **Source CSV**: `plot_data/task_outcome.csv`
- **Columns**: `condition_id`
- **Rows**: `task_outcome` (discrete/categorical axis, not a measure -- this column has no numeric or status counterpart)
- **Detail**: `trial_id` (so each of the 5 trials per condition renders as its own mark within its outcome row/category)
- **Outcome/category field**: `task_outcome` used for color (expected values: `SUCCESS`, `SAFE_DEGRADATION`, `UNSAFE_FAILURE`, `NOT_EVALUABLE` -- keep all four as defined categories in the color legend even though `SAFE_DEGRADATION`/`NOT_EVALUABLE` do not occur in the current 35 rows, so the legend does not silently imply they are impossible)
- **Required reference line**: none (categorical axis)
- **Missing-value treatment**: `task_outcome` is never `NOT_AVAILABLE` in this schema (it is always one of the four enum values for a counted trial); no null/empty handling is needed for this figure.
- **Required caption**: "n=5 formal valid trials per condition; descriptive results only; no statistical-significance or broad-generalisation claim." Additionally state: "SAFE_DEGRADATION and NOT_EVALUABLE do not occur in this A-G dataset to date; their absence is not evidence they cannot occur."

## 4. Authoritative realised packet loss by mechanism

- **Source CSV**: `plot_data/realised_loss.csv` (columns: `condition_id`, `trial_id`, `drop_mechanism`, `authoritative_drop_fraction_epuck1_to_epuck2`, `authoritative_drop_fraction_epuck1_to_epuck2_value_status`, `authoritative_drop_fraction_epuck2_to_epuck1`, `authoritative_drop_fraction_epuck2_to_epuck1_value_status`)
- **Data-prep step required first**: in Tableau's data source pane, use **Pivot** on the two `authoritative_drop_fraction_*` numeric columns to produce `direction` and `authoritative_drop_fraction` fields. Pivot the two matching `..._value_status` columns the same way (into a parallel `value_status` field) so each pivoted numeric row keeps its own availability flag.
- **Columns**: `condition_id`
- **Rows**: `authoritative_drop_fraction` (individual marks, not aggregated)
- **Detail**: `trial_id`, `direction`
- **Outcome/category field**: `drop_mechanism` for color (expected values: `NONE`, `INDEPENDENT_BERNOULLI`, `SCHEDULED_OUTAGE`, `COMBINED` -- this is a configuration-derived tag, not a measured value; never use it as a substitute for the plotted fraction itself)
- **Required reference line**: none fixed across conditions (configured `drop_probability` differs by condition: 0 / 0 / 0 / 0 / 0.15 / 0(outage-only) / 0.10). If a per-condition configured-vs-realised comparison is wanted, add a second reference band per condition from `configured_vs_realised_impairment.csv`'s `configured_drop_probability` column rather than one fixed line.
- **Missing-value treatment**: currently `AVAILABLE` for all 35 rows both directions. If a future direction's `received_count` is 0, its fraction cell is empty and `value_status=NOT_AVAILABLE` -- filter it out of the fraction axis explicitly and report the excluded trial count in the caption, never treat as 0% loss.
- **Required caption**: "n=5 formal valid trials per condition; descriptive results only; no statistical-significance or broad-generalisation claim." Additionally state: "Realised loss is read only from each trial's own relay counters (matrix_analysis.json), never inferred from the configured drop_probability. Different conditions use different loss mechanisms (independent Bernoulli, scheduled outage, or combined) -- do not visually compare loss fractions across mechanism types as if they were the same phenomenon."

## 5. Measured message age by condition

- **Source CSV**: `plot_data/message_age.csv` (columns: `condition_id`, `trial_id`, `measured_message_age_mean_epuck1_to_epuck2`, `measured_message_age_mean_epuck1_to_epuck2_value_status`, `measured_message_age_mean_epuck2_to_epuck1`, `measured_message_age_mean_epuck2_to_epuck1_value_status`)
- **Data-prep step required first**: pivot the two numeric `measured_message_age_mean_*` columns into `direction` and `measured_message_age_mean_s`, and pivot the two matching `..._value_status` columns in parallel, as in figure 4.
- **Columns**: `condition_id`
- **Rows**: `measured_message_age_mean_s` (individual marks, not aggregated)
- **Detail**: `trial_id`, `direction`
- **Outcome/category field**: `direction` for color/shape (two values: `epuck1_to_epuck2`, `epuck2_to_epuck1`)
- **Required reference line**: optional -- per-condition configured `delay_s` reference line if blended with `configured_vs_realised_impairment.csv` (values: A=0.0, B=0.2, C=1.0, D=0.15, E=0.0, F=0.0, G=0.2), to show measured-vs-configured directly. Not a single fixed line across all conditions.
- **Missing-value treatment**: currently `AVAILABLE` for all 35 rows both directions; same null-handling rule as figure 4 for any future gap.
- **Required caption**: "n=5 formal valid trials per condition; descriptive results only; no statistical-significance or broad-generalisation claim." Additionally state: "Condition A's near-zero message-age values are a simulation-clock-resolution artifact (RESOLUTION_LIMITED), not a measurement of real physical network transport delay, and must not be visually compared to B-G's values as if on the same physical footing."

## 6. Reordered-message count by condition

- **Source CSV**: `plot_data/reordered_count.csv` (columns: `condition_id`, `trial_id`, `reordered_count_epuck1_to_epuck2`, `reordered_count_epuck1_to_epuck2_value_status`, `reordered_count_epuck2_to_epuck1`, `reordered_count_epuck2_to_epuck1_value_status`)
- **Data-prep step required first**: pivot the two numeric `reordered_count_*` columns into `direction` and `reordered_count`, and pivot the two matching `..._value_status` columns in parallel, as in figures 4-5.
- **Columns**: `condition_id`
- **Rows**: `reordered_count` (individual marks, not aggregated)
- **Detail**: `trial_id`, `direction`
- **Outcome/category field**: `direction` for color/shape
- **Required reference line**: none
- **Missing-value treatment**: a value of **0 with `value_status=AVAILABLE` is genuine, not missing**, for every A/B/C/E/F trial (these conditions have `jitter_s=0` by frozen configuration, so zero reordering is the correct, expected measured result) -- do not filter out or visually suppress these zero-valued, `AVAILABLE` points; they are the true no-jitter reference points against D and G. Only a row with `value_status=NOT_AVAILABLE` (empty numeric cell) represents an actual missing measurement, and none currently exists in this file.
- **Required caption**: "n=5 formal valid trials per condition; descriptive results only; no statistical-significance or broad-generalisation claim." Additionally state: "Zero reordered-message counts for Conditions A, B, C, E and F reflect jitter_s=0 by frozen configuration, not a missing measurement; only Conditions D (jitter_s=0.30) and G (jitter_s=0.20) exercise message reordering in this matrix."
