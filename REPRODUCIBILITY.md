# Reproducibility and Evidence Guide

This guide identifies the software, evidence and verification steps supporting
the dissertation results. It supports inspection and reconstruction of the
recorded procedure; it does not promise identical trajectories on another
operating system, simulator release or physical platform.

## 1. Canonical submission version

The assessor-facing version is the annotated tag
`dissertation-submission-final-v2`. Resolve the tag to its full commit identifier
after cloning:

```bash
git rev-list -n 1 dissertation-submission-final-v2
git status --short
```

The second command should print nothing. `PROJECT_HANDOFF.md` is retained as
development provenance, but this file is the primary assessor entry point.

## 2. Supported scope

The formal evidence comprises:

- A-G: five valid two-robot simulation trials under each of seven
  communication conditions (35 included trials). The retained D04 attempt is
  exclusionary evidence for the measurement-chain correction, not an extra
  included trial.
- N2: five matched `COMM_OFF`/`COMM_ON` two-robot shared-exit pairs.
- N3: five matched `COMM_OFF`/`COMM_ON` three-robot shared-exit pairs.
- Stage 4: one bounded physical HIL event using one physical e-puck2 and one
  software-only virtual peer.

Pilot, diagnostic, legacy and assessor-demo material is not part of the formal
comparison. The six assessor-demo directories exercise exclusion and
source-identity checks only; they are deliberately omitted from the repository.

## 3. Environment

The simulation studies used Webots R2025a and ROS 2 Humble on Ubuntu 22.04
under WSL2. The Stage 4 path joined this environment to a Raspberry Pi running
Raspbian GNU/Linux 10, ROS 2 Foxy and Python 3.7.3. Exact study parameters are
retained with the corresponding evidence. Rebuilding the software on another
platform does not recreate the original execution timing.

## 4. Clone and materialise Git LFS evidence

Install Git LFS before cloning. On Windows, clone to a short path and enable
long-path support; WSL/Linux is preferred.

```bash
git lfs install
git -c core.longpaths=true clone --branch dissertation-submission-final-v2 \
  https://github.com/luyiming0610-cmyk/msc-communication-assisted-navigation.git
cd msc-communication-assisted-navigation
git config core.longpaths true
git lfs pull
git lfs ls-files
```

The repository contains 163 `.db3` bag databases and their corresponding
`metadata.yaml` files. Their expected paths, byte sizes and SHA-256 values are
listed in `docs/RAW_ROSBAG_INVENTORY.csv`.

## 5. Build and unit tests

From a ROS 2 Humble environment:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to epuck2_comm --symlink-install
colcon test --packages-select epuck2_comm
colcon test-result --verbose
```

The interface package is built as a dependency of `epuck2_comm`. Webots and
hardware-dependent trial execution additionally requires the frozen local
environment described in the experiment documentation.

## 6. Evidence entry points

| Study | Formal evidence and canonical summary |
|---|---|
| A-G | `experiments/05_objective5_impairment_matrix/aggregation/` and the condition-specific `*_analysis/` directories |
| N2 | `experiments/10_cooperative_exit_navigation_20260720/shared_exit_formal_batch_summary/` |
| N3 | `experiments/10_cooperative_exit_navigation_20260720/shared_exit_n3_formal_batch_summary/` |
| Stage 4 | `experiments/07_reality_gap/hil_single_real_shared_exit_20260723/formal_evidence/stage4_20260803_144220/` |

`experiments/EXPERIMENT_INDEX.md` records the wider experiment taxonomy and
`experiments/experiment_registry.csv` provides a machine-readable catalogue.

The Stage 4 directory is a sealed copy of the WSL evidence root. Its
`FINAL_SHA256SUMS.txt` covers the final WSL, Raspberry Pi and physical-process
records. The run is an `n=1` proof of the virtual-to-physical event path, not a
two-physical-robot cooperation experiment or statistical reality-gap study.

## 7. Integrity verification

After `git lfs pull`, verify all formal bag files and Stage 4 evidence:

```bash
python3 tools/verify_evidence.py
```

For a metadata-only checkout in which Git LFS objects have not been
materialised, the pointer identities can be checked without downloading the
1 GB evidence set:

```bash
python3 tools/verify_evidence.py --lfs-mode pointer
```

Either command is read-only. It reports missing files, size differences and
SHA-256 mismatches and exits non-zero when verification fails.

## 8. Derived results

The retained dissertation-facing outputs are:

- A-G: `aggregation/per_trial_canonical.csv`,
  `aggregation/per_condition_summary.csv` and `aggregation/plot_data/`;
- N2 and N3: each batch summary's `paired_trial_results.csv`,
  `descriptive_statistics.csv`, `batch_summary.json` and event audit;
- Stage 4: `post_run_verification.json`, `pi_verifier_verdict.json` and
  `physical_measurements.json`.

The A-G aggregation pipeline can be checked without overwriting the canonical
outputs by writing to a new empty directory:

```bash
python3 experiments/05_objective5_impairment_matrix/tools/aggregate_objective5_matrix_a_to_g.py \
  --root experiments/05_objective5_impairment_matrix \
  --out /tmp/a_to_g_recomputed --skip-plots
```

Canonical derived files remain separate from regenerated outputs so that
verification does not silently alter the submitted evidence.

## 9. Known limits

- Application-level impairment represents the stream presented to the
  controller, not a full wireless network model.
- Five trials or pairs support descriptive comparison, not population-level
  inference.
- N2 and N3 differ in geometry and peer processing and do not establish a
  general team-size effect.
- `COMM_ON` combines goal sharing with peer-aware avoidance, so its effect
  cannot be attributed to `GoalAnnouncement` alone.
- Stage 4 contains one physical robot and one bounded event. Hardware timing,
  battery state and floor interaction are not controlled as in simulation.
