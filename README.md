# Communication-Assisted Multi-Robot Navigation

This repository contains the ROS 2 source code, frozen configurations,
derived evidence and analysis materials used in an MSc dissertation on
communication-assisted navigation with e-puck2 robots.

The work evaluates an implemented communication and navigation system under
controlled conditions. It does not propose a new network protocol or claim a
general solution to multi-robot collision avoidance.

## Evidence covered by the dissertation

| Study | Scope | Formal comparison |
|---|---|---|
| A–G | Two simulated e-puck2 robots under controlled delay, jitter, loss, outage and reordering | Five valid trials per condition; 35 trials in total |
| N2 | Two-robot shared-exit task | Five matched `COMM_OFF`/`COMM_ON` pairs |
| N3 | Three-robot shared-exit extension with multi-peer selection | Five matched `COMM_OFF`/`COMM_ON` pairs |
| Stage 4 | One physical e-puck2 and one software-only virtual peer | One bounded HIL proof of the virtual-to-physical event path |

Stage 4 is not a two-physical-robot experiment and is not a statistical
simulation-to-reality comparison. Pilot, diagnostic, legacy and exclusionary
runs are kept separate from formal results.

## Repository structure

| Path | Contents |
|---|---|
| `src/` | ROS 2 interfaces, state exchange, impairment relay, navigation controller and tests |
| `experiments/` | Frozen configurations, trial indices, derived records, validation outputs and evidence manifests |
| `docs/` | Design notes and physical-validation scope documentation |
| `REPRODUCIBILITY.md` | Assessor-facing environment, evidence and verification instructions |
| `PROJECT_HANDOFF.md` | Development history and detailed project provenance; not the primary reproduction guide |
| `experiments/EXPERIMENT_INDEX.md` | Experiment taxonomy and evidence entry points |
| `experiments/experiment_registry.csv` | Machine-readable classification of formal, diagnostic and excluded work |

## Software environment

The simulation studies used Webots R2025a with ROS 2 Humble under Ubuntu
22.04 in WSL2. Stage 4 connected the WSL2 ROS 2 Humble environment to a
Raspberry Pi running Raspbian GNU/Linux 10, ROS 2 Foxy and Python 3.7.3.
Exact versions and study-specific settings are retained in the experiment
metadata and dissertation.

## Obtain the submission snapshot

The assessor-facing snapshot is identified by the annotated tag
`dissertation-submission-final`. Git LFS is required to materialise the ROS bag
databases. On Windows, use a short destination path and enable long paths for
the clone because several retained evidence names are necessarily descriptive.

```bash
git lfs install
git -c core.longpaths=true clone --branch dissertation-submission-final \
  https://github.com/luyiming0610-cmyk/msc-communication-assisted-navigation.git
cd msc-communication-assisted-navigation
git lfs pull
git rev-list -n 1 dissertation-submission-final
git lfs ls-files
```

Cloning under WSL/Linux avoids the Windows legacy path-length limit. A GitHub
source-code ZIP is not a substitute for the commands above because it may
contain Git LFS pointer files instead of the bag content.

## Build and test

The ROS 2 package is normally built from the WSL workspace:

```bash
source /opt/ros/humble/setup.bash
cd msc-communication-assisted-navigation
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to epuck2_comm --symlink-install
colcon test --packages-select epuck2_comm
colcon test-result --verbose
```

For evidence inspection, begin with `REPRODUCIBILITY.md`, then read
`experiments/EXPERIMENT_INDEX.md` and the relevant condition or study summary.
The derived records retain trial identifiers, configurations, validity
classifications and SHA-256 manifests so that reported results can be traced
to their evidence sources.

## Raw evidence

ROS 2 bag databases are binary experimental records and are stored through
Git LFS rather than as ordinary Git objects. Each available `.db3` file is
retained with its corresponding `metadata.yaml`. The complete path, byte size
and SHA-256 digest of every retained bag file are recorded in
`docs/RAW_ROSBAG_INVENTORY.csv`. Derived summaries are not a substitute for
the raw recordings.

The formal Stage 4 evidence is retained under
`experiments/07_reality_gap/hil_single_real_shared_exit_20260723/formal_evidence/`.
It contains environment-specific technical metadata that the author explicitly
approved for public release. No passwords, private keys or API credentials are
included. Altering a raw record would invalidate its retained checksum.

## Version and integrity identification

Resolve the immutable submission tag and verify the retained evidence with:

```bash
git rev-list -n 1 dissertation-submission-final
python3 tools/verify_evidence.py
```

The repository supports inspection and reconstruction of the recorded
procedure. It does not guarantee identical trajectories across different
operating systems, simulator releases or physical platforms.
