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
| `PROJECT_HANDOFF.md` | Detailed project status, evidence boundaries, build instructions and known limitations |
| `experiments/EXPERIMENT_INDEX.md` | Experiment taxonomy and evidence entry points |
| `experiments/experiment_registry.csv` | Machine-readable classification of formal, diagnostic and excluded work |

## Software environment

The simulation studies used Webots R2025a with ROS 2 Humble under Ubuntu
22.04 in WSL2. Stage 4 connected the WSL2 ROS 2 Humble environment to a
Raspberry Pi running Raspbian GNU/Linux 10, ROS 2 Foxy and Python 3.7.3.
Exact versions and study-specific settings are retained in the experiment
metadata and dissertation.

## Build and test

The ROS 2 package is normally built from the WSL workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/epuck_ws
colcon build --packages-select epuck2_comm --symlink-install
colcon test --packages-select epuck2_comm
colcon test-result --verbose
```

For evidence inspection, begin with `PROJECT_HANDOFF.md`, then read
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

Some physical and diagnostic records contain environment-specific network
information. They must not be placed in a public archive without a deliberate
access and redaction decision, because altering a raw bag would also invalidate
its recorded checksum.

## Version identification

The dissertation should identify the exact evaluated version using a tag and
full commit hash. Obtain the current values with:

```bash
git describe --tags --always
git rev-parse HEAD
```

The repository supports inspection and reconstruction of the recorded
procedure. It does not guarantee identical trajectories across different
operating systems, simulator releases or physical platforms.
