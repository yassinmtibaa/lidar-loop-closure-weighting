# LiDAR preprocessing + learned feedback loop

Goal: improve LiDAR odometry/mapping on real data with filtering and a learned
component trained against a downstream metric, then use the result to make
simulation more realistic.

## Setup
- See docs/environment.md for hardware and versions.
- ROS 2 Jazzy on Ubuntu 24.04, workspace ~/ros2_ws.
- KISS-ICP and evo installed with pipx.
- KISS-SLAM slam.py patched for env-controlled information matrices (docs/kiss_slam_weights_patch.diff);
  unset env vars = original behaviour. Main result: KS_ODOM_INFO=KS_CLOSURE_INFO=1,1,1,1e3,1e3,1e3
- KISS-ICP needs a one-line patch (TUM export crash on newer NumPy):
  see docs/pipeline_patch.diff. Reapply after any pipx upgrade/reinstall.

## Data
KITTI Odometry (Velodyne, calib, poses) unzipped to
data/kitti/dataset/{sequences,poses}. Not tracked in git.

## Reproduce the baseline
    ./scripts/run_baseline.sh baseline
    ./scripts/eval_exp.sh baseline > results/baseline.csv

## Layout
- scripts/  run and evaluation scripts
- configs/  one KISS-ICP config per experiment
- results/  experiment registry, CSV summaries (raw runs are not tracked)
- docs/     lab log, decisions, figures

## Environment notes (pipx)
- PyYAML injected into the KISS-ICP env: pipx inject kiss-icp pyyaml
- KISS-SLAM env needs: pipx inject kiss-slam pillow matplotlib
- KISS-ICP env needs a one-line patch for the TUM export (docs/pipeline_patch.diff).
  The KISS-SLAM env did not hit that bug.
