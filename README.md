# When accurate loop closures hurt local accuracy: factor weighting in KISS-SLAM on KITTI

KISS-SLAM gives every pose-graph factor an identity information matrix, so one radian of rotation costs the
same as one metre of translation. Loop-closure corrections are then absorbed as cheap heading changes, which
increases local error even when every closure is accurate. Weighting the rotational block by 10^3 removes most
of that cost on the affected KITTI sequences, including a held-out one, and improves global accuracy.

Full write-up: [`docs/report/report.pdf`](docs/report/report.pdf)

## Main result

| KITTI seq. | Role | KITTI trans. error: odometry / default SLAM / weighted | ATE (m): odometry / default / weighted |
|---|---|---|---|
| 05 | tuning | 0.342 / 0.517 / **0.333** % | 1.491 / 0.661 / **0.364** |
| 06 | validation | 0.281 / 0.325 / 0.332 % | 0.418 / 0.328 / 0.331 |
| 07 | validation | 0.375 / 0.386 / 0.391 % | 0.314 / 0.315 / 0.314 |
| 08 | test (held out) | 0.820 / 0.954 / **0.841** % | 2.266 / 2.524 / **2.126** |

"Weighted" = information matrix `diag(1,1,1,1e3,1e3,1e3)` on all factors. The weight was chosen on sequences
00-05, validated on 06-07, and run once on the test sequences 08-10 (09 and 10 have no closures and are
unchanged). Every hypothesis and acceptance rule was written down before each run: see
[`docs/decisions.md`](docs/decisions.md).

**Limits:** one dataset (KITTI), one SLAM system, one sensor; large effect on two sequences (05, 08); the local
loss on 06 is not recovered; +1.9% local error on 00; weight chosen from three candidates.

## How the diagnosis went

1. KISS-ICP baseline on KITTI 00-10; preprocessing sweep found no gain over the defaults.
2. KISS-SLAM cuts ATE by up to 74% but raises local error by up to 51% (sequence 05).
3. All 14 closures on 00/02/05 are more accurate than the odometry they replace (`scripts/audit_closures2.py`).
4. With closures disabled, local error returns to odometry level: the loss comes from applying closures.
5. Optimizer corrections are uncorrelated with where odometry actually drifted (`scripts/correction_alignment.py`).
6. Source code: every factor uses `np.eye(6)`. Rotation weighting fixes most of the loss.

Findings summaries: [`docs/findings_kiss_slam_kitti.md`](docs/findings_kiss_slam_kitti.md),
[`docs/findings_sim_noise.md`](docs/findings_sim_noise.md) (KITTI range-noise model and a range artifact in
Gazebo's `gpu_lidar`).

## Reproduce the main result

```bash
# 1. patch KISS-SLAM (env-controlled information matrices; unset = original behaviour)
S=$(pipx environment --value PIPX_LOCAL_VENVS)/kiss-slam/lib/python3.12/site-packages/kiss_slam/slam.py
patch "$S" docs/kiss_slam_weights_patch.diff

# 2. default vs rotation-weighted on one sequence
SEQS="05" ./scripts/run_slam.sh slam_default configs/kiss_slam_default.yaml 1
SEQS="05" KS_ODOM_INFO=1,1,1,1e3,1e3,1e3 KS_CLOSURE_INFO=1,1,1,1e3,1e3,1e3 \
  ./scripts/run_slam.sh w_rot1e3 configs/kiss_slam_default.yaml 1
```

Odometry baseline: `./scripts/run_baseline.sh baseline` then `./scripts/eval_exp.sh baseline > results/baseline.csv`.

## Setup

- Ubuntu 24.04, ROS 2 Jazzy; hardware and versions in [`docs/environment.md`](docs/environment.md).
- KISS-ICP, KISS-SLAM and evo installed with pipx.
- `pipx inject kiss-icp pyyaml` and `pipx inject kiss-slam pillow matplotlib` (undeclared dependencies).
- KISS-ICP needs a one-line patch for the TUM export on newer NumPy: [`docs/pipeline_patch.diff`](docs/pipeline_patch.diff).
  Reapply after any pipx upgrade or reinstall (same for the KISS-SLAM patch).

## Data

KITTI Odometry (Velodyne, calibration, poses) unzipped to `data/kitti/dataset/{sequences,poses}`. Not tracked in git.

## Layout

- `scripts/` run, evaluation and diagnostic scripts
- `configs/` one config per experiment (KISS-ICP and KISS-SLAM)
- `results/` experiment registry, CSV and text summaries (raw runs are not tracked)
- `docs/` decision log, findings, report, figures, patches
- `worlds/` Gazebo test worlds (simulator side)

## License

MIT, see [`LICENSE`](LICENSE).
