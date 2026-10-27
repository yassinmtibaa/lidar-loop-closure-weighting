# Running Phase 2.4 on the supercomputer

What it does: the 31-subset sensor sweep (noise model v5) on the three TUNE sessions, 3 KISS-ICP inputs each,
with each session's noise fitted on the other two (leave-one-out). Protocol and predictions: `../DECISIONS.md`,
entry "2026-10-27: 2.4 tune-session protocol". Validation and test sessions are not touched.

## 1. Environment
- KISS-ICP 1.3.0 with scipy and pyyaml (`pip install kiss-icp==1.3.0 scipy pyyaml`), plus the TUM-export patch
  `../../docs/pipeline_patch.diff` (otherwise sessions whose ground truth starts with NaN rows crash at export).
- Fusion env (Python 3.12): `uv venv .venv --python 3.12 && uv pip install --python .venv/bin/python -r requirements-fusion.txt`
  (gtsam 4.3.0, numpy 2.5.4, scipy 1.18.1). Record any version difference in `../STATUS.md`.
- Check: `.venv/bin/python -m pytest -q tests` (13 tests; the reader tests need the smoke session 2013-01-10).

## 2. Data (per tune session, under `data/nclt/<session>/`)
`velodyne_sync/` (unpacked scans), `ms25.csv`, `kvh.csv`, `wheels.csv`, `gps.csv`, and
`data/nclt/ground_truth/groundtruth_<session>.csv`. About 15 GB per session; no images, no velodyne_hits.bin.
Smoke session 2013-01-10 too if you want the tests.

## 3. Run
```
JOBS=32 supercomputer/run_tune.sh          # or one step: icp | noise | sweep
```
Outputs in `results/`: `icp_tune_metrics.txt`, `noise_v5_<session>.json/.txt`,
`subsets_<session>_noise_v5_<other>+<other>.csv/.log`. Then judge with the rule in DECISIONS (3-run ranges must not
overlap and the mean ATE change must exceed 5%).

## Runtime (from the smoke session, 5,104 frames, 8 cores)
- With LiDAR in the subset: seconds to a minute per run.
- IMU subsets without LiDAR: the slow ones (up to ~300 Dogleg steps, ~30 s each on 5k frames).
- Tune sessions have 12,962 / 24,014 / 33,463 frames: expect roughly 3x / 5x / 7x the smoke time, more for the slow
  IMU subsets. Memory: one Pool worker per job, each holding the session's measurements (a few GB at most).
