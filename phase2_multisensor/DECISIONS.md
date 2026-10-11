# Decisions and predictions (Phase 2)

## 2026-10-10: project start
- Question and hypothesis as in README.md.
- Primary dataset NCLT, secondary HeLiPR (see DATASETS.md).
- RTK GPS excluded from fusion inputs (feeds ground truth). Camera deferred (size).
- Phase 1 conventions kept: predictions before runs, fixed split, noise bands, test set once.
- Fusion framework for 2.2+: GTSAM (KISS-SLAM cannot fuse non-LiDAR sensors).

## 2026-10-10: Phase 2.0 predictions (fixed before download)
- P2.0a: KISS-ICP runs on an NCLT session via `--dataloader nclt` without code changes.
- P2.0b: the ground-truth CSV can be converted to a format evo reads, and KISS-ICP odometry ATE on a full
  session is under 1% of path length (~55 m on 5.5 km). If it is far larger, suspect frame or timestamp
  alignment before blaming the method.
