# Plan

## Sensors in scope (NCLT)
L = LiDAR (HDL-32E), I = IMU (GX3), F = fiber-optic gyro (KVH), W = wheel odometry, G = standard GPS.
Excluded for now: RTK GPS (part of ground truth), camera (size), Hokuyo 2D lasers.
5 sensors -> 31 subsets; subsets that cannot localize alone (I, F, I+F) are reported as such, not dropped.

## What is measured for every subset
1. Accuracy: ATE (aligned), RPE over 100 m, drift per km.
2. Error-estimation quality (consistency): NEES of the estimated covariance against the real error.
   Consistent estimator: NEES close to the state dimension.
3. Same subset run with (a) identity weights, (b) fixed physical weights, (c) predicted covariances.

Headline figure: accuracy and NEES vs number of sensors, one line per weighting scheme.

## Phases

| # | Phase | Output | Exit criterion |
|---|---|---|---|
| 2.0 | Setup | NCLT session downloaded, readers working, ground truth plotted | KISS-ICP runs on one session; evo reads its ground truth |
| 2.1 | Replicate Phase 1 on NCLT (LiDAR only) | KISS-ICP/KISS-SLAM baseline + rotation-weighted variant on the NCLT split | Pre-registered rule passes or fails on NCLT test sessions (either result goes in the report) |
| 2.2 | Fusion backbone | GTSAM factor graph: LiDAR odometry factors + IMU preintegration, any sensor switchable on/off | L-only in GTSAM matches KISS-ICP within noise; L+I runs end to end |
| 2.3 | Add sensors | W, F, G factors | Each sensor alone/combined runs; noise band per subset measured |
| 2.4 | Combination sweep, fixed weights | 31-subset table, identity vs physical weights | Table + figure; predictions checked |
| 2.5 | Classical covariance prediction | Per-constraint covariance (ICP Hessian / degeneracy, wheel slip, GPS quality) | Beats fixed weights on accuracy AND NEES on validation sessions |
| 2.6 | Learned prediction (only if 2.5 fails measurably somewhere) | Small model predicting covariance from features | Beats 2.5 where 2.5 failed, without losing elsewhere |
| 2.7 | Second dataset (HeLiPR, LiDAR ring) | Same sweep over 4 LiDARs + IMU | Generalization result |

## Timeline (applications run in parallel; deadlines Dec 2026 - Feb 2027)
- by end Oct: 2.0 + 2.1 done -> report updated with NCLT replication (strongest single upgrade for applications)
- Nov: 2.2 + 2.3
- Dec - Jan: 2.4 + 2.5
- later: 2.6, 2.7

## Split (fixed in 2.0, written to DECISIONS.md before any run)
Sessions chosen to span seasons; e.g. tune on 3, validate on 2, test on 3. Exact list decided in 2.0.
