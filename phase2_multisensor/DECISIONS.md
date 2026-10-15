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

## 2026-10-10: Phase 2.0 smoke test result (session 2013-01-10)
- P2.0a met: `kiss_icp_pipeline --dataloader nclt` ran unchanged (after `pipx inject kiss-icp scipy`).
  5104 scans, 37 frames/s.
- P2.0b met: ATE 1.926 m on a 1138.8 m ground-truth path = 0.17% (prediction < 1%). Max error 6.55 m.
  Independent check (own Umeyama alignment) reproduces 1.926 m.
- KITTI translation metric 1.874% (KITTI seqs were 0.28-0.82%): HDL-32E is sparser, campus scenes differ.
- Estimated per-frame path is 11.5% longer than ground truth, but the ratio is 1.006 at 5-frame and 1.001
  at 10-frame spacing: per-frame jitter, not a scale error.
- KISS-ICP writes frame indices (dt = 1) as TUM timestamps on NCLT: use the KITTI-format files with evo.
- 325 revisit pairs (< 3 m apart, > 60 s apart, 1-in-10 subsampling) in this short session: loops exist.

## 2026-10-10: NCLT split (fixed before downloading; spans seasons; avoids the two sessions whose
## listed sizes look like typos, 2012-02-04 and 2012-03-31)
- Smoke / development: 2013-01-10
- Tune:     2012-01-15, 2012-04-29, 2012-08-04
- Validate: 2012-05-26, 2012-10-28
- Test (run once at the end): 2012-02-12, 2012-06-15, 2013-04-05
- Download: Velodyne + sensors + ground truth + covariance only (~110 GB Velodyne total). No images.

## 2026-10-10: Phase 2.1 predictions (replicate Phase 1 on NCLT, LiDAR only), fixed before any 2.1 run
- P2.1a: default KISS-SLAM finds closures on every tune session (same campus, many revisits).
- P2.1b: default KISS-SLAM improves ATE vs KISS-ICP on every tune session.
- P2.1c: default KISS-SLAM raises the KITTI translation metric vs KISS-ICP on at least 2 of 3 tune sessions
  (the Phase 1 effect).
- P2.1d (main): rotation weighting W=1e3 (chosen on KITTI, NOT re-tuned) lowers the KITTI translation metric
  vs default KISS-SLAM on the test sessions on average, without raising ATE by more than 10% on any.
- Noise: KISS-SLAM reruns on each tune session (x3) before any comparison.

## 2026-10-11: data download
- velodyne_hits.bin (raw packets, ~21 GB per long session) deleted after unpacking: KISS-ICP reads only
  velodyne_sync/. Re-download a session if per-point timestamps are ever needed (deskew).
- Unpacked so far: 2013-01-10 (5120 scans), 2012-01-15 (33612), 2012-04-29 (12971), 2012-08-04 (24116),
  2012-05-26 (26544), 2012-10-28 (25691). Test sessions still downloading. Disk 404 GB free.
- Tuning runs may start now; validation and test sessions are not run until their phase.

## 2026-10-11: all 9 sessions downloaded and unpacked (2013-04-05: 20901 scans); disk 371 GB free.

## 2026-10-11: KISS-ICP baseline on tune sessions (results/icp_baseline.csv)
| session | path | ATE | ATE % path | RPE100 rmse | KITTI trans |
|---|---|---|---|---|---|
| 2012-01-15 | 7.54 km | 9.85 m | 0.13% | 4.27 m | 1.48% |
| 2012-04-29 | 3.17 km | 4.61 m | 0.15% | 1.82 m | 1.38% |
| 2012-08-04 | 5.47 km | 30.89 m | 0.57% | 2.42 m | 1.74% |
- Local error is ~2x KITTI's (100 m RPE median ~1.8-2.1 m vs ~0.4-1.0 m on KITTI): sparser 32-beam LiDAR.
- RPE100 rmse is tail-driven: 2012-01-15 median 1.89 m but rmse 4.27 m; 2012-08-04 has 4 segments > 5 m and
  max ATE 90 m (one bad stretch, not checked yet). Ground truth has no jumps (max step 0.69 m).

## 2026-10-11: default KISS-SLAM x3 on tune sessions (results/slam_default_r1..3.csv)
| session | ICP ATE | SLAM ATE r1/r2/r3 | ICP KITTI | SLAM KITTI r1/r2/r3 | closures |
|---|---|---|---|---|---|
| 2012-01-15 | 9.85 | 3.08 / 2.61 / 2.32 | 1.482 | 1.397 / 1.397 / 1.395 | 13/13/15 |
| 2012-04-29 | 4.61 | 6.37 / 2.14 / 2.01 | 1.383 | 1.474 / 1.437 / 1.415 | 3/3/3 |
| 2012-08-04 | 30.89 | 3.74 / 2.92 / 5.96 | 1.735 | 1.775 / 1.638 / 1.887 | 6/7/5 |
- P2.1a (closures on every tune session): MET.
- P2.1b (SLAM improves ATE on every tune session): MOSTLY MET, 8 of 9 runs; 2012-04-29 run 1 worse (6.37 vs 4.61).
- P2.1c (SLAM raises KITTI metric on >= 2 of 3): NOT MET. Clear rise only on 2012-04-29 (+2..7%, all 3 runs);
  2012-01-15 improves (-6%, all runs, RPE100 4.27 -> ~2.3 m: closures remove the bad tail);
  2012-08-04 straddles the baseline (1.638..1.887). The KITTI effect does not transfer cleanly to NCLT.
- NOISE is much larger than on KITTI: ATE 2.0-6.4 m on 2012-04-29 with the SAME closure count (3/3/3), KITTI
  metric spread 14% on 2012-08-04. Cause unknown (multi-threaded ICP or closure verification randomness).

## 2026-10-11: comparison rule under this noise (fixed before running W=1e3)
- Every configuration is run 3x on the tune sessions. A difference counts only if the 3-run ranges of the
  two configurations do not overlap; otherwise it is reported as "within noise".
- Prediction for W=1e3 (KITTI's choice, not re-tuned) on tune sessions: KITTI metric range on 2012-04-29 falls
  entirely below default's (1.415-1.474); on the other two, within noise. ATE: within noise or better.

## 2026-10-14: W=1e3 x3 on tune sessions (results/w_rot1e3_r1..3.csv) vs default x3
| session | KITTI % default (3 runs) | KITTI % W=1e3 (3 runs) | ATE default | ATE W=1e3 | verdict (non-overlap rule) |
|---|---|---|---|---|---|
| 2012-01-15 | 1.395-1.397 | 1.373-1.413 | 2.32-3.08 | 2.71-3.08 | within noise |
| 2012-04-29 | 1.415-1.474 | 1.398-1.408 | 2.01-6.37 | 1.74-2.00 | BETTER on both (ranges do not overlap) |
| 2012-08-04 | 1.638-1.887 | 1.666-1.762 | 2.92-5.96 | 2.60-5.42 | within noise |
- Prediction (04-29 KITTI range entirely below default's; others within noise; ATE within noise or better): MET.
- On 04-29 (the session where closures cost local accuracy) W=1e3 recovers ~2/3 of the gap to odometry
  (default mean 1.442%, W mean 1.403%, odometry 1.383%) and removes the 6.37 m ATE outlier.
- Observation (not pre-registered): W=1e3 runs look less spread (04-29 ATE 1.74-2.00 vs 2.01-6.37;
  08-04 KITTI 1.666-1.762 vs 1.638-1.887). 3 runs each is too few to claim it.
- Closure counts unchanged (13-16 / 3 / 6-7).

## 2026-10-14: validation rule and secondary prediction (fixed before validation runs)
- Validation sessions 2012-05-26, 2012-10-28: default x3 and W=1e3 x3 each.
- Pass: on each validation session, W=1e3 is not worse than default beyond noise (ranges overlap or W better)
  on KITTI metric and ATE. Fail on either session -> report, do not re-tune W.
- Secondary (now pre-registered): W=1e3 3-run ATE range is narrower than default's on both validation sessions.

## 2026-10-14: validation results (2012-05-26, 2012-10-28), default x3 vs W=1e3 x3
- 2012-05-26: all 6 runs crashed AFTER writing poses, in KISS-ICP's ground-truth TUM export: the ground-truth CSV
  starts with 2 NaN rows. Metrics computed with scripts/metrics.py (drops NaN frames, 1 frame dropped). The script
  reproduces eval.sh exactly for ATE and the KITTI metric (checked on 3 earlier runs); its RPE100 definition differs
  from evo's, so RPE is not used for 05-26. Accepted closures (from the log): default 10/11/10, W=1e3 10/11/10.
| session | KITTI % default | KITTI % W=1e3 | ATE default | ATE W=1e3 | verdict |
|---|---|---|---|---|---|
| 2012-05-26 | 1.333-1.409 | 1.293-1.352 | 2.39-3.49 | 1.44-2.25 | KITTI within noise (W lower); ATE BETTER (no overlap) |
| 2012-10-28 | 1.369-1.394 | 1.344-1.371 | 1.71-3.12 | 1.70-2.04 | within noise on both (W lower/tighter) |
- Validation rule (W not worse beyond noise on either session): PASSED.
- Secondary (W ATE range narrower than default's on both): MET (05-26 0.81 vs 1.10 m; 10-28 0.34 vs 1.41 m).
- Fix for test: NaN rows removed from groundtruth CSVs of 2012-05-26 (2 rows), 2012-02-12 (1), 2012-06-15 (2);
  originals kept as *.orig_with_nan. Done BEFORE any test run.

## 2026-10-14: test protocol (fixed before running)
- Test sessions 2012-02-12, 2012-06-15, 2013-04-05: default x3 and W=1e3 x3, run once, reported as-is.
- P2.1d (registered 2026-10-10): W=1e3 lowers the KITTI metric vs default on the test sessions on average, without
  raising ATE by more than 10% on any. Evaluated on 3-run means; per-session non-overlap verdicts reported too.
