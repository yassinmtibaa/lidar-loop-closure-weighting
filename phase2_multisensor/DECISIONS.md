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

## 2026-10-18: OS upgrade and reproducibility recheck (BEFORE any test run)
- System upgraded to Ubuntu 26.04.1 (system Python 3.14.4, ROS 2 Lyrical). All pipx tools broke (venvs built on 3.12).
- Reinstalled on uv-managed Python 3.12 with the SAME pinned versions: kiss-icp 1.3.0, kiss-slam 0.0.2, evo 1.37.1.
  Patched pipeline.py / slam.py restored from docs/patched_backup/ (copied before reinstall). NumPy now 2.5.4
  (kiss-icp, evo) and 2.2.6 (kiss-slam); previous NumPy versions were not recorded (lesson: record them).
- Check: scripts/check_env.sh. Weighting verified in the output pose graph (rotation info = 1000 in W run, 1 in default).
- Recheck on 2012-05-26, 1 run each, scored with metrics.py (old runs re-scored with the same script: values match the
  table above; old runs dropped 1 NaN GT frame, new runs 0 since the CSV fix):
  | run | ATE m | KITTI % | closures | old 3-run range (ATE / KITTI) | inside? |
  |---|---|---|---|---|---|
  | default | 2.450 | 1.357 | 10 | 2.39-3.49 / 1.333-1.409 | yes / yes |
  | W=1e3 | 2.571 | 1.356 | 11 | 1.44-2.25 / 1.293-1.352 | NO / NO (just above) |
- Reading: one run cannot separate "software change shifted W" from "3 runs underestimated run-to-run noise".
  Either way, the 05-26 verdict "ATE BETTER (no overlap)" no longer holds on the 4 W runs pooled (1.44-2.57 overlaps
  default 2.39-3.49). Treat 05-26 ATE as "within noise" until resolved. Validation rule (W not worse) still passes.
- Plan, fixed now: 2 more runs of each on 05-26 (3 new per condition). Environment accepted as equivalent if, for both
  conditions, the new 3-run range overlaps the old 3-run range on ATE and KITTI. If accepted: report 05-26 with all
  6 runs per condition and proceed to the test sessions on this environment. If not: stop, investigate the NumPy /
  dependency difference (try matching old versions) before any test run. No test run until this is settled.

## 2026-10-18 (later): recheck result -> environment ACCEPTED
- 2 more runs per condition on 2012-05-26 (4 run in parallel; weighting verified in each pose graph).
  | condition | new 3 runs ATE m | new KITTI % | old 3 runs ATE / KITTI | overlap? |
  |---|---|---|---|---|
  | default | 2.450, 3.756, 3.469 | 1.357, 1.422, 1.376 | 2.39-3.49 / 1.333-1.409 | yes / yes |
  | W=1e3 | 2.571, 1.769, 2.586 | 1.356, 1.331, 1.383 | 1.44-2.25 / 1.293-1.352 | yes / yes |
- Rule (fixed before these runs): overlap on both metrics for both conditions -> ACCEPTED. Test sessions run on the
  new environment.
- 2012-05-26 now reported on all 6 runs per condition:
  default ATE 2.39-3.76 (mean 3.17), KITTI 1.333-1.422 (mean 1.377);
  W=1e3  ATE 1.44-2.59 (mean 2.13), KITTI 1.293-1.383 (mean 1.343).
  Verdict changes from "ATE BETTER (no overlap)" to "within noise by the non-overlap rule; W mean ATE 33% lower,
  W lower on KITTI mean". Earlier claim retracted. Lesson: 3 runs underestimated KISS-SLAM's run-to-run spread on NCLT.
- Consequence for the test protocol: keep 3 runs per condition as registered (P2.1d is on means), but report the
  6-run 05-26 spread alongside as the noise reference.

## 2026-10-25: TEST result (2012-02-12, 2012-06-15, 2013-04-05), default x3 vs W=1e3 x3, run once -> P2.1d NOT MET
- Run once, no re-tuning, on the accepted 2026-10-18 environment (check_env.sh --smoke passed before the runs).
  6 runs in parallel per round, 3 rounds (scripts/run_test_once.sh). Weighting verified in every pose graph
  (rotation info 1000 in all 9 W runs, 1 in all 9 default runs). No crashes, no NaN frames dropped.
  Scored by scripts/summarize_test.py (metrics.py) -> results/test_summary.txt; eval.sh CSVs in results/*_test.csv.
| session | KITTI % default | KITTI % W=1e3 | ATE m default | ATE m W=1e3 | ATE mean change | closures d / W |
|---|---|---|---|---|---|---|
| 2012-02-12 | 1.609-1.715 (1.658) | 1.563-1.601 (1.580) | 2.870-3.605 (3.257) | 3.044-3.797 (3.308) | +1.6% | 8/8/8 / 8/8/8 |
| 2012-06-15 | 1.239-1.266 (1.250) | 1.196-1.253 (1.232) | 2.026-2.047 (2.033) | 2.064-2.565 (2.336) | +14.9% | 1/1/1 / 1/1/1 |
| 2013-04-05 | 2.089-2.130 (2.113) | 1.775-2.431 (2.006) | 9.192-9.784 (9.568) | 4.919-14.368 (8.340) | -12.8% | 4/4/3 / 4/4/2 |
- P2.1d part 1 (KITTI metric lower on average): yes, 1.674% -> 1.606% (-4.0%); W mean lower on all 3 sessions.
- P2.1d part 2 (no session with ATE mean raised > 10%): FAILS on 2012-06-15 (+14.9%). => P2.1d NOT MET.
- Non-overlap verdicts: 02-12 KITTI W BETTER, ATE within noise; 06-15 ATE W WORSE (W min 2.064 > default max 2.047),
  KITTI within noise; 04-05 both within noise.
- Honest reading:
  - The 06-15 ATE failure is real by the rule but small in absolute terms (+0.30 m mean; the non-overlap margin is
    0.017 m). Default happened to be very repeatable there (spread 0.02 m, 1 closure); the 6-run 05-26 noise reference
    spread is ~1.4 m. It still counts: the rule was fixed in advance and is not relaxed afterwards.
  - The validation observation "W runs are tighter" does NOT hold on test: on 04-05 W is far MORE spread (ATE
    4.9-14.4 m, KITTI 1.775-2.431%); W r1 is worse than every default run on both metrics, r2/r3 better than every
    default run. The -12.8% ATE mean there rests on 2 of 3 runs and is within noise.
  - The KITTI-metric gain is consistent in sign (3/3 sessions) but small (-1.4 to -5.1%) and beyond noise on 1 of 3.
  - Conclusion: rotation weighting W=1e3 chosen on KITTI does not transfer to NCLT as a reliable improvement.
    It does not clearly hurt local accuracy and may help it slightly; it does not reliably keep global accuracy.
    Not to be claimed as an improvement on NCLT.
- Not explored (would be post-hoc): why 04-05 W r1 diverged (same 4 closures as r2); a W below 1e3; more runs.

## 2026-10-25: Phase 2.2 start (GTSAM backbone + NCLT sensor readers). Fixed BEFORE any GTSAM run.
- Scope agreed with Yassine: (a) GTSAM pose graph taking KISS-ICP odometry + loop-closure factors, checked against the
  identity-weighting KISS-SLAM baseline on the TUNE sessions only; (b) readers for IMU, FOG, wheels, standard GPS
  (RTK stays excluded), wheels + FOG first, unit-tested on the smoke session 2013-01-10. Test sessions are spent:
  never used in 2.2. No full sweeps here (supercomputer).
- Environment: separate uv venv `phase2_multisensor/.venv` (uv Python 3.12.15): gtsam 4.3.0, numpy 2.5.4,
  scipy 1.18.1, pytest 9.1.1. The KISS tools (pipx venvs) are untouched.
- What KISS-SLAM outputs (read from its code, kiss_slam 0.0.2): stage 1 = keypose graph (one node per ~100 m local
  map, odometry factor per local map, closure factors, node 0 fixed), written to local_maps/local_map_graph.g2o with
  the final keyposes. Stage 2 = per-frame graph: per-frame odometry factors, and in each local map the last frame is
  held fixed at (optimized keypose x local end pose); written to trajectory.g2o (vertices = the output poses, equal to
  <seq>_poses.npy to 5e-4, the g2o text precision). Edge convention checked: measurement = inv(T_i) T_j.
  Information order in g2o is (translation, rotation); GTSAM Pose3 tangent order is (rotation, translation).
- Reproduction check: GTSAM rebuilds both stages from the factors in those two g2o files (no new KISS-SLAM runs, no
  new closure detection) for the 9 existing default runs (identity weights) on the 3 tune sessions, plus the 9 existing
  W=1e3 runs (rotation information 1000 on odometry), which also checks the information-order permutation.
  Stage 1: Levenberg-Marquardt from the odometry-chain initial guess (not from KISS-SLAM's answer).
- Acceptance (each of the 18 graphs, fixed now; tolerances are not widened afterwards):
  - A1: stage-1 keyposes vs KISS-SLAM's final keyposes: max translation difference < 0.10 m and max rotation
    difference < 0.1 deg.
  - A2: final per-frame trajectory scored with the same code as metrics.py: |ATE difference| < 0.02 m and
    |KITTI-metric difference| < 0.002 percentage points vs KISS-SLAM's own output. For scale: the smallest 3-run
    spread on tune sessions is 0.76 m ATE (default, 2012-01-15) and 0.002 pp KITTI (same session).
  - Determinism: two GTSAM runs on the same graph agree to < 1e-9 m.
  - Pass = all 18 meet A1 and A2. Any failure is reported with its cause (iterations, local minimum, conventions).
- P2.2c (secondary): the odometry-only chain (each default run's per-frame odometry, no closures) has a KITTI metric
  within 5% of the KISS-ICP baseline on each tune session, for all 3 runs. ATE is reported without a prediction
  (one bad stretch drives 2012-08-04's 30.9 m).
- Sensor readers, all developed and checked on 2013-01-10 only. Predictions:
  - P2.2d: wheel speed + FOG heading 2D dead reckoning on 2013-01-10 has 2D ATE (SE2-aligned) under 5% of the
    ground-truth path (1138.8 m), i.e. < 57 m.
  - P2.2e: wheel-only dead reckoning (heading from differential wheel speed) is at least 3x worse in 2D ATE than
    wheels + FOG. Its track width is fitted on 2013-01-10 against FOG yaw rate (in-sample; frozen afterwards and
    written here when fitted).

## 2026-10-25: Phase 2.2 GTSAM reproduction result (tune sessions only) -> PASSED after a convention fix
- scripts/gtsam_reproduce.py -> results/gtsam_reproduction.csv / .txt, results/gtsam_odom_chain.csv.
- First attempt FAILED A1 on most graphs: the g2o information matrix was only permuted (t,R) -> (R,t). Keyposes then
  differed by up to 3.35 m (2012-04-29 default r1) and ATE by up to 1.04 m. Diagnosis: GTSAM reached the same minimum
  from the odometry guess and from KISS-SLAM's answer, with a LOWER cost than KISS-SLAM's answer, so KISS-SLAM was
  minimizing a different cost, not stopping early. Cause: KISS-SLAM's C++ uses g2o EdgeSE3 (symbols checked in
  kiss_slam_pybind.so), whose rotation error is the quaternion vector part (~angle/2), while GTSAM's BetweenFactor
  uses the angle. Fix: rotation information x 1/4 (cross terms x 1/2) when converting g2o information
  (fusion/graph.py, info_g2o_to_gtsam). The permute-only results stay in the CSV (mapping=permute_only):
  A1 6/18, A2 8/18. Tolerances were not changed.
- With the conversion (mapping=quaternion), all 18 graphs (9 default + 9 W=1e3):
  | check | worst over 18 graphs | tolerance | result |
  |---|---|---|---|
  | A1 keypose translation | 0.0096 m | 0.10 m | 18/18 |
  | A1 keypose rotation | 0.0026 deg | 0.1 deg | 18/18 |
  | A2 ATE difference | 0.0011 m | 0.02 m | 18/18 |
  | A2 KITTI-metric difference | 0.00005 pp | 0.002 pp | 18/18 |
  | any frame, position difference | 0.0103 m | (reported) | |
  | determinism (two GTSAM runs) | 0 m | 1e-9 m | met |
- Meaning: given KISS-SLAM's own odometry and closure factors, the GTSAM backbone reproduces its trajectories and
  metrics far below run-to-run noise. Closure DETECTION is still KISS-SLAM's; 2.2 does not replace it.
- Lesson for later phases: any covariance fed to GTSAM is on the angle (radians), not g2o's half-angle. The W=1e3
  rotation information from Phase 1 equals 250 in GTSAM's convention.
- P2.2c (odometry chain KITTI within 5% of KISS-ICP on each tune session): NOT MET, 6/9 runs.
  2012-01-15 and 2012-04-29 met in all runs (-4.2% to +0.5%); 2012-08-04 failed in all 3 runs: chain KITTI
  1.959 / 1.619 / 2.061 % vs KISS-ICP 1.735 % (+12.9 / -6.7 / +18.8 %). Chain ATE 08-04: 15.4 / 21.4 / 13.6 m vs
  KISS-ICP 30.9 m.
  Observation (not pre-registered, not a claim): on 2012-08-04 KISS-SLAM's own odometry differs a lot from run to run,
  so part of KISS-SLAM's run-to-run spread there comes from odometry, not only from closures; the final SLAM KITTI
  order of the 3 runs (r2 < r1 < r3) matches the chain's. Checking this would need its own pre-registered test.

## 2026-10-25: sensor-reader conventions found on 2013-01-10 (before evaluating P2.2d / P2.2e)
- kvh.csv heading is counterclockwise-positive: its increments correlate -0.999 with ground-truth yaw. The reader
  negates it so FOG heading, ground-truth yaw and the z-down body frame agree (heading grows turning right).
- Ground truth is NED-like: direction of motion atan2(dy, dx) matches ground-truth yaw (median offset -0.001 rad).
- Wheel track width fitted against FOG yaw rate (samples moving > 0.2 m/s): b = 0.6125 m. FROZEN for P2.2e and later.
- Seen while checking units (before P2.2d was evaluated): wheel-integrated distance 1233.6 m vs ground-truth path
  1138.8 m, i.e. wheel speeds read ~8% high on this session. Not corrected (P2.2d is evaluated as registered);
  a wheel scale factor is a Phase 2.3 calibration question.
- IMU (ms25) at rest: |acc| = 9.805 m/s^2, gyro mean < 0.004 rad/s per axis (first 30 samples).
- GPS (gps.csv): mode 2 (2940 rows) or 3 (4246 rows); num_sats is always 0 on this session (unused); one 39.6 s gap.

## 2026-10-25: L+I end to end on the smoke session (fixed BEFORE the first L+I run)
- Yassine asked to continue and push straight to master. Smoke session 2013-01-10 only (not in the split).
- LiDAR-to-IMU rotation estimated on 2013-01-10 (in-sample): 1 s LiDAR rotations (KISS-ICP smoke run) vs gyro
  integration (bias = mean of the first second at rest), 734 windows turning > 0.05 rad, Kabsch fit.
  R_LI ~ diag(1, -1, -1) (the KISS flip) with a 0.7 deg yaw and 0.2 deg pitch offset; median residual 0.019 rad on
  windows averaging 0.17 rad. Lever arm ignored (slow platform). Frozen for the L+I runs.
- Graph: one Pose3 + velocity per LiDAR frame (5104 frames), per-frame KISS-ICP odometry as BetweenFactorPose3,
  IMU preintegration (ImuFactor) between consecutive frames, bias with a random-walk factor between frames, gravity-
  aligned start (roll/pitch from the accelerometer at rest), zero-velocity and bias priors at the start. Noise values
  are placeholders (LiDAR 0.002 rad / 0.02 m per frame; IMU as in sensors.imu_params), NOT tuned on the outcome.
- P2.2f: (1) L+I runs end to end with finite output for every frame; (2) its KITTI metric is within +-10% of L-only
  (the KISS-ICP smoke run, 1.874%). No improvement is predicted or claimed: noise values are placeholders.
  L-only through the same GTSAM graph (IMU factors switched off) must equal the KISS-ICP trajectory (< 1 mm).

## 2026-10-25: L+I smoke result (scripts/li_smoke.py -> results/li_smoke_2013-01-10.txt) -> P2.2f MET
| run | ATE m | RPE100 m | KITTI % |
|---|---|---|---|
| KISS-ICP (input) | 1.926 | | 1.874 |
| L-only through GTSAM | 1.926 | 2.539 | 1.874 |
| L+I | 1.630 | 2.018 | 1.983 |
- L-only through GTSAM equals KISS-ICP to 4e-12 m. L+I runs end to end, all frames finite. KITTI +5.8% (within the
  +-10% band); ATE -15%, RPE100 -21%. 16 s for 5104 frames, batch.
- Reading: the exit criterion "L+I runs end to end" is met. The metric changes are NOT a result: one session,
  one KISS-ICP input run, placeholder noise, in-sample R_LI, and they point in opposite directions (ATE and RPE
  better, KITTI worse). GTSAM is deterministic, so the 3-run rule applies once there are 3 KISS-ICP inputs to compare.
- preintegrate() now uses zero-order hold over [t0, t1) (the first version skipped the gap before the first sample).
