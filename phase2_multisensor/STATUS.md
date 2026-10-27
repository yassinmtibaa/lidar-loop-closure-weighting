# Status (update at the end of every session)

**Last update:** 2026-10-25
**Current phase:** 2.3 started on the smoke session (all sensors in the graph); 2.1 write-up pending
**Done:** structure, dataset choice (NCLT), smoke test (ATE 0.17% of path), split fixed, 2.1 predictions
written, tune + validation sessions unpacked.

## Next action
1. `pipx inject kiss-slam scipy` (the NCLT loader needs it in the KISS-SLAM env too).
2. DONE: KISS-ICP baseline (ATE 0.13 / 0.15 / 0.57% of path), see DECISIONS.
3. DONE: KISS-SLAM default x3. P2.1a met, P2.1b mostly, P2.1c not met; SLAM noise large (see DECISIONS).
4. DONE: W=1e3 x3 on tune: better on 2012-04-29, within noise elsewhere (prediction met).
5. DONE: validation passed (W not worse; after the 6-run 05-26 recheck everything is within noise).
6. DONE 2026-10-25: test sessions run once. P2.1d NOT MET (KITTI mean -4.0% but 2012-06-15 ATE mean +14.9%);
   see DECISIONS 2026-10-25. Test sessions are used up: never rerun them for 2.1.
7. NEXT: write up Phase 2.1 as-is.
8. DONE 2026-10-25 (Phase 2.2, tune sessions + smoke session only):
   - GTSAM backbone (fusion/graph.py) rebuilds KISS-SLAM's two-stage optimization from its g2o files: 18/18 tune
     graphs reproduce KISS-SLAM (worst keypose 0.0096 m, ATE diff 0.0011 m). Needed the g2o half-angle information
     conversion (first attempt failed; see DECISIONS). P2.2c (odometry chain vs KISS-ICP) NOT MET on 2012-08-04.
   - NCLT readers (fusion/nclt.py) for wheels, FOG, IMU, standard GPS; factor helpers (fusion/sensors.py);
     12 tests pass (tests/). Wheels+FOG dead reckoning 1.08% of path (P2.2d met), wheels-only 9.2x worse (P2.2e met).
9. DONE 2026-10-25: L+I end to end on the smoke session (scripts/li_smoke.py): runs, P2.2f met; metric changes
   mixed (ATE -15%, KITTI +5.8%) with placeholder noise, not a result. Phase 2.2 exit criteria met.
10. DONE 2026-10-25 (2.3, smoke session only): per-sensor noise estimated; all 31 subsets run. Solver: Dogleg with
   QR (Cholesky stalled or threw on IMU subsets); 28/31 converge (not WG, WFG, LIWG). No subset beats L alone
   (1.92 m ATE); 19 of 61 decided additions make things worse (P2.3d met, P2.3c not met). See DECISIONS.
11. DONE 2026-10-25: noise model v2 (drift-equivalent sigmas over 50-frame windows + wheel scale 0.925): damage
   much smaller (L+W 131 -> 20 m, L+I 34 -> 3.3 m) but 18 of 65 additions still worse; P2.3h/i/j not met, k met.
12. DONE 2026-10-25: noise model v3 (turn-dependent wheel sigma, GPS correlation inflation x7.1): GPS harm halved
   (L+G 7.8 -> 3.9 m) but 18 additions still worse; P2.5a/b/c not met.
13. DONE 2026-10-25: IMU axis check: axes are as assumed (gyro correlations +0.95/+0.99/+1.00); acc_x test
   inconclusive (P2.3m), exploratory 2 s smoothing +0.76. Wheel-vs-IMU disagreement is not an axis bug.
14. DONE 2026-10-26: wheel tilt test: tilt is not the cause (leveled factor changes I+L subsets by < 0.5%; I+W still
   348 m). P2.3n(1) not met, (2)(3) met. Body-frame wheel factor stays.
15. DONE 2026-10-26: IMU noise test (v4: densities while moving, x15): L+I now better than L (1.18 vs 1.93 m);
   I+W 242 -> 162 m; 16 worse additions. P2.4a/b/c met. Remaining harm: wheels, GPS on LiDAR, FOG on LiDAR.
16. DONE 2026-10-27: wheel calibration (v5: per-wheel scales + track width): wheel heading bias was the wheel
   problem (W 111 -> 25 m, LW 18.5 -> 2.55 m, WF 1.85 m). All 31 converge. Remaining harm: GPS (9 of 15
   additions worse), IMU without LiDAR (I+W 127 m). P2.6a met, b/c/d not met.
17. NEXT (agreed order): step 2 = prepare tune-session runs (supercomputer); step 3 = Phase 2.1 write-up.
All 9 sessions are on disk. Test sessions were run once (2026-10-25); never rerun them for 2.1.

## Facts to remember
- Fusion code runs in phase2_multisensor/.venv (uv Python 3.12.15; gtsam 4.3.0, numpy 2.5.4, scipy 1.18.1,
  pytest 9.1.1; pinned in requirements-fusion.txt). Created 2026-10-25. Tests: `.venv/bin/python -m pytest -q tests`.
- g2o (KISS-SLAM) rotation information weights the half angle: divide by 4 for GTSAM (W=1e3 -> 250).
- KISS-ICP/KISS-SLAM frame k = nclt.lidar_timestamps()[k]; KISS frame = NCLT body/velodyne with diag(1,-1,-1) flip.
- kvh.csv heading is counterclockwise-positive; the reader negates it. Wheel track width 0.6125 m (fitted, frozen).
- LiDAR-to-IMU rotation: sensors.fit_imu_rotation(), ~diag(1,-1,-1) plus 0.7 deg yaw (fitted on 2013-01-10).
- Wheel speeds read ~8% long on 2013-01-10 (scale not calibrated yet).
- Disk: 404 GB free (2026-10-11). velodyne_hits.bin deleted after unpacking (see DECISIONS).
- KISS-ICP nclt loader: data/nclt/<date>/velodyne_sync/<timestamp>.bin and
  data/nclt/ground_truth/groundtruth_<date>.csv; needs scipy (kiss-icp env: installed).
- Sensor files per session: ms25.csv (IMU), kvh.csv (fiber-optic gyro), wheels.csv, odometry_mu/cov.csv
  (wheel odometry + covariance), gps.csv, gps_rtk.csv (excluded: feeds ground truth).
- Use *_kitti.txt with evo; KISS-ICP's TUM timestamps on NCLT are frame indices.
- ~37 frames/s KISS-ICP on NCLT: 2012-01-15 (33.6k scans) ~15 min.
- KISS-SLAM in this machine is the patched one: KS_ODOM_INFO / KS_CLOSURE_INFO env vars, unset = original.

## Open questions
- Bako Motors role and the 92% figure for the CV (applications, separate).

## For whoever picks this up (Yassine or Claude)
Read README.md, then this file, then the last entries of DECISIONS.md. Phase 1 lives in the repo root.
