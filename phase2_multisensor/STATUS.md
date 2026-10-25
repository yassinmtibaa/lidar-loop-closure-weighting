# Status (update at the end of every session)

**Last update:** 2026-10-25
**Current phase:** 2.1 (replicate Phase 1 on NCLT, LiDAR only)
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
7. NEXT: write up Phase 2.1 as-is; then Phase 2.2 (fusion, GTSAM) on the supercomputer when access arrives.
All 9 sessions are on disk. Test sessions were run once (2026-10-25); never rerun them for 2.1.

## Facts to remember
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
