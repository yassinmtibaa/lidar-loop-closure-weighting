# Findings: KISS-ICP / KISS-SLAM on KITTI odometry (2026-10-03)

Setup: KITTI 00-10, evo ATE (aligned) and RPE (100 m), KITTI translation error from KISS logs.
Split: tune 00-05, validate 06-07, test 08-10 (untouched by any tuning).

1. KISS-ICP baseline: ATE 0.24-6.46 m, translation error 0.28-0.82%. Deterministic (parallel rerun identical).
2. Classical preprocessing (deskew, max/min range, voxel size): no config met the acceptance rule.
   Deskew has no measurable effect on KITTI.
3. KISS-SLAM defaults: ATE -74% (00), -71% (02), -56% (05), -22% (06); no gain on 07 (1 closure) and
   09 (0 closures); 08 worse (+11%, test seq). Mean KITTI translation error ~+10%, mostly 05 (+51%).
4. KISS-SLAM is not fully deterministic: 02's closure count flips 1 <-> 2 (ATE spread 3.3%); 00 has one
   odd run in ~8; 05/06/07 are stable.
5. All 14 accepted closures on 00/02/05 are more accurate than the odometry they replace (2-10x).
   Overlap score does not visibly predict error.
6. With closures disabled, KISS-SLAM matches standalone odometry (05: 0.349 vs 0.342%).
   The local-accuracy cost appears only when closures are applied.
7. Corrections are not targeted at odometry error (corr 0.13 / delivered 0.05 on 05) and are spread
   over frames, not concentrated at map boundaries (translation and rotation).
8. On 05, dropping 3 of 5 closures (overlap_threshold 0.6) cut the corrections in the closure-cluster
   span by 91% and restored odometry-level local accuracy (0.335%). Not reproduced on 00/02: single-sequence effect.

Retractions along the way: deskew "no-op" explanation, "closures on 05 are bad", "boundary kinks"
(translation-only test), "internal odometry is the cause".
