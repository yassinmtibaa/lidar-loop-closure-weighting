# Experiment registry

| name | date | commit | config | change vs baseline | hypothesis | result | verdict |
|---|---|---|---|---|---|---|---|
| baseline | 2026-10-03 | 27c1c63 | configs/baseline.yml | none | n/a | see results/baseline.csv | reference |
| deskew_off | 2026-10-03 | 1542e57 | configs/deskew_off.yml | deskew false | KITTI may be pre-deskewed; small change either way | identical to baseline on all 6 seqs; trajectories match to <1e-6 m | no effect (cause unverified; likely no per-point timestamps in KITTI) |
| range60 | 2026-10-03 | 1542e57 | configs/range60.yml | max_range 60 | far points noisy, RPE may improve; seq 01 may get worse | | |
| minrange3 | 2026-10-03 | 1542e57 | configs/minrange3.yml | min_range 3 | removes self-returns; tiny effect | | |
| vox05 | 2026-10-03 | 1542e57 | configs/vox05.yml | voxel 0.5 | finer map, slightly better RPE, slower | | |
| vox2 | 2026-10-03 | 1542e57 | configs/vox2.yml | voxel 2.0 | coarser map, faster, worse | | |
| slam_default | 2026-10-03 | fe28141 | configs/kiss_slam_default.yaml | KISS-SLAM defaults (loop closure + pose graph) | ATE drops clearly on looped seqs (00,02,05,06,07,09); ~equal to baseline on loop-free seqs (01,03,04,10); seq 08 uncertain; false closures are the main risk | ATE -74/-71/-56/-22% on 00/02/05/06; 07 (1 closure) and 09 (0) no gain; 08 +11%; RPE +39% on 05, mean RPE +5.1% | win on ATE where closures found; RPE regression, missed closures on 07/09, 08 worse |
| slam_ovl05 | 2026-10-03 | 15afec0 | configs/slam_ovl05.yaml | overlap_threshold 0.5 | stricter closure acceptance; fewer closures; 05 RPE improves, ATE gain shrinks; risk: lose 00/02 gains | mean RPE +1.4%; 05 ATE +48%, RPE +9.9%; closures 6/1/4 (00/02/05) | rejected |
| slam_ovl06 | 2026-10-03 | 15afec0 | configs/slam_ovl06.yaml | overlap_threshold 0.6 | same, stronger | mean RPE -4.4%; 05 ATE -44%, RPE -25%; 00/02 ATE +2.7/+3.3%; closures 5/1/2 | provisional: big real gain on 05 (5->2 closures), small real costs on 00 (+2.7% ATE) and 07 (+4.8% ATE, loses its only closure); evidence is one sequence |
| slam_inl10 | 2026-10-03 | 15afec0 | configs/slam_inl10.yaml | inliers_threshold 10 | stricter detector; fewer weak closures | 02 loses both closures, ATE +270% | rejected |
| slam_ham35 | 2026-10-03 | 15afec0 | configs/slam_ham35.yaml | hamming_distance_threshold 35 | stricter feature matching | mean RPE 0.0%; 05 ATE +19% | rejected |
| slam_ovl05 | 2026-10-03 | 15afec0 | configs/slam_ovl05.yaml | overlap_threshold 0.5 | stricter closure acceptance; fewer closures; 05 RPE improves, ATE gain shrinks; risk: lose 00/02 gains | mean RPE +1.4%; 05 ATE +48%, RPE +9.9%; closures 6/1/4 (00/02/05) | rejected |
| slam_ovl06 | 2026-10-03 | 15afec0 | configs/slam_ovl06.yaml | overlap_threshold 0.6 | same, stronger | mean RPE -4.4%; 05 ATE -44%, RPE -25%; 00/02 ATE +2.7/+3.3%; closures 5/1/2 | provisional: big real gain on 05 (5->2 closures), small real costs on 00 (+2.7% ATE) and 07 (+4.8% ATE, loses its only closure); evidence is one sequence |
| slam_inl10 | 2026-10-03 | 15afec0 | configs/slam_inl10.yaml | inliers_threshold 10 | stricter detector; fewer weak closures | 02 loses both closures, ATE +270% | rejected |
| slam_ham35 | 2026-10-03 | 15afec0 | configs/slam_ham35.yaml | hamming_distance_threshold 35 | stricter feature matching | mean RPE 0.0%; 05 ATE +19% | rejected |
| slam_pgo50 | 2026-10-03 | (fill) | configs/slam_pgo50.yaml | pose-graph max_iterations 50 | default may stop before convergence; if so KITTI trans. error on 05 improves; if graph already converged, no change | | |
| slam_pgo100 | 2026-10-03 | (fill) | configs/slam_pgo100.yaml | pose-graph max_iterations 100 | same, stronger | | |
