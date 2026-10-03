# Experiment registry

| name | date | commit | config | change vs baseline | hypothesis | result | verdict |
|---|---|---|---|---|---|---|---|
| baseline | 2026-10-03 | 27c1c63 | configs/baseline.yml | none | n/a | see results/baseline.csv | reference |
| deskew_off | 2026-10-03 | 1542e57 | configs/deskew_off.yml | deskew false | KITTI may be pre-deskewed; small change either way | identical to baseline on all 6 seqs; trajectories match to <1e-6 m | no effect (cause unverified; likely no per-point timestamps in KITTI) |
| range60 | 2026-10-03 | 1542e57 | configs/range60.yml | max_range 60 | far points noisy, RPE may improve; seq 01 may get worse | | |
| minrange3 | 2026-10-03 | 1542e57 | configs/minrange3.yml | min_range 3 | removes self-returns; tiny effect | | |
| vox05 | 2026-10-03 | 1542e57 | configs/vox05.yml | voxel 0.5 | finer map, slightly better RPE, slower | | |
| vox2 | 2026-10-03 | 1542e57 | configs/vox2.yml | voxel 2.0 | coarser map, faster, worse | | |
| slam_default | 2026-10-03 | (fill) | configs/kiss_slam_default.yaml | KISS-SLAM defaults (loop closure + pose graph) | ATE drops clearly on looped seqs (00,02,05,06,07,09); ~equal to baseline on loop-free seqs (01,03,04,10); seq 08 uncertain; false closures are the main risk | | |
