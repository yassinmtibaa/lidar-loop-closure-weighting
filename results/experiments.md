# Experiment registry

| name | date | commit | config | change vs baseline | hypothesis | result | verdict |
|---|---|---|---|---|---|---|---|
| baseline | 2026-10-03 | 27c1c63 | configs/baseline.yml | none | n/a | see results/baseline.csv | reference |
| deskew_off | 2026-10-03 | (fill) | configs/deskew_off.yml | deskew false | KITTI may be pre-deskewed; small change either way | | |
| range60 | 2026-10-03 | (fill) | configs/range60.yml | max_range 60 | far points noisy, RPE may improve; seq 01 may get worse | | |
| minrange3 | 2026-10-03 | (fill) | configs/minrange3.yml | min_range 3 | removes self-returns; tiny effect | | |
| vox05 | 2026-10-03 | (fill) | configs/vox05.yml | voxel 0.5 | finer map, slightly better RPE, slower | | |
| vox2 | 2026-10-03 | (fill) | configs/vox2.yml | voxel 2.0 | coarser map, faster, worse | | |
