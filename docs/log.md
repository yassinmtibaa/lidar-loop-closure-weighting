# Lab log

## 2026-10-03
**Goal:** KITTI baseline with KISS-ICP.
**Did:** fixed ROS/NVIDIA setup earlier; built KISS-ICP; patched TUM export bug;
ran sequences 00-10; evaluated with evo.
**Result:** ATE 0.24-6.46 m, translation error 0.28-0.82% (results/baseline.csv).
**Surprises:** seq 04 rotational error (0.623 rad) looks like an alignment
artifact on a short straight track; seq 08 has heavy-tailed RPE.
**Next:** inspect configs/baseline.yml, design first preprocessing sweep.
