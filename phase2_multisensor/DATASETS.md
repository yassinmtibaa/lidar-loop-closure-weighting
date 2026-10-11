# Dataset comparison (2026-10-10)

Needs: many sensor types, full ground truth, many revisits (loop closures), several sequences, ideally a
KISS-ICP dataloader (`nclt`, `helipr`, `boreas` exist; `mulran` too).

| Dataset | Platform | Sensors | Ground truth | Sequences | Loops | Size notes | KISS-ICP loader |
|---|---|---|---|---|---|---|---|
| **NCLT** | Segway, campus | Velodyne HDL-32E, Ladybug3 omni camera, Microstrain GX3 IMU, KVH fiber-optic gyro, wheel odometry, standard + RTK GPS, 2 Hokuyo 2D lasers | SLAM graph (LiDAR + RTK), ~100 Hz, keyframe covariances provided | 27 sessions, 15 months, ~5.5 km each, 147 km total | Same campus every session: many revisits | Velodyne ~8-20 GB/session, other sensors ~100 MB, camera ~85-130 GB (skip) | yes |
| **HeLiPR** | Car, urban | 4 LiDARs: Ouster OS2-128, Velodyne VLP-16, Livox Avia, Aeva Aeries II (FMCW); Xsens IMU; SPAN-CPT7 RTK INS | INS (independent of LiDAR), B-spline interpolated per scan | Roundabout 01-03, Town 01-03, Bridge 01-04, + KAIST/DCC/Riverside 04-06 | Designed for revisits; sessions ~2 weeks apart | not checked | yes |
| **Boreas** | Car, Toronto | 128-beam Velodyne, Navtech radar, 5 MP camera; Applanix GNSS-INS (= ground truth) | Post-processed INS, 2-4 cm nominal, 20-40 cm in urban canyons | 44 sequences, >350 km | One route repeated over a year | not checked | yes |
| **M2DGR** | Ground robot | LiDAR, 6 fisheye + 1 sky camera, infrared, event camera, VI-sensor, IMU, consumer GNSS, RTK GNSS-IMU | Motion capture / laser tracker / RTK | 36 sequences (~1 TB) indoor + outdoor | varies | ~1 TB total | no (rosbag) |
| Newer College ext. | Handheld | multi-camera, LiDAR, IMU (hardware-synced) | 6-DoF at 10 Hz | 3 collections, 4.5 km | some | not checked | rosbag |

## Choice

**Primary: NCLT.** Most distinct sensor *types* relevant to a vehicle (LiDAR, IMU, gyro, wheel odometry,
GPS), 27 sessions over the same campus (plenty of loops, room for a clean split), KISS-ICP loader, and the
non-camera data is manageable (~15 GB per session). Caveat: its ground truth was built with LiDAR + RTK
SLAM, so LiDAR-only errors are measured against a reference that partly shares LiDAR. Report this.
RTK GPS is excluded from fusion experiments because it feeds the ground truth.

**Secondary: HeLiPR.** For the "ring of LiDARs" experiment (4 different LiDARs + IMU) with ground truth
independent of LiDAR. Decide after NCLT Phase 2.1 works.

**Not now:** Boreas (its IMU is the ground-truth INS, so it can't be a fusion input), M2DGR (1 TB, no
loader), Newer College (handheld, short).

Sources: NCLT https://robots.engin.umich.edu/nclt ; HeLiPR arXiv:2309.14590 ; Boreas arXiv:2203.10168 ;
M2DGR arXiv:2112.13659 ; Newer College multi-camera extension arXiv:2112.08854.
