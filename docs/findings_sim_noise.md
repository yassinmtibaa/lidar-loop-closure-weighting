# Findings: KITTI range-noise model and Gazebo gpu_lidar (2026-10-03 .. 2026-10-08)

Established
1. KITTI .bin files are beam-ordered (64 beams/scan); beam elevations -23.59..+2.70 deg, non-uniform.
2. Plane-fit road residuals (4-8 cm) measure road shape, not sensor noise.
3. Second-difference range noise along a beam (estimator verified on synthetic white noise to 1%):
   ~0.8-0.9 cm near, ~1.6-1.8 cm at 30-40 m on asphalt; fit sigma(r)=sqrt(a^2+(kr)^2), a=0.86 cm, k=0.044 cm/m
   (all-triples; ~10% high near range). Upper bound on white noise. Varies ~+-20% between drives (06 unexplained).
4. Intensity: dim (<0.1) returns 1.3-1.4x noisier; bright (paint) returns read ~6-10 mm closer (bias, not noise).
   The earlier "bright = 1.7x noisier" was an edge artifact (retracted).
5. Gazebo Harmonic gpu_lidar: laser_retro sets intensity; ring field = beam; 360 deg config produces
   near-field (1-1.6 m) non-ground returns over most azimuths, 120 deg sector does not (cause unverified).
6. Noise node (scripts/noise_node.py) injects white Gaussian noise of the intended size (direct check 0.995-1.011,
   estimator on smooth triples 0.98-1.02).
7. gpu_lidar ideal ranges on flat ground contain deterministic jumps (~40% of triples, +- pairs), ~0.19% of range
   beyond ~8 m; not affected by <min>. Beyond ~8 m this exceeds real sensor noise (~10x at 30 m).

Not established: cause of the gpu_lidar jumps and of the 360 deg artifact; cause of seq 06's higher noise;
dropout model; realism of any simulated scan beyond the flat-ground range statistic.
No learned model was justified at any point: every measured effect fits a small parametric table.
