# Decisions

## 2026-10-03: evo is the official metric
ATE (aligned, -a) and RPE (-d 100 -u m) from evo, independent of the pipeline
being improved. Alternatives: KISS-ICP's built-in metrics (kept as a cross-check;
they matched evo ATE on all 11 sequences).

## 2026-10-03: split by sequence, never by frame
Consecutive scans are near-duplicates, so frame-level splits leak.
Proposed split: tune on 00-05, validate on 06-07, test on 08-10 (touch the
test set only at the end).

## 2026-10-03: judge preprocessing on RPE and translation error %, not ATE
ATE is dominated by drift that accumulates with trajectory length, which
filtering can't fix. ATE is reserved for the loop-closure stage.

## 2026-10-03: staged plan
Baseline -> classical preprocessing -> learned component only if a specific
failure remains -> simplest closed loop -> simulation link.

## 2026-10-03: acceptance rule for preprocessing experiments
Run on tuning sequences 00-05 only. A config is a candidate if mean RPE
improves by >= 2% AND no single tuning sequence gets worse by more than 5%.
Candidates are then checked once on validation sequences 06-07. Only configs
that also hold there are kept. Sequences 08-10 are not run until the end.
Parallel runs are deterministic (baseline_rerun identical), so differences are
real effects of the config, but may still not generalize.

## 2026-10-03: acceptance rule for loop-closure experiments
Run on tuning sequences 00-05, compared against slam_default (not odometry).
Candidate if mean RPE over 00-05 improves >= 2% vs slam_default AND mean ATE over
looped seqs (00, 02, 05) worsens <= 10% AND no tuning seq worsens ATE > 25%.
Candidates then checked once on validation 06-07. Only 3 looped tuning seqs: high
overfitting risk, so any winner needs the validation check before it is kept.

## 2026-10-03: acceptance rule for loop-closure experiments
Run on tuning sequences 00-05, compared against slam_default (not odometry).
Candidate if mean RPE over 00-05 improves >= 2% vs slam_default AND mean ATE over
looped seqs (00, 02, 05) worsens <= 10% AND no tuning seq worsens ATE > 25%.
Candidates then checked once on validation 06-07. Only 3 looped tuning seqs: high
overfitting risk, so any winner needs the validation check before it is kept.

## 2026-10-03: validation criterion for slam_ovl06
slam_ovl06 passes on validation seqs 06-07 if, vs slam_default, mean RPE does not
worsen by more than 2% and neither sequence's ATE worsens by more than 25%.
Test seqs 08-10 stay untouched. Determinism of KISS-SLAM is checked first
(rerun of slam_default on 00, 02, 05).

## 2026-10-03: KISS-SLAM is not deterministic
Rerun of slam_default: 00 ATE +1.1%, 02 ATE +3.3%, RPE -1.2%; 05 identical to 1e-5.
Differences of this size between configs on 00/02 are noise. The 05 effects (ovl05 +48%,
ovl06 -44% ATE) exceed noise. From now on, an effect counts only if it exceeds the
measured spread over 5 runs. slam_ovl06 passes the validation rule on 06-07 but the
validation is uninformative (06 identical, 07 lost its only closure): provisional only.

## 2026-10-03: KISS-SLAM noise measured (6 runs of slam_default)
Seqs 05, 06, 07: identical on every run. 00: ATE spread 1.1%, 7 closures every run.
02: ATE spread 3.3%, closure count flips between 1 and 2 (likely a candidate at the
acceptance threshold; cause unverified). Corrections: the 02 differences in the closure
sweep are noise; 07's +4.8% ATE under ovl06 is real (07 has zero spread), as is 00's +2.7%
(two fewer closures). ovl06 is a provisional tradeoff, not a new default.
Rule from now on: compare 02 only over repeated runs; 05/06/07 single runs are enough.

## 2026-10-03: closure audit definition
For each closure edge in KISS-SLAM's local_map_graph.g2o, compare its relative pose with the
ground-truth relative pose between the matched scans. BAD if translation error > 1.0 m or
rotation error > 2.0 deg; otherwise GOOD. Reference: same error for consecutive (odometry)
edges. Audited sequences: 00, 02, 05 only (tuning set). 06-07 (validation) and 08-10 (test)
are not audited now, to keep them clean. Caveat: the g2o file has no closure scores.

## 2026-10-03: closure audit v2 (vs odometry-chain error)
All 14 accepted closures on 00/02/05 are more accurate than the odometry between the same
scans (ratios ~2-10x; weakest 9<->0 on 00: 3.29 vs 3.76 m). No bad closures on tuning seqs,
so a learned verifier has no target there. Overlap (0.47-0.82 accepted) does not visibly
predict error. KITTI translation error (100-800 m) is worse with SLAM: 00 +8.5%, 02 +8.0%,
05 +51%, 06 +16%, 08 +16% (test seq, not tuned on), mean ~+10%. SLAM improves ATE but costs
local accuracy even with accurate closures. Suspect: pose-graph weighting (all edges carry
identical information matrices) and/or iteration limit (10). Both untested.

## 2026-10-03: pose-graph iteration test
max_iterations 10 vs 50 vs 100: identical on 05 (deterministic) to 6 digits; KITTI trans. error
0.517% in all three. Rejected (rule: >=10% gain on 05). The pgo50/100 numbers on 00 and 02 equal
the slam_default_rerun numbers, so the first default run is the odd one (00: 0.9072 vs 0.9168,
cause unknown, 1 odd run in ~8; 02: 2-closure state 1.8537 vs 1-closure 1.9157).
Next hypothesis: piecewise-rigid per-local-map correction creates steps at 100 m boundaries.
Test: scripts/step_jumps.py (control 04, no closures).

## 2026-10-03: no-closure test (previous hypothesis rejected)
overlap_threshold 1.0 -> 0 closures on 00/02/04/05. KITTI trans%: 05 odom 0.342, noclosure 0.349,
default 0.517; 00: 0.528/0.532/0.573; 02: 0.537/0.543/0.580; 04 identical. 86-96% of the
odometry->default gap appears only when closures are applied. KISS-SLAM internal odometry is NOT the
cause (retracting the 04-control inference). ATE without closures still moves -4%/+6% vs baseline
(noise-level). Remaining cause: how the pose-graph correction is distributed over frames. All 14
closures are accurate (audit v2), no boundary spikes (step test), all graph edges carry identical
information matrices. Example: 05 seg 10 changed 1.30 m vs odometry error 0.50 m (RPE 0.50 -> 1.89).
Next: read slam.py / pose_graph_optimizer.py to see how corrected poses are produced.

## 2026-10-03: code read (slam.py, pose_graph_optimizer.py); step test was blind to rotation
Final poses = node.keypose @ rel_pose: rigid per 100 m local map. All factors (odometry between maps
AND closures) use np.eye(6): radians and metres weighted equally, so rotation residuals are nearly
free. Maps split on straight-line distance from origin. fine_grained_optimization (per-scan smoother)
exists; unknown whether saved poses use it. RETRACTION: step_jumps.py measured translation only, so
"no boundary kinks" was wrong; a heading kink of 0.5 deg displaces the next 100 m by ~0.9 m, the
size of the RPE regressions (unverified). Prediction for step_jumps2: >=25% of rotation diff within
+-1 frame of boundaries on 05/00; control 04 ~2%.

## 2026-10-03: code read + rotation-aware step test
Code: final poses are node.keypose @ rel_pose per map, but pipeline.py line 81 assigns the saved poses
from fine_grained_optimization() (per-scan smoother, all factors np.eye(6)). Rigid per-map kinks
hypothesis REJECTED: step_jumps2 shows 3% (05), 2% (00), 4% (04 control) of rotation difference within
+-1 frame of map boundaries, equal to the fraction of such frames; predicted >=25%. Rotation
differences tiny (median 0.02 deg, max 0.24 deg). The -rs flag does not change saved poses and crashes
(map_closures lacks align_map_to_local_ground, library mismatch); irrelevant to our metrics.
Current hypothesis (from code reading, unverified): identity weights spread each correction evenly
over a map's scans regardless of where odometry actually drifted. Prediction for
correction_alignment.py on 05: corr(|change|,|odo err|) < 0.3 and mean delivered < 0.5; corr >= 0.6
with delivered near 1 would mean corrections are well targeted and the cause is elsewhere (e.g. closure noise).

## 2026-10-03: correction_alignment results
Prediction (05: corr<0.3, delivered<0.5) met: 05 corr 0.13 / delivered 0.05; 00 -0.11 / 0.01;
02 0.23 / -0.02 (04 has 3 segments, uninformative). RMS change per segment on 05 ~0.49 m vs odometry
RMS error 0.48 m; ATE 1.49 m over 21 segments is ~0.07 m/segment if spread evenly, so changes are
~7x larger than drift correction needs. Changes concentrate in segs 3-11 (frames ~420-1617, RMS ~0.72 m),
the span of the closure cluster 7<->3, 8<->4, 9<->5 (frames 445-1502); segs 12-20 mostly < 0.13 m.
Hypothesis (untested): redundant, mutually inconsistent closures + identity weights inject error.
Check: slam_ovl06 (2 closures) segs 3-11 RMS change predicted < 0.4 m; > 0.6 m would refute.

## 2026-10-03: correction_alignment results
Prediction (05: corr<0.3, delivered<0.5) met: 05 corr 0.13 / delivered 0.05; 00 -0.11 / 0.01;
02 0.23 / -0.02 (04 has 3 segments, uninformative). RMS change per segment on 05 ~0.49 m vs odometry
RMS error 0.48 m; ATE 1.49 m over 21 segments is ~0.07 m/segment if spread evenly, so changes are
~7x larger than drift correction needs. Changes concentrate in segs 3-11 (frames ~420-1617, RMS ~0.72 m),
the span of the closure cluster 7<->3, 8<->4, 9<->5 (frames 445-1502); segs 12-20 mostly < 0.13 m.
Hypothesis (untested): redundant, mutually inconsistent closures + identity weights inject error.
Check: slam_ovl06 (2 closures) segs 3-11 RMS change predicted < 0.4 m; > 0.6 m would refute.

## 2026-10-03: closure-cluster check (prediction confirmed)
slam_ovl06 on 05 keeps 2 of 5 closures (8<->4 err 0.12 m, 15<->0 err 0.24 m; dropped 7<->3 0.87 m,
9<->5 0.45, 17<->5 0.42). RMS change in segs 3-11: default 0.724 m -> ovl06 0.064 m (predicted <0.4,
refute >0.6). 05 per-segment RPE rmse: odo 0.481, default 0.694, ovl06 0.505; ATE ~0.37 vs 0.66 m.
Established: the large corrections come from the dropped closures. Not established: which one(s) or
why. Suspect 7<->3 (largest error, parallel to 8<->4). Revised plan: leave-one-out on 05 (needs a
small slam.py patch to skip closure pairs), NOT uniform downweighting. 2 survivors also have the
highest overlaps (0.70, 0.82): confound, 5 points. Free generalization check on 00/02 first.

## 2026-10-03: generalization check failed; KISS-SLAM investigation closed
Predictions: 00 RMS change ratio (ovl06/default) < 0.8 -> observed 0.89 (not met); 05 KITTI trans
0.35-0.40 -> observed 0.335 (better than predicted, ~= odometry 0.342). 00: 0.573 vs 0.574;
02: 0.580 vs 0.576 (within noise). Closure-cluster harm is specific to 05 in our data (3 looped
tuning seqs, 5 closures on 05). Hypothesis: matters on 05 because odometry is already accurate there.
Decision: do not adopt ovl06 as default (07: -1 closure, ATE +4.8%); drop learned closure
selection (insufficient labeled data); stop mining KISS-SLAM; validation (06-07) and test (08-10)
remain unused for further tuning. Next direction: simulation / sensor noise model.

## 2026-10-03: direction C started (sensor noise model for simulation)
Gazebo Sim 8.15.0 (Harmonic), gpu_lidar available, ros_gz installed, no Velodyne packages.
Step 1: KITTI ground-plane residual sigma by range (upper bound: includes road roughness).
Prediction: MAD-sigma at 30-40 m >= 2x sigma at 5-10 m. Fit on seqs 00-05, check on 06-07, 08-10 untouched.

## 2026-10-03: KITTI ground noise measurement (prediction failed)
Prediction sigma(30-40 m)/sigma(5-10 m) >= 2: observed 1.37 (00), 1.14 (02), 1.20 (05). Curve flat ~7-8 cm
beyond 10-20 m; sequences differ at 5-10 m (5.5/6.9/5.9 cm). Conclusion: road geometry (one plane per
annulus) dominates, so the number is an upper bound, not sensor noise (HDL-64E datasheet ~2 cm is from
memory, unverified). No noise model fitted from this. Usable: ~122-126k points/scan mean (max ~128k),
99th/99.9th pct range 50/74 m, elevation -24.3..-24.6 deg at 0.1th pct (99.9th pct +4.7..+4.9 deg, above
the recalled +2: unresolved).
Next: per-beam range deviation along azimuth on ground points (needs beam layout), fit on 00-05,
check 06-07, 08-10 untouched. Prediction: a constant-sigma Gaussian (1-3 cm) will describe it, in which
case no learned model is justified for this part.

## 2026-10-03: KITTI beam layout (seq 00 scan 0)
Prediction 55-64 peaks, non-uniform spacing, span +2..-25: observed 67 peaks (not met; likely
double-counted beams, eight 0.10 deg gaps), spacing 0.10-2.65 deg non-uniform (met), peaks +2.43..-21.57
deg (bottom ~3 deg missing though 0.1th pct reaches -24.3, cause unknown). Usable for a rough sensor
(64 beams, ~-24.5..+2.4 deg, 360 deg, range ~75 m), not as exact beam angles. Gazebo gpu_lidar example
has uniform vertical spacing, so a uniform sensor will misplace beams by a fraction of a degree.
Next: check whether .bin order encodes the beam (wrap-arounds ~64?) for a per-beam noise measurement.

## 2026-10-03: KITTI .bin ordering
Prediction (beam-ordered file): met. 99% of consecutive points < 0.15 deg elevation apart (median
0.003), azimuth step ~0.18 deg. 72 seam wrap-arounds != 64 (unexplained; wraps are the wrong boundary
detector, beams are separated by azimuth drops). Dropouts are structured: top beam (+2.1..2.2 deg) has
a 7.8 deg gap in its first points (sky), so dropout depends on elevation.
Next: ring_noise.py (second-difference range noise per beam on ground, fit on 00-05).
Predictions: 60-70 beams/scan; sigma 1-3 cm per bin; ratio sigma(30-40)/sigma(5-10) < 2.
If sigma > 4 cm: KITTI cannot give a sensor-noise number, stop that line.

## 2026-10-03: ring noise measurement (00/02/05, every 20th scan)
Predictions: 60-70 beams/scan -> exactly 64 (met); sigma 1-3 cm per bin -> MAD 0.84-1.83 cm, 3-10 m bins
just under 1 cm (mostly met); sigma(30-40)/sigma(5-10) < 2 -> 1.83/2.08/1.83 (2 of 3; std-based 2.3-2.6).
MAD sigma 4-8x below plane-fit (road shape dominated plane-fit). Consistent across seqs (+-5% near,
+-10% far). std/MAD 1.3-2.2: heavy tails (sensor vs scene unresolved). Flat road <= 40 m only.
Checks (predictions fixed now): stride-2 sigma within 15% of stride-1 at 5-10 m (else noise is
correlated, sigma underestimated); fit on 00-05 predicts 06/07 per-bin within 20%; 64 distinct beam
elevations, ~+2..2.6 to ~-24..-25 deg, min gap >= 0.1 deg; fill < 0.6 above 0 deg, > 0.85 below -10 deg.

## 2026-10-03: ring noise checks (8 seqs, every 20th scan; fit on 00-05)
Whiteness (stride-2/stride-1, 5-10 m): pooled 1.05, per seq 0.98-1.08 -> met (<=1.15). At 30-40 m
1.14-1.44 (scene curvature or correlation; if curvature variance scales 16x for stride 2 the
correction to sigma1 is <=~5%: assumption).
Fit on 00-05: sigma(r)=sqrt(a^2+(kr)^2), a=0.86 cm, k=0.044 cm/m (0.88 cm @4 m, 1.76 cm @35 m).
Held-out (<=20% per bin): NOT met. 06: -19.7/-20.4/-29.1/-6.2/+3.9%; 07: +1.7..+6.9%. Training seqs
01/03/04 are +11..+22% above model at 30-40 m, 01 +20% at 5-10 m: noise varies ~+-20% between sequences,
cause unknown. Fixed-sigma(r) Gaussian = average within ~+-30%.
Beams: 64 distinct, -23.59..+2.70 deg, smallest gap 0.23 deg (earlier 67 peaks/0.10 gaps = double counts
confirmed). Bottom predicted -24..-25: not met.
Fill: above 0 deg 0.98 (predicted <0.6: not met), below -10 deg 0.98 (met), lowest beams (<-18 deg)
56-87%; fill > 1 is a denominator artifact (2014 samples/rev = global median step).
RETRACTED: 'dropouts depend on elevation' (one scan); lowest beams do have lower fill, cause unverified.
Next: intensity vs sigma. Predictions: at 5-10 m sigma(lowest populated intensity bin) >= 1.2x
sigma(highest); median road intensity of 06 < that of 00/02/05/07. If neither: intensity does not
explain the +-20% between-sequence spread.

## 2026-10-03: ring noise checks (8 seqs, every 20th scan; fit on 00-05)
Whiteness (stride-2/stride-1, 5-10 m): pooled 1.05, per seq 0.98-1.08 -> met (<=1.15). At 30-40 m
1.14-1.44 (scene curvature or correlation; if curvature variance scales 16x for stride 2 the
correction to sigma1 is <=~5%: assumption).
Fit on 00-05: sigma(r)=sqrt(a^2+(kr)^2), a=0.86 cm, k=0.044 cm/m (0.88 cm @4 m, 1.76 cm @35 m).
Held-out (<=20% per bin): NOT met. 06: -19.7/-20.4/-29.1/-6.2/+3.9%; 07: +1.7..+6.9%. Training seqs
01/03/04 are +11..+22% above model at 30-40 m, 01 +20% at 5-10 m: noise varies ~+-20% between sequences,
cause unknown. Fixed-sigma(r) Gaussian = average within ~+-30%.
Beams: 64 distinct, -23.59..+2.70 deg, smallest gap 0.23 deg (earlier 67 peaks/0.10 gaps = double counts
confirmed). Bottom predicted -24..-25: not met.
Fill: above 0 deg 0.98 (predicted <0.6: not met), below -10 deg 0.98 (met), lowest beams (<-18 deg)
56-87%; fill > 1 is a denominator artifact (2014 samples/rev = global median step).
RETRACTED: 'dropouts depend on elevation' (one scan); lowest beams do have lower fill, cause unverified.
Next: intensity vs sigma. Predictions: at 5-10 m sigma(lowest populated intensity bin) >= 1.2x
sigma(highest); median road intensity of 06 < that of 00/02/05/07. If neither: intensity does not
explain the +-20% between-sequence spread.

## 2026-10-03: ring noise checks (8 seqs, every 20th scan; fit on 00-05)
Whiteness (stride-2/stride-1, 5-10 m): pooled 1.05, per seq 0.98-1.08 -> met (<=1.15). At 30-40 m
1.14-1.44 (scene curvature or correlation; if curvature variance scales 16x for stride 2 the
correction to sigma1 is <=~5%: assumption).
Fit on 00-05: sigma(r)=sqrt(a^2+(kr)^2), a=0.86 cm, k=0.044 cm/m (0.88 cm @4 m, 1.76 cm @35 m).
Held-out (<=20% per bin): NOT met. 06: -19.7/-20.4/-29.1/-6.2/+3.9%; 07: +1.7..+6.9%. Training seqs
01/03/04 are +11..+22% above model at 30-40 m, 01 +20% at 5-10 m: noise varies ~+-20% between sequences,
cause unknown. Fixed-sigma(r) Gaussian = average within ~+-30%.
Beams: 64 distinct, -23.59..+2.70 deg, smallest gap 0.23 deg (earlier 67 peaks/0.10 gaps = double counts
confirmed). Bottom predicted -24..-25: not met.
Fill: above 0 deg 0.98 (predicted <0.6: not met), below -10 deg 0.98 (met), lowest beams (<-18 deg)
56-87%; fill > 1 is a denominator artifact (2014 samples/rev = global median step).
RETRACTED: 'dropouts depend on elevation' (one scan); lowest beams do have lower fill, cause unverified.
Next: intensity vs sigma. Predictions: at 5-10 m sigma(lowest populated intensity bin) >= 1.2x
sigma(highest); median road intensity of 06 < that of 00/02/05/07. If neither: intensity does not
explain the +-20% between-sequence spread.

## 2026-10-03: intensity vs ring noise (00/02/05/06/07, ground, every 20th scan)
Predictions: sigma(lowest bin) >= 1.2x sigma(highest) at 3-10 m -> observed 0.53-0.78 (reversed);
06 median ground intensity lower than others -> 0.320 vs 0.300-0.310 (reversed, small).
Finding: sigma is U-shaped in intensity. 3-10 m: dim (<0.2) 1.2-1.4x the 0.2-0.5 bins (4/5 seqs);
bright (>0.5, ~3% of ground points) 1.7-1.8x (by hand: 1.83/1.67/1.83/1.72/1.76); 10-20 m 1.8-2.5x;
20-40 m 1.6-2.2x. Effect consistent across seqs, bigger than the +-20% between-sequence spread.
06 is not explained by intensity: its dominant 0.3-0.5 bin is +40% (3-10 m) and +75% (10-20 m)
vs the others, gone at 20-40 m; cause unknown. Alternatives for the bright-bin effect: genuine noise
vs range bias at paint edges. Test (ring_intensity2.py): predictions A: sigma_same(0.5-1.0, 3-10 m)
< 1.2 cm in all 5 seqs; B: |median d2| in 0.5-1.0 >= 1 mm, <= 0.3 mm in 0.3-0.5. A fails
(>= 1.35 cm): genuine intensity-dependent noise.

## 2026-10-03: intensity test v2 (same-bin triples), 00/02/05/06/07
A (bright-bin sigma_same < 1.2 cm at 3-10 m, all seqs): met, 0.88/0.73/0.76/0.77/0.88 (all-triples 1.37-2.03).
B (median d2 >= 1 mm in bright bin; <= 0.3 mm in 0.3-0.5): first half met (+12.1..+20.9 mm, 5/5);
second half 3/5 (-0.23/-0.20/-0.20/-0.32/-1.31 mm).
RETRACTED: 'bright points 1.7-1.8x noisier'. Edge artifact. Real effects: (1) bright returns read ~6-10 mm
closer than asphalt neighbours (d2/2; range walk is a plausible, unverified mechanism); (2) dim (<0.1)
returns 1.3-1.4x noisier than the 0.2-0.3 bin (homogeneous, all 5 seqs); (3) transitions inflated sigma
~10% in main bins -> earlier fit a=0.86 cm likely ~10% high at near range, needs refit.
06 unexplained: 0.3-0.5 bin sigma_same 1.15 (3-10 m) / 1.71 (10-20 m) vs 0.78-0.82 / 0.94-0.97 elsewhere.
Model status: parametric table sufficient; no learned model justified. Validation caveat: injecting
sigma and re-measuring with the same estimator matches by construction (tests wiring, not realism).
Next: check whether Gazebo gpu_lidar output has intensity (read-only), then refit with homogeneous triples.

## 2026-10-03: Gazebo gpu_lidar output (example world, headless server works)
Topics /lidar, /lidar/points. PointCloud2 fields: x,y,z (float32), intensity (float32, offset 16),
ring (uint16, offset 24); organized 16x640 = the example's 16 vertical x 640 horizontal samples.
Ring gives beam id directly. Not yet known: intensity values, point_step, non-return encoding.
Predictions for inspect_cloud.py: 10,240 points; intensity 1-3 distinct values (laser_retro default);
ring 0-15 = row index; misses are non-finite coordinates in the organized cloud.
If intensity is constant, intensity effects need laser_retro assigned per surface in our own world.

## 2026-10-03: Gazebo cloud inspection (example world)
Predictions: 10,240 points (met); intensity 1-3 distinct values (met: 1 value, 0.000 everywhere, no
information); ring 0-15 = row index (met); misses = non-finite coordinates (met: 2784/10240 = 27%, NaN vs
inf not checked). point_step 32; frame_id via ROS bridge 'model_with_lidar/link/gpu_lidar'. World file has
no laser_retro and no <noise>. Intensity-dependent terms (bright offset, dim-noise factor) need surfaces
with laser_retro. Prediction: laser_retro=R on a visual -> hit points report intensity R (or a fixed
multiple); stays 0 -> not wired as recalled (placement or scale unknown). Stray parameter_bridge children
survive `kill` of the ros2 run wrapper.

## 2026-10-03: Gazebo test world worlds/velodyne_flat.sdf (predictions fixed before the run)
Derived from the gpu_lidar example (asserted edits): 64 x 2014 samples, 360 deg horizontal,
vertical -23.59..+2.70 deg (uniform spacing, approximation of KITTI's non-uniform), range 0.08-80 m,
range resolution 0.0001 m, sensor 1.73 m above ground (z=0 top), ground 200 x 200 m split in three
coplanar boxes, bright band y in [2,5] with laser_retro 0.9 (ground 0.3), red box removed.
Predictions: (1) cloud 64 x 2014 = 128,896 points; (2) wall rate >= 8 Hz of the 10 Hz setting
(below 5 Hz: reduce the sensor); (3) 53-55 rings have finite points (downward beams below ~-1.24 deg
hit ground inside 80 m); first finite ring is ring 0 or ring 63 (direction unknown), spacing ~0.417 deg;
(4) median z of down-beam points -1.73 +- 0.03 m; (5) intensity: two distinct values near 0.3 and 0.9
(or a fixed multiple); only 0.000 means laser_retro is not wired as recalled.

## 2026-10-03: velodyne_flat.sdf first run
(1) 64x2014 cloud: met. (2) rate >= 8 Hz: NOT met, 4.23 Hz (sim 4.22, RTF 1.00; sensor, bridge or my
subscriber unresolved; 4.1 MB/message). (3) 53-55 rings finite: NOT met, 64/64; ring 63 (+2.70 deg) 54% finite,
ring 0 88% finite; finite 115,084 vs ~108,800 predicted from ground geometry. (4) ground z -1.73+-0.03: -1.770
(narrow miss, link offset inside the lidar model unseen). (5) intensity: 0.305 and 0.885 present (laser_retro
works) but 62,524 finite points (54%) at 0.000; nonzero-intensity ground hits 52,560 = ~48% of ~108,800.
Diagnostic predictions: (1) gz-transport rate without the bridge >= 8 Hz (else sensor/GPU limit);
(2) >= 30% of ground-height points have intensity 0 (<5%: zeros are non-ground);
(3) finite points above 0 deg: median range < 3 m (own body), not ~80 m.

## 2026-10-03: velodyne_flat diagnostic (cloud has near-field returns)
Results: ground-height points 52,550 (predicted ~108,800 from beam geometry); their intensity is correct
(0.305: 41,634, 0.885: 10,810, zero: 106), so laser_retro works; non-ground finite points 62,534, 99.8% intensity 0,
median range 1.5-1.6 m (rings 30-53), ~1.0 m (rings 54-63), range 0.08-74 m. Non-ground share by ring: 28/27/39%
(rings 0/10/20), ~64% (rings 30-53), 100% of finite on rings 54-63 (54-59% finite). Azimuth-dependent (ring 20:
non-ground ~all of -150..-60, half of +30..+150). Own cube (0.1 m, centred on sensor) too small to explain 1 m ranges.
Retractions: 'half the ground hits miss intensity'; the nvidia-smi compute-apps GPU test (invalid: graphics contexts
are not listed); ground z prediction (sensor is at 1.78 m: model 1.73 + link 0.05). gz topic -hz is not a flag
(use -f -t -m). Rate still unmeasured.
Predictions for the A/B (flat vs nocube vs narrow120): P1 nocube non-ground count within 10% of flat (cube too small);
P2 narrow120: non-ground share on rings 30-53 inside its sector < 20% (360 deg multi-camera artifact);
>50% means world or sensor geometry. P3 gz-transport rate of the full sensor ~4 Hz (sensor-limited), narrow120 >= 8 Hz;
full >= 8 Hz would mean my bridge/subscriber was the bottleneck.

## 2026-10-03: sim A/B (flat vs nocube vs narrow120), results
P1 (nocube within 10% of flat): met, identical (ground 52,570 / non-ground 62,514 / non-finite 13,812).
P2 (narrow120 < 20% non-ground on rings 30-53 in sector): met (ring 40/50 all ground; sector ground 74% of
points vs 41% for 360 deg). P3 (rates): not evaluated; gz topic -f printed nothing.
Finding: near-field non-ground returns (1.0-1.6 m, intensity 0) occur in the 360 deg sensor over most azimuths
(rings 30-63) and not in the 120 deg sector. Cause not verified (suspected: seam/stitching of the multi-camera
360 deg render). Cube not the cause. pmon shows velodyne_flat.s as a G (graphics) process: GPU context
exists; utilization '-' = no samples. 'Software rendering' suspicion dropped (no evidence); 4.2 Hz unexplained.
Decision: noise-model work uses the 120 deg x 671-sample sensor (az step 0.1788 deg); 360 deg later via three
120 deg sensors merged in the node.
Predictions for ideal-sensor ring check: sigma < 0.1 cm at 5-40 m; after injecting the fitted noise: measured
within 10% of injected per bin; az step 0.179 deg.
