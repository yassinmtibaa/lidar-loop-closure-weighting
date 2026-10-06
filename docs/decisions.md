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
