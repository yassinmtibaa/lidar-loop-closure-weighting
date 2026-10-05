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
