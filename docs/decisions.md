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
