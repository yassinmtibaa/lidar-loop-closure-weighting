# Phase 2: Multi-sensor error prediction for SLAM

**Question.** Does adding sensors always improve localization accuracy? Hypothesis: only when each
sensor's error is predicted correctly. With well-predicted covariances, every added sensor improves (or at
least never worsens) accuracy and keeps the uncertainty estimate honest. With fixed or identity weights,
adding a sensor can make things worse.

**Where this comes from.** Phase 1 (this repo's root, `docs/report/report.pdf`) showed on KITTI that
identity information matrices in KISS-SLAM let accurate loop closures degrade local accuracy, and that a
physically sensible weighting removes most of the loss. Phase 2 replaces fixed weights with predicted ones,
per constraint and per sensor, and tests every sensor combination.

**Files**
- `PLAN.md`      phases, milestones, exit criteria, timeline
- `STATUS.md`    where the project is right now and the next action (read this first)
- `DECISIONS.md` dated decisions and predictions, written BEFORE each run
- `DATASETS.md`  dataset comparison and choice
- `scripts/ configs/ results/ data/` same conventions as Phase 1 (`data/` is not tracked)

**Rules carried over from Phase 1**
1. Predictions and acceptance rules go in `DECISIONS.md` before a run, never after.
2. Fixed split per dataset: tune / validate / test. The test set is run once, at the end.
3. Measure run-to-run noise before comparing configurations.
4. Failed predictions and null results are recorded, not deleted.
