# Related work: has this been done before?

Written 2026-11-01. Question asked: has anyone reweighted the information matrices of loop-closure and
odometry factors in a LiDAR pose graph the way this repo does (fixed W = 1e3·diag(1,1,1,1e3,1e3,1e3) on
KISS-SLAM factors; Phase 2: identity vs fixed vs predicted weights in a multi-sensor GTSAM graph on NCLT,
scored with ATE, drift and NEES), or anything close to it?

Every paper below was found by web search on 2026-11-01 and its existence checked (arXiv id, publisher or
author page). Where I could only read a secondary summary instead of the paper itself, I say so. Nothing here
is cited from memory alone.

## Short answer

1. **The exact Phase 1 method has not been published by anyone else.** The only public trace of a fixed
   rotation-heavy scaling on KISS-SLAM's identity factors is our own GitHub issue
   ([PRBonn/kiss-slam#61](https://github.com/PRBonn/kiss-slam/issues/61), opened 2026-10-10). The maintainer
   (Saurabh Gupta, PRBonn) replied on 2026-10-15 that "the original work did not investigate it in detail",
   called a fixed scale "an easy fix", and invited a PR with evaluation on more benchmarks.
2. **But the idea itself is not new.** Weighting rotation more heavily than translation with fixed,
   hand-picked values is ordinary engineering practice in established LiDAR SLAM systems (LIO-SAM,
   Cartographer, hdl_graph_slam; source lines below). So "weight rotation more" is not a novelty claim we can
   make. What is new is a controlled, pre-registered measurement of it on KISS-SLAM with a held-out split.
3. **The principled version has just been done by KISS-SLAM's own group.** Gupta, Trekel, Wiesmann,
   Stachniss (arXiv 2607.13516, July 2026, "Retro-SLAM" in the maintainer's reply) replace KISS-ICP's
   identity odometry information with a geometry-dependent Hessian estimate, and their ablation shows large
   ATE gains over a no-uncertainty variant on HeLiPR, Apollo, MulRan and Newer College. This covers the
   "predicted beats identity" direction for LiDAR-only graphs. It does **not** use KITTI or NCLT, does not fuse
   other sensors, does not say what fixed weight their "no-info" ablation used, and does not test covariance
   consistency (no NEES).
4. **ICP covariance estimation is a mature field** (Censi 2007 onward), and its main finding agrees with our
   T7 result: closed-form ICP covariances are overconfident. A 2026 paper (Rønning et al.) finds LiDAR-inertial
   filters report covariances 2 to 7 orders of magnitude too tight.
5. **Robust back-ends** (switchable constraints, DCS, max-mixtures, RRR, GNC) solve a different problem:
   rejecting wrong loop closures. Our KITTI audit found every accepted closure was correct, so they do not
   explain or replace the effect we measured.
6. **On NCLT, loosely coupled fusion is far behind the state of the art.** Tightly coupled LiDAR-inertial-wheel
   odometry (LIW-OAM) reports 1.3 to 1.5 m ATE on four of our sessions. Our Phase 2 numbers must be framed as a
   study of weighting, never as a competitive fusion system.

**What gap is left for us** (details in the last section): (a) nobody has measured how much of a
Hessian-based weighting's gain a single fixed rotation/translation ratio recovers, on the same benchmarks;
(b) nobody has tested whether KISS-ICP-family covariances (identity, fixed, or Retro-SLAM's Hessian) are
consistent (NEES or proper scoring rules); (c) our negative transfer result (W = 1e3 helps on KITTI, not
reliably on NCLT) is itself a reportable finding, and the Retro-SLAM results give a reason to expect it.
Each is a modest, workshop-scale contribution. None of them is a new method.

## 1. KISS-ICP / KISS-SLAM and their direct follow-ups

| Work | What it does with factor weights | Overlap with us |
|---|---|---|
| Vizzo et al., KISS-ICP, RA-L 2023 (cited in report) | Odometry only, no pose graph. | Base system. |
| Guadagnino et al., KISS-SLAM, [arXiv 2503.12660](https://arxiv.org/abs/2503.12660) (IROS 2025, per [IPB pdf](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/guadagnino2025iros.pdf)) | All factors added with `np.eye(6)` (confirmed in our patched copy, `docs/patched_backup/slam.py`). The abstract does not discuss weights. | This is the default we changed. |
| Gupta et al., MapClosures, [arXiv 2501.07399](https://arxiv.org/abs/2501.07399), IJRR 2026 ([DOI 10.1177/02783649261449269](https://www.citedrive.com/en/discovery/efficiently-closing-loops-in-lidar-based-slam-using-point-cloud-density-maps/)) | Loop-closure *detection* and registration used by KISS-SLAM. The parts I could read do not say how edges are weighted. | Upstream of our closures; not a weighting paper. |
| Guadagnino et al., Kinematic-ICP, [arXiv 2410.10277](https://arxiv.org/abs/2410.10277v2) | Adds wheel-odometry kinematic constraints inside KISS-ICP's registration (front end, not the pose graph). | Related to our L+W fusion, but tightly coupled in the front end, not a pose-graph weighting study. |
| **Gupta, Trekel, Wiesmann, Stachniss, [arXiv 2607.13516](https://arxiv.org/abs/2607.13516) (July 2026)** | KISS-ICP front end; odometry information Λ_odo = H⁺ from a sampled local Hessian (perturbations ±0.1 m, ±0.01 rad, block-diagonal: translation and rotation fitted separately, eigenvalues clamped at 1e-3). Loop closures get "the associated information matrix Λ_lc" from fine registration, method not detailed. | **Closest work.** See below. |

**Gupta et al. 2026 in detail** (read from the arXiv HTML and PDF; Tables I-II):

- Datasets: HeLiPR (Bridge, Roundabout, Town × Aeva/Avia/Ouster), Apollo (5 sequences), MulRan (KAIST,
  Riverside, Sejong), Newer College (01, 02). No KITTI, no NCLT.
- Ablation "no-info" ("without uncertainty estimation module") vs full, ATE / RPE: HeLiPR Bridge-Aeva
  16.83 / 0.47 vs 107.34 / 1.46; Apollo SB 1.61 / 0.17 vs 22.13 / 0.45; Apollo BTS 4.22 vs 9.45 m;
  Newer College 01 0.34 vs 0.36 m, 02 0.42 vs 0.42 m. So the gain is large on some sequences and nil on others.
- The replacement weight in "no-info" is not stated (presumably identity, as in KISS-SLAM, but that is my guess).
- LiDAR only, no NEES or any consistency check, no repeats or ranges reported in what I read.

What this means for us: the claim "uncertainty-aware weights beat identity weights in a KISS-ICP pose graph"
now belongs to the authors of the system. Our Phase 1 result is a cheaper, cruder version of the same idea.
Our honest framing is "a fixed ratio recovers part of the loss on KITTI; does it recover as much as the Hessian?"
and that question is open.

## 2. Fixed, hand-tuned rotation/translation weights in existing systems

None of these are papers about weighting, but they show the practice is common. Checked in source on 2026-11-01:

- **LIO-SAM** ([`mapOptmization.cpp`](https://github.com/TixiaoShan/LIO-SAM), line ~1389): odometry factor
  variances `1e-6` for rotation, `1e-4` for translation (GTSAM order, rotation first), i.e. rotation
  information 100× translation. Loop closures use an *isotropic* variance equal to the ICP fitness score
  (line ~595), so rotation and translation are weighted equally there, like KISS-SLAM's identity.
- **Cartographer** ([`pose_graph.lua`](https://github.com/cartographer-project/cartographer)):
  `loop_closure_translation_weight = 1.1e4`, `loop_closure_rotation_weight = 1e5` (about 9× in weight; if, as I
  understand, these scale the residual, about 80× in information).
- **hdl_graph_slam** ([`information_matrix_calculator.cpp`](https://github.com/koide3/hdl_graph_slam)):
  information from the scan-matching fitness score, mapped into separate translation stddev range 0.1-5 m and
  rotation range 0.05-0.2 rad, with a constant-matrix option. Rotation is again weighted more than
  translation per unit.

Our W = 1e3 (rotation information 1000×, ≈ 32× in standard deviation) sits at the high end of these ratios.
None of these projects publishes an evaluation of the choice, which is the part we add.

## 3. ICP covariance estimation (the "predicted" alternative)

| Work | Verified at | Finding relevant to us |
|---|---|---|
| A. Censi, "An accurate closed-form estimate of ICP's covariance", ICRA 2007 | Implemented in Censi's CSM library ([`icp_covariance.c`](https://docs.ros.org/en/api/csm/html/icp__covariance_8c.html)) | The classic closed form from the cost Hessian. |
| S. M. Prakhya et al., closed-form 3D ICP covariance, MVA 2015 | [mva-org.jp](https://www.mva-org.jp/Proceedings/2015USB/papers/14-27.pdf) | Extends Censi to 3D point-to-point. |
| D. Landry, F. Pomerleau, P. Giguère, CELLO-3D, ICRA 2019 | [arXiv 1810.01470](https://arxiv.org/abs/1810.01470) | Data-driven ICP covariance; closed forms underestimate real-world error. |
| M. Brossard, S. Bonnabel, A. Barrau, "A New Approach to 3D ICP Covariance Estimation", ICRA 2020 | [arXiv 1909.05722](https://arxiv.org/abs/1909.05722) | Censi-type estimates are overconfident because they ignore initialisation error; adds that term. |
| F. A. Maken, F. Ramos, L. Ott, Stein ICP | [arXiv 2106.03287](https://arxiv.org/abs/2106.03287) | Sampling-based ICP posterior. |
| A. De Maio, S. Lacroix, Deep Bayesian ICP Covariance Estimation, ICRA 2022 | [arXiv 2202.11607](https://arxiv.org/abs/2202.11607), [dblp](https://dblp1.uni-trier.de/rec/conf/icra/MaioL22.html) | Learned ICP covariance with uncertainty. |
| K. Liu, K. Ok, W. Vega-Brown, N. Roy, Deep Inference for Covariance Estimation, ICRA 2018 | [MIT page](https://people.csail.mit.edu/wrvb/publication/liu2018deep) | Learns measurement noise models for state estimation. |
| M. Dolatabadi et al., Deep-learning LiDAR uncertainty | [arXiv 2509.18954](https://arxiv.org/abs/2509.18954v1) | Predicts 6×6 ICP covariance per scan on KITTI, used in an EKF (no pose graph). Gains vs fixed covariance are small and mixed (APE −0.19% to +10.6%). |

The KISS-SLAM maintainer's reply matches this literature: the Gauss-Newton Hessian is unreliable for
point-to-point ICP, and better estimators were too slow, which is why KISS-SLAM ships identity weights.
Our T7 result (predicted weights not consistent) is what this literature would predict.

## 4. Covariance consistency (NEES and alternatives)

- **O. Rønning, U. Saqib, A. Wąsowski, "You Should Be Properly Scoring Your Odometry",
  [arXiv 2609.25900](https://arxiv.org/html/2609.25900) (Sept 2026).** Four LiDAR-inertial filters (incl.
  FAST-LIO2) on 67 tracks: reported covariances are 2 to 7 orders of magnitude too tight; the covariance is
  set mainly by configuration parameters, not residuals. Proposes strictly proper scoring rules (energy score,
  log score, which contains NEES as its consistency term) with block-bootstrap intervals, tool `smfeval`.
  Relevance: directly supports our over-confidence finding, and offers a better metric than NEES alone. It
  covers filters, not pose graphs, and not KISS-ICP.
- I found **no paper reporting NEES (or any consistency test) for KISS-ICP / KISS-SLAM or Gupta et al. 2026
  covariances**. That is a real, if small, gap.
- Our own NEES under identity weights currently gives negative values (see STATUS.md); this must be fixed
  before any consistency claim, whatever the literature says.

## 5. Robust back-ends (outlier closures)

- N. Sünderhauf, P. Protzel, Switchable Constraints, IROS 2012 ([QUT eprints](https://eprints.qut.edu.au/109582)).
- P. Agarwal, G. D. Tipaldi, L. Spinello, C. Stachniss, W. Burgard, Dynamic Covariance Scaling, ICRA 2013
  ([IPB pdf](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/agarwal13icra.pdf)).
- E. Olson, P. Agarwal, Max-Mixtures, RSS 2012 / IJRR 2013 ([RSS](https://roboticsproceedings.org/rss08/p40.html),
  [IJRR pdf](https://april.eecs.umich.edu/media/pdfs/olson2013ijrr.pdf)).
- Y. Latif, C. Cadena, J. Neira, RRR, RSS 2012 ([RSS](https://roboticsproceedings.org/rss08/p30.html)).
- H. Yang, P. Antonante, V. Tzoumas, L. Carlone, GNC, RA-L 2020 ([arXiv 1909.08605](https://alphaxiv.org/abs/1909.08605)).
- R. Pfeifer, P. Protzel, EM for adaptive mixture models in graph optimization, ICRA 2019
  ([arXiv 1811.04748](https://www.arxiv.org/abs/1811.04748)): learns the noise model online, the closest
  robust-back-end relative of our "predicted" weights (applied to GNSS).

DCS is literally a per-closure rescaling of the information matrix, but driven by each residual and aimed at
outliers. None of these targets the inlier rotation/translation balance we changed. A fair reviewer will still
ask whether a robust kernel alone gives the same KITTI gain; we have not tested that.

## 6. Why rotation weight should matter (theory)

- S. Huang, Y. Wang, G. Dissanayake et al., "How far is SLAM from a linear least squares problem?", IROS 2010
  ([pdf](https://www.dfki.de/fileadmin/user_upload/import/5033_huang_iros_10.pdf)): with orientations known,
  pose-graph SLAM becomes linear; the nonlinearity comes from rotation.
- L. Carlone, R. Tron, K. Daniilidis, F. Dellaert, "Initialization techniques for 3D SLAM: a survey on rotation
  estimation", ICRA 2015 ([GT repository](https://repository.gatech.edu/entities/publication/ae22ec18-65d1-4075-8c0b-55c694ac5467)).
- D. Rosen, L. Carlone, A. Bandeira, J. Leonard, SE-Sync, IJRR 2019 ([arXiv 1612.07386](https://arxiv.org/abs/1612.07386)):
  the noise model has separate rotational (κ) and translational (τ) precisions, so the ratio we tuned is a
  named parameter of the standard model.
- J. Zhang, M. Kaess, S. Singh, degeneracy, ICRA 2016 ([CMU](https://www.ri.cmu.edu/publications/on-degeneracy-of-optimization-based-state-estimation-problems)):
  per-direction information varies with scene geometry, which is why a single fixed ratio cannot be right
  everywhere (our NCLT transfer failure is consistent with this).

These give a story for *why* our KITTI result could happen. They do not show it is right for KISS-SLAM;
that remains our measurement.

## 7. NCLT multi-sensor baselines

- Z. Yuan, F. Lang, T. Xu, X. Yang, LIW-OAM, [arXiv 2302.14298](https://arxiv.org/abs/2302.14298): tightly
  coupled LiDAR-IMU-wheel on 12 NCLT sessions. ATE RMSE on our sessions: 2012-01-15 1.46 m, 2012-05-26 1.41 m,
  2012-06-15 1.31 m, 2012-08-04 1.46 m. Baselines in the same table: FAST-LIO2 1.65 / 2.62 / 2.09 / 2.43 m,
  LIO-SAM 2.12 / failed / 1.51 / 2.26 m. Their alignment and ground-truth handling may differ from ours, so
  the numbers are not directly comparable, but the gap to our loosely coupled results (e.g. mean L+I ATE
  32.9 m on 2012-01-15 in the tune sweep) is too big to be a protocol difference.
- N. Carlevaris-Bianco, A. Ushani, R. Eustice, NCLT dataset paper (cited in report).
- Zhang et al., M3DGR / Ground-Fusion++, [arXiv 2507.08364](https://arxiv.org/abs/2507.08364): 40-system
  benchmark with wheel, IMU, GNSS, LiDAR on a ground robot; finds wheel-integrated systems more robust. Not NCLT.

I found no NCLT paper that compares identity vs fixed vs fitted noise models across sensor subsets with
repeats. That design is ours, but its value is diagnostic (how much weighting matters, and which sensor
additions it breaks), not a new fusion method.

## 8. Where that leaves the project

Honest position:

- **Not novel:** weighting rotation more than translation; using estimated (Hessian/learned) covariances in
  a LiDAR pose graph; showing estimated weights beat identity in KISS-ICP-based SLAM (Gupta et al. 2026);
  robust loop-closure back-ends; multi-sensor fusion on NCLT.
- **Not found anywhere else (as of 2026-11-01):**
  1. A held-out, pre-registered evaluation of a fixed rotation/translation ratio on KISS-SLAM (Phase 1). Small.
     Most useful as the benchmark PR the maintainer invited.
  2. Fixed ratio vs Gupta et al.'s Hessian weights vs identity, on the same sequences (KITTI, NCLT, and at
     least one of their datasets, e.g. HeLiPR, which is already planned). This is the most promising next
     step: if a single constant recovers most of the Hessian's gain at no runtime cost, that is useful; if it
     does not, that explains our NCLT failure. Needs their code (the maintainer named `PRBonn/retro_slam`; I
     could not confirm it is public yet).
  3. Consistency (NEES and proper scoring rules, per Rønning et al.) of these weightings in a LiDAR pose graph.
     Needs our identity-NEES bug fixed first.
  4. The negative transfer result itself (KITTI gain does not carry to NCLT), reported with the split protocol.
- **Things a reviewer will ask that we cannot answer yet:** does a robust kernel (DCS/GNC) alone give the KITTI
  gain; how does the loosely coupled NCLT graph compare to LIW-OAM-class systems; what weight did Gupta et al.'s
  "no-info" ablation use.

## Limits of this search

About 30 web searches plus source checks on 2026-11-01. arXiv full text could not be downloaded directly from
this environment; Gupta et al. 2026 was read through the arXiv HTML/PDF via a fetch tool, and some of its
details came first from a secondary summary (pith.science) and were then confirmed in the PDF text. I could not
read the full KISS-SLAM or MapClosures papers, only abstracts and partial HTML. Paywalled venues (IEEE Xplore)
were not searched directly. A paper that does exactly what we did could still exist under different wording;
I found none.
