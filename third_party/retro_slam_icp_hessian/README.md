# ICP Gauss-Newton information (Retro-SLAM `icp-hessian` ablation)

`Registration.cpp` is the snippet Saurabh Gupta (KISS-SLAM maintainer) posted on PRBonn/kiss-slam#61 on
2026-11-14, copied verbatim (only HTML entities decoded). MIT licensed; the header is kept as received.
It replaces `cpp/kiss_icp/core/Registration.cpp` of KISS-ICP. It is **not compiled or used yet**: see
`phase2_multisensor/DECISIONS.md`, entry 2026-11-14, for the pre-registered experiment.

What it adds after the ICP loop converges:
- `hessian_` = (sum_i w_i J_i^T J_i) / N, with J_i = [I, -hat(p_i)], w_i the Geman-McClure weight, N the number of
  correspondences, eigenvalues clamped from below at 1e-3.
- `fitness_` = N / number of voxels in the map.

Not in the snippet (needed before use): the `hessian_` / `fitness_` members and accessors in `Registration.hpp`,
and Python bindings so KISS-SLAM can read them.

Conventions to get right (our notes, not the maintainer's):
1. Order is Sophus/KISS (translation, rotation). GTSAM Pose3 is (rotation, translation): permute.
2. p_i are the source points already transformed into the **map frame**, and the update is applied on the left
   (`T_icp = estimation * T_icp`). So `hessian_` is the information of a left (map-frame) perturbation. For a
   body-frame relative-pose factor convert with the adjoint: H_body = Ad_T^T H_map Ad_T. Using `hessian_` directly
   would make the rotation weight grow with the distance from the map origin.
3. Dividing by N removes the dependence on point count, and no residual variance enters. The absolute scale is
   therefore arbitrary: it is not a calibrated covariance. Only its shape (rotation vs translation, degenerate
   directions) carries information.
4. g2o (KISS-SLAM) rotation information weights the half angle: GTSAM rotation block x4 for g2o (cross block x2).
