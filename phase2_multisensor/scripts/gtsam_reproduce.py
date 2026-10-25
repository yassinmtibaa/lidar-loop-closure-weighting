#!/usr/bin/env python3
"""Phase 2.2 check: rebuild each existing KISS-SLAM run on the TUNE sessions in GTSAM and compare
(DECISIONS 2026-10-25). Writes results/gtsam_reproduction.csv and results/gtsam_odom_chain.csv.
usage: .venv/bin/python scripts/gtsam_reproduce.py"""
import csv
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [ROOT, os.path.join(ROOT, "scripts")]
import metrics  # noqa: E402
from fusion import graph  # noqa: E402
from fusion.g2o_io import read_g2o  # noqa: E402

TUNE = ["2012-01-15", "2012-04-29", "2012-08-04"]   # never validation or test sessions here
CONDS = ["slam_default", "w_rot1e3"]
RES = os.path.join(ROOT, "results")


def rot_deg(A, B):
    R = A[:, :3, :3].transpose(0, 2, 1) @ B[:, :3, :3]
    return np.degrees(np.arccos(np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1))).max()


def two_stage(loc, fine, quaternion_error):
    orig = graph.noise_from_g2o_info
    graph.noise_from_g2o_info = lambda info: orig(info, quaternion_error)
    try:
        return graph.kiss_slam_two_stage(loc, fine)
    finally:
        graph.noise_from_g2o_info = orig


rows, chain_rows = [], []
for seq in TUNE:
    icp = os.path.join(RES, f"icp_baseline_seq{seq}", "latest")
    icp_m = metrics.compute(metrics.load(f"{icp}/{seq}_gt_kitti.txt"), metrics.load(f"{icp}/{seq}_poses_kitti.txt"))
    for cond in CONDS:
        for r in (1, 2, 3):
            d = os.path.join(RES, f"{cond}_r{r}_seq{seq}", "latest")
            loc = read_g2o(f"{d}/local_maps/local_map_graph.g2o")
            fine = read_g2o(f"{d}/trajectory.g2o")
            G = metrics.load(f"{d}/{seq}_gt_kitti.txt")
            E_kiss = np.load(f"{d}/{seq}_poses.npy")
            kp_kiss = np.array([loc.vertices[k] for k in range(len(loc.vertices))])
            ate_k, _, kitti_k, _ = metrics.compute(G, E_kiss)
            for mode, qe in (("quaternion", True), ("permute_only", False)):
                kp, fr = two_stage(loc, fine, qe)
                ate_g, _, kitti_g, _ = metrics.compute(G, fr)
                a1_t = np.linalg.norm(kp[:, :3, 3] - kp_kiss[:, :3, 3], axis=1).max()
                a1_r = rot_deg(kp, kp_kiss)
                row = dict(seq=seq, cond=cond, run=r, mapping=mode, keyposes=len(kp),
                           closures=sum(e.j != e.i + 1 for e in loc.edges),
                           a1_max_trans_m=a1_t, a1_max_rot_deg=a1_r,
                           frame_max_trans_m=np.linalg.norm(fr[:, :3, 3] - E_kiss[:, :3, 3], axis=1).max(),
                           ate_kiss=ate_k, ate_gtsam=ate_g, kitti_kiss=kitti_k, kitti_gtsam=kitti_g,
                           d_ate=ate_g - ate_k, d_kitti=kitti_g - kitti_k)
                row["A1"] = a1_t < 0.10 and a1_r < 0.1
                row["A2"] = abs(row["d_ate"]) < 0.02 and abs(row["d_kitti"]) < 0.002
                if qe:
                    _, fr2 = two_stage(loc, fine, qe)
                    row["determinism_max_m"] = np.abs(fr2[:, :3, 3] - fr[:, :3, 3]).max()
                rows.append(row)
                print(f"{seq} {cond} r{r} {mode:12s} A1 {a1_t:.4f} m {a1_r:.4f} deg | ATE {ate_k:.4f}->{ate_g:.4f} "
                      f"KITTI {kitti_k:.4f}->{kitti_g:.4f} | A1 {row['A1']} A2 {row['A2']}", flush=True)
            if cond == "slam_default":
                ch = graph.chain(graph.fine_odometry(fine))
                ate_c, _, kitti_c, _ = metrics.compute(G, ch)
                chain_rows.append(dict(seq=seq, run=r, ate_chain=ate_c, kitti_chain=kitti_c, ate_icp=icp_m[0],
                                       kitti_icp=icp_m[2], kitti_rel_diff=kitti_c / icp_m[2] - 1,
                                       P2_2c=abs(kitti_c / icp_m[2] - 1) < 0.05))

for name, rs in (("gtsam_reproduction.csv", rows), ("gtsam_odom_chain.csv", chain_rows)):
    keys = list(dict.fromkeys(k for r in rs for k in r))
    with open(os.path.join(RES, name), "w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows({k: (f"{v:.6f}" if isinstance(v, float) else v) for k, v in r.items()} for r in rs)
q = [r for r in rows if r["mapping"] == "quaternion"]
print(f"quaternion mapping: A1 {sum(r['A1'] for r in q)}/{len(q)}, A2 {sum(r['A2'] for r in q)}/{len(q)}, "
      f"determinism max {max(r['determinism_max_m'] for r in q):.2e} m")
n = [r for r in rows if r["mapping"] == "permute_only"]
print(f"permute-only mapping: A1 {sum(r['A1'] for r in n)}/{len(n)}, A2 {sum(r['A2'] for r in n)}/{len(n)}")
print(f"P2.2c: {sum(r['P2_2c'] for r in chain_rows)}/{len(chain_rows)}")
