#!/usr/bin/env python3
"""usage: metrics.py <run_dir> <seq>   (run_dir = the dated folder with <seq>_gt_kitti.txt / <seq>_poses_kitti.txt)
Prints ATE rmse (SE3-aligned), RPE over 100 m (consecutive pairs, like evo -d 100), KITTI translation metric (%).
Frames whose ground truth contains NaN are dropped from both trajectories (NCLT has a few NaN rows).
Importable: compute(G, E) on (N,4,4) arrays returns (ate, rpe, kitti, dropped)."""
import sys
import numpy as np

def load(p):
    a = np.loadtxt(p).reshape(-1, 3, 4)
    T = np.tile(np.eye(4), (len(a), 1, 1)); T[:, :3, :] = a
    return T

def compute(G, E):
    n = min(len(G), len(E)); G, E = G[:n], E[:n]
    ok = np.isfinite(G).all(axis=(1, 2)) & np.isfinite(E).all(axis=(1, 2))
    dropped = int((~ok).sum()); G, E = G[ok], E[ok]

    pg, pe = G[:, :3, 3], E[:, :3, 3]
    mg, me = pg.mean(0), pe.mean(0)
    U, S, Vt = np.linalg.svd((pe - me).T @ (pg - mg)); D = np.eye(3)
    if np.linalg.det(Vt.T @ U.T) < 0: D[2, 2] = -1
    R = Vt.T @ D @ U.T; t = mg - R @ me
    ate = np.sqrt((np.linalg.norm(pe @ R.T + t - pg, axis=1) ** 2).mean())

    dist = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pg, axis=0), axis=1))]
    rel = lambda T, i, j: np.linalg.inv(T[i]) @ T[j]
    segs, i = [], 0
    while True:
        j = int(np.searchsorted(dist, dist[i] + 100.0))
        if j >= n - dropped: break
        segs.append(np.linalg.norm((np.linalg.inv(rel(G, i, j)) @ rel(E, i, j))[:3, 3])); i = j
    rpe = np.sqrt((np.array(segs) ** 2).mean())

    errs = []
    for first in range(0, len(pg), 10):
        for L in (100, 200, 300, 400, 500, 600, 700, 800):
            last = int(np.searchsorted(dist, dist[first] + L))
            if last >= len(pg): continue
            e = np.linalg.inv(rel(G, first, last)) @ rel(E, first, last)
            errs.append(np.linalg.norm(e[:3, 3]) / L)
    kitti = 100 * np.mean(errs)
    return ate, rpe, kitti, dropped

if __name__ == "__main__":
    run, seq = sys.argv[1], sys.argv[2]
    ate, rpe, kitti, dropped = compute(load(f"{run}/{seq}_gt_kitti.txt"), load(f"{run}/{seq}_poses_kitti.txt"))
    print(f"{seq},{ate:.6f},{rpe:.6f},{kitti:.3f},dropped={dropped}")
