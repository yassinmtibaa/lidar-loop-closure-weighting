#!/usr/bin/env python3
"""usage: rpe_segments.py <gt_kitti.txt> <est0.txt> [est1.txt ...]
Per-100m-segment translation RPE, same idea as evo_rpe -d 100 -u m."""
import sys
import numpy as np

def load(p):
    a = np.loadtxt(p).reshape(-1, 3, 4)
    T = np.tile(np.eye(4), (len(a), 1, 1))
    T[:, :3, :] = a
    return T

gt = load(sys.argv[1])
ests = [load(p) for p in sys.argv[2:]]
pos = gt[:, :3, 3]
dist = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))]

pairs, i = [], 0
while True:
    j = int(np.searchsorted(dist, dist[i] + 100.0))
    if j >= len(dist):
        break
    pairs.append((i, j))
    i = j

def rpe(est, i, j):
    rel_gt = np.linalg.inv(gt[i]) @ gt[j]
    rel_est = np.linalg.inv(est[i]) @ est[j]
    return np.linalg.norm((np.linalg.inv(rel_gt) @ rel_est)[:3, 3])

errs = [[rpe(e, i, j) for i, j in pairs] for e in ests]
print("seg  frames        " + "  ".join(f"est{k}" for k in range(len(ests))))
for s, (i, j) in enumerate(pairs):
    print(f"{s:3d}  {i:5d}-{j:5d}  " + "  ".join(f"{e[s]:6.3f}" for e in errs))
for k, e in enumerate(errs):
    e = np.array(e)
    print(f"est{k}: rmse {np.sqrt((e**2).mean()):.4f}  mean {e.mean():.4f}  max {e.max():.3f}")
