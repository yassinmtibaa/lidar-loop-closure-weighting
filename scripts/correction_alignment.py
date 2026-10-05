#!/usr/bin/env python3
"""usage: correction_alignment.py <seq> <slam_run> [odo_run]
Per 100 m segment: odometry error vector, SLAM error vector, and the change SLAM made.
delivered = projection of the change onto the needed direction (-odometry error) divided by
|odometry error|^2: 1 = perfect, 0 = unrelated, <0 = wrong direction. Translation only."""
import os, sys
import numpy as np
seq, run = sys.argv[1], sys.argv[2]
odo = sys.argv[3] if len(sys.argv) > 3 else "baseline"
h = os.path.expanduser("~/lidar_loop/results")

def load(p):
    a = np.loadtxt(p).reshape(-1, 3, 4)
    T = np.tile(np.eye(4), (len(a), 1, 1)); T[:, :3, :] = a
    return T

gt = load(f"{h}/{run}_seq{seq}/latest/{seq}_gt_kitti.txt")
S = load(f"{h}/{run}_seq{seq}/latest/{seq}_poses_kitti.txt")
O = load(f"{h}/{odo}_seq{seq}/latest/{seq}_poses_kitti.txt")
n = min(len(gt), len(S), len(O))
pos = gt[:n, :3, 3]
dist = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))]
pairs, i = [], 0
while True:
    j = int(np.searchsorted(dist, dist[i] + 100.0))
    if j >= n: break
    pairs.append((i, j)); i = j

def err(est, i, j):
    rel_gt = np.linalg.inv(gt[i]) @ gt[j]
    rel_est = np.linalg.inv(est[i]) @ est[j]
    return (np.linalg.inv(rel_gt) @ rel_est)[:3, 3]

rows = []
print("seg   frames       |odo err|  |slam err|   |change|  delivered")
for s, (i, j) in enumerate(pairs):
    eo, es = err(O, i, j), err(S, i, j)
    c = es - eo
    no = np.linalg.norm(eo)
    deliv = float(np.dot(c, -eo) / (no * no + 1e-9))
    rows.append((no, np.linalg.norm(es), np.linalg.norm(c), deliv))
    print(f"{s:3d}  {i:5d}-{j:5d}   {no:8.3f}  {np.linalg.norm(es):9.3f}  {np.linalg.norm(c):8.3f}  {deliv:8.2f}")
r = np.array(rows)
print(f"seq {seq}: {len(r)} segments, rmse odo {np.sqrt((r[:,0]**2).mean()):.3f}  slam {np.sqrt((r[:,1]**2).mean()):.3f}")
print(f"improved {int((r[:,1] < r[:,0]).sum())}, worsened {int((r[:,1] > r[:,0]).sum())}")
print(f"mean delivered {r[:,3].mean():.2f}  median {np.median(r[:,3]):.2f}  segments with delivered < 0: {int((r[:,3] < 0).sum())}")
print(f"corr(|change|, |odo err|) = {np.corrcoef(r[:,2], r[:,0])[0,1]:.2f}")
