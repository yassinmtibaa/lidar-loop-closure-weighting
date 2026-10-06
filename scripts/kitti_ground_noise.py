#!/usr/bin/env python3
"""usage: kitti_ground_noise.py <seq> [every_nth_scan]
KITTI Velodyne scan statistics + ground-plane residual sigma per range bin.
Residual sigma mixes sensor noise and road roughness: an UPPER BOUND on sensor noise."""
import os, sys
import numpy as np

seq = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 20
d = os.path.expanduser(f"~/lidar_loop/data/kitti/dataset/sequences/{seq}/velodyne")
files = sorted(os.listdir(d))[::step]
bins = [(3, 5), (5, 10), (10, 20), (20, 30), (30, 40)]
res = {b: [] for b in bins}
counts, elev, rng = [], [], []

def fit_residuals(g):
    A = np.c_[g[:, 0], g[:, 1], np.ones(len(g))]
    coef = np.linalg.lstsq(A, g[:, 2], rcond=None)[0]
    r = g[:, 2] - A @ coef
    keep = np.abs(r) < 0.15
    if keep.sum() < 50:
        return None
    coef = np.linalg.lstsq(A[keep], g[keep, 2], rcond=None)[0]
    r = g[:, 2] - A @ coef
    return r[np.abs(r) < 0.15]

for f in files:
    P = np.fromfile(f"{d}/{f}", dtype=np.float32).reshape(-1, 4)
    xyz = P[:, :3].astype(np.float64)
    r = np.linalg.norm(xyz[:, :2], axis=1)
    counts.append(len(P))
    elev.append(np.degrees(np.arctan2(xyz[:, 2], r)))
    rng.append(np.sqrt((xyz ** 2).sum(1)))
    ground = (xyz[:, 2] > -2.3) & (xyz[:, 2] < -1.3)
    for lo, hi in bins:
        m = ground & (r >= lo) & (r < hi)
        if m.sum() > 200:
            out = fit_residuals(xyz[m])
            if out is not None:
                res[(lo, hi)].append(out)

c = np.array(counts); E = np.concatenate(elev); R = np.concatenate(rng)
print(f"seq {seq}: {len(files)} scans sampled (every {step}th)")
print(f"points/scan: mean {c.mean():.0f} min {c.min()} max {c.max()}")
print("elevation deg percentiles 0.1/50/99.9:", np.round(np.percentile(E, [0.1, 50, 99.9]), 2))
print("range m percentiles 50/99/99.9:", np.round(np.percentile(R, [50, 99, 99.9]), 1))
print("range bin   n_points   sigma_MAD(cm)   std(cm)")
for b in bins:
    if res[b]:
        x = np.concatenate(res[b])
        sig = 1.4826 * np.median(np.abs(x - np.median(x)))
        print(f"{b[0]:3d}-{b[1]:2d} m  {len(x):9d}   {100 * sig:8.2f}   {100 * x.std():8.2f}")
