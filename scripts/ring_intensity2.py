#!/usr/bin/env python3
"""usage: ring_intensity2.py <seq> [every_nth_scan]
Like ring_intensity.py, adding sigma from triples whose 3 points share one intensity bin
(no paint/asphalt transitions) and the median second difference in mm (bias of the middle point)."""
import os, sys
import numpy as np

seq = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 20
d = os.path.expanduser(f"~/lidar_loop/data/kitti/dataset/sequences/{seq}/velodyne")
files = sorted(os.listdir(d))[::step]
rbins = [(3, 10), (10, 20), (20, 40)]
edges = np.array([0, 0.1, 0.2, 0.3, 0.5, 1.01]); nb = len(edges) - 1
A = {(rb, k): [] for rb in rbins for k in range(nb)}
S = {(rb, k): [] for rb in rbins for k in range(nb)}

for f in files:
    P = np.fromfile(f"{d}/{f}", dtype=np.float32).reshape(-1, 4).astype(np.float64)
    xyz, inten = P[:, :3], P[:, 3]
    az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])) % 360
    rng = np.linalg.norm(xyz, axis=1); rh = np.linalg.norm(xyz[:, :2], axis=1); z = xyz[:, 2]
    segs = np.split(np.arange(len(P)), np.where(np.diff(az) < -90)[0] + 1)
    if len(segs) != 64:
        continue
    for s in segs:
        if len(s) < 50:
            continue
        a, r, h, zz, it = az[s], rng[s], rh[s], z[s], inten[s]
        da, dr = np.diff(a), np.abs(np.diff(r))
        good = (da > 0.12) & (da < 0.25) & (dr < 0.3)
        i = np.where(good[:-1] & good[1:])[0] + 1
        if len(i) == 0:
            continue
        gnd = np.ones(len(i), bool)
        for k in (-1, 0, 1):
            gnd &= (zz[i + k] < -1.3) & (zz[i + k] > -2.3)
        i = i[gnd]
        if len(i) == 0:
            continue
        d2 = r[i - 1] - 2 * r[i] + r[i + 1]
        b0 = np.digitize(it[i], edges) - 1
        homog = (b0 == np.digitize(it[i - 1], edges) - 1) & (b0 == np.digitize(it[i + 1], edges) - 1)
        for rb in rbins:
            mr = (h[i] >= rb[0]) & (h[i] < rb[1])
            for k in range(nb):
                m = mr & (b0 == k)
                if m.any():
                    A[(rb, k)].append(d2[m])
                    if (m & homog).any():
                        S[(rb, k)].append(d2[m & homog])

sig = lambda x: 100 * 1.4826 * np.median(np.abs(x - np.median(x))) / np.sqrt(6)
print(f"seq {seq}")
print("range    intensity     n_all  sigma_all   n_same  sigma_same  median_d2(mm)")
for rb in rbins:
    for k in range(nb):
        if not A[(rb, k)] or not S[(rb, k)]:
            continue
        xa, xs = np.concatenate(A[(rb, k)]), np.concatenate(S[(rb, k)])
        if len(xs) > 2000:
            print(f"{rb[0]:2d}-{rb[1]:2d} m  {edges[k]:.1f}-{min(edges[k+1], 1.0):.1f}  {len(xa):9d}  {sig(xa):8.2f}  {len(xs):8d}  {sig(xs):9.2f}  {1000*np.median(xa):10.2f}")
