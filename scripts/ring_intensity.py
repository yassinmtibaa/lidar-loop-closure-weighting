#!/usr/bin/env python3
"""usage: ring_intensity.py <seq> [every_nth_scan]
Range-noise sigma (second differences along a beam, stride 1, ground points) by range bin and by
intensity of the middle point, plus ground-intensity statistics."""
import os, sys
import numpy as np

seq = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 20
d = os.path.expanduser(f"~/lidar_loop/data/kitti/dataset/sequences/{seq}/velodyne")
files = sorted(os.listdir(d))[::step]
rbins = [(3, 10), (10, 20), (20, 40)]
ibins = [(0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5), (0.5, 1.01)]
acc = {(rb, ib): [] for rb in rbins for ib in ibins}
gi = []

for f in files:
    P = np.fromfile(f"{d}/{f}", dtype=np.float32).reshape(-1, 4).astype(np.float64)
    xyz, inten = P[:, :3], P[:, 3]
    az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])) % 360
    rng = np.linalg.norm(xyz, axis=1)
    rh = np.linalg.norm(xyz[:, :2], axis=1)
    z = xyz[:, 2]
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
        gi.append(it[i][h[i] < 20])
        for lo, hi in rbins:
            mr = (h[i] >= lo) & (h[i] < hi)
            for a0, a1 in ibins:
                m = mr & (it[i] >= a0) & (it[i] < a1)
                if m.any():
                    acc[((lo, hi), (a0, a1))].append(d2[m])

g = np.concatenate(gi)
print(f"seq {seq}: ground intensity (<20 m): median {np.median(g):.3f}  p10 {np.percentile(g, 10):.3f}  p90 {np.percentile(g, 90):.3f}  max {g.max():.2f}")
print("range    intensity       n      sigma(cm)")
for rb in rbins:
    for ib in ibins:
        lst = acc[(rb, ib)]
        if not lst:
            continue
        x = np.concatenate(lst)
        if len(x) > 2000:
            sig = 1.4826 * np.median(np.abs(x - np.median(x))) / np.sqrt(6)
            print(f"{rb[0]:2d}-{rb[1]:2d} m  {ib[0]:.1f}-{min(ib[1], 1.0):.1f}  {len(x):9d}   {100 * sig:7.2f}")
