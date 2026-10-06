#!/usr/bin/env python3
"""usage: ring_noise.py <seq> [every_nth_scan]
Per-beam range noise from second differences along azimuth on ground points.
Beams = runs of the file between large azimuth drops. Residual road roughness still included."""
import os, sys
import numpy as np

seq = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 20
d = os.path.expanduser(f"~/lidar_loop/data/kitti/dataset/sequences/{seq}/velodyne")
files = sorted(os.listdir(d))[::step]
bins = [(3, 5), (5, 10), (10, 20), (20, 30), (30, 40)]
d2s = {b: [] for b in bins}
nseg, sizes = [], []

for f in files:
    P = np.fromfile(f"{d}/{f}", dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
    az = np.degrees(np.arctan2(P[:, 1], P[:, 0])) % 360
    rng = np.linalg.norm(P, axis=1)
    rh = np.linalg.norm(P[:, :2], axis=1)
    z = P[:, 2]
    brk = np.where(np.diff(az) < -90)[0] + 1
    segs = np.split(np.arange(len(P)), brk)
    nseg.append(len(segs))
    for s in segs:
        sizes.append(len(s))
        if len(s) < 50:
            continue
        a, r, h, zz = az[s], rng[s], rh[s], z[s]
        da = np.diff(a)
        ok = (da > 0.12) & (da < 0.25)
        i = np.where(ok[:-1] & ok[1:])[0] + 1
        if len(i) == 0:
            continue
        d2 = r[i - 1] - 2 * r[i] + r[i + 1]
        gnd = np.ones(len(i), bool)
        for k in (-1, 0, 1):
            gnd &= (zz[i + k] < -1.3) & (zz[i + k] > -2.3)
        smooth = (np.abs(r[i] - r[i - 1]) < 0.3) & (np.abs(r[i + 1] - r[i]) < 0.3)
        m = gnd & smooth
        for lo, hi in bins:
            k = m & (h[i] >= lo) & (h[i] < hi)
            d2s[(lo, hi)].append(d2[k])

sz = np.array(sizes)
print(f"seq {seq}: {len(files)} scans; beams per scan: min {min(nseg)} median {int(np.median(nseg))} max {max(nseg)}")
print(f"points per beam: median {int(np.median(sz))} min {sz.min()} max {sz.max()}; beams under 1000 pts: {100*(sz<1000).mean():.0f}%")
print("range bin   n_triples   sigma_MAD(cm)   sigma_std(cm)")
for b in bins:
    if d2s[b]:
        x = np.concatenate(d2s[b])
        if len(x) > 100:
            mad = 1.4826 * np.median(np.abs(x - np.median(x))) / np.sqrt(6)
            print(f"{b[0]:3d}-{b[1]:2d} m  {len(x):10d}   {100*mad:8.2f}   {100*x.std()/np.sqrt(6):8.2f}")
