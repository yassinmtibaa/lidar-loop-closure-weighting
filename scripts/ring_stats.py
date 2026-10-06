#!/usr/bin/env python3
"""usage: ring_stats.py <seq> [every_nth_scan]
Writes results/ring_stats_<seq>.json: per range bin robust range-noise sigma (second differences
along a beam, stride 1 and 2) and per beam (file order): median elevation, fill fraction, median range."""
import json, os, sys
import numpy as np

seq = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 20
home = os.path.expanduser("~/lidar_loop")
d = f"{home}/data/kitti/dataset/sequences/{seq}/velodyne"
files = sorted(os.listdir(d))[::step]
bins = [(3, 5), (5, 10), (10, 20), (20, 30), (30, 40)]
acc = {s: {b: [] for b in bins} for s in (1, 2)}
el_b = [[] for _ in range(64)]; n_b = [[] for _ in range(64)]; r_b = [[] for _ in range(64)]
daz = []

def triples(a, r, h, zz, stride):
    w = 2 * stride
    if len(a) < w + 3:
        return None
    da, dr = np.diff(a), np.abs(np.diff(r))
    good = ((da > 0.12) & (da < 0.25) & (dr < 0.3)).astype(int)
    ok = np.convolve(good, np.ones(w, int), "valid") == w
    i = np.where(ok)[0] + stride
    if len(i) == 0:
        return None
    d2 = r[i - stride] - 2 * r[i] + r[i + stride]
    g = np.ones(len(i), bool)
    for k in (-stride, 0, stride):
        g &= (zz[i + k] < -1.3) & (zz[i + k] > -2.3)
    return d2[g], h[i][g]

for f in files:
    P = np.fromfile(f"{d}/{f}", dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
    az = np.degrees(np.arctan2(P[:, 1], P[:, 0])) % 360
    rng = np.linalg.norm(P, axis=1); rh = np.linalg.norm(P[:, :2], axis=1)
    el = np.degrees(np.arctan2(P[:, 2], rh)); z = P[:, 2]
    segs = np.split(np.arange(len(P)), np.where(np.diff(az) < -90)[0] + 1)
    if len(segs) != 64:
        continue
    for k, s in enumerate(segs):
        el_b[k].append(np.median(el[s])); n_b[k].append(len(s)); r_b[k].append(np.median(rng[s]))
        da = np.diff(az[s])
        da = da[(da > 0.05) & (da < 0.5)]
        if len(da) > 5:
            daz.append(np.median(da))
        if len(s) < 50:
            continue
        for stride in (1, 2):
            t = triples(az[s], rng[s], rh[s], z[s], stride)
            if t is None:
                continue
            d2, h = t
            for lo, hi in bins:
                acc[stride][(lo, hi)].append(d2[(h >= lo) & (h < hi)])

az_step = float(np.median(daz)); full = 360.0 / az_step
out = {"seq": seq, "scans": len(files), "az_step_deg": az_step, "bins": {}, "beams": []}
print(f"seq {seq}: az step {az_step:.3f} deg ({full:.0f} samples/rev)")
print("bin      n(stride1)  sigma1(cm)  sigma2(cm)  ratio2/1")
for lo, hi in bins:
    row = {}
    for stride in (1, 2):
        lst = acc[stride][(lo, hi)]
        x = np.concatenate(lst) if lst else np.array([])
        row[f"n{stride}"] = int(len(x))
        row[f"sigma{stride}"] = float(1.4826 * np.median(np.abs(x - np.median(x))) / np.sqrt(6)) if len(x) > 100 else None
    out["bins"][f"{lo}-{hi}"] = row
    if row["sigma1"] and row["sigma2"]:
        print(f"{lo:3d}-{hi:2d} m  {row['n1']:10d}  {100*row['sigma1']:9.2f}  {100*row['sigma2']:9.2f}  {row['sigma2']/row['sigma1']:7.2f}")
for k in range(64):
    out["beams"].append({"file_index": k, "el_deg": float(np.median(el_b[k])),
                          "fill": float(np.median(n_b[k]) / full), "median_range_m": float(np.median(r_b[k]))})
json.dump(out, open(f"{home}/results/ring_stats_{seq}.json", "w"), indent=1)
