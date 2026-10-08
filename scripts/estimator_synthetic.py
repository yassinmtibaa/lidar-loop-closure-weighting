#!/usr/bin/env python3
"""White Gaussian noise of known sigma through the two second-difference estimators used so far."""
import numpy as np
rng = np.random.default_rng(0)
mad6 = lambda x: 1.4826 * np.median(np.abs(x - np.median(x))) / np.sqrt(6)

def triples(a, r, h, zz, stride):          # copied from scripts/ring_stats.py
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

for sigma in (0.005, 0.009, 0.015):
    R = 8.0 + sigma * rng.normal(size=(64, 671))                      # checker style: rows of ranges
    d2 = R[:, :-2] - 2 * R[:, 1:-1] + R[:, 2:]
    est1 = mad6(d2) / sigma
    out = []
    for _ in range(64):                                               # ring_stats style: one beam at a time
        n = 2000
        a = 0.179 * np.arange(n); r = 8.0 + sigma * rng.normal(size=n)
        hh = np.sqrt(np.maximum(r * r - 1.75 ** 2, 0)); zz = np.full(n, -1.75)
        t = triples(a, r, hh, zz, 1)
        if t is not None:
            out.append(t[0])
    est2 = mad6(np.concatenate(out)) / sigma
    print(f"sigma {100*sigma:.2f} cm: checker-style ratio {est1:.3f}   ring_stats.triples ratio {est2:.3f}   (want 1.00 +-0.02)")
