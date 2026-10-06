#!/usr/bin/env python3
"""Fit sigma(r) = sqrt(a^2 + (k r)^2) on seqs 00-05, check on 06-07; pooled beam table."""
import json, os
import numpy as np
home = os.path.expanduser("~/lidar_loop")
J = lambda s: json.load(open(f"{home}/results/ring_stats_{s}.json"))
centers = {"3-5": 4.0, "5-10": 7.5, "10-20": 15.0, "20-30": 25.0, "30-40": 35.0}

def pooled(seqs, key="sigma1"):
    out = {}
    for b in centers:
        num = den = 0.0
        for s in seqs:
            r = J(s)["bins"][b]
            if r[key] is not None:
                num += r[key] * r["n1"]; den += r["n1"]
        out[b] = num / den
    return out

train = ["00", "01", "02", "03", "04", "05"]
fit, fit2 = pooled(train), pooled(train, "sigma2")
R = np.array(list(centers.values())); S = np.array([fit[b] for b in centers])
best = None
for a in np.arange(0.002, 0.020, 0.0002):
    for k in np.arange(0.0, 0.002, 0.00002):
        e = np.sum(((np.sqrt(a * a + (k * R) ** 2) - S) / S) ** 2)
        if best is None or e < best[0]:
            best = (e, a, k)
_, a, k = best
print(f"fit on 00-05: sigma(r) = sqrt(a^2 + (k r)^2), a = {100*a:.2f} cm, k = {100*k:.3f} cm/m")
print(f"whiteness: pooled sigma2/sigma1 at 5-10 m = {fit2['5-10'] / fit['5-10']:.2f} (white noise ~1.0)")
print("bin      model(cm)   06 meas  err%    07 meas  err%")
worst = 0.0
chk = {s: pooled([s]) for s in ("06", "07")}
for b, r in centers.items():
    m = np.sqrt(a * a + (k * r) ** 2)
    line = f"{b:6s}   {100*m:8.2f}"
    for s in ("06", "07"):
        e = 100 * (m - chk[s][b]) / chk[s][b]; worst = max(worst, abs(e))
        line += f"   {100*chk[s][b]:6.2f} {e:+6.1f}"
    print(line)
print(f"worst held-out error: {worst:.1f}% (prediction: <= 20%)")

beams = [J(s)["beams"] for s in train]
tab = sorted(((float(np.median([b[i]["el_deg"] for b in beams])),
               float(np.median([b[i]["fill"] for b in beams]))) for i in range(64)))
el = np.array([t[0] for t in tab]); fill = np.array([t[1] for t in tab])
print(f"\n64 beams, elevation {el.min():.2f} .. {el.max():.2f} deg, smallest gap {np.diff(el).min():.2f} deg")
print(f"median fill: beams above 0 deg {np.median(fill[el > 0]):.2f} (n={int((el > 0).sum())}), "
      f"below -10 deg {np.median(fill[el < -10]):.2f} (n={int((el < -10).sum())})")
print("el:fill% ", " ".join(f"{e:.2f}:{100*f:.0f}" for e, f in tab))
json.dump({"a_m": a, "k": k, "beam_elevation_deg": el.tolist(), "beam_fill": fill.tolist()},
          open(f"{home}/configs/velodyne_fit.json", "w"), indent=1)
