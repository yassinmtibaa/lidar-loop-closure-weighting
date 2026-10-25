#!/usr/bin/env python3
"""P2.2d / P2.2e (DECISIONS 2026-10-25): 2D dead reckoning on the smoke session vs ground truth.
usage: .venv/bin/python scripts/dead_reckoning_check.py [session]   (default 2013-01-10; smoke session only)"""
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from fusion import nclt, sensors  # noqa: E402

TRACK_WIDTH = 0.6125   # fitted on 2013-01-10, frozen (DECISIONS 2026-10-25)


def ate_2d(p_est, p_gt):
    """RMSE after the best 2D rigid alignment (rotation + translation, no scale)."""
    me, mg = p_est.mean(0), p_gt.mean(0)
    U, _, Vt = np.linalg.svd((p_est - me).T @ (p_gt - mg))
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return np.sqrt((np.linalg.norm((p_est - me) @ R.T + mg - p_gt, axis=1) ** 2).mean())


seq = sys.argv[1] if len(sys.argv) > 1 else "2013-01-10"
S = os.path.join(ROOT, "data", "nclt", seq)
gt_t, gt_p, _ = nclt.read_ground_truth(os.path.join(ROOT, "data", "nclt", "ground_truth", f"groundtruth_{seq}.csv"))
w, f = nclt.read_wheels(S), nclt.read_fog(S)
path = np.linalg.norm(np.diff(gt_p[:, :2], axis=0), axis=1).sum()
res = {}
for name, fog, b in (("wheels+FOG", f, None), ("wheels only", None, TRACK_WIDTH)):
    t, x, y, _ = sensors.dead_reckon(w, fog, b, t0=gt_t[0], t1=gt_t[-1])
    gx, gy = np.interp(t, gt_t, gt_p[:, 0]), np.interp(t, gt_t, gt_p[:, 1])
    res[name] = ate_2d(np.c_[x, y], np.c_[gx, gy])
    dist = np.hypot(np.diff(x), np.diff(y)).sum()
    print(f"{seq} {name:12s} 2D ATE {res[name]:7.2f} m = {100 * res[name] / path:5.2f}% of {path:.1f} m path; "
          f"integrated distance {dist:.1f} m")
print(f"P2.2d (wheels+FOG ATE < 5% of path): {'MET' if res['wheels+FOG'] < 0.05 * path else 'NOT MET'}")
r = res["wheels only"] / res["wheels+FOG"]
print(f"P2.2e (wheels-only ATE >= 3x wheels+FOG): ratio {r:.1f} -> {'MET' if r >= 3 else 'NOT MET'}")
