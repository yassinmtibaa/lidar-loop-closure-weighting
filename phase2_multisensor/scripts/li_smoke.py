#!/usr/bin/env python3
"""P2.2f (DECISIONS 2026-10-25): LiDAR odometry + IMU preintegration in one GTSAM graph on the smoke session.
usage: .venv/bin/python scripts/li_smoke.py   -> results/li_smoke_2013-01-10.txt"""
import os
import sys

import gtsam
import numpy as np
from gtsam.symbol_shorthand import B, V, X

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [ROOT, os.path.join(ROOT, "scripts")]
import metrics  # noqa: E402
from fusion import graph, nclt, sensors  # noqa: E402

SEQ = "2013-01-10"
S = os.path.join(ROOT, "data", "nclt", SEQ)
GT = os.path.join(ROOT, "data", "nclt", "ground_truth", f"groundtruth_{SEQ}.csv")
RUN = os.path.join(ROOT, "results", "nclt_smoke_2013-01-10", "2026-10-10_16-39-54")
LIDAR_SIGMA = (0.002, 0.02)          # rad, m per frame (placeholder)
BIAS_PRIOR = (0.1, 0.01)             # acc m/s^2, gyro rad/s
BIAS_WALK = (1e-3, 1e-4)             # per LiDAR frame

P = np.load(f"{RUN}/{SEQ}_poses.npy")
G = metrics.load(f"{RUN}/{SEQ}_gt_kitti.txt")
t = nclt.lidar_timestamps(S, GT)
assert len(t) == len(P) == len(G)
imu = nclt.read_imu(S)

R_LI, resid, nwin = sensors.fit_imu_rotation(imu, t, P)
rest = imu.t < imu.t[0] + 1_000_000
f = R_LI @ imu.acc[rest].mean(0)                      # specific force at rest, LiDAR axes
roll, pitch = np.arctan2(f[1], f[2]), np.arctan2(-f[0], np.hypot(f[1], f[2]))
T0 = gtsam.Pose3(gtsam.Rot3.Ypr(0.0, pitch, roll), np.zeros(3)).matrix()   # gravity-aligned start (z up)
bias0 = gtsam.imuBias.ConstantBias(np.zeros(3), R_LI @ imu.gyro[rest].mean(0))
params = sensors.imu_params(z_up=True)

g = graph.PoseGraph()
for k in range(len(t)):
    Tk = T0 @ P[k]
    g.add_variable(X(k), Tk)
    g.initial.insert(V(k), np.zeros(3) if k == 0 else (Tk[:3, 3] - (T0 @ P[k - 1])[:3, 3]) / ((t[k] - t[k - 1]) / 1e6))
    g.initial.insert(B(k), bias0)
g.add_factor(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(T0), gtsam.noiseModel.Isotropic.Sigma(6, 1e-6)), "prior")
g.add_factor(gtsam.PriorFactorVector(V(0), np.zeros(3), gtsam.noiseModel.Isotropic.Sigma(3, 0.01)), "imu")
bias_prior = gtsam.noiseModel.Diagonal.Sigmas(np.r_[[BIAS_PRIOR[0]] * 3, [BIAS_PRIOR[1]] * 3])
g.add_factor(gtsam.PriorFactorConstantBias(B(0), bias0, bias_prior), "imu")
lidar_noise = sensors.between_noise([LIDAR_SIGMA[0]] * 3, [LIDAR_SIGMA[1]] * 3)
walk = gtsam.noiseModel.Diagonal.Sigmas(np.r_[[BIAS_WALK[0]] * 3, [BIAS_WALK[1]] * 3])
for k in range(len(t) - 1):
    g.add_between(X(k), X(k + 1), np.linalg.inv(P[k]) @ P[k + 1], lidar_noise, "lidar_odom")
    pim = sensors.preintegrate(imu, t[k], t[k + 1], params, bias0, R_LI)
    g.add_factor(gtsam.ImuFactor(X(k), V(k), X(k + 1), V(k + 1), B(k), pim), "imu")
    g.add_factor(gtsam.BetweenFactorConstantBias(B(k), B(k + 1), gtsam.imuBias.ConstantBias(), walk), "imu")

keys = [X(k) for k in range(len(t))]
lines = [f"R_LI fit: median residual {resid:.4f} rad over {nwin} windows; R_LI =\n{np.round(R_LI, 4)}"]
out = {}
for name, enabled in (("L-only (GTSAM)", ["prior", "lidar_odom"]), ("L+I", ["prior", "lidar_odom", "imu"])):
    if name == "L-only (GTSAM)":     # IMU variables unused when imu is off: optimize the pose subgraph only
        sub = graph.PoseGraph()
        sub.factors = [(s, fac) for s, fac in g.factors if s in enabled]
        for k in keys:
            sub.initial.insert(k, g.initial.atPose3(k))
        est = sub.optimize()
    else:
        est = g.optimize(enabled=enabled)
    E = graph.poses(est, keys)
    out[name] = E
    ok = np.isfinite(E).all()
    ate, rpe, kitti, _ = metrics.compute(G, E)
    lines.append(f"{name:15s} finite={ok} ATE {ate:.3f} m  RPE100 {rpe:.3f} m  KITTI {kitti:.3f} %")
ate0, _, kitti0, _ = metrics.compute(G, P)
lines.append(f"{'KISS-ICP':15s} ATE {ate0:.3f} m  KITTI {kitti0:.3f} %")
dev = np.abs((np.linalg.inv(T0) @ out["L-only (GTSAM)"])[:, :3, 3] - P[:, :3, 3]).max()
lines.append(f"L-only GTSAM vs KISS-ICP max position difference {dev:.2e} m (must be < 1e-3)")
_, _, kitti_li, _ = metrics.compute(G, out["L+I"])
rel = kitti_li / kitti0 - 1
lines.append(f"P2.2f: finite {np.isfinite(out['L+I']).all()}, KITTI change {100 * rel:+.1f}% "
             f"-> {'MET' if np.isfinite(out['L+I']).all() and abs(rel) <= 0.10 and dev < 1e-3 else 'NOT MET'}")
print("\n".join(lines))
open(os.path.join(ROOT, "results", f"li_smoke_{SEQ}.txt"), "w").write("\n".join(lines) + "\n")
