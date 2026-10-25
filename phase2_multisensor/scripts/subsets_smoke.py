#!/usr/bin/env python3
"""Phase 2.3 on the smoke session (DECISIONS 2026-10-25): estimate per-sensor noise, then run all 31 sensor subsets
on each of the 3 KISS-ICP inputs. usage: .venv/bin/python scripts/subsets_smoke.py [--noise-only] [jobs]
Writes results/noise_2013-01-10.txt and results/subsets_smoke_2013-01-10.csv."""
import csv
import itertools
import os
import sys
from multiprocessing import Pool

import gtsam
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [ROOT, os.path.join(ROOT, "scripts")]
import metrics  # noqa: E402
from fusion import nclt, sensors, subsets  # noqa: E402

SEQ = "2013-01-10"
S = os.path.join(ROOT, "data", "nclt", SEQ)
GT = os.path.join(ROOT, "data", "nclt", "ground_truth", f"groundtruth_{SEQ}.csv")
RUNS = [os.path.join(ROOT, "results", f"icp_smoke_r{r}_seq{SEQ}", "latest", f"{SEQ}_poses.npy") for r in (1, 2, 3)]
TRACK_WIDTH = 0.6125
OUT = os.path.join(ROOT, "results")


def robust(x, axis=0):
    return 1.4826 * np.median(np.abs(x - np.median(x, axis=axis)), axis=axis)


def setup():
    t = nclt.lidar_timestamps(S, GT)
    gt = nclt.read_ground_truth(GT)
    wheels, fog, imu, gps = nclt.read_wheels(S), nclt.read_fog(S), nclt.read_imu(S), nclt.read_gps(S)
    m = subsets.Measurements(t, wheels, fog, imu, gps, TRACK_WIDTH)
    P = [np.load(p) for p in RUNS]
    R_LI = sensors.fit_imu_rotation(imu, t, P[0])[0]     # run 1 only, as in li_smoke.py
    odom = [subsets.lidar_body_odometry(p, R_LI) for p in P]
    G = subsets.gt_body_poses(gt, t)
    return t, m, odom, G, gps, R_LI


def estimate_noise(t, m, odom, G, gps):
    rel = np.array([np.linalg.inv(G[k]) @ G[k + 1] for k in range(len(G) - 1)])
    lines = []
    # L: Logmap residual inv(z_gt) z per frame, all 3 runs pooled
    r = np.array([gtsam.Pose3.Logmap(gtsam.Pose3(np.linalg.inv(g) @ z)) for o in odom for g, z in zip(rel, o)])
    lidar = robust(r)
    # W: planar residuals; F: yaw residuals
    gyaw = np.arctan2(rel[:, 1, 0], rel[:, 0, 0])
    wr = m.wheel - np.c_[rel[:, 0, 3], rel[:, 1, 3], gyaw]
    wr[:, 2] = np.angle(np.exp(1j * wr[:, 2]))
    wheel = robust(wr)
    fog = float(robust(np.angle(np.exp(1j * (m.fog - gyaw)))))
    # G: per fix mode, after a translation-only fit on 3D fixes
    k = np.array([g[0] for g in m.gps]); ned = np.array([g[1] for g in m.gps]); mode = np.array([g[2] for g in m.gps])
    gp = G[k, :3, 3]
    off = np.nanmedian((gp - ned)[mode == 3], axis=0)
    e = ned + off - gp
    gps3 = robust(e[mode == 3])
    gps2 = robust(e[mode == 2][:, :2])
    # I: white-noise densities from the first second at rest
    imu = m.imu
    rest = imu.t < imu.t[0] + 1_000_000
    dt = np.median(np.diff(imu.t[rest])) / 1e6
    acc_d = float(np.mean(imu.acc[rest].std(0)) * np.sqrt(dt))
    gyro_d = float(np.mean(imu.gyro[rest].std(0)) * np.sqrt(dt))
    noise = subsets.Noise(lidar, wheel, fog, gps3, gps2, acc_d, gyro_d)
    lines += [f"L per-frame sigmas (rx ry rz rad, x y z m): {np.array2string(lidar, precision=5)}",
              f"W per-frame sigmas (x m, y m, yaw rad): {np.array2string(wheel, precision=5)}; "
              f"median x residual {np.median(wr[:, 0]):.4f} m (scale bias kept)",
              f"F per-frame yaw sigma: {fog:.6f} rad",
              f"G 3D-fix sigmas (N E D m): {np.array2string(gps3, precision=3)} ({(mode == 3).sum()} fixes)",
              f"G 2D-fix sigmas (N E m): {np.array2string(gps2, precision=3)} ({(mode == 2).sum()} fixes)",
              f"I densities: acc {acc_d:.5f} m/s^2/sqrt(Hz), gyro {gyro_d:.6f} rad/s/sqrt(Hz) (at rest, dt {dt:.3f} s)"]
    return noise, lines


def run_one(args):
    subset, r = args
    g, init, keys = subsets.build(subset, M, NOISE, ODOM[r] if "L" in subset else None)
    E, iters, converged, solver = subsets.solve(g, init, keys)
    finite = bool(np.isfinite(E).all())
    ate, rpe, kitti, _ = metrics.compute(GT_POSES, E) if finite else (np.nan,) * 4
    return dict(subset=subset, run=r + 1, finite=finite, converged=converged, iterations=iters, solver=solver,
                ate=ate, rpe100=rpe, kitti=kitti)


def init_worker(m, noise, odom, G):
    global M, NOISE, ODOM, GT_POSES
    M, NOISE, ODOM, GT_POSES = m, noise, odom, G


if __name__ == "__main__":
    t, m, odom, G, gps, R_LI = setup()
    noise, lines = estimate_noise(t, m, odom, G, gps)
    print("\n".join(lines))
    open(os.path.join(OUT, f"noise_{SEQ}.txt"), "w").write("\n".join(lines) + "\n")
    if "--noise-only" in sys.argv:
        sys.exit()
    jobs = int(sys.argv[-1]) if sys.argv[-1].isdigit() else 6
    combos = ["".join(c) for n in range(1, 6) for c in itertools.combinations(subsets.SENSORS, n)]
    tasks = [(s, r) for s in combos for r in ((0, 1, 2) if "L" in s else (0,))]
    with Pool(jobs, init_worker, (m, noise, odom, G)) as pool:
        rows = []
        for row in pool.imap_unordered(run_one, tasks):
            rows.append(row)
            print(f"{row['subset']:6s} r{row['run']} finite={row['finite']} conv={row['converged']} "
                  f"it={row['iterations']} ATE {row['ate']:.3f} "
                  f"KITTI {row['kitti']:.3f}", flush=True)
    for row in [r for r in rows if "L" not in r["subset"]]:      # inputs without L do not depend on the run
        rows += [dict(row, run=k) for k in (2, 3)]
    rows.sort(key=lambda r: (len(r["subset"]), r["subset"], r["run"]))
    with open(os.path.join(OUT, f"subsets_smoke_{SEQ}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0]))
        w.writeheader()
        w.writerows({k: (f"{v:.6f}" if isinstance(v, float) else v) for k, v in r.items()} for r in rows)
