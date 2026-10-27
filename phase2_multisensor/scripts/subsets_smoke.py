#!/usr/bin/env python3
"""Phase 2.3 on the smoke session (DECISIONS 2026-10-25): estimate per-sensor noise, then run all 31 sensor subsets
on each of the 3 KISS-ICP inputs. usage: .venv/bin/python scripts/subsets_smoke.py [--noise-only] [--v2 | --v3 | --v4 | --v5] [--tilt] [--only=A,B] [jobs]
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


def compose(steps):
    T = np.eye(4)
    for z in steps:
        T = T @ z
    return T


def calibrate_wheels(t, m, G):
    """Per-wheel calibration (DECISIONS 2026-10-27): speed scales s_left, s_right and track width b, least squares on
    per-frame distance and yaw against ground truth (frames moving > 0.2 m). Replaces m.wheel."""
    from scipy.optimize import least_squares
    rel = np.array([np.linalg.inv(G[k]) @ G[k + 1] for k in range(len(G) - 1)])
    gyaw = np.arctan2(rel[:, 1, 0], rel[:, 0, 0])
    d = np.linalg.norm(rel[:, :2, 3], axis=1)
    w = nclt.read_wheels(S)
    wt = w.t.astype(float)
    L, R = np.zeros(len(t) - 1), np.zeros(len(t) - 1)
    for k in range(len(t) - 1):
        tt = np.r_[t[k], w.t[(w.t > t[k]) & (w.t < t[k + 1])], t[k + 1]].astype(float)
        vl, vr, dt = np.interp(tt, wt, w.v_left), np.interp(tt, wt, w.v_right), np.diff(tt) / 1e6
        L[k], R[k] = np.sum(0.5 * (vl[1:] + vl[:-1]) * dt), np.sum(0.5 * (vr[1:] + vr[:-1]) * dt)
    mv = np.r_[d > 0.2, d > 0.2]
    res = lambda p: np.r_[(p[0] * L + p[1] * R) / 2 - d, (p[0] * L - p[1] * R) / p[2] - gyaw][mv]
    sl, sr, b = least_squares(res, [1.0, 1.0, TRACK_WIDTH]).x
    cal = nclt.Wheels(w.t, w.v_left * sl, w.v_right * sr)
    m.wheel = np.array([sensors.wheel_increment(cal, t[k], t[k + 1], None, b) for k in range(len(t) - 1)])
    return sl, sr, b


def estimate_noise_v2(t, m, odom, G, gps, window=50, wheel_cal=False):
    """Noise model v2 (DECISIONS 2026-10-25): wheel scale, then drift-equivalent sigmas for L, W, F
    (robust sigma of residuals over non-overlapping `window`-frame spans, divided by sqrt(window))."""
    base, _ = estimate_noise(t, m, odom, G, gps)
    rel = np.array([np.linalg.inv(G[k]) @ G[k + 1] for k in range(len(G) - 1)])
    d_gt = np.linalg.norm(rel[:, :2, 3], axis=1)
    d_w = np.hypot(m.wheel[:, 0], m.wheel[:, 1])
    moving = d_gt > 0.2
    if wheel_cal:
        cal = calibrate_wheels(t, m, G)
        scale = None
    else:
        scale = float(np.median(d_gt[moving] / d_w[moving]))
        m.wheel = m.wheel * scale
    spans = [(a, a + window) for a in range(0, len(rel) - window + 1, window)]
    gt_w = [compose(rel[a:b]) for a, b in spans]
    lr = np.array([gtsam.Pose3.Logmap(gtsam.Pose3(np.linalg.inv(g) @ compose(o[a:b])))
                   for o in odom for g, (a, b) in zip(gt_w, spans)])
    lidar = robust(lr) / np.sqrt(window)
    wr = []
    for g, (a, b) in zip(gt_w, spans):
        e = np.linalg.inv(g) @ compose([subsets.planar(*w) for w in m.wheel[a:b]])
        wr.append([e[0, 3], e[1, 3], np.arctan2(e[1, 0], e[0, 0])])
    wheel = robust(np.array(wr)) / np.sqrt(window)
    gyaw = np.array([np.arctan2(g[1, 0], g[0, 0]) for g in gt_w])
    fr = np.array([m.fog[a:b].sum() for a, b in spans]) - gyaw
    fog = float(robust(np.angle(np.exp(1j * fr)))) / np.sqrt(window)
    noise = subsets.Noise(lidar, wheel, fog, base.gps3, base.gps2, base.acc_density, base.gyro_density)
    lines = [f"noise model v2: {len(spans)} windows of {window} frames, sigma = robust(window residual)/sqrt({window})",
             (f"wheel scale s = {scale:.4f} (median ground-truth / wheel distance, frames moving > 0.2 m)" if scale
              else "wheel calibration: s_left {:.4f}, s_right {:.4f}, track width {:.4f} m".format(*cal)),
             f"L drift-equivalent per-frame sigmas (rx ry rz rad, x y z m): {np.array2string(lidar, precision=5)}",
             f"W drift-equivalent per-frame sigmas (x m, y m, yaw rad), scaled wheels: {np.array2string(wheel, precision=5)}",
             f"F drift-equivalent per-frame yaw sigma: {fog:.6f} rad",
             f"G and I unchanged: G3 {np.array2string(base.gps3, precision=3)}, G2 {np.array2string(base.gps2, precision=3)}, "
             f"acc {base.acc_density:.5f}, gyro {base.gyro_density:.6f}"]
    return noise, lines


def estimate_noise_v3(t, m, odom, G, gps, bins=5, wheel_cal=False):
    """Noise model v3 (DECISIONS 2026-10-25): v2 plus per-frame wheel sigmas growing with the wheel-derived turn,
    and GPS sigmas inflated for correlated fixes."""
    v2, lines = estimate_noise_v2(t, m, odom, G, gps, wheel_cal=wheel_cal)   # also scales/calibrates m.wheel
    rel = np.array([np.linalg.inv(G[k]) @ G[k + 1] for k in range(len(G) - 1)])
    gyaw = np.arctan2(rel[:, 1, 0], rel[:, 0, 0])
    r = m.wheel - np.c_[rel[:, 0, 3], rel[:, 1, 3], gyaw]
    r[:, 2] = np.angle(np.exp(1j * r[:, 2]))
    u = np.abs(m.wheel[:, 2])
    edges = np.quantile(u, np.linspace(0, 1, bins + 1))
    which = np.clip(np.searchsorted(edges, u, side="right") - 1, 0, bins - 1)
    centers = np.array([np.median(u[which == b]) for b in range(bins)])
    sig = np.array([robust(r[which == b]) for b in range(bins)])          # (bins, 3)
    per_frame = np.empty_like(r)
    fits = []
    for c in range(3):
        b_, a_ = np.polyfit(centers, sig[:, c], 1)
        raw = np.maximum(a_ + b_ * u, 1e-6)
        per_frame[:, c] = raw * v2.wheel[c] / np.median(raw)          # median frame keeps its v2 sigma
        fits.append((a_, b_))
    # GPS residual autocorrelation (3D fixes, horizontal)
    k = np.array([g[0] for g in m.gps]); ned = np.array([g[1] for g in m.gps]); mode = np.array([g[2] for g in m.gps])
    sel = mode == 3
    e = ned[sel, :2] - G[k[sel], :2, 3]
    e = e - np.median(e, axis=0)
    tt = t[k[sel]] / 1e6
    rate = (len(tt) - 1) / (tt[-1] - tt[0])
    tau = None
    for lag in np.arange(1.0, 600.0, 1.0):
        i = np.searchsorted(tt, tt + lag)
        ok = i < len(tt)
        ac = np.mean([np.corrcoef(e[ok, d], e[i[ok], d])[0, 1] for d in (0, 1)])
        if ac < 1 / np.e:
            tau = lag
            break
    tau = tau or 600.0
    infl = float(np.sqrt(max(1.0, tau * rate)))
    v3 = subsets.Noise(v2.lidar, v2.wheel, v2.fog_yaw, v2.gps3, v2.gps2, v2.acc_density, v2.gyro_density,
                       wheel_per_frame=per_frame, gps_inflation=infl)
    lines += ["noise model v3:",
              "W sigma(u) line fits per component (a, b) on per-frame residuals vs |wheel yaw|: "
              + ", ".join(f"({a_:.4g}, {b_:.4g})" for a_, b_ in fits),
              f"W bins (|wheel yaw| median rad): {np.array2string(centers, precision=4)}; per-bin sigmas x/y/yaw:\n"
              + np.array2string(sig, precision=4),
              f"W per-frame sigma ranges: x {per_frame[:, 0].min():.4f}-{per_frame[:, 0].max():.4f} m, "
              f"yaw {per_frame[:, 2].min():.4f}-{per_frame[:, 2].max():.4f} rad",
              f"G residual correlation time tau = {tau:.0f} s at {rate:.2f} fixes/s -> sigma inflation x{infl:.2f}"]
    return v3, lines


def estimate_noise_v4(t, m, odom, G, gps, wheel_cal=False):
    """Noise model v4 (DECISIONS 2026-10-26): v3 with IMU densities measured while moving (1 s high-pass).
    wheel_cal=True gives v5 (per-wheel calibration instead of one distance scale, DECISIONS 2026-10-27)."""
    v3, lines = estimate_noise_v3(t, m, odom, G, gps, wheel_cal=wheel_cal)
    imu, w = m.imu, nclt.read_wheels(S)
    dt = np.median(np.diff(imu.t)) / 1e6
    v = 0.5 * (np.interp(imu.t, w.t, w.v_left) + np.interp(imu.t, w.t, w.v_right))
    mov = (np.abs(v) > 0.5)[100:-100]
    k = np.ones(50) / 50

    def density(x):
        hp = np.stack([x[:, i] - np.convolve(x[:, i], k, "same") for i in range(3)], 1)[100:-100]
        return float(hp[mov].std(0).mean() * np.sqrt(dt))

    v3.acc_density, v3.gyro_density = density(imu.acc), density(imu.gyro)
    lines += ["noise model v4:", f"IMU densities while moving (1 s high-pass, speed > 0.5 m/s): "
              f"acc {v3.acc_density:.5f} m/s^2/sqrt(Hz), gyro {v3.gyro_density:.5f} rad/s/sqrt(Hz)"]
    return v3, lines


def run_one(args):
    subset, r = args
    g, init, keys = subsets.build(subset, M, NOISE, ODOM[r] if "L" in subset else None, leveled_wheels=LEVELED)
    E, iters, converged, solver = subsets.solve(g, init, keys)
    finite = bool(np.isfinite(E).all())
    ate, rpe, kitti, _ = metrics.compute(GT_POSES, E) if finite else (np.nan,) * 4
    return dict(subset=subset, run=r + 1, finite=finite, converged=converged, iterations=iters, solver=solver,
                ate=ate, rpe100=rpe, kitti=kitti)


def init_worker(m, noise, odom, G, leveled=False):
    global M, NOISE, ODOM, GT_POSES, LEVELED
    M, NOISE, ODOM, GT_POSES, LEVELED = m, noise, odom, G, leveled


if __name__ == "__main__":
    t, m, odom, G, gps, R_LI = setup()
    model = ("v5" if "--v5" in sys.argv else "v4" if "--v4" in sys.argv else "v3" if "--v3" in sys.argv else "v2" if "--v2" in sys.argv
             else "v1")
    leveled = "--tilt" in sys.argv          # leveled wheel factor, W subsets only
    suffix = ("" if model == "v1" else f"_{model}") + ("tilt" if leveled else "")
    noise, lines = {"v1": estimate_noise, "v2": estimate_noise_v2, "v3": estimate_noise_v3,
                     "v4": estimate_noise_v4,
                     "v5": lambda *a: estimate_noise_v4(*a, wheel_cal=True)}[model](t, m, odom, G, gps)
    print("\n".join(lines))
    open(os.path.join(OUT, f"noise_{SEQ}{suffix}.txt"), "w").write("\n".join(lines) + "\n")
    if "--noise-only" in sys.argv:
        sys.exit()
    jobs = int(sys.argv[-1]) if sys.argv[-1].isdigit() else 6
    combos = ["".join(c) for n in range(1, 6) for c in itertools.combinations(subsets.SENSORS, n)]
    if leveled:
        combos = [c for c in combos if "W" in c]
    only = [a.split("=", 1)[1].split(",") for a in sys.argv if a.startswith("--only=")]
    if only:                                # rerun some subsets; results go to a *_part CSV
        combos = [c for c in combos if c in only[0]]
        suffix += "_part"
    tasks = [(s, r) for s in combos for r in ((0, 1, 2) if "L" in s else (0,))]
    with Pool(jobs, init_worker, (m, noise, odom, G, leveled)) as pool:
        rows = []
        for row in pool.imap_unordered(run_one, tasks):
            rows.append(row)
            print(f"{row['subset']:6s} r{row['run']} finite={row['finite']} conv={row['converged']} "
                  f"it={row['iterations']} ATE {row['ate']:.3f} "
                  f"KITTI {row['kitti']:.3f}", flush=True)
    for row in [r for r in rows if "L" not in r["subset"]]:      # inputs without L do not depend on the run
        rows += [dict(row, run=k) for k in (2, 3)]
    rows.sort(key=lambda r: (len(r["subset"]), r["subset"], r["run"]))
    with open(os.path.join(OUT, f"subsets_smoke_{SEQ}{suffix}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0]))
        w.writeheader()
        w.writerows({k: (f"{v:.6f}" if isinstance(v, float) else v) for k, v in r.items()} for r in rows)
