#!/usr/bin/env python3
"""Run the sensor-subset sweep (noise model v5) on any NCLT session. For the supercomputer (Phase 2.4).

Fit the noise model on a session (uses its ground truth; tune sessions only) and save it:
  .venv/bin/python scripts/subsets_run.py --seq 2012-04-29 --runs icp_r1,icp_r2,icp_r3 --fit-noise results/noise_v5_2012-04-29.json
Run all 31 subsets with a frozen noise model (no ground truth used except for scoring):
  .venv/bin/python scripts/subsets_run.py --seq 2012-04-29 --runs icp_r1,icp_r2,icp_r3 --noise results/noise_v5_2012-04-29.json --jobs 32
--runs: KISS-ICP run names; poses are read from results/<name>_seq<seq>/latest/<seq>_poses.npy.
--weights: predicted (noise model v5 as fitted), fixed (one sigma per sensor: no turn-dependent wheel sigma, no GPS
correlation inflation) or identity (unit sigma on every observed axis of L, W, F, G; IMU keeps its densities, it has
no identity analogue). Wheel calibration is applied in all three (it corrects the measurement, not its weight).
--nees N: position NEES every N frames (0 = off).
Writes results/subsets_<seq>_<tag>.csv (tag from the noise fit and weighting) and a .log, one line per finished run."""
import argparse
import csv
import itertools
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [ROOT, os.path.join(ROOT, "scripts")]
import subsets_smoke as ss  # noqa: E402
from fusion import subsets  # noqa: E402


def configure(seq, runs):
    ss.SEQ = seq
    ss.S = os.path.join(ROOT, "data", "nclt", seq)
    ss.GT = os.path.join(ROOT, "data", "nclt", "ground_truth", f"groundtruth_{seq}.csv")
    ss.RUNS = [os.path.join(ROOT, "results", f"{r}_seq{seq}", "latest", f"{seq}_poses.npy") for r in runs]


def save_noise(noise, path, source_seq):
    d = dict(model="v5", fitted_on=source_seq, lidar=noise.lidar.tolist(), wheel=noise.wheel.tolist(),
             fog_yaw=noise.fog_yaw, gps3=noise.gps3.tolist(), gps2=noise.gps2.tolist(),
             acc_density=noise.acc_density, gyro_density=noise.gyro_density, gps_inflation=noise.gps_inflation,
             wheel_cal=noise.wheel_cal, wheel_line=noise.wheel_line)
    json.dump(d, open(path, "w"), indent=1)


def apply_weights(noise, weights):
    if weights == "fixed":
        noise.wheel_per_frame, noise.gps_inflation = None, 1.0
    elif weights == "identity":
        noise.lidar, noise.wheel, noise.fog_yaw = np.ones(6), np.ones(3), 1.0
        noise.gps3, noise.gps2, noise.wheel_per_frame, noise.gps_inflation = np.ones(3), np.ones(2), None, 1.0
    return noise


def load_noise(path, t, m):
    """Frozen noise for a new session: applies the stored wheel calibration and per-frame wheel sigma line.
    path may list several JSON files separated by commas: their parameters are averaged (e.g. leave-one-out)."""
    ds = [json.load(open(p)) for p in path.split(",")]
    d = {k: (ds[0][k] if isinstance(ds[0][k], str) else np.mean([x[k] for x in ds], axis=0).tolist())
         for k in ds[0] if k not in ("model", "fitted_on")}
    d["fitted_on"] = "+".join(x["fitted_on"] for x in ds)
    ss.apply_wheel_cal(t, m, d["wheel_cal"])
    u = np.abs(m.wheel[:, 2])
    per_frame = np.stack([np.maximum(a + b * u, 1e-6) for a, b in d["wheel_line"]], axis=1)
    return subsets.Noise(np.array(d["lidar"]), np.array(d["wheel"]), d["fog_yaw"], np.array(d["gps3"]),
                         np.array(d["gps2"]), d["acc_density"], d["gyro_density"], wheel_per_frame=per_frame,
                         gps_inflation=d["gps_inflation"]), d


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", required=True)
    ap.add_argument("--runs", required=True)
    ap.add_argument("--fit-noise")
    ap.add_argument("--noise")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--only")
    ap.add_argument("--weights", choices=["predicted", "fixed", "identity"], default="predicted")
    ap.add_argument("--nees", type=int, default=0)
    a = ap.parse_args()
    configure(a.seq, a.runs.split(","))
    t, m, odom, G, gps, R_LI = ss.setup()
    if a.fit_noise:
        noise, lines = ss.estimate_noise_v4(t, m, odom, G, gps, wheel_cal=True)
        save_noise(noise, a.fit_noise, a.seq)
        print("\n".join(lines))
        sys.exit()
    noise, d = load_noise(a.noise, t, m)
    noise = apply_weights(noise, a.weights)
    tag = "noise_v5_" + d["fitted_on"] + ("" if a.weights == "predicted" else "_" + a.weights)
    combos = ["".join(c) for n in range(1, 6) for c in itertools.combinations(subsets.SENSORS, n)]
    if a.only:
        combos = [c for c in combos if c in a.only.split(",")]
    tasks = [(s, r) for s in combos for r in (range(len(odom)) if "L" in s else (0,))]
    out = os.path.join(ROOT, "results", f"subsets_{a.seq}_{tag}")
    rows = []
    with Pool(a.jobs, ss.init_worker, (m, noise, odom, G, False, a.nees)) as pool, open(out + ".log", "a") as log:
        log.write(f"seq {a.seq} runs {a.runs} noise {a.noise} (fitted on {d['fitted_on']}) weights {a.weights}\n")
        for row in pool.imap_unordered(ss.run_one, tasks):
            rows.append(row)
            log.write(f"{row['subset']:6s} r{row['run']} finite={row['finite']} conv={row['converged']} "
                      f"it={row['iterations']} ATE {row['ate']:.3f} KITTI {row['kitti']:.3f}"
                      + (f" NEES {row['nees_mean']:.2f} in95 {row['nees_in95']:.2f}" if "nees_mean" in row else "")
                      + "\n")
            log.flush()
    for row in [r for r in rows if "L" not in r["subset"]]:
        rows += [dict(row, run=k) for k in range(2, len(odom) + 1)]
    rows.sort(key=lambda r: (len(r["subset"]), r["subset"], r["run"]))
    with open(out + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0]))
        w.writeheader()
        w.writerows({k: (f"{v:.6f}" if isinstance(v, float) else v) for k, v in r.items()} for r in rows)
