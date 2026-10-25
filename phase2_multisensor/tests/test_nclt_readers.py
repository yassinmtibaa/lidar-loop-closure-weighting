"""Reader and factor checks on the smoke session 2013-01-10 (development only; not part of the split)."""
import os

import gtsam
import numpy as np
import pytest

from fusion import nclt, sensors

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
S = os.path.join(ROOT, "data", "nclt", "2013-01-10")
GT = os.path.join(ROOT, "data", "nclt", "ground_truth", "groundtruth_2013-01-10.csv")
pytestmark = pytest.mark.skipif(not os.path.isdir(S), reason="smoke session not on disk")


@pytest.fixture(scope="module")
def data():
    return dict(w=nclt.read_wheels(S), f=nclt.read_fog(S), imu=nclt.read_imu(S), gps=nclt.read_gps(S),
                gt=nclt.read_ground_truth(GT))


def _rate(t):
    return 1e6 / np.median(np.diff(t))


def test_shapes_and_rates(data):
    w, f, imu, gps = data["w"], data["f"], data["imu"], data["gps"]
    assert len(w.t) == 42276 and 35 < _rate(w.t) < 140
    assert len(f.t) == 97007 and 90 < _rate(f.t) < 110
    assert len(imu.t) == 48324 and 45 < _rate(imu.t) < 55
    assert imu.acc.shape == imu.gyro.shape == imu.mag.shape == (48324, 3)
    assert set(np.unique(gps.mode)) == {2, 3}


def test_same_clock(data):
    """All sensors cover the same ~1026 s on one microsecond clock, as do the LiDAR frames."""
    starts = [data[k].t[0] for k in ("w", "f", "imu", "gps")]
    ends = [data[k].t[-1] for k in ("w", "f", "imu", "gps")]
    assert max(starts) - min(starts) < 1e6 and max(ends) - min(ends) < 1e6
    lt = nclt.lidar_timestamps(S, GT)
    assert len(lt) == 5104                     # KISS-ICP smoke run had 5104 scans (DECISIONS 2026-10-10)
    assert data["gt"][0][0] < lt[0] and lt[-1] < data["gt"][0][-1]


def test_wheel_speeds_plausible(data):
    w = data["w"]
    assert np.abs(w.v_left).max() < 3 and np.abs(w.v_right).max() < 3
    first = w.t < w.t[0] + 1_000_000
    assert np.abs(w.v_left[first]).max() < 0.05 and np.abs(w.v_right[first]).max() < 0.05   # starts at rest


def test_fog_matches_ground_truth_yaw(data):
    """After the reader's sign flip, FOG yaw increments follow ground-truth yaw (z-down)."""
    gt_t, _, e = data["gt"]
    gy = np.unwrap(e[:, 2])[::100]
    fy = sensors.heading_at(data["f"], gt_t[::100])
    assert np.corrcoef(np.diff(gy), np.diff(fy))[0, 1] > 0.99
    assert abs((fy[-1] - fy[0]) - (gy[-1] - gy[0])) < 0.2           # total heading change, rad
    assert np.abs(np.diff(data["f"].heading)).max() < 0.1            # unwrapped: no 2*pi jumps


def test_track_width(data):
    assert abs(sensors.fit_track_width(data["w"], data["f"]) - 0.6125) < 1e-3


def test_wheel_fog_between_matches_ground_truth(data):
    """10 s planar increments from wheels + FOG vs ground truth: heading error small, distance within 15%."""
    gt_t, p, e = data["gt"]
    errs_yaw, ratios = [], []
    for t0 in np.arange(gt_t[0] + 60e6, gt_t[-1] - 20e6, 60e6).astype(np.int64):
        t1 = t0 + 10_000_000
        z = sensors.wheel_fog_between(data["w"], data["f"], t0, t1)
        i, j = np.searchsorted(gt_t, [t0, t1])
        d_gt = np.linalg.norm(p[j, :2] - p[i, :2])
        yaw_gt = np.angle(np.exp(1j * (e[j, 2] - e[i, 2])))
        errs_yaw.append(abs(np.arctan2(z[1, 0], z[0, 0]) - yaw_gt))
        if d_gt > 5:
            ratios.append(np.linalg.norm(z[:2, 3]) / d_gt)
    assert np.median(errs_yaw) < 0.03                # rad over 10 s (median seen: 0.014)
    assert 0.95 < np.median(ratios) < 1.15          # wheels read ~8% long on this session (DECISIONS)


def test_imu_at_rest_preintegrates_to_zero_velocity(data):
    imu = data["imu"]
    t0 = imu.t[0]
    t1 = t0 + 1_000_000
    rest = (imu.t >= t0) & (imu.t < t1)
    f = imu.acc[rest].mean(0)
    assert abs(np.linalg.norm(f) - 9.81) < 0.05
    roll, pitch = np.arctan2(-f[1], -f[2]), np.arctan2(f[0], np.hypot(f[1], f[2]))
    bias = gtsam.imuBias.ConstantBias(np.zeros(3), imu.gyro[rest].mean(0))
    pim = sensors.preintegrate(imu, t0, t1, sensors.imu_params(), bias)
    state = pim.predict(gtsam.NavState(gtsam.Rot3.Ypr(0.0, pitch, roll), np.zeros(3), np.zeros(3)), bias)
    assert np.linalg.norm(state.velocity()) < 0.05
    assert np.linalg.norm(state.position()) < 0.03


def test_gps_local_frame_matches_ground_truth_axes(data):
    """GPS north/east vs ground truth x/y: 3D fixes ~5 m RMS after a translation-only fit, axes aligned."""
    g, (gt_t, p, _) = data["gps"], data["gt"]
    k = int(np.argmax(np.isfinite(g.alt)))
    ned = nclt.geodetic_to_ned(g.lat, g.lon, g.alt, (g.lat[k], g.lon[k], g.alt[k]))
    m = (g.t > gt_t[0]) & (g.t < gt_t[-1]) & (g.mode == 3)
    a = ned[m, :2]
    b = np.c_[np.interp(g.t[m], gt_t, p[:, 0]), np.interp(g.t[m], gt_t, p[:, 1])]
    U, _, Vt = np.linalg.svd((a - a.mean(0)).T @ (b - b.mean(0)))
    R = Vt.T @ U.T
    assert abs(np.degrees(np.arctan2(R[1, 0], R[0, 0]))) < 1.0
    r = np.linalg.norm(a + (b - a).mean(0) - b, axis=1)
    assert np.sqrt((r ** 2).mean()) < 8
    assert np.isnan(nclt.geodetic_to_ned(g.lat, g.lon, g.alt, (g.lat[k], g.lon[k], g.alt[k]))[g.mode == 2, 2]).all()


def test_factors_build(data):
    w, f, gps = data["w"], data["f"], data["gps"]
    t0, t1 = int(w.t[1000]), int(w.t[1400])
    fw = sensors.wheel_fog_factor(0, 1, w, f, t0, t1)
    ff = sensors.fog_factor(0, 1, f, t0, t1)
    fg = sensors.gps_factor(0, [1.0, 2.0, 3.0])
    v = gtsam.Values()
    v.insert(0, gtsam.Pose3())
    v.insert(1, gtsam.Pose3(sensors.wheel_fog_between(w, f, t0, t1)))
    assert fw.error(v) < 1e-9
    assert ff.error(v) < 1e-4          # only the loose translation (sigma 1000 m) contributes
    assert fg.error(v) > 0


def test_wheel_increment_chains_to_dead_reckoning(data):
    """Per-interval wheel+FOG increments composed over 60 s match one dead-reckoning pass over the same span."""
    w, f = data["w"], data["f"]
    t0 = int(w.t[0]) + 100_000_000
    grid = np.arange(t0, t0 + 60_000_000, 100_000)
    T = np.eye(4)
    for a, b in zip(grid[:-1], grid[1:]):
        T = T @ sensors.planar_pose(*sensors.wheel_increment(w, a, b, f))
    z = sensors.planar_pose(*sensors.wheel_increment(w, grid[0], grid[-1], f))
    assert np.linalg.norm(T[:2, 3] - z[:2, 3]) < 0.02 * max(1.0, np.linalg.norm(z[:2, 3]))
