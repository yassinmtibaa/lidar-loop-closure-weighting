"""Readers for NCLT sensor files (one session directory, e.g. data/nclt/2013-01-10).

Formats (NCLT README / paper, checked on 2013-01-10). All timestamps are microseconds (int64), one shared clock.
  wheels.csv  t, v_left, v_right                         [m/s]                       ~41 Hz
  kvh.csv     t, heading                                 [rad, wrapped to 0..2pi]    ~100 Hz  (fiber-optic gyro, yaw only)
              The raw heading is counterclockwise-positive (z up). read_fog() negates it so it follows the z-down body
              frame and the ground-truth yaw (2013-01-10: correlation of yaw increments -0.999 before negation).
  ms25.csv    t, mag xyz [Gauss], acc xyz [m/s^2], gyro xyz [rad/s]                   ~50 Hz  (Microstrain GX3 IMU)
  gps.csv     t, mode, num_sats, lat [rad], lon [rad], alt [m], track [rad], speed [m/s]   (standard GPS)
Body frame is x forward, y right, z down (the IMU reads acc_z ~ -9.81 at rest); heading grows turning right.
gps_rtk.csv is NOT read here on purpose: RTK feeds the ground truth (DECISIONS 2026-10-10)."""
import os
from dataclasses import dataclass

import numpy as np

# WGS84
_A = 6378137.0
_E2 = 6.69437999014e-3


def _load(session, name):
    a = np.loadtxt(os.path.join(session, name), delimiter=",", ndmin=2)
    t = a[:, 0].astype(np.int64)
    assert (np.diff(t) > 0).all(), f"{name}: timestamps not strictly increasing"
    return t, a[:, 1:]


@dataclass
class Wheels:
    t: np.ndarray        # us
    v_left: np.ndarray   # m/s
    v_right: np.ndarray


@dataclass
class Fog:
    t: np.ndarray
    heading: np.ndarray  # rad, unwrapped, z-down convention (= minus the raw kvh value)


@dataclass
class Imu:
    t: np.ndarray
    mag: np.ndarray      # (N,3) Gauss
    acc: np.ndarray      # (N,3) m/s^2, specific force, body frame
    gyro: np.ndarray     # (N,3) rad/s


@dataclass
class Gps:
    t: np.ndarray
    mode: np.ndarray     # 2 = 2D fix, 3 = 3D fix (other values: no fix)
    lat: np.ndarray      # rad
    lon: np.ndarray      # rad
    alt: np.ndarray      # m
    track: np.ndarray
    speed: np.ndarray


def read_wheels(session):
    t, a = _load(session, "wheels.csv")
    return Wheels(t, a[:, 0], a[:, 1])


def read_fog(session):
    t, a = _load(session, "kvh.csv")
    return Fog(t, -np.unwrap(a[:, 0]))


def read_imu(session):
    t, a = _load(session, "ms25.csv")
    return Imu(t, a[:, 0:3], a[:, 3:6], a[:, 6:9])


def read_gps(session, min_mode=2):
    t, a = _load(session, "gps.csv")
    ok = (a[:, 0] >= min_mode) & np.isfinite(a[:, 2:4]).all(axis=1)    # 2D fixes (mode 2) have alt = NaN
    a, t = a[ok], t[ok]
    return Gps(t, a[:, 0].astype(int), a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6])


def read_ground_truth(gt_csv):
    """NCLT ground truth: t, x, y, z, roll, pitch, yaw (NED-like local frame). NaN rows dropped."""
    a = np.loadtxt(gt_csv, delimiter=",", ndmin=2)
    a = a[np.isfinite(a).all(axis=1)]
    return a[:, 0].astype(np.int64), a[:, 1:4], a[:, 4:7]


def lidar_timestamps(session, gt_csv):
    """Timestamps of the scans KISS-ICP/KISS-SLAM use, in frame order (same filter as kiss_icp's NCLT loader:
    scans strictly inside the ground-truth time range). Frame k of a KISS run is at element k."""
    files = sorted(os.listdir(os.path.join(session, "velodyne_sync")))
    t = np.array([int(f.split(".")[0]) for f in files], dtype=np.int64)
    gt_t = np.loadtxt(gt_csv, delimiter=",", usecols=0, ndmin=1)
    return t[(t > gt_t.min()) & (t < gt_t.max())]


def geodetic_to_ned(lat, lon, alt, origin):
    """lat/lon in rad, origin = (lat0, lon0, alt0). Local tangent plane: north, east, down [m].
    Where alt is NaN (2D fix), north/east are computed at the origin altitude and down is NaN."""
    lat0, lon0, alt0 = origin

    def ecef(la, lo, h):
        n = _A / np.sqrt(1 - _E2 * np.sin(la) ** 2)
        return np.stack([(n + h) * np.cos(la) * np.cos(lo), (n + h) * np.cos(la) * np.sin(lo),
                         (n * (1 - _E2) + h) * np.sin(la)], axis=-1)

    alt = np.asarray(alt, float)
    known = np.isfinite(alt)
    d = ecef(np.asarray(lat), np.asarray(lon), np.where(known, alt, alt0)) - ecef(lat0, lon0, alt0)
    sl, cl, so, co = np.sin(lat0), np.cos(lat0), np.sin(lon0), np.cos(lon0)
    R = np.array([[-sl * co, -sl * so, cl], [-so, co, 0.0], [-cl * co, -cl * so, -sl]])
    ned = d @ R.T
    ned[..., 2] = np.where(known, ned[..., 2], np.nan)   # 2D fix: north/east valid, down unknown
    return ned
