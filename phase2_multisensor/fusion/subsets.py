"""Per-frame fusion graph for any subset of sensors L (LiDAR odometry), I (IMU), W (wheels), F (FOG), G (GPS).
Body frame: NCLT body (x fwd, y right, z down); navigation frame: NED. See DECISIONS 2026-10-25 (Phase 2.3)."""
from dataclasses import dataclass

import gtsam
import numpy as np
from gtsam.symbol_shorthand import B, V, X
from scipy.spatial.transform import Rotation

from fusion import sensors

LOOSE = 1e3
REG_SIGMA = (1.0, 10.0)            # rad, m per frame: weak smoothness, always on
BIAS_PRIOR = (0.1, 0.01)           # acc m/s^2, gyro rad/s
BIAS_WALK = (1e-3, 1e-4)           # per frame
SENSORS = "LIWFG"


@dataclass
class Noise:
    lidar: np.ndarray        # 6 sigmas, GTSAM order (rx, ry, rz, x, y, z)
    wheel: np.ndarray        # (x, y, yaw)
    fog_yaw: float
    gps3: np.ndarray         # (N, E, D) for 3D fixes
    gps2: np.ndarray         # (N, E) for 2D fixes
    acc_density: float
    gyro_density: float


def planar(dx, dy, dyaw):
    return sensors.planar_pose(dx, dy, dyaw)


def gt_body_poses(gt, t):
    """Ground-truth body poses (NED) at times t: positions interpolated, attitude from the nearest row."""
    gt_t, p, e = gt
    pos = np.stack([np.interp(t, gt_t, p[:, i]) for i in range(3)], axis=1)
    idx = np.clip(np.searchsorted(gt_t, t), 0, len(gt_t) - 1)
    T = np.tile(np.eye(4), (len(t), 1, 1))
    T[:, :3, :3] = Rotation.from_euler("ZYX", e[idx][:, ::-1]).as_matrix()
    T[:, :3, 3] = pos
    return T


class Measurements:
    """Everything per frame that does not depend on the LiDAR input run."""

    def __init__(self, t, wheels, fog, imu, gps, track_width):
        self.t = t
        n = len(t) - 1
        self.wheel = np.array([sensors.wheel_increment(wheels, t[k], t[k + 1], None, track_width) for k in range(n)])
        yaw = sensors.heading_at(fog, t)
        self.fog = np.diff(yaw)
        self.imu = imu
        rest = imu.t < imu.t[0] + 1_000_000
        f = imu.acc[rest].mean(0)
        self.roll, self.pitch = np.arctan2(-f[1], -f[2]), np.arctan2(f[0], np.hypot(f[1], f[2]))
        self.bias0 = gtsam.imuBias.ConstantBias(np.zeros(3), imu.gyro[rest].mean(0))
        k3 = int(np.argmax(np.isfinite(gps.alt) & (gps.mode == 3)))
        ned = sensors_ned(gps, k3)
        node = np.searchsorted(t, gps.t)
        node = np.clip(node, 1, len(t) - 1)
        node = np.where(np.abs(t[node - 1] - gps.t) < np.abs(t[node] - gps.t), node - 1, node)
        ok = np.abs(t[node] - gps.t) < 50_000
        self.gps = [(int(k), ned[i], int(gps.mode[i])) for i, k in enumerate(node) if ok[i]]


def sensors_ned(gps, k_origin):
    from fusion import nclt
    return nclt.geodetic_to_ned(gps.lat, gps.lon, gps.alt, (gps.lat[k_origin], gps.lon[k_origin], gps.alt[k_origin]))


def lidar_body_odometry(P, R_LI):
    """Per-frame KISS-ICP motion expressed in the body (IMU) frame: E^-1 z E with E = [R_LI, 0]."""
    E = np.eye(4)
    E[:3, :3] = R_LI
    Ei = np.linalg.inv(E)
    return np.array([Ei @ np.linalg.inv(P[k]) @ P[k + 1] @ E for k in range(len(P) - 1)])


def _sig(rot, trans):
    return gtsam.noiseModel.Diagonal.Sigmas(np.r_[rot, trans].astype(float))


def _kabsch2d(a, b):
    ma, mb = a.mean(0), b.mean(0)
    U, _, Vt = np.linalg.svd((a - ma).T @ (b - mb))
    R = Vt.T @ np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    return R, mb - R @ ma


def _local_step(factors):
    """Relative pose minimizing the given (measurement, noise) between factors for one step."""
    if len(factors) == 1:
        return factors[0][0].matrix()
    g = gtsam.NonlinearFactorGraph()
    for z, nm in factors:
        g.add(gtsam.BetweenFactorPose3(0, 1, z, nm))
    g.add(gtsam.NonlinearEqualityPose3(0, gtsam.Pose3()))
    v = gtsam.Values()
    v.insert(0, gtsam.Pose3())
    v.insert(1, factors[0][0])
    return gtsam.LevenbergMarquardtOptimizer(g, v, gtsam.LevenbergMarquardtParams()).optimize().atPose3(1).matrix()


def build(subset, m, noise, lidar_odom=None):
    """Returns (graph, initial values, node keys). subset: string of letters from 'LIWFG'."""
    n = len(m.t)
    graph, init = gtsam.NonlinearFactorGraph(), gtsam.Values()
    # initial guess: each step is the local optimum of that step's relative factors (L, W, F) alone, so the
    # chain starts from the weighted combination instead of one sensor's (LM from a one-sensor chain did not
    # converge in 50 iterations on some subsets, DECISIONS 2026-10-25)
    rel = []
    if "L" in subset:
        rel.append(([gtsam.Pose3(z) for z in lidar_odom], gtsam.noiseModel.Diagonal.Sigmas(noise.lidar)))
    if "W" in subset:
        rel.append(([gtsam.Pose3(planar(*w)) for w in m.wheel],
                    _sig([LOOSE, LOOSE, noise.wheel[2]], [noise.wheel[0], noise.wheel[1], LOOSE])))
    if "F" in subset:
        rel.append(([gtsam.Pose3(planar(0, 0, d)) for d in m.fog], _sig([LOOSE, LOOSE, noise.fog_yaw], [LOOSE] * 3)))
    steps = [_local_step([(zs[k], nm) for zs, nm in rel]) for k in range(n - 1)] if rel else [np.eye(4)] * (n - 1)
    T0 = gtsam.Pose3(gtsam.Rot3.Ypr(0.0, m.pitch, m.roll) if "I" in subset else gtsam.Rot3(), np.zeros(3)).matrix()
    traj = [T0]
    for s in steps:
        traj.append(traj[-1] @ s)
    traj = np.array(traj)
    if "G" in subset:
        k = np.array([g[0] for g in m.gps])
        ne = np.array([g[1][:2] for g in m.gps])
        if "L" in subset or "W" in subset:
            R2, t2 = _kabsch2d(traj[k, :2, 3], ne)
            Rz = np.eye(4)
            Rz[:2, :2] = R2
            Rz[:2, 3] = t2
            traj = Rz @ traj
        else:
            for i in range(2):
                traj[:, i, 3] = np.interp(np.arange(n), k, ne[:, i])
    for k in range(n):
        init.insert(X(k), gtsam.Pose3(traj[k]))
    # prior on the first node
    rp = 0.01 if "I" in subset else 1e-6
    if "G" in subset:
        graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(traj[0]), _sig([rp, rp, LOOSE], [LOOSE] * 3)))
    else:
        graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(T0), _sig([rp, rp, 1e-6], [1e-6] * 3)))
    reg = _sig([REG_SIGMA[0]] * 3, [REG_SIGMA[1]] * 3)
    for k in range(n - 1):
        graph.add(gtsam.BetweenFactorPose3(X(k), X(k + 1), gtsam.Pose3(), reg))
    if "L" in subset:
        nl = gtsam.noiseModel.Diagonal.Sigmas(noise.lidar)
        for k in range(n - 1):
            graph.add(gtsam.BetweenFactorPose3(X(k), X(k + 1), gtsam.Pose3(lidar_odom[k]), nl))
    if "W" in subset:
        nw = _sig([LOOSE, LOOSE, noise.wheel[2]], [noise.wheel[0], noise.wheel[1], LOOSE])
        for k in range(n - 1):
            graph.add(gtsam.BetweenFactorPose3(X(k), X(k + 1), gtsam.Pose3(planar(*m.wheel[k])), nw))
    if "F" in subset:
        nf = _sig([LOOSE, LOOSE, noise.fog_yaw], [LOOSE] * 3)
        for k in range(n - 1):
            graph.add(gtsam.BetweenFactorPose3(X(k), X(k + 1), gtsam.Pose3(planar(0, 0, m.fog[k])), nf))
    if "G" in subset:
        n3 = gtsam.noiseModel.Diagonal.Sigmas(noise.gps3)
        n2 = gtsam.noiseModel.Diagonal.Sigmas(np.r_[noise.gps2, LOOSE])
        for k, ned, mode in m.gps:
            p = ned if mode == 3 else np.r_[ned[:2], traj[k, 2, 3]]
            graph.add(gtsam.GPSFactor(X(k), gtsam.Point3(*p), n3 if mode == 3 else n2))
    if "I" in subset:
        params = sensors.imu_params(noise.acc_density, noise.gyro_density)
        walk = _sig([BIAS_WALK[0]] * 3, [BIAS_WALK[1]] * 3)
        graph.add(gtsam.PriorFactorVector(V(0), np.zeros(3), gtsam.noiseModel.Isotropic.Sigma(3, 0.01)))
        graph.add(gtsam.PriorFactorConstantBias(B(0), m.bias0, _sig([BIAS_PRIOR[0]] * 3, [BIAS_PRIOR[1]] * 3)))
        for k in range(n):
            dt = (m.t[k] - m.t[k - 1]) / 1e6 if k else 1.0
            init.insert(V(k), np.zeros(3) if k == 0 else (traj[k, :3, 3] - traj[k - 1, :3, 3]) / dt)
            init.insert(B(k), m.bias0)
        for k in range(n - 1):
            pim = sensors.preintegrate(m.imu, m.t[k], m.t[k + 1], params, m.bias0)
            graph.add(gtsam.ImuFactor(X(k), V(k), X(k + 1), V(k + 1), B(k), pim))
            graph.add(gtsam.BetweenFactorConstantBias(B(k), B(k + 1), gtsam.imuBias.ConstantBias(), walk))
    return graph, init, [X(k) for k in range(n)]


def solve(graph, init, keys, max_iterations=500):
    """LM; returns (poses, iterations, converged)."""
    p = gtsam.LevenbergMarquardtParams()
    p.setMaxIterations(max_iterations)
    p.setRelativeErrorTol(1e-8)
    opt = gtsam.LevenbergMarquardtOptimizer(graph, init, p)
    est = opt.optimize()
    return np.array([est.atPose3(k).matrix() for k in keys]), opt.iterations(), opt.iterations() < max_iterations
