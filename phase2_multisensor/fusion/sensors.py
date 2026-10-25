"""Turn NCLT sensor readings into relative measurements and GTSAM factors (body frame x fwd, y right, z down).
Noise values here are placeholders until Phase 2.3 measures them; every factor carries a source tag so the
backbone can switch it on or off (PoseGraph.add_factor(factor, source))."""
import gtsam
import numpy as np


def heading_at(fog, t):
    """FOG heading (unwrapped, rad) interpolated at times t (us)."""
    return np.interp(np.asarray(t, float), fog.t.astype(float), fog.heading)


def planar_pose(x, y, yaw):
    """4x4 pose for a planar motion in a z-down frame (yaw about +z, positive turning right)."""
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:2, 3] = x, y
    return T


def dead_reckon(wheels, fog=None, track_width=None, t0=None, t1=None):
    """2D dead reckoning at wheel timestamps between t0 and t1 (us). Speed = mean of both wheels; heading from the
    FOG if given, else integrated from the wheel speed difference: d(yaw)/dt = (v_left - v_right) / track_width.
    Returns t, x, y, yaw (yaw relative to the start)."""
    m = np.ones(len(wheels.t), bool)
    if t0 is not None:
        m &= wheels.t >= t0
    if t1 is not None:
        m &= wheels.t <= t1
    t, vl, vr = wheels.t[m], wheels.v_left[m], wheels.v_right[m]
    dt = np.diff(t) / 1e6
    v = 0.5 * (vl + vr)
    v_mid = 0.5 * (v[1:] + v[:-1])
    if fog is not None:
        yaw = heading_at(fog, t)
        yaw = yaw - yaw[0]
    else:
        w = (vl - vr) / track_width
        yaw = np.r_[0.0, np.cumsum(0.5 * (w[1:] + w[:-1]) * dt)]
    yaw_mid = 0.5 * (yaw[1:] + yaw[:-1])
    x = np.r_[0.0, np.cumsum(v_mid * np.cos(yaw_mid) * dt)]
    y = np.r_[0.0, np.cumsum(v_mid * np.sin(yaw_mid) * dt)]
    return t, x, y, yaw


def fit_track_width(wheels, fog, min_speed=0.2):
    """Least-squares track width b from FOG yaw rate = (v_left - v_right) / b, using samples while moving."""
    t = wheels.t.astype(float)
    yaw = heading_at(fog, wheels.t)
    rate = np.gradient(yaw, t / 1e6)
    diff = wheels.v_left - wheels.v_right
    moving = 0.5 * np.abs(wheels.v_left + wheels.v_right) > min_speed
    k = (diff[moving] @ rate[moving]) / (diff[moving] @ diff[moving])   # rate = k * diff, k = 1/b
    return 1.0 / k


def wheel_fog_between(wheels, fog, t0, t1):
    """Relative planar motion from t0 to t1 (us): wheel speed integrated along FOG heading. inv(T(t0)) @ T(t1)."""
    t, x, y, yaw = dead_reckon(wheels, fog, t0=t0, t1=t1)
    return planar_pose(x[-1], y[-1], yaw[-1]) if len(t) > 1 else np.eye(4)


def fog_yaw_between(fog, t0, t1):
    """Relative rotation about z from t0 to t1, as a 4x4 (no translation)."""
    a, b = heading_at(fog, [t0, t1])
    return planar_pose(0.0, 0.0, b - a)


def between_noise(sigma_rot, sigma_trans):
    """Diagonal noise in GTSAM Pose3 order (roll, pitch, yaw, x, y, z); use large sigmas for unobserved axes."""
    return gtsam.noiseModel.Diagonal.Sigmas(np.r_[sigma_rot, sigma_trans].astype(float))


def wheel_fog_factor(i, j, wheels, fog, t_i, t_j, sigma_xy=0.05, sigma_yaw=0.002, loose=1e3):
    """Planar between factor: x, y, yaw observed; z, roll, pitch left free (loose)."""
    z = wheel_fog_between(wheels, fog, t_i, t_j)
    return gtsam.BetweenFactorPose3(i, j, gtsam.Pose3(z),
                                    between_noise([loose, loose, sigma_yaw], [sigma_xy, sigma_xy, loose]))


def fog_factor(i, j, fog, t_i, t_j, sigma_yaw=0.002, loose=1e3):
    """FOG alone: relative yaw only; everything else free."""
    z = fog_yaw_between(fog, t_i, t_j)
    return gtsam.BetweenFactorPose3(i, j, gtsam.Pose3(z), between_noise([loose, loose, sigma_yaw], [loose] * 3))


def gps_factor(key, ned, sigma=(3.0, 3.0, 6.0)):
    """Position factor in the GPS local NED frame (nclt.geodetic_to_ned). The map frame must be tied to this frame
    (an extra alignment variable) before GPS can be fused with LiDAR; that is Phase 2.3."""
    return gtsam.GPSFactor(key, gtsam.Point3(*ned), gtsam.noiseModel.Diagonal.Sigmas(np.asarray(sigma, float)))


def imu_params(acc_sigma=0.05, gyro_sigma=0.005, integration_sigma=1e-4, z_up=False):
    """Preintegration parameters; navigation frame z down (gravity +z) or z up. Noise densities are placeholders."""
    p = gtsam.PreintegrationParams.MakeSharedU(9.81) if z_up else gtsam.PreintegrationParams.MakeSharedD(9.81)
    p.setAccelerometerCovariance(np.eye(3) * acc_sigma ** 2)
    p.setGyroscopeCovariance(np.eye(3) * gyro_sigma ** 2)
    p.setIntegrationCovariance(np.eye(3) * integration_sigma ** 2)
    return p


def preintegrate(imu, t0, t1, params, bias=None, R=None):
    """Preintegrated IMU measurements from t0 to t1 (us), zero-order hold: each piece of [t0, t1) uses the latest
    sample at or before its start. R (3x3) rotates IMU axes into the target body frame (e.g. the LiDAR frame)."""
    pim = gtsam.PreintegratedImuMeasurements(params, bias or gtsam.imuBias.ConstantBias())
    inside = imu.t[(imu.t > t0) & (imu.t < t1)]
    edges = np.r_[t0, inside, t1]
    src = np.searchsorted(imu.t, edges[:-1], side="right") - 1
    for (a, b), k in zip(zip(edges[:-1], edges[1:]), np.maximum(src, 0)):
        acc, gyro = imu.acc[k], imu.gyro[k]
        if R is not None:
            acc, gyro = R @ acc, R @ gyro
        pim.integrateMeasurement(acc, gyro, (b - a) / 1e6)
    return pim


def fit_imu_rotation(imu, frame_t, frame_poses, window=10, min_angle=0.05):
    """Rotation R mapping IMU axes into the frame of frame_poses (e.g. KISS-ICP LiDAR poses at frame_t):
    Kabsch fit of LiDAR rotation vectors over `window` frames vs gyro integration (bias = mean of the first second,
    assumed at rest). Returns R, median residual (rad), number of windows used."""
    from scipy.spatial.transform import Rotation
    bg = imu.gyro[imu.t < imu.t[0] + 1_000_000].mean(0)
    A, B = [], []
    for k in range(0, len(frame_t) - window, window // 2):
        rl = Rotation.from_matrix(frame_poses[k][:3, :3].T @ frame_poses[k + window][:3, :3]).as_rotvec()
        if np.linalg.norm(rl) <= min_angle:
            continue
        idx = np.flatnonzero((imu.t >= frame_t[k]) & (imu.t < frame_t[k + window]))
        Ri = Rotation.identity()
        for n, i in enumerate(idx):
            tn = imu.t[idx[n + 1]] if n + 1 < len(idx) else frame_t[k + window]
            Ri = Ri * Rotation.from_rotvec((imu.gyro[i] - bg) * (tn - imu.t[i]) / 1e6)
        A.append(Ri.as_rotvec())
        B.append(rl)
    A, B = np.array(A), np.array(B)
    U, _, Vt = np.linalg.svd(A.T @ B)
    R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    return R, float(np.median(np.linalg.norm(A @ R.T - B, axis=1))), len(A)
