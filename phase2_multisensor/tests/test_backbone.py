"""Synthetic checks of the GTSAM backbone (no data needed)."""
import gtsam
import numpy as np
from scipy.spatial.transform import Rotation

from fusion import graph


def _pose(rotvec, t):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    T[:3, 3] = t
    return T


def test_g2o_information_matches_g2o_cost():
    """GTSAM cost with the converted information equals g2o's EdgeSE3 cost (error [t, q_vec]) for small errors."""
    rng = np.random.default_rng(0)
    A = rng.normal(size=(6, 6))
    info = A @ A.T + 6 * np.eye(6)                       # g2o order (t, R)
    for _ in range(20):
        z = _pose(rng.normal(size=3) * 0.3, rng.normal(size=3))
        Ti = _pose(rng.normal(size=3) * 0.3, rng.normal(size=3))
        delta = _pose(rng.normal(size=3) * 1e-3, rng.normal(size=3) * 1e-3)
        Tj = Ti @ z @ delta                               # inv(z) inv(Ti) Tj = delta
        q = Rotation.from_matrix(delta[:3, :3]).as_quat()  # x y z w
        e = np.r_[delta[:3, 3], np.sign(q[3]) * q[:3]]
        cost_g2o = 0.5 * e @ info @ e
        f = gtsam.BetweenFactorPose3(0, 1, gtsam.Pose3(z), graph.noise_from_g2o_info(info))
        v = gtsam.Values()
        v.insert(0, gtsam.Pose3(Ti))
        v.insert(1, gtsam.Pose3(Tj))
        assert abs(f.error(v) - cost_g2o) < 1e-3 * cost_g2o
        # the permute-only mapping overweights rotation (what failed A1 first)
        f_bad = gtsam.BetweenFactorPose3(0, 1, gtsam.Pose3(z), graph.noise_from_g2o_info(info, False))
        assert f_bad.error(v) > cost_g2o


def test_sources_switch_on_and_off():
    """A loop factor changes the answer only when its source is enabled."""
    g = graph.PoseGraph()
    step = _pose([0, 0, 0], [1, 0, 0])
    for k in range(4):
        g.add_variable(k, np.linalg.matrix_power(step, k))
    g.fix(0, np.eye(4))
    unit = gtsam.noiseModel.Unit.Create(6)
    for k in range(3):
        g.add_between(k, k + 1, step, unit, "lidar_odom")
    g.add_between(0, 3, _pose([0, 0, 0], [2.4, 0, 0]), unit, "loop")
    assert g.sources() == ["fixed", "lidar_odom", "loop"]
    x_odom = graph.poses(g.optimize(enabled=["lidar_odom"]), [3])[0][0, 3]
    x_all = graph.poses(g.optimize(), [3])[0][0, 3]
    assert abs(x_odom - 3.0) < 1e-6
    assert abs(x_all - 2.55) < 1e-6        # 3 odometry edges vs 1 loop, equal weights: 3 - 0.6 * 3/4


def test_chain_composes_relative_motion():
    z = _pose([0, 0, np.pi / 2], [1, 0, 0])
    P = graph.chain([(z, None)] * 4)
    assert np.allclose(P[-1], np.eye(4), atol=1e-12)
