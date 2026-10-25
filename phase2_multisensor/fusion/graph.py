"""GTSAM pose-graph backbone.

PoseGraph holds Pose3 variables and tagged factors; optimize() can switch any factor source (sensor) on or off.
kiss_slam_two_stage() rebuilds KISS-SLAM's own two-stage optimization from its g2o output, to check the backbone
reproduces the baseline. joint_graph() is the single per-frame graph that later sensors attach to."""
import gtsam
import numpy as np

# g2o orders the tangent space (t, R); GTSAM Pose3 orders it (R, t).
_G2O_TO_GTSAM = np.r_[3, 4, 5, 0, 1, 2]
# g2o's EdgeSE3 error is [t, q_vec] of inv(z) inv(T_i) T_j, and q_vec ~ angle/2; GTSAM's BetweenFactor error is
# Logmap = [angle, t] to first order. Same cost to first order: rotation information x 1/4, cross terms x 1/2.
_HALF_ANGLE = np.diag([0.5, 0.5, 0.5, 1.0, 1.0, 1.0])


def info_g2o_to_gtsam(info, quaternion_error=True):
    """quaternion_error=False only permutes (treats the g2o rotation information as if it weighted the angle)."""
    P = info[np.ix_(_G2O_TO_GTSAM, _G2O_TO_GTSAM)]
    return _HALF_ANGLE @ P @ _HALF_ANGLE if quaternion_error else P


def noise_from_g2o_info(info, quaternion_error=True):
    return gtsam.noiseModel.Gaussian.Information(info_g2o_to_gtsam(info, quaternion_error))


class PoseGraph:
    def __init__(self):
        self.factors = []          # (source, factor)
        self.initial = gtsam.Values()

    def add_variable(self, key, T):
        self.initial.insert(key, gtsam.Pose3(T))

    def fix(self, key, T):
        self.factors.append(("fixed", gtsam.NonlinearEqualityPose3(key, gtsam.Pose3(T))))

    def add_between(self, i, j, z, noise, source):
        """z = inv(T_i) @ T_j."""
        self.factors.append((source, gtsam.BetweenFactorPose3(i, j, gtsam.Pose3(z), noise)))

    def add_factor(self, factor, source):
        self.factors.append((source, factor))

    def sources(self):
        return sorted({s for s, _ in self.factors})

    def optimize(self, enabled=None, max_iterations=100):
        """enabled: iterable of sources to use (None = all). 'fixed' factors are always kept."""
        graph = gtsam.NonlinearFactorGraph()
        for s, f in self.factors:
            if enabled is None or s == "fixed" or s in enabled:
                graph.add(f)
        params = gtsam.LevenbergMarquardtParams()
        params.setMaxIterations(max_iterations)
        params.setRelativeErrorTol(1e-12)
        params.setAbsoluteErrorTol(1e-12)
        return gtsam.LevenbergMarquardtOptimizer(graph, self.initial, params).optimize()


def poses(values, keys):
    return np.array([values.atPose3(k).matrix() for k in keys])


def fine_odometry(fine):
    """Per-frame odometry from KISS-SLAM's trajectory.g2o: list of (z, info) for frames k -> k+1."""
    by_i = {e.i: e for e in fine.edges}
    n = len(fine.vertices)
    assert all(by_i[k].j == k + 1 for k in range(n - 1)), "fine graph is not a chain"
    return [(by_i[k].z, by_i[k].info) for k in range(n - 1)]


def chain(odom, T0=np.eye(4)):
    out = [T0]
    for z, _ in odom:
        out.append(out[-1] @ z)
    return np.array(out)


def kiss_slam_two_stage(local, fine, max_iterations=100):
    """Rebuild KISS-SLAM's optimization in GTSAM from its two g2o files.
    Returns (keyposes (K,4,4), frame poses (N,4,4))."""
    K = len(local.vertices)
    odom_edges = {e.i: e for e in local.edges if e.j == e.i + 1}
    closures = [e for e in local.edges if e.j != e.i + 1]
    assert len(odom_edges) == K - 1

    # Stage 1: keypose graph, initial guess = odometry chain (not KISS-SLAM's answer).
    g1 = PoseGraph()
    T = np.eye(4)
    for k in range(K):
        g1.add_variable(k, T)
        if k < K - 1:
            T = T @ odom_edges[k].z
    g1.fix(0, np.eye(4))
    for k, e in odom_edges.items():
        g1.add_between(e.i, e.j, e.z, noise_from_g2o_info(e.info), "lidar_odom")
    for e in closures:
        g1.add_between(e.i, e.j, e.z, noise_from_g2o_info(e.info), "loop")
    keyposes = poses(g1.optimize(max_iterations=max_iterations), range(K))

    # Stage 2, mirroring KissSLAM.fine_grained_optimization(): local map k covers frames s_k..s_{k+1} (the boundary
    # frame is shared); frames are initialized from their own local map's optimized keypose; frame 0 and the
    # second-to-last frame of every local map are held fixed at that initial value (KISS-SLAM fixes id_-1).
    odom = fine_odometry(fine)
    N = len(odom) + 1
    fixed = sorted(set(fine.fixed))
    starts = [0] + [f + 1 for f in fixed[1:]]
    if len(starts) == K - 1:               # the final local map is empty (split at the last frame): no fix of its own
        starts.append(N - 1)
    assert fixed[0] == 0 and len(starts) == K, (len(starts), K)
    bounds = starts[1:] + [N - 1]
    g2 = PoseGraph()
    init = {0: keyposes[0]}
    for k in range(K):
        local_traj = chain(odom[starts[k]:bounds[k]])       # frames starts[k]..bounds[k], relative to keypose k
        for m, rel in enumerate(local_traj[1:], start=starts[k] + 1):
            init[m] = keyposes[k] @ rel
    for m in range(N):
        g2.add_variable(m, init[m])
    for f in fixed:
        g2.fix(f, init[f])
    for m, (z, info) in enumerate(odom):
        g2.add_between(m, m + 1, z, noise_from_g2o_info(info), "lidar_odom")
    frames = poses(g2.optimize(max_iterations=max_iterations), range(N))
    return keyposes, frames


def joint_graph(local, fine, odom_info=None, loop_info=None):
    """Single per-frame graph (the fusion backbone): per-frame LiDAR odometry factors and loop closures attached to
    the frames where their local maps start. Information matrices default to the ones in the g2o files (g2o order)."""
    odom = fine_odometry(fine)
    fixed = sorted(set(fine.fixed))
    starts = [0] + [f + 1 for f in fixed[1:]]          # frame of keypose k (see kiss_slam_two_stage)
    g = PoseGraph()
    for m, T in enumerate(chain(odom)):
        g.add_variable(m, T)
    g.fix(0, np.eye(4))
    for m, (z, info) in enumerate(odom):
        g.add_between(m, m + 1, z, noise_from_g2o_info(info if odom_info is None else odom_info), "lidar_odom")
    for e in local.edges:
        if e.j != e.i + 1:
            g.add_between(starts[e.i], starts[e.j], e.z,
                          noise_from_g2o_info(e.info if loop_info is None else loop_info), "loop")
    return g
