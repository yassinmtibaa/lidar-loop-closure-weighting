"""Read the g2o files KISS-SLAM writes (VERTEX_SE3:QUAT, EDGE_SE3:QUAT, FIX).
Edge measurement convention (checked on KISS-SLAM output): z_ij = inv(T_i) @ T_j.
Information matrices are in g2o order (x, y, z, rx, ry, rz)."""
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation


def pose_from_quat(v):
    """[x y z qx qy qz qw] -> 4x4."""
    T = np.eye(4)
    T[:3, 3] = v[:3]
    T[:3, :3] = Rotation.from_quat(v[3:7]).as_matrix()
    return T


def info_from_upper(u):
    """21 upper-triangular entries (row-major) -> symmetric 6x6."""
    I = np.zeros((6, 6))
    I[np.triu_indices(6)] = u
    return I + np.triu(I, 1).T


@dataclass
class Edge:
    i: int
    j: int
    z: np.ndarray       # 4x4, inv(T_i) @ T_j
    info: np.ndarray    # 6x6, g2o order (t, R)


@dataclass
class G2oGraph:
    vertices: dict = field(default_factory=dict)   # id -> 4x4
    edges: list = field(default_factory=list)
    fixed: list = field(default_factory=list)


def read_g2o(path):
    g = G2oGraph()
    with open(path) as f:
        for line in f:
            p = line.split()
            if not p:
                continue
            if p[0] == "VERTEX_SE3:QUAT":
                g.vertices[int(p[1])] = pose_from_quat(np.array(p[2:9], float))
            elif p[0] == "EDGE_SE3:QUAT":
                g.edges.append(Edge(int(p[1]), int(p[2]), pose_from_quat(np.array(p[3:10], float)),
                                    info_from_upper(np.array(p[10:31], float))))
            elif p[0] == "FIX":
                g.fixed += [int(x) for x in p[1:]]
    return g
