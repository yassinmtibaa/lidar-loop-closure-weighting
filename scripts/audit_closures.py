#!/usr/bin/env python3
"""usage: audit_closures.py <seq> <run_name>   e.g. audit_closures.py 05 slam_default
Labels each loop-closure edge of a KISS-SLAM run against KITTI ground truth."""
import os, sys
import numpy as np

seq, run = sys.argv[1], sys.argv[2]
home = os.path.expanduser("~/lidar_loop")
res = f"{home}/results/{run}_seq{seq}/latest"
kitti = f"{home}/data/kitti/dataset"

def load_poses(p):
    a = np.loadtxt(p).reshape(-1, 3, 4)
    T = np.tile(np.eye(4), (len(a), 1, 1)); T[:, :3, :] = a
    return T

def read_tr(p):
    for line in open(p):
        if line.startswith("Tr:"):
            T = np.eye(4); T[:3, :] = np.array(line.split()[1:], float).reshape(3, 4)
            return T
    raise SystemExit("no Tr line in calib.txt")

def to_T(v):
    x, y, z, qx, qy, qz, qw = v
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw); qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    R = np.array([[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                  [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                  [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [x, y, z]
    return T

def rot_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))

verts, edges = {}, []
for line in open(f"{res}/local_maps/local_map_graph.g2o"):
    t = line.split()
    if not t: continue
    if t[0] == "VERTEX_SE3:QUAT":
        verts[int(t[1])] = to_T([float(x) for x in t[2:9]])
    elif t[0] == "EDGE_SE3:QUAT":
        edges.append((int(t[1]), int(t[2]), to_T([float(x) for x in t[3:10]])))

Tr = read_tr(f"{kitti}/sequences/{seq}/calib.txt"); Tri = np.linalg.inv(Tr)
est = np.array([Tri @ P @ Tr for P in load_poses(f"{res}/{seq}_poses_kitti.txt")])
gt = np.array([Tri @ P @ Tr for P in load_poses(f"{res}/{seq}_gt_kitti.txt")])

idx, resid, prev = {}, [], 0
for k in sorted(verts):
    d = np.linalg.norm(est[prev:prev + 600, :3, 3] - verts[k][:3, 3], axis=1)
    idx[k] = prev + int(np.argmin(d)); resid.append(d.min()); prev = idx[k]
print(f"seq {seq} run {run}: {len(verts)} vertices, vertex->scan residual median "
      f"{np.median(resid):.2f} m, max {max(resid):.2f} m  (large = mapping unreliable)")

odom, clos = [], []
for i, j, Tm in edges:
    E = np.linalg.inv(np.linalg.inv(gt[idx[i]]) @ gt[idx[j]]) @ Tm
    row = (i, j, idx[i], idx[j], np.linalg.norm(Tm[:3, 3]),
           np.linalg.norm(E[:3, 3]), rot_deg(E[:3, :3]))
    (odom if abs(i - j) == 1 else clos).append(row)

print("odometry edges (reference): mean trans err %.2f m, max %.2f m, max rot err %.2f deg"
      % (np.mean([r[5] for r in odom]), max(r[5] for r in odom), max(r[6] for r in odom)))
print("closure   i<->j   scans         |t| meas   trans err   rot err   label")
nbad = 0
for i, j, si, sj, dist, te, re in clos:
    bad = te > 1.0 or re > 2.0; nbad += bad
    print(f"          {i:2d}<->{j:2d}  {si:5d}-{sj:5d}  {dist:8.1f} m  {te:8.2f} m  {re:6.2f}°   {'BAD' if bad else 'good'}")
print(f"SUMMARY seq {seq} {run}: {len(clos)} closures, {len(clos) - nbad} good, {nbad} bad")
