#!/usr/bin/env python3
"""usage: audit_closures2.py <seq> <run_name> [odometry_run]
Per closure edge: error vs ground truth, error of the pure-odometry chain between the
same two scans, and the overlap score from the run log."""
import os, re, sys
import numpy as np

seq, run = sys.argv[1], sys.argv[2]
odo_run = sys.argv[3] if len(sys.argv) > 3 else "baseline"
home = os.path.expanduser("~/lidar_loop")
res = f"{home}/results/{run}_seq{seq}/latest"
odo_file = f"{home}/results/{odo_run}_seq{seq}/latest/{seq}_poses_kitti.txt"
log_file = f"{home}/results/{run}_seq{seq}.log"
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

verts, edges = {}, []
for line in open(f"{res}/local_maps/local_map_graph.g2o"):
    t = line.split()
    if not t: continue
    if t[0] == "VERTEX_SE3:QUAT": verts[int(t[1])] = to_T([float(x) for x in t[2:9]])
    elif t[0] == "EDGE_SE3:QUAT": edges.append((int(t[1]), int(t[2]), to_T([float(x) for x in t[3:10]])))

Tr = read_tr(f"{kitti}/sequences/{seq}/calib.txt"); Tri = np.linalg.inv(Tr)
conv = lambda p: np.array([Tri @ P @ Tr for P in load_poses(p)])
est, gt, odo = conv(f"{res}/{seq}_poses_kitti.txt"), conv(f"{res}/{seq}_gt_kitti.txt"), conv(odo_file)

idx, prev = {}, 0
for k in sorted(verts):
    d = np.linalg.norm(est[prev:prev + 600, :3, 3] - verts[k][:3, 3], axis=1)
    idx[k] = prev + int(np.argmin(d)); prev = idx[k]

def chain_err(a, b):
    E = np.linalg.inv(np.linalg.inv(gt[a]) @ gt[b]) @ (np.linalg.inv(odo[a]) @ odo[b])
    return np.linalg.norm(E[:3, 3])

cand = []
txt = open(log_file, errors="ignore").read()
for m in re.finditer(r"LocalMaps Overlap: ([0-9.eE+-]+)|Closure rejected for low overlap", txt):
    if m.group(1) is not None: cand.append([float(m.group(1)), True])
    elif cand: cand[-1][1] = False
accepted = [c[0] for c in cand if c[1]]
print(f"seq {seq} {run}: {len(cand)} candidates reached validation, {len(accepted)} accepted")
print("  overlaps (A=accepted, r=rejected):", " ".join(f"{o:.2f}{'A' if a else 'r'}" for o, a in cand))

clos = [(i, j, Tm) for i, j, Tm in edges if abs(i - j) != 1]
join = len(accepted) == len(clos)
if not join: print("  WARNING: accepted overlaps != closure edges, overlap column omitted")
print("    i<->j   scans         |t|     closure err   odom-chain err   overlap   vs odometry")
for n, (i, j, Tm) in enumerate(clos):
    si, sj = idx[i], idx[j]
    E = np.linalg.inv(np.linalg.inv(gt[si]) @ gt[sj]) @ Tm
    ce, oe = np.linalg.norm(E[:3, 3]), chain_err(si, sj)
    ov = f"{accepted[n]:.2f}" if join else "-"
    print(f"  {i:2d}<->{j:2d}  {si:5d}-{sj:5d}  {np.linalg.norm(Tm[:3,3]):6.1f} m  {ce:8.2f} m  {oe:10.2f} m   {ov:>6}   {'better' if ce < oe else 'WORSE'}")
