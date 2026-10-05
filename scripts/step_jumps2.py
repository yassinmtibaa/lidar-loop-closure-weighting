#!/usr/bin/env python3
"""usage: step_jumps2.py <seq> <slam_run> [odo_run]
SLAM-vs-odometry difference per consecutive scan, translation (m) AND rotation (deg),
and how much of it sits within 1 frame of a local-map boundary (graph vertex)."""
import os, sys
import numpy as np
seq, run = sys.argv[1], sys.argv[2]
odo = sys.argv[3] if len(sys.argv) > 3 else "baseline"
home = os.path.expanduser("~/lidar_loop"); h = f"{home}/results"

def load(p):
    a = np.loadtxt(p).reshape(-1, 3, 4)
    T = np.tile(np.eye(4), (len(a), 1, 1)); T[:, :3, :] = a
    return T

def read_tr(p):
    for line in open(p):
        if line.startswith("Tr:"):
            T = np.eye(4); T[:3, :] = np.array(line.split()[1:], float).reshape(3, 4)
            return T
    raise SystemExit("no Tr line in calib.txt")

S = load(f"{h}/{run}_seq{seq}/latest/{seq}_poses_kitti.txt")
O = load(f"{h}/{odo}_seq{seq}/latest/{seq}_poses_kitti.txt")
n = min(len(S), len(O))
dt, dr = np.zeros(n - 1), np.zeros(n - 1)
for i in range(n - 1):
    E = np.linalg.inv(np.linalg.inv(O[i]) @ O[i + 1]) @ (np.linalg.inv(S[i]) @ S[i + 1])
    dt[i] = np.linalg.norm(E[:3, 3])
    dr[i] = np.degrees(np.arccos(np.clip((np.trace(E[:3, :3]) - 1) / 2, -1, 1)))

Tr = read_tr(f"{home}/data/kitti/dataset/sequences/{seq}/calib.txt"); Tri = np.linalg.inv(Tr)
pos = np.array([(Tri @ P @ Tr)[:3, 3] for P in S])
verts = []
for line in open(f"{h}/{run}_seq{seq}/latest/local_maps/local_map_graph.g2o"):
    t = line.split()
    if t and t[0] == "VERTEX_SE3:QUAT": verts.append(np.array(t[2:5], float))
bound, prev = [], 0
for v in verts:
    prev += int(np.argmin(np.linalg.norm(pos[prev:prev + 600] - v, axis=1))); bound.append(prev)
bound = bound[1:]
near = np.zeros(n - 1, bool)
for b in bound: near[max(b - 1, 0):b + 2] = True

o = np.argsort(dr)[::-1]
print(f"seq {seq}: {len(bound)} map boundaries; frames within +-1 of a boundary = {100*near.mean():.1f}% of all")
print(f"rotation diff: median {np.median(dr):.3f} deg, max {dr.max():.2f} deg, sum {dr.sum():.1f} deg")
print(f"  share within +-1 frame of a boundary: {100*dr[near].sum()/dr.sum():.0f}%   top-10 steps hold {100*dr[o[:10]].sum()/dr.sum():.0f}%")
print(f"translation diff: share within +-1 frame of a boundary: {100*dt[near].sum()/dt.sum():.0f}%")
print("largest rotation steps (frame:deg):", "  ".join(f"{i}:{dr[i]:.2f}" for i in o[:10]))
print("boundary frames:", bound)
