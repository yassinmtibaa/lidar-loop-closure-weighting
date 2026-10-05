#!/usr/bin/env python3
"""usage: step_jumps.py <seq> <slam_run> [odo_run]
Per-consecutive-scan difference between SLAM and odometry relative motion.
Piecewise-rigid corrections show up as isolated spikes at local-map boundaries."""
import os, sys
import numpy as np
seq, run = sys.argv[1], sys.argv[2]
odo = sys.argv[3] if len(sys.argv) > 3 else "baseline"
h = os.path.expanduser("~/lidar_loop/results")
def load(p):
    a = np.loadtxt(p).reshape(-1, 3, 4)
    T = np.tile(np.eye(4), (len(a), 1, 1)); T[:, :3, :] = a
    return T
S = load(f"{h}/{run}_seq{seq}/latest/{seq}_poses_kitti.txt")
O = load(f"{h}/{odo}_seq{seq}/latest/{seq}_poses_kitti.txt")
n = min(len(S), len(O))
d = np.zeros(n - 1)
for i in range(n - 1):
    rs = np.linalg.inv(S[i]) @ S[i + 1]
    ro = np.linalg.inv(O[i]) @ O[i + 1]
    d[i] = np.linalg.norm((np.linalg.inv(ro) @ rs)[:3, 3])
order = np.argsort(d)[::-1]; tot = d.sum()
print(f"seq {seq}: {n-1} steps, median step diff {np.median(d)*1000:.1f} mm, sum {tot:.2f} m")
print(f"top 10 steps hold {d[order[:10]].sum()/tot*100:.0f}% of the total, top 30 hold {d[order[:30]].sum()/tot*100:.0f}%")
print("largest (frame:diff m):", "  ".join(f"{i}:{d[i]:.2f}" for i in order[:12]))
