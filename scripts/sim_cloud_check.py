#!/usr/bin/env python3
"""usage: sim_cloud_check.py [n_msgs]
Collect /lidar/points; report size, rates, per-ring finite fraction and elevation, ground height, intensity values."""
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

FMT = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
N = int(sys.argv[1]) if len(sys.argv) > 1 else 20

class C(Node):
    def __init__(self):
        super().__init__("sim_cloud_check")
        self.create_subscription(PointCloud2, "/lidar/points", self.cb, qos_profile_sensor_data)
        self.rec = []
    def cb(self, m):
        dt = np.dtype({"names": [f.name for f in m.fields], "formats": [FMT[f.datatype] for f in m.fields],
                       "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
        a = np.frombuffer(bytes(m.data), dtype=dt).copy()
        self.rec.append((time.time(), m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m.height, m.width, a))

rclpy.init(); node = C(); t0 = time.time()
while rclpy.ok() and len(node.rec) < N and time.time() - t0 < 90:
    rclpy.spin_once(node, timeout_sec=0.5)
if len(node.rec) < 2:
    print(f"only {len(node.rec)} message(s) in 90 s"); sys.exit(1)
wall = np.array([r[0] for r in node.rec]); sim = np.array([r[1] for r in node.rec])
_, _, h, w, a = node.rec[-1]
print(f"{len(node.rec)} messages; cloud {h} x {w} = {len(a)} points")
print(f"wall rate {(len(wall)-1)/(wall[-1]-wall[0]):.2f} Hz, sim rate {(len(sim)-1)/(sim[-1]-sim[0]):.2f} Hz, "
      f"real-time factor {(sim[-1]-sim[0])/(wall[-1]-wall[0]):.2f}")
x, y, z = (a[k].astype(float) for k in "xyz")
fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
ring = a["ring"].astype(int)
print(f"finite points: {fin.sum()} of {len(a)} ({100 * fin.mean():.1f}%)")
el = np.degrees(np.arctan2(z, np.hypot(x, y)))
nf = np.array([int(((ring == r) & fin).sum()) for r in range(h)])
rf = np.where(nf > 0)[0]
print(f"rings with finite points: {len(rf)} of {h}" + (f" (ring {rf.min()}..{rf.max()})" if len(rf) else ""))
if len(rf) > 1:
    med = np.array([np.median(el[(ring == r) & fin]) for r in rf])
    print(f"elevation of first/last finite ring: {med[0]:.2f} / {med[-1]:.2f} deg; median spacing {np.median(np.abs(np.diff(med))):.3f} deg")
    print(f"ring 0 finite share {nf[0]/w:.2f}, ring {h-1} finite share {nf[h-1]/w:.2f}")
down = fin & (el < -5)
if down.any():
    print(f"median z of finite points with elevation < -5 deg: {np.median(z[down]):.3f} m (expected -1.73 if the sensor frame is the model origin)")
it = a["intensity"][fin]
u, c = np.unique(np.round(it, 3), return_counts=True)
print("intensity distinct values (value:count):", ", ".join(f"{v:.3f}:{k}" for v, k in list(zip(u, c))[:8]))
node.destroy_node(); rclpy.shutdown()
