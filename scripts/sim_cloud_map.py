#!/usr/bin/env python3
"""Coarse azimuth x ring map of /lidar/points. '.' >=80% ground hits, '#' >=80% finite non-ground,
'-' >=80% non-finite, '+' mixed, ' ' no beams in the cell. One column = 10 deg of azimuth."""
import time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

FMT = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}

class C(Node):
    def __init__(self):
        super().__init__("sim_cloud_map")
        self.create_subscription(PointCloud2, "/lidar/points", self.cb, qos_profile_sensor_data)
        self.n, self.a = 0, None
    def cb(self, m):
        dt = np.dtype({"names": [f.name for f in m.fields], "formats": [FMT[f.datatype] for f in m.fields],
                       "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
        self.a = np.frombuffer(bytes(m.data), dtype=dt).copy(); self.n += 1

rclpy.init(); node = C(); t0 = time.time()
while rclpy.ok() and node.n < 3 and time.time() - t0 < 60:
    rclpy.spin_once(node, timeout_sec=0.5)
a = node.a
if a is None:
    print("no message in 60 s"); raise SystemExit(1)
x, y, z = (a[k].astype(float) for k in "xyz")
fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
ring = a["ring"].astype(int)
el = np.degrees(np.arctan2(z, np.hypot(x, y))); az = np.degrees(np.arctan2(y, x))
zg = np.median(z[(ring == 0) & fin])
ground = fin & (np.abs(z - zg) < 0.1); nong = fin & ~ground
print(f"points {len(a)}: ground {ground.sum()} ({100*ground.mean():.0f}%), non-ground finite {nong.sum()} "
      f"({100*nong.mean():.0f}%), non-finite {(~fin).sum()} ({100*(~fin).mean():.0f}%); ground z {zg:.3f}")
bins = np.floor((az + 180) / 10).astype(int).clip(0, 35)
print("columns: |=-180 deg, |=-90, |=0, |=+90 (10 deg per column)")
print("                   " + "".join("|" if b in (0, 9, 18, 27) else " " for b in range(36)))
for r in (0, 10, 20, 30, 40, 50, 54, 63):
    line = ""
    for b in range(36):
        m = (ring == r) & (bins == b); n = int(m.sum())
        if n == 0: line += " "; continue
        g, k, f = (m & ground).sum() / n, (m & nong).sum() / n, (m & ~fin).sum() / n
        line += "." if g >= .8 else "#" if k >= .8 else "-" if f >= .8 else "+"
    e = np.median(el[(ring == r) & fin]) if ((ring == r) & fin).any() else float("nan")
    print(f"ring {r:2d} el {e:6.2f} |{line}|")
node.destroy_node(); rclpy.shutdown()
