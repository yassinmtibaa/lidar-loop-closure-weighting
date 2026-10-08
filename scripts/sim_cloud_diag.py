#!/usr/bin/env python3
"""One cloud from /lidar/points: where do finite points and intensity-0 points sit?"""
import time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

FMT = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}

class C(Node):
    def __init__(self):
        super().__init__("sim_cloud_diag")
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
rng = np.sqrt(x * x + y * y + z * z)
el = np.degrees(np.arctan2(z, np.hypot(x, y))); az = np.degrees(np.arctan2(y, x))
it = a["intensity"].astype(float); ring = a["ring"].astype(int)
zg = np.median(z[fin & (el < -5)])
print(f"ground height estimate: {zg:.3f} m")
print("ring  el(deg)  finite%  zero-int% (of finite)  median range  median z")
for r in (0, 10, 20, 30, 40, 50, 53, 54, 56, 58, 60, 63):
    m = ring == r; f = m & fin
    if f.sum() == 0:
        print(f"{r:4d}  no finite points"); continue
    print(f"{r:4d}  {np.median(el[f]):7.2f}  {100*f.sum()/m.sum():6.1f}  {100*(it[f] < 0.1).mean():12.1f}  "
          f"{np.median(rng[f]):12.2f}  {np.median(z[f]):8.3f}")
g = fin & (np.abs(z - zg) < 0.1); ng = fin & ~g
cls = lambda m: (int(((it < 0.1) & m).sum()), int(((it >= 0.1) & (it < 0.6) & m).sum()), int(((it >= 0.6) & m).sum()))
print("ground-height points, intensity 0 / ~0.3 / ~0.9:", cls(g))
print("non-ground finite points, intensity 0 / ~0.3 / ~0.9:", cls(ng))
if ng.any():
    print("non-ground range min / median / max:", np.round([rng[ng].min(), np.median(rng[ng]), rng[ng].max()], 2))
sec = ((az + 180) // 30).astype(int)
for r in (0, 20):
    m = ring == r
    print(f"ring {r} per 30-deg sector from -180: finite% " +
          " ".join(f"{100*(fin & m & (sec == s)).sum()/max((m & (sec == s)).sum(), 1):3.0f}" for s in range(12)))
print("ring 20 zero-intensity% of finite, same sectors: " +
      " ".join(f"{100*((it < 0.1) & fin & (ring == 20) & (sec == s)).sum()/max((fin & (ring == 20) & (sec == s)).sum(), 1):3.0f}" for s in range(12)))
node.destroy_node(); rclpy.shutdown()
