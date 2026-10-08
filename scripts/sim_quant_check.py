#!/usr/bin/env python3
"""usage: sim_quant_check.py [n_clouds]  (gz sim + bridge running). Ideal /lidar/points only:
per ring, ground second differences along rows: std, outlier share, jump-pair signature, outlier size."""
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

FMT = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
NC = int(sys.argv[1]) if len(sys.argv) > 1 else 5
T = 0.005

class C(Node):
    def __init__(self):
        super().__init__("sim_quant_check"); self.c = []
        self.create_subscription(PointCloud2, "/lidar/points", self.cb, qos_profile_sensor_data)
    def cb(self, m):
        dt = np.dtype({"names": [f.name for f in m.fields], "formats": [FMT[f.datatype] for f in m.fields],
                       "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
        self.c.append((np.frombuffer(bytes(m.data), dtype=dt).copy(), m.height, m.width))

rclpy.init(); node = C(); t0 = time.time()
while rclpy.ok() and len(node.c) < NC and time.time() - t0 < 60:
    rclpy.spin_once(node, timeout_sec=0.3)
if not node.c:
    print("no clouds"); sys.exit(1)
stats = {}
for p, h, w in node.c:
    x, y, z = (p[k].astype(np.float64).reshape(h, w) for k in "xyz")
    it = p["intensity"].astype(np.float64).reshape(h, w)
    with np.errstate(invalid="ignore"):
        r = np.sqrt(x * x + y * y + z * z); rh = np.hypot(x, y)
        fin = np.isfinite(r); el = np.degrees(np.arctan2(z, rh)); zg = np.median(z[fin & (el < -5)])
        g = fin & (np.abs(z - zg) < 0.1) & (it < 0.6)
    tri = g[:, :-2] & g[:, 1:-1] & g[:, 2:]
    d2 = np.where(tri, r[:, :-2] - 2 * r[:, 1:-1] + r[:, 2:], np.nan)
    for ring in range(h):
        v = d2[ring]; ok = ~np.isnan(v)
        if ok.sum() < 50:
            continue
        s = stats.setdefault(ring, {"d": [], "r": [], "out": 0, "pair": 0})
        s["d"].append(v[ok]); s["r"].append(np.nanmedian(r[ring][g[ring]]))
        out = np.where(ok & (np.abs(v) > T))[0]
        for j in out:
            s["out"] += 1
            for k in (j - 1, j + 1):
                if 0 <= k < len(v) and not np.isnan(v[k]) and v[k] * v[j] < 0 and abs(v[k] + v[j]) < 0.2 * abs(v[j]):
                    s["pair"] += 1; break
print(f"{len(node.c)} clouds, {h} x {w}")
print("ring  r(m)   n_triples  std/sqrt6(cm)  outlier%  paired%  median|outlier d2|(cm)")
allo = allp = 0
for ring in sorted(stats)[::3]:
    s = stats[ring]; d = np.concatenate(s["d"]); o = np.abs(d) > T
    allo += s["out"]; allp += s["pair"]
    print(f"{ring:4d} {np.median(s['r']):6.1f} {len(d):10d} {100*d.std()/np.sqrt(6):12.3f} {100*o.mean():9.1f} "
          f"{100*s['pair']/max(s['out'],1):8.1f} {100*np.median(np.abs(d[o])) if o.any() else float('nan'):14.3f}")
print(f"paired share over printed rings: {100*allp/max(allo,1):.1f}%")
node.destroy_node(); rclpy.shutdown()
