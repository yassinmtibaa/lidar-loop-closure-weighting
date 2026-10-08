#!/usr/bin/env python3
"""usage: sim_noise_check2.py [n_pairs]  (gz sim + bridge + noise_node with default parameters running)
Is the injected noise white along rows, and where does the second-difference excess come from?"""
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

FMT = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
NP = int(sys.argv[1]) if len(sys.argv) > 1 else 8

class C(Node):
    def __init__(self):
        super().__init__("sim_noise_check2")
        self.ideal, self.noisy = {}, {}
        self.create_subscription(PointCloud2, "/lidar/points", lambda m: self.cb(m, self.ideal), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/lidar/points_noisy", lambda m: self.cb(m, self.noisy), qos_profile_sensor_data)
    def cb(self, m, store):
        dt = np.dtype({"names": [f.name for f in m.fields], "formats": [FMT[f.datatype] for f in m.fields],
                       "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
        store[(m.header.stamp.sec, m.header.stamp.nanosec)] = (np.frombuffer(bytes(m.data), dtype=dt).copy(), m.height, m.width)

rclpy.init(); node = C(); t0 = time.time()
while rclpy.ok() and time.time() - t0 < 90:
    rclpy.spin_once(node, timeout_sec=0.3)
    if len([s for s in node.ideal if s in node.noisy]) >= NP:
        break
pairs = sorted(s for s in node.ideal if s in node.noisy)
if len(pairs) < 2:
    print("too few matched pairs"); sys.exit(1)

mad = lambda x: 1.4826 * np.median(np.abs(x - np.median(x)))
bins = [(5, 10), (20, 30)]
acc = {b: {k: [] for k in ("e", "a1", "b1", "a2", "b2", "d1", "d2e", "d2i", "d2n")} for b in bins}
for s in pairs:
    (pi, h, w), (pn, _, _) = node.ideal[s], node.noisy[s]
    g3 = lambda p: [p[c].astype(np.float64).reshape(h, w) for c in "xyz"]
    (xi, yi, zi), (xn, yn, zn) = g3(pi), g3(pn)
    with np.errstate(invalid="ignore"):
        ri, rn = np.sqrt(xi**2 + yi**2 + zi**2), np.sqrt(xn**2 + yn**2 + zn**2)
        rh = np.hypot(xi, yi); it = pi["intensity"].astype(np.float64).reshape(h, w)
        fin = np.isfinite(ri) & np.isfinite(rn)
        el = np.degrees(np.arctan2(zi, rh)); zg = np.median(zi[fin & (el < -5)])
        g = fin & (np.abs(zi - zg) < 0.1) & (it < 0.6)
        for b in bins:
            m = g & (rh >= b[0]) & (rh < b[1])
            e = np.where(m, rn - ri, np.nan); I = np.where(m, ri, np.nan); N = np.where(m, rn, np.nan)
            d = acc[b]
            v = ~np.isnan(e[:, :-1]) & ~np.isnan(e[:, 1:]); d["a1"].append(e[:, :-1][v]); d["b1"].append(e[:, 1:][v])
            v = ~np.isnan(e[:, :-2]) & ~np.isnan(e[:, 2:]); d["a2"].append(e[:, :-2][v]); d["b2"].append(e[:, 2:][v])
            d["e"].append(e[~np.isnan(e)])
            d1 = e[:, 1:] - e[:, :-1]; d["d1"].append(d1[~np.isnan(d1)])
            for key, A in (("d2e", e), ("d2i", I), ("d2n", N)):
                x = A[:, :-2] - 2 * A[:, 1:-1] + A[:, 2:]
                x2 = np.where(np.isnan(e[:, :-2]) | np.isnan(e[:, 1:-1]) | np.isnan(e[:, 2:]), np.nan, x)
                d[key].append(x2[~np.isnan(x2)])
print(f"{len(pairs)} matched pairs")
for b in bins:
    d = {k: np.concatenate(v) for k, v in acc[b].items()}
    rho = lambda a, c: np.corrcoef(a, c)[0, 1]
    print(f"\nbin {b[0]}-{b[1]} m: {len(d['e'])} points, {len(d['d2e'])} triples")
    print(f"  injected noise std {100*d['e'].std():.3f} cm; autocorrelation lag1 {rho(d['a1'], d['b1']):+.3f}, lag2 {rho(d['a2'], d['b2']):+.3f}")
    print(f"  first difference of noise: MAD/sqrt2 {100*mad(d['d1'])/np.sqrt(2):.3f} cm, std/sqrt2 {100*d['d1'].std()/np.sqrt(2):.3f} cm")
    for key, name in (("d2e", "second diff of INJECTED NOISE only"), ("d2i", "second diff of IDEAL ranges      "), ("d2n", "second diff of NOISY ranges      ")):
        print(f"  {name}: MAD/sqrt6 {100*mad(d[key])/np.sqrt(6):.3f} cm, std/sqrt6 {100*d[key].std()/np.sqrt(6):.3f} cm")
    q = np.percentile(np.abs(d["d2i"]), [50, 99, 99.9])
    print(f"  ideal second-difference |d2| percentiles 50/99/99.9: {np.round(100*q, 3)} cm")
node.destroy_node(); rclpy.shutdown()
