#!/usr/bin/env python3
"""usage: sim_noise_check.py [n_pairs] [a_cm k_cm_per_m scale]   (defaults 8 0.86 0.044 1.0)
Compares /lidar/points (ideal) with /lidar/points_noisy: direct noise statistics per range bin and the
second-difference ring estimator (as used on KITTI) on both clouds, same triples."""
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

FMT = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
NP = int(sys.argv[1]) if len(sys.argv) > 1 else 8
A, K, SC = (float(v) for v in (sys.argv[2:5] if len(sys.argv) > 4 else (0.86, 0.044, 1.0)))
A, K = A / 100.0, K / 100.0
sig_model = lambda r: SC * np.sqrt(A ** 2 + (K * r) ** 2)

class C(Node):
    def __init__(self):
        super().__init__("sim_noise_check")
        self.ideal, self.noisy, self.ti, self.tn = {}, {}, [], []
        self.create_subscription(PointCloud2, "/lidar/points", lambda m: self.cb(m, self.ideal, self.ti), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/lidar/points_noisy", lambda m: self.cb(m, self.noisy, self.tn), qos_profile_sensor_data)
    def cb(self, m, store, tl):
        dt = np.dtype({"names": [f.name for f in m.fields], "formats": [FMT[f.datatype] for f in m.fields],
                       "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
        store[(m.header.stamp.sec, m.header.stamp.nanosec)] = (np.frombuffer(bytes(m.data), dtype=dt).copy(), m.height, m.width)
        tl.append(time.time())

rclpy.init(); node = C(); t0 = time.time()
while rclpy.ok() and time.time() - t0 < 90:
    rclpy.spin_once(node, timeout_sec=0.3)
    if len([s for s in node.ideal if s in node.noisy]) >= NP:
        break
pairs = sorted(s for s in node.ideal if s in node.noisy)
rate = lambda t: (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else float("nan")
print(f"matched pairs {len(pairs)}; wall rate ideal {rate(node.ti):.2f} Hz, noisy {rate(node.tn):.2f} Hz")
if len(pairs) < 2:
    print("too few matched pairs: is noise_node running and the bridge up?"); sys.exit(1)

bins = [(3, 5), (5, 10), (10, 20), (20, 30), (30, 40)]
direct = {b: [] for b in bins}; d2i = {b: [] for b in bins}; d2n = {b: [] for b in bins}; rmid = {b: [] for b in bins}
for s in pairs:
    (pi, h, w), (pn, _, _) = node.ideal[s], node.noisy[s]
    f = lambda p: [p[c].astype(np.float64).reshape(h, w) for c in "xyz"]
    (xi, yi, zi), (xn, yn, zn) = f(pi), f(pn)
    ri, rn = np.sqrt(xi**2 + yi**2 + zi**2), np.sqrt(xn**2 + yn**2 + zn**2)
    rh = np.hypot(xi, yi); it = pi["intensity"].astype(np.float64).reshape(h, w)
    fin = np.isfinite(ri) & np.isfinite(rn)
    el = np.degrees(np.arctan2(zi, rh))
    zg = np.median(zi[fin & (el < -5)])
    g = fin & (np.abs(zi - zg) < 0.1) & (it < 0.6)
    for b in bins:
        m = g & (rh >= b[0]) & (rh < b[1])
        direct[b].append(np.c_[(rn - ri)[m], sig_model(ri[m])])
    cm = lambda a: a[:, 1:-1]
    tri = g[:, :-2] & g[:, 1:-1] & g[:, 2:]
    tri &= (np.abs(it[:, :-2] - cm(it)) < 0.1) & (np.abs(it[:, 2:] - cm(it)) < 0.1)
    tri &= (np.abs(ri[:, 1:-1] - ri[:, :-2]) < 0.3) & (np.abs(ri[:, 2:] - ri[:, 1:-1]) < 0.3)
    di = ri[:, :-2] - 2 * cm(ri) + ri[:, 2:]; dn = rn[:, :-2] - 2 * cm(rn) + rn[:, 2:]
    for b in bins:
        m = tri & (cm(rh) >= b[0]) & (cm(rh) < b[1])
        d2i[b].append(di[m]); d2n[b].append(dn[m]); rmid[b].append(cm(ri)[m])

mad = lambda x: 1.4826 * np.median(np.abs(x - np.median(x))) / np.sqrt(6)
print("\nDIRECT (noisy range - ideal range, ground, asphalt): bin   n    mean(mm)  std(cm)  rms sigma_model(cm)  ratio")
for b in bins:
    d = np.concatenate(direct[b]) if direct[b] else np.zeros((0, 2))
    if len(d) > 500:
        rm = np.sqrt((d[:, 1] ** 2).mean())
        print(f"  {b[0]:2d}-{b[1]:2d} m {len(d):9d} {1000*d[:,0].mean():9.3f} {100*d[:,0].std():8.3f} {100*rm:12.3f} {d[:,0].std()/rm:12.3f}")
print("\nESTIMATOR (second differences along rows): bin   n_triples  sigma_ideal(cm)  sigma_noisy(cm)  sigma_model@median r (cm)  noisy/model")
for b in bins:
    xi = np.concatenate(d2i[b]) if d2i[b] else np.array([])
    if len(xi) > 500:
        xn = np.concatenate(d2n[b]); rm = np.median(np.concatenate(rmid[b]))
        print(f"  {b[0]:2d}-{b[1]:2d} m {len(xi):9d} {100*mad(xi):14.3f} {100*mad(xn):16.3f} {100*sig_model(rm):20.3f} {mad(xn)/sig_model(rm):12.3f}")
node.destroy_node(); rclpy.shutdown()
