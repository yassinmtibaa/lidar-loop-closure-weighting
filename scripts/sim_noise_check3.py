#!/usr/bin/env python3
"""usage: sim_noise_check3.py [n_pairs]   (gz sim + bridge + noise_node with default parameters running)
A: per-ring elevation spread and ideal second-difference spread on ground.  B: where do large ideal d2 sit (columns, rings)?
C: estimator on the noisy cloud, all triples vs triples whose ideal d2 is small."""
import sys, time
from collections import defaultdict
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

FMT = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
NP = int(sys.argv[1]) if len(sys.argv) > 1 else 8
A0, K0 = 0.0086, 0.00044
sig_model = lambda r: np.sqrt(A0 ** 2 + (K0 * r) ** 2)
mad6 = lambda x: 1.4826 * np.median(np.abs(x - np.median(x))) / np.sqrt(6)

class C(Node):
    def __init__(self):
        super().__init__("sim_noise_check3")
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

bins = [(5, 10), (10, 20), (20, 30), (30, 40)]
Cb = {b: {"di": [], "dn": [], "r": []} for b in bins}
elr, rr, d2r = defaultdict(list), defaultdict(list), defaultdict(list)
colhits = ringhits = ringtri = None
for s in pairs:
    (pi, h, w), (pn, _, _) = node.ideal[s], node.noisy[s]
    if colhits is None:
        colhits, ringhits, ringtri = np.zeros(w), np.zeros(h), np.zeros(h)
    g3 = lambda p: [p[c].astype(np.float64).reshape(h, w) for c in "xyz"]
    (xi, yi, zi), (xn, yn, zn) = g3(pi), g3(pn)
    with np.errstate(invalid="ignore"):
        ri, rn = np.sqrt(xi ** 2 + yi ** 2 + zi ** 2), np.sqrt(xn ** 2 + yn ** 2 + zn ** 2)
        rh = np.hypot(xi, yi); it = pi["intensity"].astype(np.float64).reshape(h, w)
        fin = np.isfinite(ri) & np.isfinite(rn)
        el = np.degrees(np.arctan2(zi, rh)); zg = np.median(zi[fin & (el < -5)])
        g = fin & (np.abs(zi - zg) < 0.1) & (it < 0.6)
        cm = lambda a: a[:, 1:-1]
        tri = g[:, :-2] & g[:, 1:-1] & g[:, 2:]
        di = ri[:, :-2] - 2 * cm(ri) + ri[:, 2:]; dn = rn[:, :-2] - 2 * cm(rn) + rn[:, 2:]
        rhm = cm(rh)
    for b in bins:
        m = tri & (rhm >= b[0]) & (rhm < b[1])
        Cb[b]["di"].append(di[m]); Cb[b]["dn"].append(dn[m]); Cb[b]["r"].append(cm(ri)[m])
    use = tri & (rhm >= 5) & (rhm < 40)
    out = use & (np.abs(di) > 0.02)
    colhits[1:-1] += out.sum(0); ringhits += out.sum(1); ringtri += use.sum(1)
    for r in range(h):
        if g[r].any():
            elr[r].append(el[r][g[r]]); rr[r].append(ri[r][g[r]])
        if tri[r].any():
            d2r[r].append(di[r][tri[r]])

print(f"{len(pairs)} matched pairs, cloud {h} x {w}")
print("\nA. ideal ground, per ring: ring  el(deg)  r(m)  el spread(1e-3 deg)  predicted range jitter(cm)  ideal d2 MAD/sqrt6(cm)  std/sqrt6(cm)")
rings = [r for r in sorted(elr) if sum(len(v) for v in elr[r]) > 1000]
for r in rings[::max(1, len(rings) // 14)]:
    e = np.concatenate(elr[r]); rm = np.median(np.concatenate(rr[r])); em = np.median(e)
    sp = 1.4826 * np.median(np.abs(e - em))
    pred = 100 * rm / np.tan(np.radians(abs(em))) * np.radians(sp)
    d = np.concatenate(d2r[r]) if d2r[r] else np.array([])
    if len(d) > 100:
        print(f"  {r:3d} {em:8.2f} {rm:7.1f} {1000*sp:12.2f} {pred:16.2f} {100*mad6(d):18.3f} {100*d.std()/np.sqrt(6):12.3f}")
tot = colhits.sum()
print(f"\nB. ground triples with |ideal d2| > 2 cm (5-40 m): {int(ringhits.sum())} of {int(ringtri.sum())} = {100*ringhits.sum()/max(ringtri.sum(), 1):.2f}%")
if tot > 0:
    order = np.argsort(colhits)[::-1]; cs = np.cumsum(colhits[order]) / tot
    print("   top columns (col:count): " + " ".join(f"{c}:{int(colhits[c])}" for c in order[:10]))
    print(f"   share held by the top 10 / 30 / 100 of {w} columns: {100*cs[9]:.0f}% / {100*cs[29]:.0f}% / {100*cs[99]:.0f}%")
    rf = ringhits / np.maximum(ringtri, 1); ro = np.argsort(rf)[::-1][:6]
    print("   rings with highest outlier fraction (ring:%): " + " ".join(f"{r}:{100*rf[r]:.1f}" for r in ro if ringtri[r] > 500))
print("\nC. estimator on the NOISY cloud: bin  n_all  ratio_all  |  n_smooth(|ideal d2| < 1 cm)  kept%  ratio_smooth   (ratio = sigma_noisy / sigma_model at median r)")
for b in bins:
    di_, dn_, r_ = (np.concatenate(Cb[b][k]) if Cb[b][k] else np.array([]) for k in ("di", "dn", "r"))
    if len(di_) < 500:
        continue
    rm = np.median(r_); line = f"  {b[0]:2d}-{b[1]:2d} m {len(di_):8d} {mad6(dn_) / sig_model(rm):9.3f}  |"
    sm = np.abs(di_) < 0.01
    line += f" {int(sm.sum()):8d} {100*sm.mean():7.1f}% " + (f"{mad6(dn_[sm]) / sig_model(rm):12.3f}" if sm.sum() > 500 else "   too few")
    print(line)
node.destroy_node(); rclpy.shutdown()
