#!/usr/bin/env python3
"""Print structure and statistics of one PointCloud2 from /lidar/points."""
import sys
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

FMT = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}

class N(Node):
    def __init__(self):
        super().__init__("inspect_cloud")
        self.create_subscription(PointCloud2, "/lidar/points", self.cb, qos_profile_sensor_data)
        self.done = False

    def cb(self, m):
        dt = np.dtype({"names": [f.name for f in m.fields],
                       "formats": [FMT[f.datatype] for f in m.fields],
                       "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
        a = np.frombuffer(bytes(m.data), dtype=dt)
        print(f"frame {m.header.frame_id}, height {m.height}, width {m.width}, point_step {m.point_step}, "
              f"row_step {m.row_step}, is_dense {m.is_dense}, points {len(a)}")
        fin = np.isfinite(a["x"]) & np.isfinite(a["y"]) & np.isfinite(a["z"])
        print(f"finite points: {fin.sum()} of {len(a)}  (non-finite: {(~fin).sum()})")
        r = np.sqrt(a["x"][fin] ** 2 + a["y"][fin] ** 2 + a["z"][fin] ** 2)
        if len(r):
            print(f"range of finite points: min {r.min():.3f}  median {np.median(r):.3f}  max {r.max():.3f}")
        it = a["intensity"]
        u, c = np.unique(it[np.isfinite(it)], return_counts=True)
        print(f"intensity: {len(u)} distinct finite values; first few (value:count):",
              ", ".join(f"{v:.3f}:{n}" for v, n in list(zip(u, c))[:8]))
        print(f"intensity non-finite: {(~np.isfinite(it)).sum()}")
        ring = a["ring"]
        print(f"ring: min {ring.min()} max {ring.max()} distinct {len(np.unique(ring))}")
        print("ring of first 3 points:", ring[:3], " ring at index 640..642:", ring[640:643])
        if m.height > 1:
            print("ring equals row index everywhere:", bool(np.all(ring.reshape(m.height, m.width) == np.arange(m.height)[:, None])))
        self.done = True

rclpy.init(); n = N()
t = n.get_clock().now()
while rclpy.ok() and not n.done and (n.get_clock().now() - t).nanoseconds < 20e9:
    rclpy.spin_once(n, timeout_sec=0.5)
if not n.done:
    print("no message received in 20 s")
n.destroy_node(); rclpy.shutdown()
