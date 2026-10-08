#!/usr/bin/env python3
"""usage: python3 scripts/noise_node.py --ros-args -p a_cm:=0.86 -p k_cm_per_m:=0.044 -p scale:=1.0 -p seed:=1
/lidar/points -> /lidar/points_noisy. Gaussian noise along each ray, sigma(r) = scale*sqrt(a^2 + (k r)^2)."""
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

FMT = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}

class NoiseNode(Node):
    def __init__(self):
        super().__init__("lidar_noise")
        for n, v in (("a_cm", 0.86), ("k_cm_per_m", 0.044), ("scale", 1.0), ("seed", 1)):
            self.declare_parameter(n, v)
        self.a0 = self.get_parameter("a_cm").value / 100.0
        self.k = self.get_parameter("k_cm_per_m").value / 100.0
        self.scale = float(self.get_parameter("scale").value)
        self.rng = np.random.default_rng(int(self.get_parameter("seed").value))
        self.pub = self.create_publisher(PointCloud2, "/lidar/points_noisy", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/lidar/points", self.cb, qos_profile_sensor_data)
        self.n = 0
        self.get_logger().info(f"a={100*self.a0:.3f} cm k={100*self.k:.4f} cm/m scale={self.scale}")

    def cb(self, m):
        dt = np.dtype({"names": [f.name for f in m.fields], "formats": [FMT[f.datatype] for f in m.fields],
                       "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
        p = np.frombuffer(bytes(m.data), dtype=dt).copy()
        x, y, z = (p[c].astype(np.float64) for c in "xyz")
        fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        r = np.sqrt(x * x + y * y + z * z)
        fin &= r > 0
        sig = self.scale * np.sqrt(self.a0 ** 2 + (self.k * r) ** 2)
        s = np.ones_like(r)
        s[fin] = (r[fin] + self.rng.normal(0.0, 1.0, int(fin.sum())) * sig[fin]) / r[fin]
        p["x"], p["y"], p["z"] = (x * s).astype(np.float32), (y * s).astype(np.float32), (z * s).astype(np.float32)
        o = PointCloud2()
        o.header, o.height, o.width, o.fields = m.header, m.height, m.width, m.fields
        o.is_bigendian, o.point_step, o.row_step, o.is_dense = m.is_bigendian, m.point_step, m.row_step, m.is_dense
        o.data = p.tobytes()
        self.pub.publish(o)
        self.n += 1

rclpy.init(); node = NoiseNode()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
