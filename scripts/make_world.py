#!/usr/bin/env python3
"""Derive worlds/velodyne_flat.sdf from Gazebo's gpu_lidar example. Every edit is asserted."""
import os, re
import xml.etree.ElementTree as ET
src = "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/gpu_lidar_sensor.sdf"
dst = os.path.expanduser("~/lidar_loop/worlds/velodyne_flat.sdf")
t = open(src).read()

def rep(old, new):
    global t
    n = t.count(old)
    assert n == 1, f"expected exactly 1 of {old!r}, found {n}"
    t = t.replace(old, new)

def box(name, y, w, retro):
    return (f'\n    <visual name="{name}">\n      <pose>0 {y} -0.05 0 0 0</pose>\n'
            f'      <geometry><box><size>200 {w} 0.1</size></box></geometry>\n'
            f'      <laser_retro>{retro}</laser_retro>\n'
            f'      <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>\n    </visual>')

# ground: three coplanar, non-overlapping boxes, top at z=0; bright band y in [2,5] (laser_retro 0.9)
ground = ('<model name="ground_plane">\n  <static>true</static>\n  <link name="link">'
          + box("ground_a", -49, 102, 0.3) + box("bright_band", 3.5, 3, 0.9) + box("ground_b", 52.5, 95, 0.3)
          + '\n  </link>\n</model>')
t, n = re.subn(r'<model name="ground_plane">.*?</model>', lambda m: ground, t, count=1, flags=re.S); assert n == 1
t, n = re.subn(r'<model name="box">.*?</model>', "", t, count=1, flags=re.S); assert n == 1

m = re.search(r'<model name="model_with_lidar">(.*?)</model>', t, flags=re.S)
assert m, "model_with_lidar not found"
blk = m.group(1)
print("model_with_lidar has <static>:", "<static>" in blk)
if "<static>" in blk:
    assert "true" in re.search(r"<static>(.*?)</static>", blk).group(1), "lidar model is non-static"
else:
    t = t.replace('<model name="model_with_lidar">', '<model name="model_with_lidar">\n      <static>true</static>', 1)

rep("<pose>4 0 0.5 0 0.0 3.14</pose>", "<pose>0 0 1.73 0 0 0</pose>")        # 1.73 m above ground top
rep("<samples>640</samples>", "<samples>2014</samples>")                      # ~0.179 deg azimuth step
rep("<min_angle>-1.396263</min_angle>", "<min_angle>-3.141593</min_angle>")   # 360 deg
rep("<max_angle>1.396263</max_angle>", "<max_angle>3.141593</max_angle>")
rep("<samples>16</samples>", "<samples>64</samples>")
rep("<min_angle>-0.261799</min_angle>", "<min_angle>-0.411732</min_angle>")   # -23.59 deg
rep("<max_angle>0.261799</max_angle>", "<max_angle>0.047124</max_angle>")     # +2.70 deg
rep("<max>10.0</max>", "<max>80.0</max>")
rep("<resolution>0.01</resolution>", "<resolution>0.0001</resolution>")       # range quantization 0.1 mm

ET.fromstring(t)   # well-formedness check
open(dst, "w").write(t)
print("wrote", dst, f"({len(t)} bytes)")
