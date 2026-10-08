#!/usr/bin/env python3
"""From worlds/velodyne_flat.sdf: (1) nocube = lidar link without collision/visual box; (2) narrow120 = 120 deg x 671 samples."""
import os, re
import xml.etree.ElementTree as ET
d = os.path.expanduser("~/lidar_loop/worlds")
base = open(f"{d}/velodyne_flat.sdf").read()
def must(t, old, new):
    assert t.count(old) == 1, f"{old!r}: {t.count(old)} matches"
    return t.replace(old, new)
blk = re.search(r'<model name="model_with_lidar">.*?</model>', base, flags=re.S).group(0)
b2, n1 = re.subn(r'<collision name="collision">.*?</collision>', '', blk, count=1, flags=re.S)
b2, n2 = re.subn(r'<visual name="visual">.*?</visual>', '', b2, count=1, flags=re.S)
assert n1 == 1 and n2 == 1 and "<sensor" in b2
t = base.replace(blk, b2); ET.fromstring(t)
open(f"{d}/velodyne_flat_nocube.sdf", "w").write(t)
t = must(base, "<samples>2014</samples>", "<samples>671</samples>")
t = must(t, "<min_angle>-3.141593</min_angle>", "<min_angle>-1.047198</min_angle>")
t = must(t, "<max_angle>3.141593</max_angle>", "<max_angle>1.047198</max_angle>"); ET.fromstring(t)
open(f"{d}/velodyne_flat_narrow120.sdf", "w").write(t)
print("wrote nocube and narrow120 variants")
