#!/usr/bin/env python3
"""Draw the Gazebo world's collision geometry (sliced at lidar height) on top of the
occupancy map, to check that the map frame coincides with the Gazebo world frame.

usage: python3 map_vs_world.py <world.sdf> <map.yaml> <out.png> [slice_height_m]
  e.g. python3 tools/map_vs_world.py testbed_gazebo/worlds/testbed_playground.world \
           testbed_bringup/maps/testbed_world.yaml /tmp/overlay.png 0.25
red = world walls/boxes, blue = cylinders/spheres, black = map, green ring = world (0,0),
orange ring = robot spawn pose (0,5)."""
import sys, math
import numpy as np
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw
import yaml

world_file, map_yaml, out_png = sys.argv[1], sys.argv[2], sys.argv[3]
SLICE_Z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25

def parse_pose(txt):
    v = [float(x) for x in (txt or "0 0 0 0 0 0").split()]
    return v + [0.0] * (6 - len(v))

def compose(a, b):
    # 2.5D compose (yaw only, roll/pitch are zero everywhere in this world)
    x, y, z, _, _, yaw = a
    bx, by, bz, _, _, byaw = b
    c, s = math.cos(yaw), math.sin(yaw)
    return [x + c*bx - s*by, y + s*bx + c*by, z + bz, 0, 0, yaw + byaw]

root = ET.parse(world_file).getroot()
world = root.find("world")
state = world.find("state")
state_link_pose = {}
if state is not None:
    for m in state.findall("model"):
        for l in m.findall("link"):
            state_link_pose[(m.get("name"), l.get("name"))] = parse_pose(l.findtext("pose"))

shapes = []  # (kind, params)
for m in world.findall("model"):
    mname = m.get("name")
    if mname == "ground_plane":
        continue
    mpose = parse_pose(m.findtext("pose"))
    for l in m.findall("link"):
        key = (mname, l.get("name"))
        lpose = state_link_pose.get(key) or compose(mpose, parse_pose(l.findtext("pose")))
        for c in l.findall("collision"):
            cp = compose(lpose, parse_pose(c.findtext("pose")))
            g = c.find("geometry")
            if g.find("box") is not None:
                sx, sy, sz = [float(v) for v in g.find("box").findtext("size").split()]
                if cp[2] - sz/2 <= SLICE_Z <= cp[2] + sz/2:
                    shapes.append(("box", cp, sx, sy, mname + "/" + key[1]))
            elif g.find("cylinder") is not None or g.find("sphere") is not None:
                shape = g.find("cylinder") if g.find("cylinder") is not None else g.find("sphere")
                r = float(shape.findtext("radius"))
                shapes.append(("circle", cp, r, r, mname))

meta = yaml.safe_load(open(map_yaml))
img_path = meta["image"]
import os
if not os.path.isabs(img_path):
    img_path = os.path.join(os.path.dirname(map_yaml), img_path)
im = Image.open(img_path).convert("RGB")
W, H = im.size
res = meta["resolution"]; ox, oy = meta["origin"][:2]
scale = 2
im = im.resize((W*scale, H*scale), Image.NEAREST)
d = ImageDraw.Draw(im)

def w2p(x, y):
    return ((x - ox) / res * scale, (H - (y - oy) / res) * scale)

for kind, p, a, b, name in shapes:
    if kind == "box":
        c, s = math.cos(p[5]), math.sin(p[5])
        pts = []
        for dx, dy in [(-a/2,-b/2),(a/2,-b/2),(a/2,b/2),(-a/2,b/2)]:
            pts.append(w2p(p[0] + c*dx - s*dy, p[1] + s*dx + c*dy))
        d.polygon(pts, outline=(255, 0, 0))
    else:
        cx, cy = w2p(p[0], p[1]); r = a / res * scale
        d.ellipse([cx-r, cy-r, cx+r, cy+r], outline=(0, 0, 255))

# world origin + candidate spawn
for (x, y, col) in [(0, 0, (0, 160, 0)), (0, 5, (255, 140, 0))]:
    px, py = w2p(x, y)
    d.ellipse([px-6, py-6, px+6, py+6], outline=col, width=3)
im.save(out_png)
print(f"{len(shapes)} shapes intersect z={SLICE_Z}; map {W}x{H} px, res {res}, origin {ox},{oy}")
print(f"map covers x:[{ox:.2f},{ox+W*res:.2f}] y:[{oy:.2f},{oy+H*res:.2f}]")
