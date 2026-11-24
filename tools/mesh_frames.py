#!/usr/bin/env python3
"""For every URDF link with a mesh, print where the mesh really is (bounding box in the
parent base_link frame) and where the mesh centre lies in the link's OWN frame.
A sensor frame should sit on its sensor: an offset of several cm means a wrong joint origin.

usage: python3 mesh_frames.py <path/to/robot.urdf> <path/to/meshes_dir>
  e.g. xacro testbed_description/urdf/testbed.xacro > /tmp/testbed.urdf
       python3 tools/mesh_frames.py /tmp/testbed.urdf testbed_description/meshes
(xacro needs the workspace sourced because the file uses $(find testbed_description))"""
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np


def load_stl(path):
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    rec = np.frombuffer(data[84:84 + n * 50],
                        dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return rec['v'].reshape(-1, 3).astype(float)


def xyz(elem):
    return np.array([float(v) for v in elem.get('xyz', '0 0 0').split()])


urdf, mesh_dir = sys.argv[1], sys.argv[2]
root = ET.parse(urdf).getroot()
joint_of = {j.find('child').get('link'): j for j in root.findall('joint')}
print(f"{'link':14s} {'joint xyz (parent)':24s} {'mesh centre in base_link':28s} {'mesh centre in own frame':28s} offset")
for link in root.findall('link'):
    mesh = link.find('visual/geometry/mesh')
    if mesh is None:
        continue
    scale = float(mesh.get('scale', '1 1 1').split()[0])
    verts = load_stl(f"{mesh_dir}/{mesh.get('filename').split('/')[-1]}") * scale
    verts += xyz(link.find('visual/origin'))                  # -> link frame
    centre_own = (verts.min(0) + verts.max(0)) / 2
    j = joint_of.get(link.get('name'))
    jxyz = xyz(j.find('origin')) if j is not None and j.find('parent').get('link') == 'base_link' else np.zeros(3)
    centre_base = centre_own + jxyz                           # all sensors hang off base_link
    off = np.linalg.norm(centre_own[:2]) * 100
    print(f"{link.get('name'):14s} {str(np.round(jxyz, 4)):24s} {str(np.round(centre_base, 4)):28s} "
          f"{str(np.round(centre_own, 4)):28s} {off:5.1f} cm")
