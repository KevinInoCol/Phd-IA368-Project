#!/usr/bin/env python3
"""Plano 2D del museo: corta la malla a una altura dada y rasteriza las paredes.

    python3 tools/museum_floorplan.py [z=0.2] [res=0.1]

Lee models/iscas_museum/meshes/ISCAS_museum.dae, aplica la pose del modelo (yaw −1.57 del
visual en model.sdf) y la del museo en los mundos (MUSEUM_POSE de convert_worlds.py), y escribe
maps/museum_walls.txt: una celda ocupada por línea, "ix iy" con x = ix·res, y = iy·res en el
frame del mundo. Lo usan la figura del HTML y el muestreo de poses de pepper_visitors.
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DAE = os.path.join(HERE, "..", "models", "iscas_museum", "meshes", "ISCAS_museum.dae")
OUT = os.path.join(HERE, "..", "maps", "museum_walls.txt")
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
VISUAL_YAW = -1.57
MUSEUM_XY = (1.86134, -2.16704)


def load_triangles(path):
    root = ET.parse(path).getroot()
    geoms = {}
    for g in root.iterfind(".//c:library_geometries/c:geometry", NS):
        m = g.find("c:mesh", NS)
        srcs = {s.get("id"): np.array(s.find("c:float_array", NS).text.split(), float).reshape(-1, 3)
                for s in m.findall("c:source", NS)
                if s.find("c:float_array", NS) is not None and s.find(".//c:accessor", NS).get("stride") == "3"}
        verts = {v.get("id"): srcs[i.get("source")[1:]]
                 for v in m.findall("c:vertices", NS)
                 for i in v.findall("c:input", NS) if i.get("semantic") == "POSITION"}
        tris = []
        for t in m.findall("c:triangles", NS):
            ins = t.findall("c:input", NS)
            stride = max(int(i.get("offset")) for i in ins) + 1
            vin = next(i for i in ins if i.get("semantic") == "VERTEX")
            idx = np.array(t.find("c:p", NS).text.split(), int).reshape(-1, stride)[:, int(vin.get("offset"))]
            tris.append(verts[vin.get("source")[1:]][idx].reshape(-1, 3, 3))
        if tris:
            geoms[g.get("id")] = np.concatenate(tris)
    out = []

    def walk(node, M):
        mt = node.find("c:matrix", NS)
        if mt is not None:
            M = M @ np.array(mt.text.split(), float).reshape(4, 4)
        for ig in node.findall("c:instance_geometry", NS):
            T = geoms.get(ig.get("url")[1:])
            if T is not None:
                h = np.concatenate([T, np.ones(T.shape[:2] + (1,))], 2) @ M.T
                out.append(h[..., :3])
        for ch in node.findall("c:node", NS):
            walk(ch, M)

    for vs in root.iterfind(".//c:visual_scene", NS):
        for n in vs.findall("c:node", NS):
            walk(n, np.eye(4))
    T = np.concatenate(out)
    c, s = math.cos(VISUAL_YAW), math.sin(VISUAL_YAW)
    x = T[..., 0] * c - T[..., 1] * s + MUSEUM_XY[0]
    y = T[..., 0] * s + T[..., 1] * c + MUSEUM_XY[1]
    return x, y, T[..., 2]


def main():
    z0 = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
    res = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1
    x, y, z = load_triangles(DAE)
    cells = set()
    for k in np.where((z.min(1) <= z0) & (z.max(1) >= z0))[0]:
        pts = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            za, zb = z[k, a], z[k, b]
            if (za - z0) * (zb - z0) < 0:
                t = (z0 - za) / (zb - za)
                pts.append((x[k, a] + t * (x[k, b] - x[k, a]), y[k, a] + t * (y[k, b] - y[k, a])))
        if len(pts) == 2:
            (x0, y0), (x1, y1) = pts
            n = max(1, int(math.hypot(x1 - x0, y1 - y0) / (res / 3)))
            for i in range(n + 1):
                cells.add((math.floor((x0 + (x1 - x0) * i / n) / res), math.floor((y0 + (y1 - y0) * i / n) / res)))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write(f"# res {res} z {z0}\n")
        f.writelines(f"{cx} {cy}\n" for cx, cy in sorted(cells))
    print(f"{OUT}: {len(cells)} celdas ocupadas (res {res} m, z {z0} m)")


if __name__ == "__main__":
    main()
