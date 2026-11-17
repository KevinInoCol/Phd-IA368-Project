#!/usr/bin/env python3
"""Mapa de navegación del museo: el mapa del hokuyo + lo que el láser no ve a la altura de Pepper.

    python3 tools/museum_nav_map.py [zmin=0.25] [zmax=1.25]

Los láseres van bajos, así que el mapa del hokuyo (maps/museum_hokuyo.*) no tiene los paneles
colgantes, los salientes de las paredes ni las vitrinas por encima de su plano, pero Pepper mide
1.2 m y sus hombros los golpean: el 2026-11-16 Pepper chocó con la punta de un panel colgante en
(−1.9, −5.7) del mapa y quedó doblado por la cadera. Esta herramienta corta la malla del museo
(ISCAS_museum.dae) cada 5 cm entre zmin y zmax, marca como ocupadas esas celdas y escribe
maps/museum_nav.{pgm,yaml} (mismo origen y resolución que el del hokuyo). Lo usan Nav2/AMCL en
cleaning_demo.launch.py y el planificador de limpieza.
"""

import math
import os
import sys

import numpy as np
import yaml
from PIL import Image

from museum_floorplan import DAE, load_triangles

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = os.path.join(HERE, "..", "maps")
MAP_OFFSET = (1.5, -1.0)   # map = mundo + offset


def main():
    zmin = float(sys.argv[1]) if len(sys.argv) > 1 else 0.25
    zmax = float(sys.argv[2]) if len(sys.argv) > 2 else 1.25
    with open(os.path.join(MAPS, "museum_hokuyo.yaml")) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(MAPS, meta["image"])))
    h, w = img.shape
    res = meta["resolution"]
    ox, oy = meta["origin"][:2]

    x, y, z = load_triangles(DAE)
    x, y = x + MAP_OFFSET[0], y + MAP_OFFSET[1]
    occ = np.zeros_like(img, dtype=bool)
    for z0 in np.arange(zmin, zmax + 1e-6, 0.05):
        for k in np.where((z.min(1) <= z0) & (z.max(1) >= z0))[0]:
            pts = []
            for a, b in ((0, 1), (1, 2), (2, 0)):
                za, zb = z[k, a], z[k, b]
                if (za - z0) * (zb - z0) < 0:
                    t = (z0 - za) / (zb - za)
                    pts.append((x[k, a] + t * (x[k, b] - x[k, a]), y[k, a] + t * (y[k, b] - y[k, a])))
            if len(pts) != 2:
                continue
            (x0, y0), (x1, y1) = pts
            n = max(1, int(math.hypot(x1 - x0, y1 - y0) / (res / 3)))
            for i in range(n + 1):
                c = int((x0 + (x1 - x0) * i / n - ox) / res)
                r = h - 1 - int((y0 + (y1 - y0) * i / n - oy) / res)
                if 0 <= c < w and 0 <= r < h:
                    occ[r, c] = True
    added = occ & (img > 250)
    out = img.copy()
    out[occ] = 0
    Image.fromarray(out).save(os.path.join(MAPS, "museum_nav.pgm"))
    with open(os.path.join(MAPS, "museum_nav.yaml"), "w") as f:
        f.write(f"# Mapa de navegación = museum_hokuyo + malla del museo entre {zmin} y {zmax} m\n"
                f"# (paneles colgantes, salientes y vitrinas que el láser no ve). tools/museum_nav_map.py\n")
        yaml.safe_dump({**meta, "image": "museum_nav.pgm"}, f, sort_keys=False)
    print(f"maps/museum_nav: {added.sum()} celdas nuevas ocupadas ({added.sum() * res * res:.2f} m²)")


if __name__ == "__main__":
    main()
