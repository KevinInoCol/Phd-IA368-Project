"""Utilidades compartidas: configuración de la base de carga y conversión mundo ↔ mapa."""

import math
import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


def default_map():
    return os.path.join(get_package_share_directory("pepper_museum"), "maps", "museum_nav.yaml")


def default_config():
    return os.path.join(get_package_share_directory("pepper_cleaning"), "config", "dock.yaml")


def load_dock(path=""):
    with open(path or default_config()) as f:
        cfg = yaml.safe_load(f)
    d = cfg["dock"]
    return {
        "contact": tuple(d["contact"]),
        "facing": math.radians(d.get("facing_deg", 0.0)),
        "predock": (d["predock"][0], d["predock"][1], math.radians(d["predock"][2])),
        "offset": tuple(cfg.get("map_offset", [1.5, -1.0])),
    }


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FreeMap:
    """Mapa de ocupación (yaml + pgm de map_server) para saber qué celdas son piso libre."""

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            m = yaml.safe_load(f)
        from PIL import Image
        img = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), m["image"])).convert("L"))
        self.res = m["resolution"]
        self.ox, self.oy = m["origin"][:2]
        self.h, self.w = img.shape
        self.free = img > 250            # trinary: 254 libre, 0 ocupado, 205 desconocido

    def clear(self, x, y, r):
        """Todas las celdas del cuadrado de semilado r alrededor de (x, y) son libres."""
        c0, c1 = int((x - r - self.ox) / self.res), int((x + r - self.ox) / self.res)
        r0, r1 = (self.h - 1 - int((y + r - self.oy) / self.res), self.h - 1 - int((y - r - self.oy) / self.res))
        if c0 < 0 or r0 < 0 or c1 >= self.w or r1 >= self.h:
            return False
        return bool(self.free[r0:r1 + 1, c0:c1 + 1].all())

    def free_at(self, x, y):
        """La celda de (x, y) es libre en el mapa (fuera del mapa: no)."""
        c = int((x - self.ox) / self.res)
        r = self.h - 1 - int((y - self.oy) / self.res)
        return 0 <= c < self.w and 0 <= r < self.h and bool(self.free[r, c])
